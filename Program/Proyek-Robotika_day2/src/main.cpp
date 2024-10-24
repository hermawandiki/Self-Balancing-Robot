#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "math.h"

#define ENCA_MKa 13
#define ENCB_MKa 12
#define ENCA_MKi 14
#define ENCB_MKi 27

#define IN1_MKa 15
#define IN2_MKa 2
#define PWM_MKa 4

#define IN1_MKi 16
#define IN2_MKi 17
#define PWM_MKi 5

#define ServoPin1 18
#define ServoPin2 19

BluetoothSerial blutut;
MPU6050 mpu;
Servo servo1, servo2;
ESP32Encoder encoder_MKa;
ESP32Encoder encoder_MKi;

const int PWM_CHANNEL_MKa = 2;
const int PWM_CHANNEL_MKi = 3;
const int PWM_FREQ = 30000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX_DUTY = (int)(pow(2, PWM_RESOLUTION) - 1);

#define INTERRUPT_PIN 23
#define LED_PIN 2
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

unsigned int Yaw, Pitch, Roll;
char buffRx;
bool flag = 0, konek = 0;
long lastTime = 0;
uint8_t step = 0;

double originalSP = 180.0;
double SP = originalSP;
double input, output;
double Kp = 10.0;
double Ki = 1.0;
double Kd = 0.001;
PID pid(&input, &output, &SP, Kp, Ki, Kd, DIRECT);
int error, lastError;

void motor(int pwm1, int pwm2){
  if(pwm1>0){
    digitalWrite(IN1_MKi, HIGH);
    digitalWrite(IN2_MKi, LOW);
  }else{
    digitalWrite(IN1_MKi, LOW);
    digitalWrite(IN2_MKi, HIGH);
  }

  if(pwm2>0){
    digitalWrite(IN1_MKa, HIGH);
    digitalWrite(IN2_MKa, LOW);
  }else{
    digitalWrite(IN1_MKa, LOW);
    digitalWrite(IN2_MKa, HIGH);
  }

  pwm1 = constrain(abs(pwm1) + 10, 10, 255);
  pwm2 = constrain(abs(pwm2) + 10, 10, 255);
  
  ledcWrite(PWM_CHANNEL_MKi, pwm1);
  ledcWrite(PWM_CHANNEL_MKa, pwm2);
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  blutut.begin("Robot Dinasti");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  // mpu.setXGyroOffset(0);
  // mpu.setYGyroOffset(0);
  // mpu.setZGyroOffset(0);
  // mpu.setZAccelOffset(0);

  if(devStatus == 0){
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else{
    // ga lapo-lapo
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo1.attach(ServoPin1, 500, 2500);
  servo2.setPeriodHertz(50);
  servo2.attach(ServoPin2, 500, 2500);
  
  pinMode(IN1_MKa, OUTPUT);
  pinMode(IN2_MKa, OUTPUT);
  pinMode(IN1_MKi, OUTPUT);
  pinMode(IN2_MKi, OUTPUT);
  ledcSetup(PWM_CHANNEL_MKa, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_MKi, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_MKa, PWM_CHANNEL_MKa);
  ledcAttachPin(PWM_MKi, PWM_CHANNEL_MKi);

  for(uint8_t i=0; i<7;i++){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  step = 1;  
}

void loop() {
  if(!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // 
    }
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if((mpuIntStatus & 0x10) || fifoCount == 1024){
    mpu.resetFIFO();
  } else if(mpuIntStatus & 0x02){
    while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Roll  = (int)(((ypr[2] * 180 / M_PI) + 180.0) + 0.5);
    // Pitch = (int)(((ypr[1] * 180 / M_PI) + 180.0) + 0.5);
    // Yaw   = (int)(((ypr[0] * 180 / M_PI) + 180.0) + 0.5);
    input = (int)(((ypr[1] * 180 / M_PI) + 180.0) + 0.5);
    pid.Compute();
  }

  // ==================================================================== USER CODE BEGIN HERE ====================================================================
  error = SP - input;
  int P = Kp * error;
  int I = Ki * (error + lastError);
  int D = Kd * (error - lastError);
  lastError = error;
  int outPID = constrain(P+I+D, -255, 255);

  Serial.print("Pitch : ");
  Serial.print(input);
  Serial.print("\t");
  Serial.print("Output PID1 : ");
  Serial.print(output);
  Serial.print("\t");
  Serial.print("Output PID2 : ");
  Serial.println(outPID);

  // Serial.print("Yaw: ");
  // Serial.print(Yaw);
  // Serial.print(" | Pitch: ");
  // Serial.print(Pitch);
  // Serial.print(" | Roll: ");
  // Serial.println(Roll);

  // if(millis() - lastTime > 1000 && step == 1){
  //   servo1.write(0);
  //   servo2.write(0);
  //   ledcWrite(PWM_CHANNEL_MKi, 150);
  //   ledcWrite(PWM_CHANNEL_MKa, 150);
  //   digitalWrite(IN1_MKa, HIGH);
  //   digitalWrite(IN2_MKa, HIGH);
  //   digitalWrite(IN1_MKi, HIGH);
  //   digitalWrite(IN2_MKi, HIGH);
  //   digitalWrite(LED_PIN, HIGH);
  //   lastTime = millis();
  //   step = 2;
  // }
  // if(millis() - lastTime > 1000 && step == 2){
  //   servo1.write(180);
  //   servo2.write(180);
  //   ledcWrite(PWM_CHANNEL_MKi, 50);
  //   ledcWrite(PWM_CHANNEL_MKa, 50);
  //   digitalWrite(IN1_MKa, LOW);
  //   digitalWrite(IN2_MKa, LOW);
  //   digitalWrite(IN1_MKi, LOW);
  //   digitalWrite(IN2_MKi, LOW);
  //   digitalWrite(LED_PIN, LOW);
  //   lastTime = millis();
  //   step = 1;
  // }

  // ===================================================================== USER CODE END HERE =====================================================================
}

