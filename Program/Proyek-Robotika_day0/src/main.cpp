#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
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
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

unsigned int Yaw, Pitch, Roll;
char buffRx;
bool flag = 0, konek = 0;
long lastTime = 0;
uint8_t step = 0;

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

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if(devStatus == 0){
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else{
    // ga lapo-lapo
  }

  pinMode(LED_PIN, OUTPUT);

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

  step = 1;  
}

void loop() {
  if(!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {}
  else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Roll  = (int)(((ypr[2] * 180 / M_PI) + 180.0) + 0.5);
    Pitch = (int)(((ypr[1] * 180 / M_PI) + 180.0) + 0.5);
    Yaw   = (int)(((ypr[0] * 180 / M_PI) + 180.0) + 0.5);

    // blinkState = !blinkState;
    // digitalWrite(LED_PIN, blinkState);
  }

  // ==================================================================== USER CODE BEGIN HERE ====================================================================
  
  Serial.print("Yaw: ");
  Serial.print(Yaw);
  Serial.print(" | Pitch: ");
  Serial.print(Pitch);
  Serial.print(" | Roll: ");
  Serial.println(Roll);

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

  // if(blutut.available()){
  //   buffRx = blutut.read();
  //   Serial.println(buffRx);
  // }

  // if(millis() - lastTime > 50){
  //   blutut.print(Yaw);
  //   blutut.print("|");
  //   blutut.print(Pitch);
  //   blutut.print("|");
  //   blutut.print(Roll);
  //   blutut.println("|");
  //   lastTime = millis();
  // }

  // if(blutut.connected() && konek==0){
  //   blutut.println("Enter pin to start!");
  //   konek = 1;
  // }
  // if(!blutut.connected()){
  //   konek = 0;
  //   flag = 0;
  // }

  // if(blutut.available()){
  //   buffRx = blutut.read();
  //   Serial.println(buffRx);
  //   if(buffRx == '~') {
  //     blutut.println("Started...");
  //     flag = 1;
  //   }else {
  //     flag = 0;
  //     blutut.println("Can't start, try again!");
  //   }
  // }
  
  // while(flag){
  //   if(blutut.available()){
  //     buffRx = blutut.read();
  //     Serial.println(buffRx);
  //     if(buffRx == '`') {
  //       blutut.println("Stopped...");
  //       flag = 0;
  //     }
  //   }
  //   if(!blutut.connected()){
  //     konek = 0;
  //     flag = 0;
  //   }
    
  //   // code begin here
  //   if(millis() - lastTime > 1000){
  //     blutut.println("Balancing...");
  //     lastTime = millis();
  //   }
  //   // code end here
  // }
}

