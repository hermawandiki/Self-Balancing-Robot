#include <Arduino.h>
#include "PID_v1.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "ESP32Encoder.h"

MPU6050 mpu;
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
int Yaw, Pitch, Roll;

double originalSetpoint = 180.0;
double setpointPitch = originalSetpoint;
double setpointYaw = originalSetpoint;
double movingAngleOffset = 0.1;
double inputPitch, outputPitch, inputYaw, outputYaw;

double KpPitch = 17; // 10 // 16 // 16 // 16
double KdPitch = 0.8; // 0.01 // 0.7 // 0.9 / 0.7
double KiPitch = 194.5; // 30 // 77 // 95 // 153
PID pidPitch(&inputPitch, &outputPitch, &setpointPitch, KpPitch, KiPitch, KdPitch, DIRECT);
double KpYaw = 14;
double KdYaw = 0.3;
double KiYaw = 20.02;
PID pidYaw(&inputYaw, &outputYaw, &setpointYaw, KpYaw, KiYaw, KdYaw, DIRECT);

ESP32Encoder encM1, encM2;
uint8_t pinMotor[] = {15, 2, 4, 16, 17, 5};
void motor(int16_t pwm1, int16_t pwm2){
  if(pwm1>0){
    digitalWrite(2, HIGH);
    digitalWrite(4, LOW);
  }else{
    digitalWrite(2, LOW);
    digitalWrite(4, HIGH);
  }

  if(pwm2>0){
    digitalWrite(16, LOW);
    digitalWrite(17, HIGH);
  }else{
    digitalWrite(16, HIGH);
    digitalWrite(17, LOW);
  }

  pwm1 = constrain(abs(pwm1), 0, 255); // 0
  pwm2 = constrain(abs(pwm2)+10, 10, 255); // 10
  
  ledcWrite(0, pwm1);
  ledcWrite(1, pwm2);
}
//
uint8_t i;

void setup(){
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);
  for(int i=0; i<6; i++){
    pinMode(pinMotor[i], OUTPUT);
  }

  ledcSetup(0, 30000, 8);
  ledcSetup(1, 30000, 8);
  ledcAttachPin(15, 0);
  ledcAttachPin(5, 1);

  mpu.initialize();
  pinMode(23, INPUT);
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
    attachInterrupt(digitalPinToInterrupt(23), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pidPitch.SetMode(AUTOMATIC);
    pidPitch.SetSampleTime(10);
    pidPitch.SetOutputLimits(-255, 255);
    pidYaw.SetMode(AUTOMATIC);
    pidYaw.SetSampleTime(10);
    pidYaw.SetOutputLimits(-100, 100);
  } else{
    // ga lapo-lapo
  }

  pinMode(26, OUTPUT);
  digitalWrite(26, LOW);
  for(uint8_t i=0; i<6;i++){
    digitalWrite(26, !digitalRead(26));
    delay(70);
  }
  
  encM1.attachFullQuad(13, 12);
  encM2.attachFullQuad(14, 27);
}

void loop(){
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Pitch = (int)(((ypr[2] * 180 / M_PI) + 180.0) + 0.5);  // depan++ belakang--
    // Roll = (int)(((ypr[1] * 180 / M_PI) + 180.0) + 0.5);  // kanan++ kiri--
    // Yaw = (int)(((ypr[0] * 180 / M_PI) + 180.0) + 0.5); // kanan++ kiri--

    // Serial.print(Yaw);
    // Serial.print('\t');
    // Serial.print(Pitch);
    // Serial.print('\t');
    // Serial.print(Roll);
    // Serial.print('\n');

    inputPitch = (ypr[2] * 180 / M_PI + 180) + 0.5;
    inputYaw = (ypr[0] * 180 / M_PI + 180) + 0.5;
  }
  pidPitch.Compute();
  pidYaw.Compute();
  motor(outputPitch - outputYaw, outputPitch + outputYaw);
  if(inputPitch<=180 && inputPitch>=179.7) digitalWrite(26, HIGH);
  else digitalWrite(26, LOW);

  // Serial.print(encM1.getCount());
  // Serial.print("  ");
  // Serial.println(encM2.getCount());
}

////// Recommended servo pins include 2,4,12-19,21-23,25-27,32-33
/*
** ledc: 0  => Group: 0, Channel: 0, Timer: 0
** ledc: 1  => Group: 0, Channel: 1, Timer: 0
** ledc: 2  => Group: 0, Channel: 2, Timer: 1
** ledc: 3  => Group: 0, Channel: 3, Timer: 1
** ledc: 4  => Group: 0, Channel: 4, Timer: 2
** ledc: 5  => Group: 0, Channel: 5, Timer: 2
** ledc: 6  => Group: 0, Channel: 6, Timer: 3
** ledc: 7  => Group: 0, Channel: 7, Timer: 3
** ledc: 8  => Group: 1, Channel: 8, Timer: 0
** ledc: 9  => Group: 1, Channel: 9, Timer: 0
** ledc: 10 => Group: 1, Channel: 10, Timer: 1
** ledc: 11 => Group: 1, Channel: 11, Timer: 1
** ledc: 12 => Group: 1, Channel: 12, Timer: 2
** ledc: 13 => Group: 1, Channel: 13, Timer: 2
** ledc: 14 => Group: 1, Channel: 14, Timer: 3
** ledc: 15 => Group: 1, Channel: 15, Timer: 3
*/