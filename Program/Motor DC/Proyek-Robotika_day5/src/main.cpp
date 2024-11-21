#include <Arduino.h>
#include <stdio.h>
#include "myDefinition.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "PID_v1.h"

// IMU
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
float Yaw, Pitch, Roll; // int
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// PID
double inputPitch, outputPitch, inputYaw, outputYaw;
PID pidPitch(&inputPitch, &outputPitch, &spPitch, KpPitch, KiPitch, KdPitch, DIRECT);
PID pidYaw(&inputYaw, &outputYaw, &spYaw, KpYaw, KiYaw, KdYaw, DIRECT);

// User Global Var
uint8_t i;
char buffer[100];

void setup(){
  Wire.begin();
  Wire.setClock(400000);
  configIO();

  ledcSetup(PWM_MKI_CHANNEL, 30000, 8);
  ledcSetup(PWM_MKA_CHANNEL, 30000, 8);
  ledcAttachPin(PWM_MKI, PWM_MKI_CHANNEL);
  ledcAttachPin(PWM_MKA, PWM_MKA_CHANNEL);

  mpu.initialize();
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
    attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pidPitch.SetMode(AUTOMATIC);
    pidPitch.SetSampleTime(10);
    pidPitch.SetOutputLimits(-255, 255);
    pidYaw.SetMode(AUTOMATIC);
    pidYaw.SetSampleTime(10);
    pidYaw.SetOutputLimits(-100, 100);
  } else{}
  BuzzTit(3);
}

void loop(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    inputPitch = (ypr[2] * 180 / M_PI + 180) + 0.5;
    inputYaw   = (ypr[0] * 180 / M_PI + 180) + 0.5;

    pidPitch.Compute();
    pidYaw.Compute();
    if(inputPitch<150 || inputPitch>210) motor(0, 0);
    else motor(outputPitch - outputYaw, outputPitch + outputYaw);
    if(inputPitch<=180 && inputPitch>=179.7) BuzzOn;
    else BuzzOff;
  }
  // sprintf(buffer, "m1 = %d, m2 = %d/n", encoderM1, encoderM2);
  delay(1);
}
