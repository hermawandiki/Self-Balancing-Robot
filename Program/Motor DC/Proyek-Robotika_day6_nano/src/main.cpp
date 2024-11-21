#include <Arduino.h>
#include <stdio.h>
#include "myDefinition.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "PID_v1.h"
#include <PinChangeInterrupt.h>
#include <Servo.h>

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
int Yaw, Pitch, Roll;
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// PID
double inputPitch, outputPitch, inputYaw, outputYaw;
PID pidPitch(&inputPitch, &outputPitch, &spPitch, KpPitch, KiPitch, KdPitch, DIRECT);
PID pidYaw(&inputYaw, &outputYaw, &spYaw, KpYaw, KiYaw, KdYaw, DIRECT);

// Servo
Servo servo1, servo2;

// User Global Var
uint8_t i;
bool flagS;
char buffer[100];
char rxData;

void setup(){
  configIO();
  Wire.begin();
  Wire.setClock(400000);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // mpu.initialize();
  // devStatus = mpu.dmpInitialize();
  // mpu.setXGyroOffset(220);
  // mpu.setYGyroOffset(76);
  // mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788);

  // if(devStatus == 0){
  //   mpu.CalibrateAccel(6);
  //   mpu.CalibrateGyro(6);
  //   mpu.PrintActiveOffsets();
  //   mpu.setDMPEnabled(true);
  //   attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(INT_PIN), dmpDataReady, RISING);
  //   mpuIntStatus = mpu.getIntStatus();
  //   dmpReady = true;
  //   packetSize = mpu.dmpGetFIFOPacketSize();

  //   pidPitch.SetMode(AUTOMATIC);
  //   pidPitch.SetSampleTime(10);
  //   pidPitch.SetOutputLimits(-255, 255);
  //   pidYaw.SetMode(AUTOMATIC);
  //   pidYaw.SetSampleTime(10);
  //   pidYaw.SetOutputLimits(-100, 100);
  // } else{}
  BuzzTit(3);
}

void loop(){
  Serial.write(0xFF);
  delay(1000);
  Serial.write(0x12);
  delay(1000);
}
