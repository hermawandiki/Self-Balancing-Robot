#include <Arduino.h>
#include <stdio.h>
#include "myDefinition.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "PID_v1.h"
#include <BluetoothSerial.h>

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
double originalSetpoint = 180.0;
double spPitch = originalSetpoint;
double spYaw = originalSetpoint;
double movingAngleOffset = 0.1;
double inputPitch, outputPitch, inputYaw, outputYaw;
PID pidPitch(&inputPitch, &outputPitch, &spPitch, KpPitch, KiPitch, KdPitch, DIRECT);
PID pidYaw(&inputYaw, &outputYaw, &spYaw, KpYaw, KiYaw, KdYaw, DIRECT);

// Bluetooth
BluetoothSerial SerialBT;

// User Global Var
uint8_t i;
bool flagS;
char buffer[100], rxData;

void setup(){
  Wire.begin();
  Wire.setClock(400000);
  configIO();

  SerialBT.begin("RototagaBot");

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
  if(millis()%10==0){
    if(!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Pitch = (ypr[2] * 180 / M_PI + 180) + 0.5;
      Roll  = (ypr[1] * 180 / M_PI + 180) + 0.5;
      Yaw   = (ypr[0] * 180 / M_PI + 180) + 0.5;

      inputPitch = Pitch;
      inputYaw = Yaw;
      pidPitch.Compute();
      pidYaw.Compute();
      // outputYaw = 0;
    }
    double PwmLeft  = outputPitch + outputYaw;
    double PwmRight = outputPitch - outputYaw;

    PwmLeft = constrain(PwmLeft, -255, 255);
    PwmRight = constrain(PwmRight, -255, 255);

    if(inputPitch<150 || inputPitch>210) motor(0, 0);
    else motor(PwmLeft, PwmRight);

    if(inputPitch<=180 && inputPitch>=179.7) BuzzOn;
    else BuzzOff;
  }

  if(millis()%33==0){
    if(SerialBT.available()){
      rxData = SerialBT.read();
    }
    if(rxData=='W')       ; // maju gan
    else if(rxData=='A')  ; // mutero kiri
    else if(rxData=='S')  ; // mundur cak
    else if(rxData=='D')  ; // mutero kanan
    else if(rxData=='G')  ; // grip servo
    else if(rxData=='R')  ; // release servo
    else if(rxData=='I')  ; // reset imu
    else {
    };

    SerialBT.print(Yaw);
    SerialBT.print("|");
    SerialBT.print(Pitch);
    SerialBT.print("|");
    SerialBT.print(Roll);
    SerialBT.print("|");
  }
  // sprintf(buffer, "encoderM1: %d, encoderM2: %d", encoderM1, encoderM2);
  // sprintf(buffer, "rxData: %c", rxData);
  // debugln(buffer);
}
