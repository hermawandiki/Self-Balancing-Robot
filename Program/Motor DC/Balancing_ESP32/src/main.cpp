#include <Arduino.h>
#include <stdio.h>
#include <EEPROM.h>
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
float Yaw, Pitch, Roll;
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// PID
double spPitch = 180.0;
double spYaw   = 180.0;
float kP, kI, kD;
double inputPitch, outputPitch, inputYaw, outputYaw;
PID pidPitch(&inputPitch, &outputPitch, &spPitch, kP, kI, kD, DIRECT);
PID pidYaw(&inputYaw, &outputYaw, &spYaw, KpYaw, KiYaw, KdYaw, DIRECT);

// Bluetooth
BluetoothSerial SerialBT;

// User Global Var
uint8_t i;
float offsetYaw = 0;
float offsetPitch = 0;
char c, buffer[100];
String rxData;
int addrP = 0;
int addrI = sizeof(float);
int addrD = 2*sizeof(float);

void saveFloatToEEPROM(int address, float value) {
  byte *dataPtr = (byte*) &value; // Ubah pointer float menjadi byte
  for (int i = 0; i < sizeof(float); i++) {
    EEPROM.write(address + i, dataPtr[i]); // Tulis setiap byte dari float ke EEPROM
  }
}

float readFloatFromEEPROM(int address) {
  float value = 0.0;
  byte *dataPtr = (byte*) &value;
  for (int i = 0; i < sizeof(float); i++) {
    dataPtr[i] = EEPROM.read(address + i); // Baca setiap byte dari EEPROM ke float
  }
  return value;
}

void parsingData(String data){
  if(data.startsWith("P")){
    kP = data.substring(1).toFloat();
    saveFloatToEEPROM(addrP, kP);
  }
  if(data.startsWith("I")){
    kI = data.substring(1).toFloat();
    saveFloatToEEPROM(addrI, kI);
  }
  if(data.startsWith("D")){
    kD = data.substring(1).toFloat();
    saveFloatToEEPROM(addrD, kD);
  }
}

void setup(){
  Wire.begin();
  Wire.setClock(400000);
  configIO();

  SerialBT.begin("RototagaBot");
  EEPROM.begin(512);

  kP = readFloatFromEEPROM(addrP);
  kI = readFloatFromEEPROM(addrI);
  kD = readFloatFromEEPROM(addrD);

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
    pidPitch.SetOutputLimits(-200, 200);
    pidYaw.SetMode(AUTOMATIC);
    pidYaw.SetSampleTime(10);
    pidYaw.SetOutputLimits(-100, 100);
  } else{}
  pidPitch.SetTunings(kP, kI, kD);
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

      spPitch = 180.0 - offsetPitch;
      spYaw  -= offsetYaw;
      if(spYaw>360) spYaw = 0;
      if(spYaw<0)   spYaw = 360;

      inputPitch = Pitch;
      inputYaw   = Yaw;
      pidPitch.Compute();
      pidYaw.Compute();
    }
    double PwmLeft  = outputPitch + outputYaw;
    double PwmRight = outputPitch - outputYaw;

    PwmLeft  = constrain(PwmLeft, -200, 200);
    PwmRight = constrain(PwmRight, -200, 200);

    if(inputPitch<150 || inputPitch>210) motor(0, 0);
    else motor(PwmLeft, PwmRight);
  }

  if(millis()%50==1){
    if (SerialBT.available()) {
      c = SerialBT.read();
    }

    if (c == '#'){
      while(c != '\n'){
        c = SerialBT.read();
        rxData += c;
      }
      parsingData(rxData);
      saveFloatToEEPROM(addrP, kP);
      saveFloatToEEPROM(addrI, kI);
      saveFloatToEEPROM(addrD, kD);
      pidPitch.SetTunings(kP, kI, kD);
      EEPROM.commit();
      rxData = "";
    }
    else if(c == 'W'){
      // Serial.println("Maju");
      if(offsetPitch > 2) offsetPitch = 2;
      else offsetPitch++;
    } 
    else if(c == 'S'){
      // Serial.println("Mundur");
      if(offsetPitch < -2) offsetPitch = -2;
      else offsetPitch--;
    } 
    else if(c == 'A'){
      // Serial.println("Kiri");
      offsetYaw++;
    } 
    else if(c == 'D'){
      // Serial.println("Kanan");
      offsetYaw--;
    } 
    else if(c == 'G'){
      // Serial.println("Grip");
    } 
    else if(c == 'R'){
      // Serial.println("Release");
    }
    else{
      offsetYaw = 0;
      if(offsetPitch>0){
        offsetPitch--;
        if (offsetPitch<0) offsetPitch = 0;
      }
      else if(offsetPitch<0){
        offsetPitch++;
        if (offsetPitch>0) offsetPitch = 0;
      }
    } 

    sprintf(buffer, "00|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|", Yaw, Pitch, Roll, kP, kI, kD);
    SerialBT.print(buffer);
    // Serial.println(buffer);
  }
}
