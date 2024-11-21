#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"

#define BuzzOn          digitalWrite(13, HIGH);
#define BuzzOff         digitalWrite(13, LOW);
#define BuzzTit(y)      {for(uint8_t ii=0; ii<y; ii++){BuzzOn; delay(30); BuzzOff; delay(30);}} 

SoftwareSerial blutut(A1, A0);
Servo RDS, SG; 

float Kp = 30;
float Ki = 1;
float Kd = 9;
float Moving_Speed = 200;
float Max_Speed = 500;

int left_motor, right_motor;
int Left_Motor_Speed, Right_Motor_Speed;
int CC_Speed_Left_Motor, CC_Speed_Right_Motor;
int Left_Motor_Speed_Prev, Right_Motor_Speed_Prev;

long Gyro_Y_Offset, Gyro_X_Offset;
unsigned long Loop_Time;
float Gyro_Angle, Acc_Angle;
float Temp_Error, PID_I, Setpoint, PID_Value, Last_D_Error;
float PID_Value_left, PID_Value_right;

MPU6050 mpu;
volatile bool mpuInterrupt = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 gy;
void dmpDataReady() {
  mpuInterrupt = true;
}
float Yaw = 0;
float Pitch = 0;
char cmd;
float offset;

void setup() {
  Serial.begin(115200);
  blutut.begin(115200);
  initIMU();

  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(13, OUTPUT);

  RDS.attach(9);
  SG.attach(10);

  TCCR2A = 0;               //Start with TCCR2A set to zero
  TCCR2B = 0;               //Start with TCCR2B set to zero
  TIMSK2 |= (1 << OCIE2A);  //Interupt enable bit OCIE2A set to 1
  TCCR2B |= (1 << CS21);    //Set CS21 bit: We set prescaler to 8
  OCR2A = 39;               //Compare register is 39, so...   20us/(1s/(16MHz/8))-1
  TCCR2A |= (1 << WGM21);   //Mode: Clear timer on compare

  BuzzTit(10);
  
  Loop_Time = micros() + 4000; //Loop time is 4000us
}

void loop() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    Yaw = gy.z;
    Pitch = ypr[2] * 180 / M_PI;

    calculatePID();
  }

  if (cmd == 'G') RDS.write(180);
  if (cmd == 'R') RDS.write(0);
  if (cmd == 'I') SG.write(180);
  if (cmd == 'J') SG.write(0);

  while (Loop_Time > micros());
  Loop_Time += 4000;
}










































void initIMU() {
  Wire.begin();
  TWBR = 12;
  mpu.initialize();
  pinMode(2, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(708);
  mpu.setYAccelOffset(-73);
  mpu.setZAccelOffset(1542);
  mpu.setXGyroOffset(72);
  mpu.setYGyroOffset(89);
  mpu.setZGyroOffset(13);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void receiveCmd() {
  if (blutut.available() > 0) {
    cmd = blutut.read();
    Serial.println(cmd);
  }
}

void calculatePID() {
  Temp_Error = Pitch - Setpoint - offset;
  if (PID_Value > 10 || PID_Value < -10) Temp_Error += PID_Value * 0.015 ;

  PID_I += Ki * Temp_Error;
  if (PID_I > 400)PID_I = 400;
  else if (PID_I < -400)PID_I = -400;

  PID_Value = Kp * Temp_Error + PID_I + Kd * (Temp_Error - Last_D_Error);
  if (PID_Value > 400)PID_Value = 400;
  else if (PID_Value < -400)PID_Value = -400;
  Last_D_Error = Temp_Error;

  if (PID_Value < 6 && PID_Value > - 6)PID_Value = 0;

  if (Pitch > 30 || Pitch < -30) PID_Value = 0;
  else PID_Value = PID_Value;

  PID_Value_left = PID_Value; //Get PID output for the left motor
  PID_Value_right = PID_Value; //Get PID output for the right motor

  receiveCmd();

  if (cmd == 'D') {
    PID_Value_left += Moving_Speed;
    PID_Value_right -= Moving_Speed;
  }
  else if (cmd == 'A') {
    PID_Value_left -= Moving_Speed;
    PID_Value_right += Moving_Speed;
  }

  else if (cmd == 'S') {
    if (offset > 5.5) offset = 5.5;
    else offset += 0.1;
  }

  else if (cmd == 'W') {
    if (offset < -5.5) offset = -5.5;
    else offset -= 0.1;
  }

  else {
    if (offset > 0) {
      offset -= 0.1;
      if (offset < 0) offset = 0.0;
    }
    if (offset < 0) {
      offset += 0.1;
      if (offset > 0) offset = 0.0;
    }
  }

  if (PID_Value_left > 0) PID_Value_left = 405 - (1 / (PID_Value_left + 9)) * 5500;
  else if (PID_Value_left < 0) PID_Value_left = -405 - (1 / (PID_Value_left - 9)) * 5500;

  if (PID_Value_right > 0) PID_Value_right = 405 - (1 / (PID_Value_right + 9)) * 5500;
  else if (PID_Value_right < 0) PID_Value_right = -405 - (1 / (PID_Value_right - 9)) * 5500;

  //Calculate the pulse time for the left and right motor
  if (PID_Value_left > 0) left_motor = 400 - PID_Value_left;
  else if (PID_Value_left < 0) left_motor = -400 - PID_Value_left;
  else left_motor = 0;

  if (PID_Value_right > 0) right_motor = 400 - PID_Value_right;
  else if (PID_Value_right < 0) right_motor = -400 - PID_Value_right;
  else right_motor = 0;

  Left_Motor_Speed = left_motor;
  Right_Motor_Speed = right_motor;
}


ISR(TIMER2_COMPA_vect) {
  // Left motor pulses
  CC_Speed_Left_Motor++;
  if (CC_Speed_Left_Motor > Left_Motor_Speed_Prev) {
    CC_Speed_Left_Motor = 0;
    Left_Motor_Speed_Prev = Left_Motor_Speed;

    if (Left_Motor_Speed_Prev < 0) {
      PORTD |= (1 << 4);  // D4 HIGH
      Left_Motor_Speed_Prev *= -1;
    } else {
      PORTD &= ~(1 << 4); // D4 LOW
    }
  }
  if (CC_Speed_Left_Motor == 1) PORTD |= (1 << 3);   // D3 HIGH
  else if (CC_Speed_Left_Motor == 2) PORTD &= ~(1 << 3); // D3 LOW

  // Right motor pulses
  CC_Speed_Right_Motor++;
  if (CC_Speed_Right_Motor > Right_Motor_Speed_Prev) {
    CC_Speed_Right_Motor = 0;
    Right_Motor_Speed_Prev = Right_Motor_Speed;

    if (Right_Motor_Speed_Prev < 0) {
      PORTD &= ~(1 << 6); // D6 LOW
      Right_Motor_Speed_Prev *= -1;
    } else {
      PORTD |= (1 << 6);  // D6 HIGH
    }
  }
  if (CC_Speed_Right_Motor == 1) PORTD |= (1 << 5);    // D5 HIGH
  else if (CC_Speed_Right_Motor == 2) PORTD &= ~(1 << 5); // D5 LOW
}
