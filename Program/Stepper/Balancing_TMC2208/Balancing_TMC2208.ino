#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"

#define BuzzOn          digitalWrite(13, HIGH);
#define BuzzOff         digitalWrite(13, LOW);
#define BuzzTit(y)      {for(uint8_t ii=0; ii<y; ii++){BuzzOn; delay(30); BuzzOff; delay(30);}}

SoftwareSerial blutut(A1, A0);
Servo RDS, SG;
uint8_t stateRDS, stateSG, stateGrip;
float offsetServo;

#define Gripper_Naik      { RDS.write(0); stateRDS = 1; }
#define Gripper_Turun     { RDS.write(60); stateRDS = 0; }
#define Gripper_Capit     { SG.write(60); stateSG = 1; }
#define Gripper_Buka      { SG.write(100); stateSG = 0; }
#define Ambil_Objek       { Gripper_Capit; offsetServo = 14.0; delay(100); Gripper_Naik; offsetServo = 7.0; stateGrip = 1; }
#define Taruh_Objek       { Gripper_Turun; offsetServo = 14.0; stateGrip = 2; }
#define Lepas_Objek       { Gripper_Buka; stateGrip = 3; }

float Kp = 20;
float Ki = 3;
float Kd = 9;
float Rotate_Speed = 200;
float Max_Speed = 800;

int left_motor, right_motor;
int Left_Motor_Speed, Right_Motor_Speed;
int CC_Speed_Left_Motor, CC_Speed_Right_Motor;
int Left_Motor_Speed_Prev, Right_Motor_Speed_Prev;

unsigned long Loop_Time;
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
  Gripper_Naik;
  Gripper_Buka;

  TCCR2A = 0;               //Start with TCCR2A set to zero
  TCCR2B = 0;               //Start with TCCR2B set to zero
  TIMSK2 |= (1 << OCIE2A);  //Interupt enable bit OCIE2A set to 1 (Output Compare Match A)
  TCCR2B |= (1 << CS21);    //Set CS21 bit: We set prescaler to 8
  OCR2A = 39;               //Compare register is 39, so...   20us/(1s/(16MHz/8))-1
  TCCR2A |= (1 << WGM21);   //Mode: Clear timer (TCNT) on compare (CTC)

  offsetServo = -2.0;
  // -2 tanpa pipa
  // 6 pipa diatas
  // 12 pipa dibawah

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

    //    Yaw = gy.z;
    Pitch = ypr[2] * 180 / M_PI;

    calculateSpeed();
  }

  Serial.println(cmd);

  if (cmd == 'G') {
    if (!stateRDS) {
      Gripper_Naik;
    }
    else Gripper_Turun;
    delay(100);
  }
  if (cmd == 'R') {
    if (!stateSG) {
      Gripper_Capit;
    }
    else Gripper_Buka;
    delay(100);
  }
  if (cmd == 'I') Ambil_Objek;
  if (cmd == 'J') {
    if (stateGrip == 1) {
      Taruh_Objek;
    } else if (stateGrip == 2) {
      Lepas_Objek;
    }
    else ;
    delay(100);
  }

  if (stateGrip == 3) {
    if (offsetServo < -2) {
      offsetServo = -2;
      stateGrip = 0;
    }
    else offsetServo -= 0.1;
  }

  while (Loop_Time > micros());
  Loop_Time += 4000;
}

//////////////////////////////////////////  USER FUNCTION BEGIN ///////////////////////////////////////

void initIMU() {
  Wire.begin();
  TWBR = 12; // Fast-Mode || 400 kHz

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
  }
}

void calculateSpeed() {
  Temp_Error = Pitch - (Setpoint + offsetServo) - offset;
  if (PID_Value > 6 || PID_Value < -6) Temp_Error += PID_Value * 0.015 ;

  PID_I += Ki * Temp_Error;
  if (PID_I > Max_Speed)        PID_I = Max_Speed;
  else if (PID_I < -Max_Speed)  PID_I = -Max_Speed;

  PID_Value = Kp * Temp_Error + PID_I + Kd * (Temp_Error - Last_D_Error);
  if (PID_Value > Max_Speed)        PID_Value = Max_Speed;
  else if (PID_Value < -Max_Speed)  PID_Value = -Max_Speed;
  Last_D_Error = Temp_Error;

  if (PID_Value < 6 && PID_Value > - 6) PID_Value = 0;

  if (Pitch > 35 || Pitch < -35) PID_Value = 0;
  else PID_Value = PID_Value;

  PID_Value_left  = PID_Value; // Get PID output for the left motor
  PID_Value_right = PID_Value; // Get PID output for the right motor

  receiveCmd();


  if (cmd == 'D') {
    PID_Value_left  += Rotate_Speed;
    PID_Value_right -= Rotate_Speed;
  }
  else if (cmd == 'A') {
    PID_Value_left  -= Rotate_Speed;
    PID_Value_right += Rotate_Speed;
  }
  else if (cmd == 'S') {
    if (offset > 9.5) offset = 9.5;
    else offset += 0.5;
  }
  else if (cmd == 'W') {
    if (offset < -9.5) offset = -9.5;
    else offset -= 0.5;
  }
  else {
    if (offset > 0) {
      offset -= 0.5;
      if (offset < 0) offset = 0.0;
    }
    if (offset < 0) {
      offset += 0.5;
      if (offset > 0) offset = 0.0;
    }
  }

  if (PID_Value_left > 0)       PID_Value_left =  Max_Speed - (1 / (PID_Value_left + 9)) * 5500;
  else if (PID_Value_left < 0)  PID_Value_left = -Max_Speed - (1 / (PID_Value_left - 9)) * 5500;

  if (PID_Value_right > 0)      PID_Value_right =  Max_Speed - (1 / (PID_Value_right + 9)) * 5500;
  else if (PID_Value_right < 0) PID_Value_right = -Max_Speed - (1 / (PID_Value_right - 9)) * 5500;

  //Calculate the pulse time for the left and right motor
  if (PID_Value_left > 0)       left_motor =  Max_Speed - PID_Value_left;
  else if (PID_Value_left < 0)  left_motor = -Max_Speed - PID_Value_left;
  else left_motor = 0;

  if (PID_Value_right > 0)      right_motor =  Max_Speed - PID_Value_right;
  else if (PID_Value_right < 0) right_motor = -Max_Speed - PID_Value_right;
  else right_motor = 0;

  Left_Motor_Speed  = left_motor;
  Right_Motor_Speed = right_motor;
}

////////////////////////////////////////// END OF USER FUNCTION ///////////////////////////////////////

///////////////////////////////////////////  TIMER ROUTINE BEGIN ////////////////////////////////////

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
  if (CC_Speed_Left_Motor == 1)       PORTD |= (1 << 3);    // D3 HIGH
  else if (CC_Speed_Left_Motor == 2)  PORTD &= ~(1 << 3);   // D3 LOW

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
  if (CC_Speed_Right_Motor == 1)      PORTD |= (1 << 5);    // D5 HIGH
  else if (CC_Speed_Right_Motor == 2) PORTD &= ~(1 << 5);   // D5 LOW
}

///////////////////////////////////////////  END OF TIMER ROUTINE ////////////////////////////////////
