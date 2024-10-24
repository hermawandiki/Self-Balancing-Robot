#ifndef __MYDEFINITION_H__
#define __MYDEFINITION_H__

#include <Arduino.h>

#define DEBUG 0

#if DEBUG == 0
#define debug(x) monitor.print(x);
#define debugln(x) monitor.println(x);
#else
#define debug(x)
#define debugln(x)
#endif

// PID
#define KpPitch 17.0  // 10 // 16 // 16 // 16
#define KdPitch 0.8  // 0.01 // 0.7 // 0.9 / 0.7
#define KiPitch 194.5  // 30 // 77 // 95 // 153

#define KpYaw 14.0
#define KdYaw 0.3
#define KiYaw 20.02

#define KpPos 0.0
#define KdPos 0.0
#define KiPos 0.0

// Motor Driver
#define PWM_MKI 9
#define IN1_MKI A3
#define IN2_MKI A2
#define IN1_MKA A1
#define IN2_MKA A0
#define PWM_MKA 10

// Serial Communication
#define RX_PIN_BT 0
#define TX_PIN_BT 1
#define RX_MONITOR 11
#define TX_MONITOR 12

// MPU6050
#define INT_PIN 4

// Encoder
#define ENCA_MKI 2
#define ENCB_MKI 7
#define ENCA_MKA 3
#define ENCB_MKA 8

// Servo
#define SERVO1_PIN 5
#define SERVO2_PIN 6

// Buzzer
#define BUZZER_PIN 13
#define BuzzOn digitalWrite(BUZZER_PIN, HIGH)
#define BuzzOff digitalWrite(BUZZER_PIN, LOW)
#define BuzzTit(y) {for(uint8_t ii=0; ii<y; ii++){BuzzOn; delay(70); BuzzOff; delay(70);}}

// Global Variable
extern uint8_t lastStateA1, lastStateB1, lastStateA2, lastStateB2;
extern volatile int encoderM1, encoderM2;
extern double spPitch, spYaw;

// Function Prototype
extern void configIO();
extern void motor(int16_t pwm1, int16_t pwm2);
extern void handleEncMki();
extern void handleEncMka();

#endif