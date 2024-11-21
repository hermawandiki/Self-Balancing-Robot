#ifndef __MYDEFINITION_H__
#define __MYDEFINITION_H__

#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x);
#define debugln(x) Serial.println(x);
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

// Motor Driver
#define PWM_MKI 15
#define IN1_MKI 2
#define IN2_MKI 4
#define IN1_MKA 16
#define IN2_MKA 17
#define PWM_MKA 5

#define PWM_MKI_CHANNEL 0
#define PWM_MKA_CHANNEL 1

// MPU6050
#define INT_PIN 23
#define LED_PIN 2

// Encoder
#define ENCA_MKI 13
#define ENCB_MKI 12
#define ENCA_MKA 14
#define ENCB_MKA 27

// Servo
#define SERVO1_PIN 18
#define SERVO2_PIN 19

// Buzzer
#define BUZZER_PIN 26
#define BuzzOn digitalWrite(BUZZER_PIN, HIGH)
#define BuzzOff digitalWrite(BUZZER_PIN, LOW)
#define BuzzTit(y) {for(uint8_t ii=0; ii<y; ii++){BuzzOn; delay(70); BuzzOff; delay(70);}}

// Global Variable

// Function
extern void configIO();
extern void motor(int16_t pwm1, int16_t pwm2);

#endif

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