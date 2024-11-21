#include <Arduino.h>
#include "myDefinition.h"   

uint8_t lastStateA1, lastStateB1, lastStateA2, lastStateB2;
volatile long encoderM1, encoderM2;
double spPitch = 180, spYaw = 180;

void configIO(){
  if(DEBUG) Serial.begin(9600);
  pinMode(PWM_MKI, OUTPUT);
  pinMode(IN1_MKI, OUTPUT);
  pinMode(IN2_MKI, OUTPUT);
  pinMode(IN1_MKA, OUTPUT);
  pinMode(IN2_MKA, OUTPUT);
  pinMode(PWM_MKA, OUTPUT);
  pinMode(INT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(ENCA_MKI, INPUT);
  pinMode(ENCB_MKI, INPUT);
  pinMode(ENCA_MKA, INPUT);
  pinMode(ENCB_MKA, INPUT);

  lastStateA1 = digitalRead(ENCA_MKI);
  lastStateB1 = digitalRead(ENCB_MKI);
  lastStateA2 = digitalRead(ENCA_MKA);
  lastStateB2 = digitalRead(ENCB_MKA);

  attachInterrupt(digitalPinToInterrupt(ENCA_MKI), handleEncMki, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_MKA), handleEncMka, CHANGE);
}

void motor(int16_t pwm1, int16_t pwm2){
  if(pwm1>0){
    digitalWrite(IN1_MKI, HIGH);
    digitalWrite(IN2_MKI, LOW);
  }else{
    digitalWrite(IN1_MKI, LOW);
    digitalWrite(IN2_MKI, HIGH);
  }

  if(pwm2>0){
    digitalWrite(IN1_MKA, LOW);
    digitalWrite(IN2_MKA, HIGH);
  }else{
    digitalWrite(IN1_MKA, HIGH);
    digitalWrite(IN2_MKA, LOW);
  }

  pwm1 = constrain(abs(pwm1), 0, 255); // 0
  pwm2 = constrain(abs(pwm2)+10, 10, 255); // 10
  
  ledcWrite(PWM_MKI_CHANNEL, pwm1);
  ledcWrite(PWM_MKA_CHANNEL, pwm2);
}

void handleEncMki(){
  uint8_t stateA = digitalRead(ENCA_MKI);
  uint8_t stateB = digitalRead(ENCB_MKI);
  if(stateA != lastStateA1){
    if(stateB != stateA){
      encoderM1++;
    } else{
      encoderM1--;
    }
  }
  lastStateA1 = stateA;
}

void handleEncMka(){
  uint8_t stateA = digitalRead(ENCA_MKA);
  uint8_t stateB = digitalRead(ENCB_MKA);
  if(stateA != lastStateA2){
    if(stateB != stateA){
      encoderM2++;
    } else{
      encoderM2--;
    }
  }
  lastStateA1 = stateA;
}
