#include <Arduino.h>
#include "myDefinition.h"   

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