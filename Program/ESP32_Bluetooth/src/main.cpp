#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial blutut;
int lastTime = 0;
char buffRx;

void setup() {
  Serial.begin(115200);
  blutut.begin("Robot Dinasti");
  lastTime = millis();
  pinMode(2, OUTPUT);
  pinMode(26, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(26, LOW);
}

void loop() {
  if(blutut.available()){
    buffRx = blutut.read();
    // Serial.println(buffRx);
  }

  if(millis() - lastTime > 50){
    blutut.print(random(100, 180));
    blutut.print("|");
    blutut.print(random(100, 180));
    blutut.print("|");
    blutut.print(random(100, 180));
    blutut.println("|");
    lastTime = millis();
  }

  switch(buffRx){
    case 'W':
      Serial.println("Forward");
      break;
    case 'A':
      Serial.println("Turn Left");
      break;
    case 'S':
      Serial.println("Backward");
      break;
    case 'D':
      Serial.println("Turn Right");
      break;
    case 'G':
      Serial.println("Grip");
      digitalWrite(2, HIGH);
      break;
    case 'R':
      Serial.println("Release");
      digitalWrite(2, LOW);
      break;
    case 'I':
      Serial.println("Reset IMU");
      digitalWrite(26, HIGH);
      break;
    default:
      Serial.println(buffRx);
      digitalWrite(26, LOW);
      break;
  }
}

