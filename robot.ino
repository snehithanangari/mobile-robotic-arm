#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
BluetoothSerial SerialBT;
// === DC MOTOR PINS ===
int motor1Pin11 = 22; // LEFT FRONT MOTOR IN1
int motor1Pin12 = 23; // LEFT FRONT MOTOR IN2
int motor2Pin21 = 18; // RIGHT FRONT MOTOR IN3
int motor2Pin22 = 19; // RIGHT FRONT MOTOR IN4
int motor1_en = 21; // ENA
int motor2_en = 5;  // ENB
int dutyCycle1 = 200; // PWM duty cycle for motor 1
int dutyCycle2 = 200; // PWM duty cycle for motor 2
// === SERVO SETTINGS ===
const int servoPins[5] = {13, 12, 14, 27, 26};
Servo servos[5];
// === INPUT BUFFER ===
String inputString = "";
bool stringComplete = false;
void setup() {
  Serial.begin(115200);
  SerialBT.begin("esp32_agri_robot");
  Serial.println("Bluetooth device ready!");
  // DC Motor setup
  pinMode(motor1Pin11, OUTPUT);
  pinMode(motor1Pin12, OUTPUT);
  pinMode(motor2Pin21, OUTPUT);
  pinMode(motor2Pin22, OUTPUT);
  ledcAttach(motor1_en, 30000, 8); // 30kHz, 8-bit
  ledcAttach(motor2_en, 30000, 8);
  // Servo Setup
  for (int i = 0; i < 5; i++) {
    servos[i].attach(servoPins[i]);
  }
}
void loop() {
  while (SerialBT.available()) {
    char inChar = (char)SerialBT.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
  if (stringComplete) {
    handleInput(inputString);
    inputString = "";
    stringComplete = false;
  }
}
// === Command Handler ===
void handleInput(String cmd) {
  cmd.trim();
  if (cmd.length() == 1) {
    char ch = cmd.charAt(0);
    switch (ch) {
      case '1': mob_forward(); break;
      case '2': mob_backward(); break;
      case '3': mob_left(); break;
      case '4': mob_right(); break;
      case '5': mob_stop(); break;
      default: Serial.println("Unknown motor command");
    }
  } else if (cmd.startsWith("s")) {
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex > 1 && spaceIndex < cmd.length() - 1) {
      int servoNum = cmd.substring(1, spaceIndex).toInt();
      int angle = cmd.substring(spaceIndex + 1).toInt();
      if (servoNum >= 1 && servoNum <= 5 && angle >= 0 && angle <= 180) {
        servos[servoNum - 1].write(angle);
        Serial.printf("Servo %d moved to %d degrees.\n", servoNum, angle);
      } else {
        Serial.println("Invalid servo number or angle (1–5, 0–180)");
      }
    } else {
      Serial.println("Invalid servo command. Format: sN angle (e.g., s2 90)");
    }
  } else {
    Serial.println("Invalid command");
  }
}
// === DC MOTOR CONTROL FUNCTIONS ===
void mob_forward() {
  ledcWrite(motor1_en, dutyCycle1);
  ledcWrite(motor2_en, dutyCycle2);
  digitalWrite(motor1Pin11, LOW); digitalWrite(motor1Pin12, HIGH);
  digitalWrite(motor2Pin21, HIGH); digitalWrite(motor2Pin22, LOW);
  Serial.println("Moving Forward");
}
void mob_backward() {
  ledcWrite(motor1_en, dutyCycle1);
  ledcWrite(motor2_en, dutyCycle2);
  digitalWrite(motor1Pin11, HIGH); digitalWrite(motor1Pin12, LOW);
  digitalWrite(motor2Pin21, LOW); digitalWrite(motor2Pin22, HIGH);
  Serial.println("Moving Backward");
}
void mob_left() {
  ledcWrite(motor1_en, dutyCycle1);
  ledcWrite(motor2_en, dutyCycle2);
  digitalWrite(motor1Pin11, LOW); digitalWrite(motor1Pin12, HIGH);
  digitalWrite(motor2Pin21, LOW); digitalWrite(motor2Pin22, HIGH);
  Serial.println("Turning Left");
}
void mob_right() {
  ledcWrite(motor1_en, dutyCycle1);
  ledcWrite(motor2_en, dutyCycle2);
  digitalWrite(motor1Pin11, HIGH); digitalWrite(motor1Pin12, LOW);
  digitalWrite(motor2Pin21, HIGH); digitalWrite(motor2Pin22, LOW);
  Serial.println("Turning Right");
}
void mob_stop() {
  digitalWrite(motor1Pin11, LOW); digitalWrite(motor1Pin12, LOW);
  digitalWrite(motor2Pin21, LOW); digitalWrite(motor2Pin22, LOW);
  Serial.println("Stopping");
}


