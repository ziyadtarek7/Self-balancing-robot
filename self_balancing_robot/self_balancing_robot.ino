// Libraries
#include "PIDF.h"
#include "Wire.h"
#include <MPU6050_light.h>

// Serial parameters
#define BAUD_RATE 9600

// L298n motor driver
// Motor 1
#define right_motor_ENA 5
#define right_motor_IN1 3
#define right_motor_IN2 4
// Motor 2
#define left_motor_ENB 6
#define left_motor_IN3 7
#define left_motor_IN4 8

// PID parameters
#define KP 10
#define KI 0
#define KD 0
#define KN 0
#define TS 0.005
#define PID_PERIOD_MS 5

MPU6050 mpu(Wire);
PID::PIDF angle_pid = PID::PIDF(KP, KI, KD, KN, TS);
float Setpoint = 0;
float error = 0;
float angle = 0;
unsigned long timer = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();
  pinMode(right_motor_ENA , OUTPUT);
  pinMode(right_motor_IN1 , OUTPUT);
  pinMode(right_motor_IN2 , OUTPUT);
  pinMode(left_motor_ENB, OUTPUT);
  pinMode(left_motor_IN3, OUTPUT);
  pinMode(left_motor_IN4, OUTPUT);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  angle_pid.setpoint(Setpoint);
  angle_pid.limitOutput(-255, 255);
}

void loop() {
  mpu.update();
  angle = mpu.getAngleY();

  if ((millis() - timer) > 5) { // print data every 10ms
    pid_routine();
    timer = millis();
  }
}

void pid_routine()
{
  float pidOutput = angle_pid.calculate(angle);      // Calculate value of pid
  if (pidOutput >= 0)
    forward();
  else
    backward();
  analogWrite(right_motor_ENA , abs(pidOutput));
  analogWrite(left_motor_ENB, abs(pidOutput));
  Serial.println(pidOutput);
}

void forward() {
  digitalWrite(right_motor_IN1 , HIGH);
  digitalWrite(right_motor_IN2 , LOW);
  digitalWrite(left_motor_IN3, HIGH);
  digitalWrite(left_motor_IN4, LOW);
}

void backward() {
  digitalWrite(right_motor_IN1 , LOW);
  digitalWrite(right_motor_IN2 , HIGH);
  digitalWrite(left_motor_IN3, LOW);
  digitalWrite(left_motor_IN4, HIGH);
}

void Stop() {
  analogWrite(right_motor_ENA, 0);
  digitalWrite(right_motor_IN1, LOW);
  digitalWrite(right_motor_IN2, LOW);
  analogWrite(left_motor_ENB, 0);
  digitalWrite(left_motor_IN3, LOW);
  digitalWrite(left_motor_IN4, LOW);
}
