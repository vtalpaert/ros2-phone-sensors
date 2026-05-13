#include "DifferentialRobot.h"

int PIController::compute(float target, float measured, float dt_s) {
  float error = target - measured;
  float output = kp * error + ki * integrator;
  if (output > -pwmMax && output < pwmMax) {
    integrator += error * dt_s;
  }
  return constrain((int)output, -(int)pwmMax, (int)pwmMax);
}

void PIController::reset() {
  integrator = 0.0f;
}

DifferentialRobot::DifferentialRobot(float kp, float ki) {
  piA = { kp, ki, (float)PWM_MAX };
  piB = { kp, ki, (float)PWM_MAX };
}

void DifferentialRobot::begin() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWriteFrequency(ENA, 20000);
  analogWriteFrequency(ENB, 20000);

  driveA(0);
  driveB(0);
}

void DifferentialRobot::setSpeed(float aTicksPerSec, float bTicksPerSec) {
  targetA = aTicksPerSec;
  targetB = bTicksPerSec;
}

float DifferentialRobot::getBatteryVoltage() const {
  return analogRead(BAT) * (ADC_VREF / ADC_MAX) * BAT_DIVIDER_INV;
}

void DifferentialRobot::update(long ticksA, long ticksB, float dt_s) {
  speedA = ticksA / dt_s;
  speedB = ticksB / dt_s;

  driveA(piA.compute(targetA, speedA, dt_s));
  driveB(piB.compute(targetB, speedB, dt_s));
}

int DifferentialRobot::applyDeadband(int pwm) {
  if (abs(pwm) < PWM_DEADBAND) return 0;
  return pwm;
}

void DifferentialRobot::driveA(int pwm) {
  pwm = applyDeadband(pwm);
  bool newForward = pwm >= 0;
  if (pwm != 0 && newForward != forwardA) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    delay(BRAKE_DELAY_MS);
  }
  if (pwm > 0) {
    forwardA = true;
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
  } else if (pwm < 0) {
    forwardA = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwm);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void DifferentialRobot::driveB(int pwm) {
  pwm = applyDeadband(pwm);
  bool newForward = pwm >= 0;
  if (pwm != 0 && newForward != forwardB) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
    delay(BRAKE_DELAY_MS);
  }
  if (pwm > 0) {
    forwardB = true;
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm);
  } else if (pwm < 0) {
    forwardB = false;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -pwm);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}
