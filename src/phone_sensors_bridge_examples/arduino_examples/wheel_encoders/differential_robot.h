#pragma once

#include <Arduino.h>

// Robot-specific parameters — update these for your hardware
// Motors:   L298N dual H-bridge
// Encoder:  IR slot sensor, single channel (direction inferred from motor command)
// Teensy:   4.0
// PWM freq: 20 kHz (above audible range)
// Deadband: 105/255 — minimum PWM at which motors overcome friction at 12V supply
// Ticks:    encoder ticks per revolution not yet calibrated
// Sides:    drives are labelled A and B (matching L298N channels); which is left/right is unknown

struct PIController {
  float kp;
  float ki;
  float pwmMax;
  float integrator = 0.0f;

  int compute(float target, float measured, float dt_s);
  void reset();
};

class DifferentialRobot {
public:
  // Pin assignments for L298N connected to Teensy 4.0
  static const int ENA = 2;
  static const int IN1 = 3;
  static const int IN2 = 4;
  static const int IN3 = 5;
  static const int IN4 = 6;
  static const int ENB = 7;

  // PWM limits (8-bit resolution, 0-255)
  static const int PWM_MAX = 250;
  // Minimum effective PWM — below this the motors stall due to friction + L298N drop
  static const int PWM_DEADBAND = 105;
  // Brake delay in ms when switching direction
  static const int BRAKE_DELAY_MS = 20;

  DifferentialRobot(float kp, float ki);

  void begin();
  void setSpeed(float aTicksPerSec, float bTicksPerSec);
  void update(long ticksA, long ticksB, float dt_s);
  float getSpeedA() const { return speedA; }
  float getSpeedB() const { return speedB; }
  bool getForwardA() const { return forwardA; }
  bool getForwardB() const { return forwardB; }

private:
  PIController piA;
  PIController piB;
  bool forwardA = true;
  bool forwardB = true;
  float speedA = 0.0f;
  float speedB = 0.0f;
  float targetA = 0.0f;
  float targetB = 0.0f;

  int applyDeadband(int pwm);
  void driveA(int pwm);
  void driveB(int pwm);
};
