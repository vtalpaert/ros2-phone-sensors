#include "differential_robot.h"

const int WEA = 14;
const int WEB = 15;
const float KP = 0.030f;
const float KI = 0.3f;
const float SPEED_TARGET = 1800.0f;

DifferentialRobot robot(KP, KI);

volatile long ticksA = 0;
volatile long ticksB = 0;
long lastTicksA = 0;
long lastTicksB = 0;
unsigned long lastTime = 0;
unsigned long phaseStart = 0;
float speedTarget = 0.0f;

void onRisingEdgeA() {
  ticksA += robot.getForwardA() ? 1 : -1;
}

void onRisingEdgeB() {
  ticksB += robot.getForwardB() ? 1 : -1;
}

void setup() {
  Serial.begin(115200);
  pinMode(WEA, INPUT);
  pinMode(WEB, INPUT);
  robot.begin();
  attachInterrupt(digitalPinToInterrupt(WEA), onRisingEdgeA, RISING);
  attachInterrupt(digitalPinToInterrupt(WEB), onRisingEdgeB, RISING);
}

void loop() {
  unsigned long now = millis();

  if (now - phaseStart >= 4000) {
    speedTarget = (speedTarget > 0.0f) ? -SPEED_TARGET : SPEED_TARGET;
    robot.setSpeed(speedTarget, 0.0f);
    phaseStart = now;
  }

  unsigned long dt = now - lastTime;
  if (dt >= 100) {
    float dt_s = dt / 1000.0f;
    long deltaA = ticksA - lastTicksA;
    long deltaB = ticksB - lastTicksB;
    lastTicksA = ticksA;
    lastTicksB = ticksB;
    lastTime = now;

    robot.update(deltaA, deltaB, dt_s);

    Serial.print("setpoint:");
    Serial.print(speedTarget);
    Serial.print(" speedA:");
    Serial.print(robot.getSpeedA());
    Serial.print(" speedB:");
    Serial.println(robot.getSpeedB());
  }
}
