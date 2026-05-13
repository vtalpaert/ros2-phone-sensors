#include <PhoneBridgeClient.h>
#include "SerialTransfer.h"

// Robot geometry (from CalibrateOdometry).
const float WHEEL_BASE       = 0.2100f;   // meters
const float TICKS_PER_METER  = 3432.17f;  // ticks/m, average of A and B
const float BATTERY_CUTOFF_V = 11.2f;     // 3S LiPo low-voltage cutoff

// PI gains (from PIDTune).
const float KP = 0.030f;
const float KI = 0.3f;

// Encoder pins.
const int WEA = 14;
const int WEB = 15;

// Periods.
const unsigned long PI_PERIOD_MS      = 50;    // 20 Hz control + odom telemetry
const unsigned long BATTERY_PERIOD_MS = 1000;  // 1 Hz battery telemetry

SerialTransfer myTransfer;
DifferentialRobot robot(KP, KI);

// Encoder counters. Signed: ISRs increment on forward, decrement on reverse,
// using the motor command direction held by DifferentialRobot.
volatile long ticksA = 0;
volatile long ticksB = 0;

long lastTicksA = 0;
long lastTicksB = 0;
unsigned long lastPiMs = 0;
unsigned long lastBatteryMs = 0;

// Pose state, integrated on-board so Python does not need geometry constants.
float odomX     = 0.0f;
float odomY     = 0.0f;
float odomTheta = 0.0f;

bool motorsCutoff = false;

// Must match the order and types sent by cmd_vel_to_arduino.py.
struct __attribute__((packed)) CmdVel {
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
} cmdVel;

// Packet ID 1: odometry telemetry, SI units, integrated on the Arduino.
struct __attribute__((packed)) OdomMsg {
  float x;      // m
  float y;      // m
  float theta;  // rad, wrapped to [-PI, PI]
  float vx;     // m/s
  float omega;  // rad/s
} odomMsg;

// Packet ID 2: battery voltage.
struct __attribute__((packed)) BatteryMsg {
  float voltage;  // V
} batteryMsg;

void onRisingEdgeA() {
  ticksA += robot.getForwardA() ? 1 : -1;
}

void onRisingEdgeB() {
  ticksB += robot.getForwardB() ? 1 : -1;
}

// Send a plain string to the ROS logger (packet ID 0).
void sendMessage(const char *text) {
  char buf[64];
  strncpy(buf, text, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  uint16_t size = myTransfer.txObj(buf);
  myTransfer.sendData(size, 0);
}

void handleCmdVel() {
  myTransfer.rxObj(cmdVel);
  if (motorsCutoff) {
    return;
  }
  float linear_x  = cmdVel.linear_x;
  float angular_z = cmdVel.angular_z;
  float left_speed  = linear_x - angular_z * (WHEEL_BASE / 2.0f);  // m/s
  float right_speed = linear_x + angular_z * (WHEEL_BASE / 2.0f);
  // Wiring on this robot: motor A drives RIGHT, motor B drives LEFT.
  // To flip, swap the two arguments below; nothing else changes.
  robot.setSpeed(right_speed * TICKS_PER_METER,  // A
                 left_speed  * TICKS_PER_METER); // B
}

void runPiAndOdom(unsigned long now) {
  unsigned long dt_ms = now - lastPiMs;
  lastPiMs = now;
  float dt_s = dt_ms / 1000.0f;

  noInterrupts();
  long snapA = ticksA;
  long snapB = ticksB;
  interrupts();
  long deltaA = snapA - lastTicksA;
  long deltaB = snapB - lastTicksB;
  lastTicksA = snapA;
  lastTicksB = snapB;

  robot.update(deltaA, deltaB, dt_s);

  // Wiring on this robot: A == RIGHT, B == LEFT. To flip, swap these two.
  float right_speed = ((float)deltaA / TICKS_PER_METER) / dt_s;
  float left_speed  = ((float)deltaB / TICKS_PER_METER) / dt_s;
  float vx    = (left_speed + right_speed) * 0.5f;
  float omega = (right_speed - left_speed) / WHEEL_BASE;

  odomTheta += omega * dt_s;
  odomTheta = atan2f(sinf(odomTheta), cosf(odomTheta));
  odomX += vx * cosf(odomTheta) * dt_s;
  odomY += vx * sinf(odomTheta) * dt_s;

  odomMsg.x     = odomX;
  odomMsg.y     = odomY;
  odomMsg.theta = odomTheta;
  odomMsg.vx    = vx;
  odomMsg.omega = omega;
  uint16_t size = myTransfer.txObj(odomMsg);
  myTransfer.sendData(size, 1);
}

void runBattery() {
  float voltage = robot.getBatteryVoltage();
  batteryMsg.voltage = voltage;
  uint16_t size = myTransfer.txObj(batteryMsg);
  myTransfer.sendData(size, 2);

  if (!motorsCutoff && voltage < BATTERY_CUTOFF_V) {
    motorsCutoff = true;
    robot.setSpeed(0.0f, 0.0f);
    sendMessage("ERROR: Battery cutoff, motors disabled, cycle power to reset");
  }
}

void setup() {
  // Serial is the USB CDC connection to the phone running the ROS2 bridge.
  // On native USB boards (Leonardo, Micro, Teensy), wait for the host to open the port.
  Serial.begin(115200);
  while (!Serial)
    ;

  myTransfer.begin(Serial);

  pinMode(WEA, INPUT);
  pinMode(WEB, INPUT);
  robot.begin();
  attachInterrupt(digitalPinToInterrupt(WEA), onRisingEdgeA, RISING);
  attachInterrupt(digitalPinToInterrupt(WEB), onRisingEdgeB, RISING);

  lastPiMs      = millis();
  lastBatteryMs = millis();

  sendMessage("cmd_vel sketch ready");
}

void loop() {
  if (myTransfer.available()) {
    handleCmdVel();
  }

  unsigned long now = millis();
  if (now - lastPiMs >= PI_PERIOD_MS) {
    runPiAndOdom(now);
  }
  if (now - lastBatteryMs >= BATTERY_PERIOD_MS) {
    lastBatteryMs = now;
    runBattery();
  }
}
