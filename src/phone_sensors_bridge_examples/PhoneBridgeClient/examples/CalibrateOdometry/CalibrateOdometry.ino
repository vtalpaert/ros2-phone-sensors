// Interactive sketch to obtain the two geometry constants needed to drive a
// differential robot in real-world units (m, m/s, rad/s):
//
//   WHEEL_BASE      meters, distance between the two driven wheels, measured
//                   center-to-center across their ground contact patches
//   TICKS_PER_METER ticks/m, encoder ticks the wheel produces per meter of
//                   forward travel
//
// Procedure: motors stay OFF. The user measures WHEEL_BASE with a tape and
// types it in. The sketch then runs 3 push trials: push the robot in a
// straight line over a measured distance, type the distance, repeat. The
// final ticks/m is the average over the 3 trials and the two wheels.
//
// Once obtained, the constants convert a (linear, angular) command into
// per-wheel setpoints for DifferentialRobot::setSpeed():
//
//   left_speed         = v - omega * (WHEEL_BASE / 2)   // m/s
//   right_speed        = v + omega * (WHEEL_BASE / 2)   // m/s
//   left_ticks_per_sec  = left_speed  * TICKS_PER_METER
//   right_ticks_per_sec = right_speed * TICKS_PER_METER

#include <PhoneBridgeClient.h>

const int WEA = 14;
const int WEB = 15;
const int NUM_TRIALS = 3;

volatile long ticksA = 0;
volatile long ticksB = 0;

void onRisingEdgeA() { ticksA++; }
void onRisingEdgeB() { ticksB++; }

void flushSerial() {
  while (Serial.available()) Serial.read();
}

float promptFloat(const char *label) {
  Serial.print(label);
  while (!Serial.available()) {}
  String s = Serial.readStringUntil('\n');
  s.trim();
  float v = s.toFloat();
  Serial.println(v, 4);
  return v;
}

void waitForChar(char target) {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == target) {
        flushSerial();
        return;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(WEA, INPUT);
  pinMode(WEB, INPUT);
  attachInterrupt(digitalPinToInterrupt(WEA), onRisingEdgeA, RISING);
  attachInterrupt(digitalPinToInterrupt(WEB), onRisingEdgeB, RISING);

  Serial.println();
  Serial.println("=== Odometry calibration ===");
  Serial.println();
  Serial.println("Step 1: measure the distance between the two driven wheels");
  Serial.println("(center to center across the ground contact patches) with a");
  Serial.println("tape measure.");
  float wheelBase = promptFloat("WHEEL_BASE in meters: ");

  Serial.println();
  Serial.println("Step 2: with motors OFF, push the robot in a straight line");
  Serial.print("over a measured distance. Repeating ");
  Serial.print(NUM_TRIALS);
  Serial.println(" times.");
  Serial.println("Push at a steady moderate pace so the encoder does not miss");
  Serial.println("edges.");

  float sumTicksPerMeterA = 0.0f;
  float sumTicksPerMeterB = 0.0f;
  int completed = 0;

  while (completed < NUM_TRIALS) {
    Serial.println();
    Serial.print("--- Trial ");
    Serial.print(completed + 1);
    Serial.print(" of ");
    Serial.print(NUM_TRIALS);
    Serial.println(" ---");
    Serial.println("Position the robot at the start mark.");
    Serial.println("Type 's' then ENTER to start counting.");
    waitForChar('s');

    noInterrupts();
    ticksA = 0;
    ticksB = 0;
    interrupts();

    Serial.println("Counting. Push to the end mark, then type 'x' ENTER.");
    unsigned long lastPrint = 0;
    while (true) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'x') {
          flushSerial();
          break;
        }
      }
      if (millis() - lastPrint >= 250) {
        Serial.print("  ticksA=");
        Serial.print(ticksA);
        Serial.print(" ticksB=");
        Serial.println(ticksB);
        lastPrint = millis();
      }
    }

    long capturedA = ticksA;
    long capturedB = ticksB;
    Serial.print("Final ticksA=");
    Serial.print(capturedA);
    Serial.print(" ticksB=");
    Serial.println(capturedB);

    float distance = promptFloat("Distance pushed in meters: ");
    if (distance <= 0.0f) {
      Serial.println("Invalid distance, trial discarded.");
      continue;
    }

    float tpmA = capturedA / distance;
    float tpmB = capturedB / distance;
    Serial.print("  trial ticks/m: A=");
    Serial.print(tpmA, 2);
    Serial.print(" B=");
    Serial.println(tpmB, 2);
    sumTicksPerMeterA += tpmA;
    sumTicksPerMeterB += tpmB;
    completed++;
  }

  float avgA = sumTicksPerMeterA / NUM_TRIALS;
  float avgB = sumTicksPerMeterB / NUM_TRIALS;
  float ticksPerMeter = (avgA + avgB) / 2.0f;

  Serial.println();
  Serial.println("=== Results ===");
  Serial.print("Average ticks/m wheel A = ");
  Serial.println(avgA, 2);
  Serial.print("Average ticks/m wheel B = ");
  Serial.println(avgB, 2);
  Serial.print("Mismatch A vs B         = ");
  Serial.print(100.0f * (avgA - avgB) / ticksPerMeter, 2);
  Serial.println(" %");
  Serial.println();
  Serial.println("Copy these into your sketch:");
  Serial.println();
  Serial.print("const float WHEEL_BASE      = ");
  Serial.print(wheelBase, 4);
  Serial.println("f;  // meters");
  Serial.print("const float TICKS_PER_METER = ");
  Serial.print(ticksPerMeter, 2);
  Serial.println("f;  // ticks per meter, average of A and B");
  Serial.println();
  Serial.println("If the A vs B mismatch is larger than a few percent, the");
  Serial.println("encoder discs may differ; consider storing per-wheel values.");
  Serial.println();
  Serial.println("Done. Press the reset button to recalibrate.");
}

void loop() {}
