#include "SerialTransfer.h"

SerialTransfer myTransfer;

// Must match the order and types sent by cmd_vel_to_arduino.py
struct __attribute__((packed)) CmdVel
{
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
} cmdVel;

// Packet ID 1 (optional): encoder ticks sent back to ROS.
// Uncomment and populate with real encoder readings.
// struct __attribute__((packed)) Encoder {
//   int32_t left_ticks;
//   int32_t right_ticks;
// } encoder;

// Send a plain string to the ROS logger (packet ID 0).
void sendMessage(const char *text)
{
  char buf[64];
  strncpy(buf, text, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  uint16_t size = myTransfer.txObj(buf);
  myTransfer.sendData(size, 0);
}

void setup()
{
  // Serial is the USB CDC connection to the phone running the ROS2 bridge.
  // On native USB boards (Leonardo, Micro, Teensy), wait for the host to open the port.
  Serial.begin(115200);
  while (!Serial)
    ;

  myTransfer.begin(Serial);
  sendMessage("cmd_vel sketch ready");
}

void loop()
{
  if (myTransfer.available())
  {
    myTransfer.rxObj(cmdVel);

    // TODO: use cmdVel values to drive motors, servos, etc.
    // Example:
    //   setMotorSpeed(cmdVel.linear_x, cmdVel.angular_z);

    // Optional: send encoder ticks back (packet ID 1).
    // encoder.left_ticks  = readLeftEncoder();
    // encoder.right_ticks = readRightEncoder();
    // uint16_t encSize = myTransfer.txObj(encoder);
    // myTransfer.sendData(encSize, 1);
  }
}
