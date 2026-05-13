# Bridge ROS2 to an arduino robot using the phone USB

## Hardware

## Arduino code

The microcontroller side of the bridge is provided as an Arduino library named **PhoneBridgeClient**, located in [src/phone_sensors_bridge_examples/PhoneBridgeClient/](../src/phone_sensors_bridge_examples/PhoneBridgeClient/). It wraps the [SerialTransfer](https://github.com/PowerBroker2/SerialTransfer) protocol used to exchange framed messages with the phone over USB CDC-ACM.

### Install the library

Symlink the library into the Arduino IDE's libraries folder so edits in this repository are picked up live:

```bash
mkdir -p ~/Arduino/libraries
ln -s "$(pwd)/src/phone_sensors_bridge_examples/PhoneBridgeClient" ~/Arduino/libraries/PhoneBridgeClient
```

Install the dependency declared in `library.properties`:

- Arduino IDE > **Tools > Manage Libraries...** > search **SerialTransfer** by PowerBroker2 > Install.

Restart the Arduino IDE (it only scans libraries on startup). The library and its examples then appear under **File > Examples > PhoneBridgeClient**.

### Examples

- **CmdVel** receives `geometry_msgs/Twist` packets from ROS2 (linear/angular floats, packed struct) and reserves a packet ID for sending encoder ticks back. Starting point for new robots. Compiles on any CDC-ACM board (Teensy 4.x / 3.x, Arduino Leonardo / Micro / Zero / MKR / Nano 33 IoT, Adafruit Feather 32u4 / M0, ESP32-S2/S3).
- **PIDTune** drives one motor with alternating speed setpoints and prints `setpoint`, `speedA`, `speedB` over USB serial. Used to tune the `KP` and `KI` gains of the bundled `DifferentialRobot` PI controller.

### Build and upload

1. **Tools > Board** and pick your target.
2. **Tools > Port** and pick the serial port.
3. Click **Verify** to compile, then **Upload** to flash.

### Tuning with the Serial Plotter

While running **PIDTune**, open **Tools > Serial Plotter** at 115200 baud. The sketch emits lines of the form

```txt
setpoint:1800.00 speedA:1755.30 speedB:0.00
```

which the Serial Plotter parses as named traces. Step responses become directly observable: adjust `KP` and `KI` at the top of the sketch, reflash, and watch the new traces converge on the alternating setpoint.

> **PIDTune is restricted to Teensy and ESP32 boards.** It calls `analogWriteFrequency()` to set the motor PWM above the audible range, which is not available on AVR (Uno/Nano/Leonardo/Micro) or SAMD cores. CmdVel has no such restriction.

## Running the server

## Nav2 integration
