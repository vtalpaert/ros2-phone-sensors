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

- **CmdVel** receives `geometry_msgs/TwistStamped` commands from ROS2 (the Python bridge unwraps `msg.twist` and ships the six linear/angular floats as a packed struct) and reserves a packet ID for sending encoder ticks back. Starting point for new robots. Compiles on any CDC-ACM board (Teensy 4.x / 3.x, Arduino Leonardo / Micro / Zero / MKR / Nano 33 IoT, Adafruit Feather 32u4 / M0, ESP32-S2/S3).
- **CalibrateOdometry** interactive sketch that asks for `WHEEL_BASE` (tape-measured) then runs 3 push trials with motors off to derive `TICKS_PER_METER` from the encoders. Outputs the two constants needed to convert `(v, omega)` commands into per-wheel tick-rate setpoints.
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

The bundled launch file starts the phone bridge server alongside the `cmd_vel_to_arduino` node:

```bash
ros2 launch phone_sensors_bridge_examples arduino_bridge.launch.py
```

It loads [config/server_params_for_arduino.yaml](../src/phone_sensors_bridge_examples/config/server_params_for_arduino.yaml), which enables the USB CDC transport (115200 baud) and streams both phone cameras at low quality (5 fps, 320x240 front / 240x320 back, JPEG quality 0.2) to leave bandwidth headroom for the USB link.

The `cmd_vel` subscription is remapped to `key_vel`, matching the topic published by `key_teleop`:

```bash
ros2 run key_teleop key_teleop
```

The node republishes wheel odometry on `arduino/odometry` and battery voltage on `arduino/battery_voltage`.

## Nav2 integration
