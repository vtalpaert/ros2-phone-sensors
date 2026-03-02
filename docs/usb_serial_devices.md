# USB Serial Devices

This page explains how to connect USB serial devices (microcontrollers, sensors) to the phone through the browser, using the [WebUSB API](https://developer.mozilla.org/en-US/docs/Web/API/WebUSB_API).

## Why WebUSB and not Web Serial?

On Android, the [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API) requires Chrome 138 or later and only exposes devices that the Android OS has already claimed through its own serial driver. Most USB-to-serial bridge chips (CP2102, CH340, FTDI) are not claimed this way, so they do not appear in the Web Serial port picker.

WebUSB bypasses the OS driver layer and talks directly to the USB device, which is why it can reach these devices. The trade-off is that the communication protocol must be implemented in JavaScript for each chip family.

## Test page

The Web USB test page (`/test-web-usb`, linked from the home page) lets you:

- Discover which USB devices the browser can see
- Open a serial connection using one of the supported protocols
- Send text and receive data in real time

Use this page to verify compatibility before integrating a device into a larger application.

## Compatible hardware

### Category 1 — Boards with native USB (CDC-ACM protocol)

These boards have a USB controller directly on the main processor. They implement the standard USB CDC-ACM class, which the test page uses to open a serial connection without any chip-specific vendor protocol.

| Board | Chip | Notes |
|---|---|---|
| Teensy 4.0 / 4.1 | iMXRT1062 | Tested and confirmed working |
| Teensy 3.x | MK20 / MK64 | Native USB |
| Arduino Leonardo | ATmega32U4 | Native USB |
| Arduino Micro | ATmega32U4 | Native USB |
| Arduino Zero / MKR series | SAMD21 | Native USB |
| Arduino Nano 33 IoT | SAMD21 | Native USB |
| Adafruit Feather 32u4 / M0 | ATmega32U4 / SAMD21 | Native USB |

**How it works:** The browser claims the CDC data interface (interface 1), sends a `SET_LINE_CODING` control request to configure the baud rate, raises DTR with `SET_CONTROL_LINE_STATE`, then reads and writes on the bulk endpoints. No vendor-specific protocol is needed.

### Category 2 — USB-to-UART bridge: CP2102

The Silicon Labs CP2102 is a dedicated USB-to-UART converter chip. It is found as a standalone USB dongle and on many ESP32 development boards. The test page implements the CP2102 vendor protocol (Silicon Labs AN571) to configure the chip and access the bulk data endpoints.

| Device | Chip | vendorId | productId |
|---|---|---|---|
| CP2102 USB-to-UART dongle | CP2102 | 0x10c4 | 0xea60 |
| Most ESP32 dev boards | CP2102 or CP2104 | 0x10c4 | 0xea60 / 0xea71 |

**How it works:** The browser claims interface 0, sends `IFC_ENABLE` and `SET_BAUDRATE` vendor control requests, then reads and writes on the bulk endpoints. The data is buffered on the JavaScript side and displayed line by line.

## Incompatible hardware

| Board / Device | Reason |
|---|---|
| Arduino Uno | Uses ATmega16U2 as a USB bridge; the main sketch has no USB control |
| Arduino Nano (classic) | Uses CH340G or FTDI as a USB bridge; same limitation as Uno |
| ESP32 (classic, with CP2102) | CP2102 protocol is supported (see above), but the ESP32 chip itself is not accessible via USB |
| CH340 / CH9102 adapters | Vendor protocol not yet implemented in the test page |
| FTDI FT232RL adapters | Vendor protocol not yet implemented in the test page |

Note: the ESP32-S2 and ESP32-S3 have native USB OTG and fall into Category 1 when programmed with the `arduino-esp32` USB CDC class.

## Example sketch (Teensy / Arduino with native USB)

The following sketch sends a periodic heartbeat on both USB Serial and UART, and echoes back anything it receives. This is useful for testing both the CDC Serial section and the CP2102 section of the test page simultaneously.

```cpp
// Wiring for CP2102 section on a Teensy 4.0:
//   TX1 (Teensy pin 1) → RX pin of CP2102 adapter
//   RX1 (Teensy pin 0) → TX pin of CP2102 adapter
//   GND → GND

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for USB CDC connection (DTR raised by browser)
  Serial1.begin(115200); // Hardware UART on pins 0/1
}

void loop() {
  // Periodic heartbeat on both ports
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 1000) {
    Serial.println("USB Serial alive");
    Serial1.println("UART alive");
    lastSend = millis();
  }

  // Echo USB Serial
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    Serial.println("Echo USB: " + msg);
  }

  // Echo UART
  if (Serial1.available()) {
    String msg = Serial1.readStringUntil('\n');
    msg.trim();
    Serial1.println("Echo UART: " + msg);
  }
}
```

**Key points:**

- `readStringUntil('\n')` reads a complete line; the test page appends `\n` to every message it sends
- `trim()` removes the trailing `\r` that is part of the `\r\n` line ending

## Testing procedure

1. Flash the sketch above to your board
2. Open the Web USB test page (`/test-web-usb`) in Chrome on Android
3. Press **Request USB device** and select your board or converter
4. For native USB boards (Teensy, Leonardo, etc.): use the **CDC Serial** section
5. For CP2102 adapters: use the **CP2102 Serial** section
6. Set the baud rate to match your sketch (115200 in the example above)
7. Press **Open**: the board should start sending heartbeat messages in the Received area
8. Type a message in the Send field and press Send: you should receive the echo back
