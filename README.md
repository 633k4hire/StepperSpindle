
# Stepper Spindle Board

Handle a stepper motor as both a PWM-controlled spindle and a positional axis. This board provides a versatile solution for applications requiring both high-precision positioning and speed control via PWM signals.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Overview](#hardware-overview)
  - [Schematic](#schematic)
  - [Board Images](#board-images)
- [Getting Started](#getting-started)
  - [Hardware Setup](#hardware-setup)
  - [Software Setup](#software-setup)
- [Code Example](#code-example)
- [Credits](#credits)
- [License](#license)

---

## Overview

The Stepper Spindle Board is designed to handle a stepper motor in two distinct modes:

1. **PWM-Controlled Spindle Mode** – Uses PWM signals (clockwise and counter-clockwise) to control the motor speed, acting as a spindle.
2. **Positional Axis (Motion Mode)** – Provides direct positional control via a set of passthrough inputs, ideal for precise positioning tasks.

The board leverages the [FastAccelStepper Library](https://github.com/gin66/FastAccelStepper/) for high-performance stepper control and utilizes a web interface for remote control and monitoring.

---

## Features

- **Dual Operation Modes:**  
  - **Spindle Mode:** Map a 10-bit PWM input to RPM control.
  - **Motion Mode:** Directly drive the stepper motor as a positional axis.
- **Real-Time Control:**  
  - Fast, interrupt-based PWM measurements and signal passthrough.
- **Web API:**  
  - Remotely control and monitor motor operations via a built-in web server.
- **Persistent Settings:**  
  - Save and load configuration parameters using non-volatile memory.

---

## Hardware Overview

### Schematic

![stepper spindle board schematic](https://github.com/633k4hire/StepperSpindle/assets/17692800/dcea1e94-c6dd-41e6-a1a5-f3d3f297fa5d)

### Board Images

- **Board Front:**

  ![stepper spindle board](https://github.com/633k4hire/StepperSpindle/assets/17692800/e3ea57b5-149e-4338-9973-37100db18a31)

- **3D View (Front):**

  ![stepper spindle board 3d](https://github.com/633k4hire/StepperSpindle/assets/17692800/5b80597f-6b44-48f8-8625-f0d189a26ccf)

- **3D View (Back):**

  ![stepper spindle board 3d back](https://github.com/633k4hire/StepperSpindle/assets/17692800/063cd7c3-6849-44cc-bde4-6e48960bef9c)

---

## Getting Started

### Hardware Setup

1. **Stepper Motor and Driver Connections:**  
   Connect the stepper motor and the corresponding driver to the board. Refer to the schematic for specific pin assignments.
   
2. **PWM Inputs for Spindle Control:**  
   Connect the PWM signals (clockwise and counter-clockwise) and the spindle enable signal as specified in the schematic.
   
3. **MKS Board Integration:**  
   The board is designed to accept directional, enable, and step signals from an MKS board. Ensure that the input pins are connected correctly:
   - **Direction:** Pin 7
   - **Enable:** Pin 8
   - **Step:** Pin 9

### Software Setup

1. **Install Required Libraries:**  
   - [FastAccelStepper Library](https://github.com/gin66/FastAccelStepper/)
   - [Preferences Library](https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences)

2. **Upload the Firmware:**  
   Use the Arduino IDE or your preferred toolchain to upload the provided firmware to your microcontroller (e.g., ESP32).

3. **Configure WiFi:**  
   Update the WiFi credentials in the main code before uploading:
   ```cpp
   const char* ssid = "SSID";
   const char* password = "PASSWORD";
   ```

---

## Code Example

Below is the main program that demonstrates how to initialize and run the Stepper Spindle Board. This code sets up the board, starts the web interface for remote control, and prints status messages to the Serial monitor every second.

```cpp
#include <Arduino.h>
#include "StepperController.h"
#include "StepperApi.h"

// Global instances
StepperController sc;
StepperApi stepperApi(&sc);

unsigned long previousMillis = 0;
const long interval = 1000; // Interval at which to print status to Serial (1 second)

// WiFi credentials (update these with your network details)
const char* ssid = "SSID";
const char* password = "PASSWORD";

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Setup the stepper controller
    sc.setup();
    
    // Start the web interface (this serves the dashboard and REST endpoints)
    stepperApi.begin(ssid, password);
}

void loop() {
    // Run the stepper controller's loop for real-time control and PWM measurement
    sc.loop();

    // Print status to Serial every second for debugging purposes
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        String mode = sc.getMode();
        String spindleInfo = sc.getSpindleInfo();

        if (mode == "Spindle") {
            Serial.printf("Current Mode: %s, Spindle Info: %s\n", mode.c_str(), spindleInfo.c_str());
        } else {
            Serial.printf("Current Mode: %s\n", mode.c_str());
        }
    }
}
```

---

## Credits

- **FastAccelStepper Library:**  
  [FastAccelStepper](https://github.com/gin66/FastAccelStepper/)

Special thanks to all contributors and the community that supports the development of advanced stepper motor control solutions.

---

## License

MIT License,

