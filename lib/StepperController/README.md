# StepperController

The **StepperController** project is designed to control a stepper motor with dual modes of operation: **Motion Mode** and **Spindle Mode**. It leverages the [FastAccelStepper](https://github.com/FastAccelStepper/FastAccelStepper) library for precise stepper control and the [Preferences](https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences) library to store and load configuration settings persistently. In addition, the controller can process PWM signals via interrupts for spindle speed control.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware & Pin Configuration](#hardware--pin-configuration)
- [Software Overview](#software-overview)
- [StepperController Class Documentation](#steppercontroller-class-documentation)
  - [Public Methods](#public-methods)
  - [Configuration Setters](#fastaccelstepper-configuration-setters)
  - [Motion Override Functions](#motion-override-functions)
  - [Web API Command Functions](#web-api-command-functions)
  - [Preferences Handling](#preferences-handling)
  - [Getter Functions](#getter-functions)
  - [Public Member Variables](#public-member-variables)
- [Interrupt Service Routines (ISRs)](#interrupt-service-routines-isrs)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

The **StepperController** enables control over a stepper motor with two distinct modes:

- **Motion Mode**: Directly passes through external control signals (direction, enable, step) via interrupts. Ideal for manual or external controller operation.
- **Spindle Mode**: Uses measured PWM inputs (clockwise and counter-clockwise) to control the stepper motor's speed (expressed as RPM). This mode detaches passthrough signals to avoid interference and maps a 10-bit PWM reading to an RPM value.

The project is targeted at microcontrollers compatible with the FastAccelStepper library (for example, ESP32) and can be extended with web-based control via the provided API functions.

---

## Features

- **Dual Operation Modes**: Easily switch between Motion Mode and Spindle Mode.
- **PWM Measurement via Interrupts**: Accurately captures PWM signal duty cycles to compute spindle RPM.
- **Configurable Motion Parameters**: Change acceleration, linear acceleration, jump start, and auto-enable settings.
- **Persistent Settings**: Uses the Preferences library to save and load configuration across resets.
- **Web API Commands**: Provides methods to remotely command both spindle speed and motion operations.
- **Motion Override**: Allows for direct motion testing by overriding passthrough interrupts.

---

## Hardware & Pin Configuration

The controller is designed with specific pins for both input and output connections:

### Input Pins (from the MKS board)
- **`dirPinIn` (Pin 7):** Direction input signal.
- **`enablePinIn` (Pin 8):** Enable input signal.
- **`stepPinIn` (Pin 9):** Step input signal.

### Output Pins (to the stepper driver)
- **`dirPinOut` (Pin 6):** Direction output signal.
- **`enablePinOut` (Pin 5):** Enable output signal.
- **`stepPinOut` (Pin 4):** Step output signal.

### Spindle Control PWM Inputs (5kHz PWM, 3.3V signals)
- **`spindle_pwm_cw_pin` (Pin 10):** PWM input for clockwise (CW) rotation.
- **`spindle_pwm_ccw_pin` (Pin 11):** PWM input for counter-clockwise (CCW) rotation.

### Spindle Mode Enable
- **`spindle_enable_pin` (Pin 2):** When set HIGH, enables spindle mode.

---

## Software Overview

The project utilizes:
- **FastAccelStepper Library:** For efficient and smooth stepper control.
- **Preferences Library:** To persist motor parameters and settings between sessions.

The controller implements several interrupt service routines (ISRs) for measuring PWM signals and for passing through control signals when in Motion Mode. It also offers a set of methods to change settings, perform test moves, and respond to web API commands.

---

## StepperController Class Documentation

### Public Methods

- **`StepperController()`**
  - **Description:** Constructor; initializes member variables and sets initial conditions.
  
- **`void setup()`**
  - **Description:** Initializes pin modes, loads persistent settings, sets up the FastAccelStepper engine, configures the stepper driver, and attaches all necessary interrupts.
  
- **`void loop()`**
  - **Description:** Main loop function that checks the state of the spindle enable pin. It switches between Spindle Mode (processing PWM inputs to control speed) and Motion Mode (using passthrough interrupts). Also, if motion override is active, it bypasses normal operation for testing.
  
- **`void switchToMotionMode()`**
  - **Description:** Switches the controller to Motion Mode by stopping spindle-related commands, disabling outputs, and reattaching passthrough interrupts.
  
- **`void handleMotionMode()`**
  - **Description:** Handles operations specific to Motion Mode. (In this implementation, motion handling is performed via interrupts.)
  
- **`void switchToSpindleMode()`**
  - **Description:** Switches the controller to Spindle Mode by detaching passthrough interrupts, enabling the stepper driver, and preparing for PWM-based control.
  
- **`void handleSpindleMode(uint16_t pwmValue, bool cwDirection)`**
  - **Description:** Converts a 10-bit PWM value into an RPM value using a mapping function. It then calculates the corresponding stepper speed in steps per second and commands the motor to run in the specified direction.
  
- **`void setStepperSpeed(long stepsPerSecond)`**
  - **Description:** Sets the stepper motor speed using the FastAccelStepper library by converting RPM to steps per second.

### FastAccelStepper Configuration Setters

- **`void setAccelerationSetting(long accel)`**
  - **Description:** Sets the acceleration (steps/s²) for the stepper and saves the value to Preferences.
  
- **`void setLinearAccelerationSetting(uint32_t linearAcc)`**
  - **Description:** Sets the linear acceleration steps for smoother motion start-up and stores the setting.
  
- **`void setJumpStartSetting(uint32_t jumpStep)`**
  - **Description:** Sets the number of jump start steps to overcome initial inertia, then saves the configuration.
  
- **`void setAutoEnableSetting(bool autoEn)`**
  - **Description:** Configures whether the stepper outputs should auto-enable on move commands; this setting is saved persistently.

### Motion Override Functions

- **`void enableMotionOverride()`**
  - **Description:** Activates motion override mode, detaching passthrough interrupts so that direct motion commands can be tested.
  
- **`void disableMotionOverride()`**
  - **Description:** Deactivates motion override mode and reattaches passthrough interrupts.
  
- **`void testMotionCommand()`**
  - **Description:** For testing purposes; if the stepper is idle, this function issues a command to start a continuous forward move.

### Web API Command Functions

- **`void webSetSpindleSpeed(unsigned long rpm, bool cwDirection)`**
  - **Description:** Ensures the controller is in Spindle Mode, calculates the corresponding stepper speed for the target RPM, and commands the motor to run in the specified direction.
  
- **`void webMotionMoveTo(int32_t position, bool blocking = true)`**
  - **Description:** In Motion Mode, commands the stepper motor to move to an absolute position. The optional `blocking` parameter allows the function to wait until the move completes.
  
- **`void webMotionMove(int32_t steps, bool blocking = true)`**
  - **Description:** In Motion Mode, commands a relative move by a given number of steps. Also supports blocking or non-blocking execution.
  
- **`void webMotionStop()`**
  - **Description:** Stops any ongoing motion command issued to the stepper motor.

### Preferences Handling

- **`void loadSettings()`**
  - **Description:** Loads configuration parameters (e.g., steps per revolution, acceleration settings) from non-volatile storage.
  
- **`void saveSettings()`**
  - **Description:** Saves current configuration settings to Preferences for persistence across power cycles.

### Getter Functions

- **`String getMode()`**
  - **Description:** Returns a string representing the current mode ("Spindle" or "Motion").
  
- **`String getSpindleInfo()`**
  - **Description:** Returns a formatted string containing spindle information, such as the current RPM.
  
- **`unsigned long getCurrentRPM()`**
  - **Description:** Calculates the RPM from the current stepper speed (obtained from the FastAccelStepper library) based on the defined steps per revolution.
  
- **`int getCurrentPosition()`**
  - **Description:** Retrieves the current step position of the stepper motor from the FastAccelStepper library.

### Public Member Variables

- **`Preferences preferences`**
  - **Description:** An instance of the Preferences class used for saving and loading configuration settings.
  
- **`FastAccelStepperEngine engine`**
  - **Description:** Engine instance that manages stepper motor commands.
  
- **`FastAccelStepper* stepper`**
  - **Description:** Pointer to the stepper motor instance connected to the engine.
  
- **`unsigned int defaultMode`**
  - **Description:** (Reserved for future use) Default operational mode.
  
- **`unsigned int currentMode`**
  - **Description:** Holds the current operation mode (either `MOTION_MODE` or `SPINDLE_MODE`).
  
- **`unsigned int stepsPerRevolution`**
  - **Description:** Number of steps per revolution of the stepper motor (default: 1600).
  
- **`long acceleration`**
  - **Description:** Acceleration rate in steps per second².
  
- **`uint32_t linearAcceleration`**
  - **Description:** Optional linear acceleration steps for smoother motor startup.
  
- **`uint32_t jumpStart`**
  - **Description:** Number of jump start steps to quickly overcome inertia.
  
- **`bool autoEnable`**
  - **Description:** Flag to automatically enable stepper outputs when a move command is issued.
  
- **`bool motionOverride`**
  - **Description:** Flag that, when true, bypasses the normal passthrough interrupts for testing direct motion commands.
  
- **Static Constants:**
  - **`static const unsigned int MOTION_MODE`**
    - **Description:** Constant representing Motion Mode.
  - **`static const unsigned int SPINDLE_MODE`**
    - **Description:** Constant representing Spindle Mode.
  - **`static const unsigned int MAX_RPM`**
    - **Description:** Maximum RPM allowed (set to 2000 RPM).

---

## Interrupt Service Routines (ISRs)

### PWM Measurement ISRs

- **`pwmCW_ISR()`**
  - **Description:** Measures the PWM signal on the clockwise channel. It calculates the time between rising edges to determine the period and captures the pulse width on the falling edge.
  
- **`pwmCCW_ISR()`**
  - **Description:** Measures the PWM signal on the counter-clockwise channel in a similar manner as `pwmCW_ISR()`.

Both ISRs are designed to be minimal, using the `micros()` function to record precise timestamps and updating global variables for further processing in the main loop.

### Motion Mode Passthrough ISRs

- **`handleEnableChangeInterrupt()`**
  - **Description:** Immediately writes the state of the `enablePinIn` to `enablePinOut`.
  
- **`handleDirectionChangeInterrupt()`**
  - **Description:** Immediately writes the state of the `dirPinIn` to `dirPinOut`.
  
- **`handleStepInterrupt()`**
  - **Description:** Immediately writes the state of the `stepPinIn` to `stepPinOut`.

These routines allow the external MKS board signals to directly control the stepper driver when the system is in Motion Mode.

---

## Usage

1. **Hardware Setup:**
   - **Connect Input Signals:** Wire the MKS board outputs to `dirPinIn` (7), `enablePinIn` (8), and `stepPinIn` (9).
   - **Connect Stepper Driver:** Connect the stepper driver's control pins to `dirPinOut` (6), `enablePinOut` (5), and `stepPinOut` (4).
   - **Spindle Control:** Connect the spindle PWM inputs to `spindle_pwm_cw_pin` (10) and `spindle_pwm_ccw_pin` (11) and the enable signal to `spindle_enable_pin` (2).

2. **Software Setup:**
   - Include `StepperController.h` and its implementation file in your project.
   - Install the **FastAccelStepper** and **Preferences** libraries.
   - In your microcontroller's `setup()` function, call `StepperController::setup()`.
   - In your `loop()` function, continuously call `StepperController::loop()`.

3. **Using the Web API:**
   - Use the provided web API command functions to remotely set the spindle speed or control motion commands:
     - `webSetSpindleSpeed(rpm, cwDirection)`
     - `webMotionMoveTo(position, blocking)`
     - `webMotionMove(steps, blocking)`
     - `webMotionStop()`

4. **Testing and Debugging:**
   - Use the motion override functions (`enableMotionOverride()`, `disableMotionOverride()`, and `testMotionCommand()`) to directly test the stepper motor without interference from the passthrough interrupts.

---

## Contributing

Contributions to enhance functionality or improve documentation are welcome. Please fork the repository, create a feature branch, and submit a pull request with your improvements. For major changes, open an issue first to discuss your ideas.

---

## License

MIT License