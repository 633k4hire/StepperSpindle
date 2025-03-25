#pragma once

#include <FastAccelStepper.h>
#include <Preferences.h>
#include <AS5600.h>       // https://github.com/RobTillaart/AS5600
#include <Button.h>       // https://github.com/madleech/Button
#include <Wire.h>

// Input pins from the MKS board
#define dirPinIn             7    // Direction input from MKS
#define enablePinIn          8    // Enable input from MKS
#define stepPinIn            9    // Step input from MKS

// PWM input pins for spindle control (5kHz PWM, 3.3V signals)
#define spindle_pwm_cw_pin   10   // Clockwise PWM input (assign valid GPIO)
#define spindle_pwm_ccw_pin  11   // Counter-clockwise PWM input (assign valid GPIO)
// Spindle mode enable pin: if HIGH, spindle mode is active
#define spindle_enable_pin   2

// Output pins (5V outputs to the stepper driver)
#define dirPinOut            6    // Direction output
#define enablePinOut         5    // Enable output
#define stepPinOut           4    // Step output

// Button for zero calibration (attached to IO0)
#define ZERO_BUTTON_PIN      0

// Logging macros
#define LOGI(x) Serial.println(x)
#define LOGE(x) Serial.println(x)

class StepperController {
public:
    StepperController();
    void setup();
    void loop();

    // Mode switching functions
    void switchToMotionMode();
    void handleMotionMode();
    void switchToSpindleMode();
    void handleSpindleMode(uint16_t pwmValue, bool cwDirection);
    void setStepperSpeed(long stepsPerSecond);

    // FastAccelStepper configuration setters
    void setAccelerationSetting(long accel);
    void setLinearAccelerationSetting(uint32_t linearAcc);
    void setJumpStartSetting(uint32_t jumpStep);
    void setAutoEnableSetting(bool autoEn);

    // Functions for motion override testing (if needed)
    void enableMotionOverride();
    void disableMotionOverride();
    void testMotionCommand();

    // Functions for encoder calibration (zeroing and calibration to set stepsPerRevolution)
    void calibrateEncoder(); // Calibration stub
    void setEncoderZero();

    void loadSettings();
    void saveSettings();

    String getMode();
    String getSpindleInfo();
    unsigned long getCurrentRPM(); 
    int getCurrentPosition();

    // --- Member variables ---
    Preferences preferences;
    FastAccelStepperEngine engine;
    FastAccelStepper* stepper;

    // Mode control variables
    unsigned int defaultMode;
    unsigned int currentMode;
    unsigned int stepsPerRevolution; // Updated via encoder calibration

    // FastAccelStepper configuration settings (defaults)
    long acceleration;
    uint32_t linearAcceleration;
    uint32_t jumpStart;
    bool autoEnable;

    // Flag for motion override testing (bypasses passthrough interrupts)
    bool motionOverride;

    // Constants for mode identification and limits
    static const unsigned int MOTION_MODE  = 0;
    static const unsigned int SPINDLE_MODE = 1;
    static const unsigned int MAX_RPM      = 2000;

    // Encoder & PID variables
    AS5600 encoder;             // AS5600 encoder instance (I²C)
    Button zeroButton;          // Button instance for zeroing (with debounce)
    float encoderZeroOffset;    // Zero reference offset (degrees)
    double lastEncoderAngle;
    unsigned long lastEncoderTime;   // in microseconds
    double actualRPM;                // Measured RPM from encoder

    // PID variables for spindle speed correction
    double pidSetpoint;         // Desired RPM (from PWM input)
    double pidInput;            // Measured RPM (from encoder)
    double pidOutput;           // PID controller output (RPM adjustment)
    double pidKp, pidKi, pidKd; // PID tuning parameters
    double pidIntegral;
    double pidLastError;
    unsigned long pidLastTime;  // in milliseconds
};

