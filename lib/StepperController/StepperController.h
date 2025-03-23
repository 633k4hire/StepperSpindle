#pragma once

#include <FastAccelStepper.h>
#include <Preferences.h>

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

    // --- Web API command functions ---
    // For spindle mode:
    void webSetSpindleSpeed(unsigned long rpm, bool cwDirection);
    // For motion mode:
    void webMotionMoveTo(int32_t position, bool blocking = true);
    void webMotionMove(int32_t steps, bool blocking = true);
    void webMotionStop();


    void loadSettings();
    void saveSettings();

    String getMode();
    String getSpindleInfo();
    unsigned long getCurrentRPM(); 
    int getCurrentPosition();


    Preferences preferences;
    FastAccelStepperEngine engine = FastAccelStepperEngine();
    FastAccelStepper* stepper = nullptr;

    // Mode control variables
    unsigned int defaultMode;
    unsigned int currentMode;
    unsigned int stepsPerRevolution = 1600;

    // FastAccelStepper configuration settings (defaults)
    long acceleration = 1000;                // steps per s²
    uint32_t linearAcceleration = 0;         // linear acceleration steps (0 = disabled)
    uint32_t jumpStart = 0;                  // jump start steps
    bool autoEnable = true;                  // auto-enable outputs

    // Flag for motion override testing (bypasses passthrough interrupts)
    bool motionOverride = false;

    // Constants for mode identification and limits
    static const unsigned int MOTION_MODE  = 0;
    static const unsigned int SPINDLE_MODE = 1;
    static const unsigned int MAX_RPM      = 2000;
};
