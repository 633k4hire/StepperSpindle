#pragma once

#include "pins.h"
#include <FastAccelStepper.h>
#include <driver/rmt.h>
#include <Preferences.h>

class StepperController {
public:
    StepperController();            // Constructor
    void setup();                   // Setup method for initialization
    void loop();                    // Main loop handling input detection and mode switching

    void switchToMotionMode();      // Switch to Motion Mode
    void handleMotionMode();        // Handle operations in Motion Mode

    void switchToSpindleMode();     // Switch to Spindle Mode
    void handleSpindleMode(int pwmValue);       // Handle operations in Spindle Mode based on PWM input

    void setStepperSpeed(long stepsPerSecond);

private:
    void initRMT();                 // Initialize RMT for PWM reading
    int16_t readPWM();              // Read PWM value using RMT
    void loadSettings();            // Load settings from preferences
    void saveSettings();            // Save settings to preferences

    // Preferences to store settings
    Preferences preferences;

    // FastAccelStepper library objects
    FastAccelStepperEngine engine;
    FastAccelStepper* stepper = nullptr;

    // Mode control
    unsigned int defaultMode;       // Default mode on startup
    unsigned int currentMode;       // Current operating mode
    unsigned int stepsPerRevolution=1600; // Steps per revolution for Spindle Mode

    // RMT
    rmt_channel_t rmtChannel = RMT_CHANNEL_0; // RMT channel
    rmt_config_t rmtConfig;         // RMT configuration structure

    // Constants for mode identification
    static const unsigned int MOTION_MODE = 0;
    static const unsigned int SPINDLE_MODE = 1;
    static const unsigned int MAX_RPM = 2000;
};