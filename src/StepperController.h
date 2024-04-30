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
    void handleSpindleMode(float duty_cycle);       // Handle operations in Spindle Mode based on PWM input

private:
    void initRMT();                 // Initialize the RMT module for PWM reading
    void readRMT();                 // Read PWM using RMT module   
    float calculateDutyCycle();     // Calculate duty cycle from RMT readings

    void loadSettings();            // Load settings from preferences
    void saveSettings();            // Save settings to preferences

    // GPIO pin configuration for inputs and outputs from pins.h

    // Preferences to store settings
    Preferences preferences;

    // FastAccelStepper library objects
    FastAccelStepperEngine engine;
    FastAccelStepper* stepper = nullptr;

    // RMT configuration details
    rmt_config_t rmt_rx;
    RingbufHandle_t rb = NULL;

    // Pulse Counter settings (if using PCNT for backup or alternative method)
    static const pcnt_unit_t pcntUnit = PCNT_UNIT_0; // PCNT unit for PWM input
    static const pcnt_channel_t pcntChannel = PCNT_CHANNEL_0; // PCNT channel for PWM input

    // Mode control
    unsigned int defaultMode;       // Default mode on startup
    unsigned int currentMode;       // Current operating mode
    unsigned int stepsPerRevolution; // Steps per revolution for Spindle Mode

    // Constants for mode identification
    static const unsigned int MOTION_MODE = 0;
    static const unsigned int SPINDLE_MODE = 1;

    static float dutyCycle; // Duty cycle calculated from PWM input
};
