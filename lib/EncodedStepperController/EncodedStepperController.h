#pragma once
#ifndef ENCODEDSTEPPERCONTROLLER_H
#define ENCODEDSTEPPERCONTROLLER_H

#include "StepperController.h"
#include <AS5600.h>    // https://github.com/RobTillaart/AS5600
#include <Button.h>    // https://github.com/madleech/Button
#include <Wire.h>

// The same button pin as defined in StepperController.h
#ifndef ZERO_BUTTON_PIN
#define ZERO_BUTTON_PIN 0
#endif

class EncodedStepperController : public StepperController {
public:
    EncodedStepperController();

    // Override setup and loop to include encoder integration.
    virtual void begin() ;
    virtual void loopWithFeedback() ;

    // Override handleSpindleMode to incorporate PID control.
    virtual void handleSpindleModeWithFeedback(uint16_t pwmValue, bool cwDirection) ;

    // Encoder calibration and zero functions.
    void setEncoderZero();
    void calibrateEncoder();

    // Optional: accessor for actual RPM.
    double getActualRPM() const;

protected:
    // Encoder instance (I2C-based).
    AS5600 encoder;

    // Button for zero calibration (with debounce).
    Button zeroButton;

    // Encoder zero offset (in degrees).
    float encoderZeroOffset;

    // Variables for computing encoder-based RPM.
    double lastEncoderAngle;
    unsigned long lastEncoderTime;   // in microseconds
    double actualRPM;                // measured RPM

    // PID variables for spindle speed correction.
    double pidSetpoint;    // Desired RPM (from PWM mapping)
    double pidInput;       // Measured RPM (from encoder)
    double pidOutput;      // PID correction output
    double pidKp;          // Proportional gain
    double pidKi;          // Integral gain
    double pidKd;          // Derivative gain
    double pidIntegral;
    double pidLastError;
    unsigned long pidLastTime;       // in milliseconds
};

#endif // ENCODEDSTEPPERCONTROLLER_H
