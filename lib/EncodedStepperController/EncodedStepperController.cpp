#include "EncodedStepperController.h"
#define LOGI(x) Serial.println(x)
#define LOGE(x) Serial.println(x)

// Constructor: initialize encoder, button and PID variables.
EncodedStepperController::EncodedStepperController() :
    StepperController(),
    encoder(), 
    zeroButton(ZERO_BUTTON_PIN),
    encoderZeroOffset(0.0),
    lastEncoderAngle(0.0),
    lastEncoderTime(0),
    actualRPM(0.0),
    pidSetpoint(0.0),
    pidInput(0.0),
    pidOutput(0.0),
    pidKp(0.1),
    pidKi(0.01),
    pidKd(0.0),
    pidIntegral(0.0),
    pidLastError(0.0),
    pidLastTime(0)
{
    // Constructor body (if needed)
}

void EncodedStepperController::begin() {
    // Call the base class setup to initialize pins, interrupts, and the stepper.
    StepperController::setup();
    // Initialize the zero-calibration button.
    zeroButton.begin();
    // (Optional) Log encoder initialization.
    LOGI("EncodedStepperController setup completed.");
}

void EncodedStepperController::loopWithFeedback() {
        // Process the zero calibration button.
   
    if (zeroButton.pressed()) {
        setEncoderZero();
    }

    // Poll the encoder for RPM measurement.
    double currentAngle = encoder.readAngle();  // angle in degrees [0, 360)
    double relativeAngle = currentAngle - encoderZeroOffset;
    if (relativeAngle < 0) {
        relativeAngle += 360;
    }
    unsigned long nowMicros = micros();
    if (lastEncoderTime != 0) {
        double deltaAngle = relativeAngle - lastEncoderAngle;
        // Handle wrap-around.
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        double deltaTime = (nowMicros - lastEncoderTime) / 1000000.0; // seconds
        double measuredRPM = (deltaAngle / 360.0) * (60.0 / deltaTime);
        actualRPM = measuredRPM;
    }
    lastEncoderAngle = relativeAngle;
    lastEncoderTime = nowMicros;

    // Call the base class loop to handle mode switching and passthrough signals.
    StepperController::loop();
}

void EncodedStepperController::handleSpindleModeWithFeedback(uint16_t pwmValue, bool cwDirection) {
    // Map the 10-bit PWM value to a target RPM.
    pidSetpoint = map(pwmValue, 0, 1023, 0, MAX_RPM);
    pidInput = actualRPM;

    unsigned long nowMillis = millis();
    double dt = (pidLastTime == 0) ? 0.01 : (nowMillis - pidLastTime) / 1000.0;
    double error = pidSetpoint - pidInput;
    pidIntegral += error * dt;
    double derivative = (error - pidLastError) / dt;
    pidOutput = pidKp * error + pidKi * pidIntegral + pidKd * derivative;
    pidLastError = error;
    pidLastTime = nowMillis;

    // Adjusted RPM using PID correction.
    double adjustedRPM = pidSetpoint + pidOutput;
    if (adjustedRPM < 0) {
        adjustedRPM = 0;
    }
    if (adjustedRPM > MAX_RPM) {
        adjustedRPM = MAX_RPM;
    }

    // Stall detection: if the encoder reads less than 1 RPM while a target RPM is set.
    if (pidInput < 1 && pidSetpoint > 0) {
        LOGE("Stall detected! Stopping spindle.");
        if (stepper) {
            stepper->stopMove();
        }
        return;
    }

    // Compute steps per second based on adjusted RPM.
    long stepsPerSecond = (adjustedRPM * stepsPerRevolution) / 60;
    setStepperSpeed(stepsPerSecond);

    // Issue the continuous-motion command.
    if (stepper) {
        if (cwDirection) {
            stepper->runForward();
        } else {
            stepper->runBackward();
        }
    }
}

void EncodedStepperController::setEncoderZero() {
    // Read the current encoder angle and store it as the zero offset.
    encoderZeroOffset = encoder.readAngle();
    preferences.begin("stepper", false);
    preferences.putFloat("encZero", encoderZeroOffset);
    preferences.end();
    LOGI("Encoder zero position set to " + String(encoderZeroOffset));
}

void EncodedStepperController::calibrateEncoder() {
    // Implement your calibration routine here.
    // For example, command a full revolution, measure the encoder delta,
    // compute the exact stepsPerRevolution, then update and save to flash.
    LOGI("Calibrating encoder... (Not implemented)");
}

double EncodedStepperController::getActualRPM() const {
    return actualRPM;
}
