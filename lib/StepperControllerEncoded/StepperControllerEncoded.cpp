#include "StepperController.h"
#include <Wire.h>

// Global variables for PWM measurement via interrupts
volatile uint32_t lastRiseCW = 0;
volatile uint32_t pulseWidthCW = 0;
volatile uint32_t periodCW = 0;
volatile bool newMeasurementCW = false;

volatile uint32_t lastRiseCCW = 0;
volatile uint32_t pulseWidthCCW = 0;
volatile uint32_t periodCCW = 0;
volatile bool newMeasurementCCW = false;

// ISR for PWM measurement (CW)
void IRAM_ATTR pwmCW_ISR() {
    uint32_t now = micros();
    int level = digitalRead(spindle_pwm_cw_pin);
    if (level == HIGH) { // Rising edge
        if (lastRiseCW != 0) {
            periodCW = now - lastRiseCW;
        }
        lastRiseCW = now;
    } else { // Falling edge
        pulseWidthCW = now - lastRiseCW;
        newMeasurementCW = true;
    }
}

// ISR for PWM measurement (CCW)
void IRAM_ATTR pwmCCW_ISR() {
    uint32_t now = micros();
    int level = digitalRead(spindle_pwm_ccw_pin);
    if (level == HIGH) { // Rising edge
        if (lastRiseCCW != 0) {
            periodCCW = now - lastRiseCCW;
        }
        lastRiseCCW = now;
    } else { // Falling edge
        pulseWidthCCW = now - lastRiseCCW;
        newMeasurementCCW = true;
    }
}

// Passthrough ISRs for Motion Mode signals
static void IRAM_ATTR handleEnableChangeInterrupt() {
    digitalWrite(enablePinOut, digitalRead(enablePinIn));
}
static void IRAM_ATTR handleDirectionChangeInterrupt() {
    digitalWrite(dirPinOut, digitalRead(dirPinIn));
}
static void IRAM_ATTR handleStepInterrupt() {
    digitalWrite(stepPinOut, digitalRead(stepPinIn));
}

//-----------------------------
// StepperController Methods
//-----------------------------

StepperController::StepperController() :
    engine(),
    stepper(nullptr),
    defaultMode(0),
    currentMode(MOTION_MODE),
    stepsPerRevolution(400),
    acceleration(1000),
    linearAcceleration(0),
    jumpStart(0),
    autoEnable(true),
    motionOverride(false),
    encoder(),
    zeroButton(ZERO_BUTTON_PIN, true),
    encoderZeroOffset(0.0),
    lastEncoderAngle(0.0),
    lastEncoderTime(0),
    actualRPM(0.0),
    pidSetpoint(0.0),
    pidInput(0.0),
    pidOutput(0.0),
    pidKp(0.1), pidKi(0.01), pidKd(0.0),
    pidIntegral(0.0),
    pidLastError(0.0),
    pidLastTime(0)
{
    // Constructor
}

void StepperController::setup() {
    LOGI("Setting up pins...");
    // Setup input pins from the MKS board
    pinMode(dirPinIn, INPUT);
    pinMode(enablePinIn, INPUT);
    pinMode(stepPinIn, INPUT);

    // Setup output pins to the stepper driver
    pinMode(dirPinOut, OUTPUT);
    pinMode(enablePinOut, OUTPUT);
    pinMode(stepPinOut, OUTPUT);

    // Setup PWM input pins and spindle enable pin
    pinMode(spindle_pwm_cw_pin, INPUT);
    pinMode(spindle_pwm_ccw_pin, INPUT);
    pinMode(spindle_enable_pin, INPUT);

    // Initialize I2C for the encoder
    Wire.begin();

    // Load settings from flash (stepsPerRevolution and encoder zero offset)
    loadSettings();
    LOGI("Settings loaded.");

    // Initialize FastAccelStepper engine and connect the stepper
    engine.init();
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
    stepper = engine.stepperConnectToPin(stepPinOut, DRIVER_DONT_CARE);
#else
    stepper = engine.stepperConnectToPin(stepPinOut);
#endif
    if (stepper) {
        stepper->setDirectionPin(dirPinOut);
        stepper->setEnablePin(enablePinOut);
        stepper->setAutoEnable(autoEnable);
        stepper->setAcceleration(acceleration);
        stepper->setLinearAcceleration(linearAcceleration);
        stepper->setJumpStart(jumpStart);
    } else {
        LOGE("Stepper not connected!");
    }
    LOGI("Stepper Engine setup completed.");

    // Attach passthrough interrupts for Motion Mode signals
    attachInterrupt(digitalPinToInterrupt(stepPinIn), handleStepInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(dirPinIn), handleDirectionChangeInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enablePinIn), handleEnableChangeInterrupt, CHANGE);
    LOGI("Motion mode interrupts attached.");

    // Attach interrupts for PWM measurement on both channels
    attachInterrupt(digitalPinToInterrupt(spindle_pwm_cw_pin), pwmCW_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(spindle_pwm_ccw_pin), pwmCCW_ISR, CHANGE);
    LOGI("PWM measurement interrupts attached.");

    // Initialize the zero calibration button (with debounce)
    zeroButton.begin();
}

void StepperController::loop() {
    // Process zero calibration button
    zeroButton.loop();
    if (zeroButton.pressed()) {
        setEncoderZero();
    }

    // Poll encoder for RPM measurement
    double currentAngle = encoder.getAngle(); // angle in degrees [0,360)
    double relativeAngle = currentAngle - encoderZeroOffset;
    if (relativeAngle < 0) relativeAngle += 360;

    unsigned long nowMicros = micros();
    if (lastEncoderTime != 0) {
        double deltaAngle = relativeAngle - lastEncoderAngle;
        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;
        double deltaTime = (nowMicros - lastEncoderTime) / 1000000.0; // seconds
        double measuredRPM = (deltaAngle / 360.0) * (60.0 / deltaTime);
        actualRPM = measuredRPM;
    }
    lastEncoderAngle = relativeAngle;
    lastEncoderTime = nowMicros;

    // Check spindle mode status
    if (digitalRead(spindle_enable_pin) == HIGH) {
        if (currentMode != SPINDLE_MODE) {
            switchToSpindleMode();
        }

        // Retrieve PWM measurements from both channels
        uint16_t pwmCWValue = 0;
        uint16_t pwmCCWValue = 0;
        uint32_t periodCWCopy = 0, periodCCWCopy = 0;
        uint32_t pulseCWCopy = 0, pulseCCWCopy = 0;
        bool newCW = false, newCCW = false;
        noInterrupts();
        newCW = newMeasurementCW;
        newCCW = newMeasurementCCW;
        if (newCW && periodCW > 0) {
            periodCWCopy = periodCW;
            pulseCWCopy = pulseWidthCW;
            newMeasurementCW = false;
        }
        if (newCCW && periodCCW > 0) {
            periodCCWCopy = periodCCW;
            pulseCCWCopy = pulseWidthCCW;
            newMeasurementCCW = false;
        }
        interrupts();

        if (newCW) {
            float duty = (float)pulseCWCopy / periodCWCopy;
            pwmCWValue = (uint16_t)(duty * 1023);
        }
        if (newCCW) {
            float duty = (float)pulseCCWCopy / periodCCWCopy;
            pwmCCWValue = (uint16_t)(duty * 1023);
        }

        const uint16_t noiseThreshold = 10;
        bool cwActive = (pwmCWValue > noiseThreshold);
        bool ccwActive = (pwmCCWValue > noiseThreshold);

        if (cwActive || ccwActive) {
            if (cwActive && (!ccwActive || pwmCWValue >= pwmCCWValue)) {
                digitalWrite(dirPinOut, HIGH);
                handleSpindleMode(pwmCWValue, true);
            } else if (ccwActive) {
                digitalWrite(dirPinOut, LOW);
                handleSpindleMode(pwmCCWValue, false);
            }
        }
    } else {
        if (currentMode != MOTION_MODE) {
            switchToMotionMode();
        }
        handleMotionMode();
    }
}

void StepperController::switchToMotionMode() {
    currentMode = MOTION_MODE;
    if (stepper) {
        stepper->stopMove();
        stepper->disableOutputs();
    }
    attachInterrupt(digitalPinToInterrupt(stepPinIn), handleStepInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(dirPinIn), handleDirectionChangeInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enablePinIn), handleEnableChangeInterrupt, CHANGE);
    LOGI("Switched to Motion Mode");
}

void StepperController::handleMotionMode() {
    // In motion mode, passthrough is handled via attached interrupts.
}

void StepperController::switchToSpindleMode() {
    currentMode = SPINDLE_MODE;
    detachInterrupt(digitalPinToInterrupt(stepPinIn));
    detachInterrupt(digitalPinToInterrupt(dirPinIn));
    detachInterrupt(digitalPinToInterrupt(enablePinIn));
    if (stepper) {
        stepper->enableOutputs();
    }
    LOGI("Switched to Spindle Mode");
}

void StepperController::handleSpindleMode(uint16_t pwmValue, bool cwDirection) {
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
    
    double adjustedRPM = pidSetpoint + pidOutput;
    if (adjustedRPM < 0) adjustedRPM = 0;
    if (adjustedRPM > MAX_RPM) adjustedRPM = MAX_RPM;

    // Stall detection: if encoder reads less than 1 RPM while a target is set, stop spindle.
    if (pidInput < 1 && pidSetpoint > 0) {
        LOGE("Stall detected! Stopping spindle.");
        if (stepper) {
            stepper->stopMove();
        }
        return;
    }

    long stepsPerSecond = (adjustedRPM * stepsPerRevolution) / 60;
    setStepperSpeed(stepsPerSecond);

    if (stepper) {
        if (cwDirection) {
            stepper->runForward();
        } else {
            stepper->runBackward();
        }
    }
}

void StepperController::setStepperSpeed(long stepsPerSecond) {
    if (stepper) {
        stepper->setSpeedInHz(stepsPerSecond);
    }
}

void StepperController::setAccelerationSetting(long accel) {
    acceleration = accel;
    if (stepper) {
        stepper->setAcceleration(acceleration);
    }
    preferences.begin("stepper", false);
    preferences.putLong("accel", acceleration);
    preferences.end();
}

void StepperController::setLinearAccelerationSetting(uint32_t linearAcc) {
    linearAcceleration = linearAcc;
    if (stepper) {
        stepper->setLinearAcceleration(linearAcceleration);
    }
    preferences.begin("stepper", false);
    preferences.putULong("linAcc", linearAcceleration);
    preferences.end();
}

void StepperController::setJumpStartSetting(uint32_t jumpStep) {
    jumpStart = jumpStep;
    if (stepper) {
        stepper->setJumpStart(jumpStart);
    }
    preferences.begin("stepper", false);
    preferences.putULong("jump", jumpStart);
    preferences.end();
}

void StepperController::setAutoEnableSetting(bool autoEn) {
    autoEnable = autoEn;
    if (stepper) {
        stepper->setAutoEnable(autoEnable);
    }
    preferences.begin("stepper", false);
    preferences.putBool("autoEn", autoEnable);
    preferences.end();
}

void StepperController::loadSettings() {
    preferences.begin("stepper", false);
    stepsPerRevolution = preferences.getUInt("stepsPerRev", 400);
    acceleration = preferences.getLong("accel", 1000);
    linearAcceleration = preferences.getULong("linAcc", 0);
    jumpStart = preferences.getULong("jump", 0);
    autoEnable = preferences.getBool("autoEn", true);
    encoderZeroOffset = preferences.getFloat("encZero", 0.0);
    preferences.end();
}

void StepperController::saveSettings() {
    preferences.begin("stepper", true);
    preferences.putUInt("stepsPerRev", stepsPerRevolution);
    preferences.putLong("accel", acceleration);
    preferences.putULong("linAcc", linearAcceleration);
    preferences.putULong("jump", jumpStart);
    preferences.putBool("autoEn", autoEnable);
    preferences.putFloat("encZero", encoderZeroOffset);
    preferences.end();
}

void StepperController::enableMotionOverride() {
    detachInterrupt(digitalPinToInterrupt(stepPinIn));
    detachInterrupt(digitalPinToInterrupt(dirPinIn));
    detachInterrupt(digitalPinToInterrupt(enablePinIn));
    motionOverride = true;
    LOGI("Motion override enabled");
}

void StepperController::disableMotionOverride() {
    motionOverride = false;
    attachInterrupt(digitalPinToInterrupt(stepPinIn), handleStepInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(dirPinIn), handleDirectionChangeInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enablePinIn), handleEnableChangeInterrupt, CHANGE);
    LOGI("Motion override disabled");
}

void StepperController::testMotionCommand() {
    if (stepper) {
        if (!stepper->isRunning()) {
            int8_t ret = stepper->runForward();
            if (ret != 0) {
                LOGI("runForward failed: " + String(ret));
            } else {
                LOGI("Motion override: Running forward");
            }
        }
    }
}

void StepperController::setEncoderZero() {
    encoderZeroOffset = encoder.getAngle();
    preferences.begin("stepper", false);
    preferences.putFloat("encZero", encoderZeroOffset);
    preferences.end();
    LOGI("Encoder zero position set to " + String(encoderZeroOffset));
}

void StepperController::calibrateEncoder() {
    // Implement your calibration routine to update stepsPerRevolution based on encoder feedback.
    LOGI("Calibrating encoder... (Not implemented)");
}

String StepperController::getMode() {
    return (currentMode == SPINDLE_MODE) ? "Spindle" : "Motion";
}

String StepperController::getSpindleInfo() {
    unsigned long rpm = getCurrentRPM();
    return "RPM: " + String(rpm);
}

unsigned long StepperController::getCurrentRPM() {
    if (stepper) {
        long stepsPerMilliSecond = stepper->getCurrentSpeedInMilliHz();
        long stepsPerSecond = stepsPerMilliSecond / 1000;
        return (stepsPerSecond * 60) / stepsPerRevolution;
    }
    return 0;
}

int StepperController::getCurrentPosition() {
    return (stepper) ? stepper->getCurrentPosition() : 0;
}
