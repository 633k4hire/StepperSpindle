#include "StepperController.h"
#define LOGI(x) Serial.println(x)
#define LOGE(x) Serial.println(x)

// --- Variables for PWM measurement via interrupts ---
// For CW PWM measurement:
volatile uint32_t lastRiseCW = 0;
volatile uint32_t pulseWidthCW = 0;
volatile uint32_t periodCW = 0;
volatile bool newMeasurementCW = false;

// For CCW PWM measurement:
volatile uint32_t lastRiseCCW = 0;
volatile uint32_t pulseWidthCCW = 0;
volatile uint32_t periodCCW = 0;
volatile bool newMeasurementCCW = false;

// --- ISRs for PWM measurement ---
// These ISRs are kept minimal to record edge timestamps.
void IRAM_ATTR pwmCW_ISR()
{
    uint32_t now = micros();
    int level = digitalRead(spindle_pwm_cw_pin);
    if (level == HIGH)
    { // Rising edge
        if (lastRiseCW != 0)
        {
            periodCW = now - lastRiseCW;
        }
        lastRiseCW = now;
    }
    else
    { // Falling edge
        pulseWidthCW = now - lastRiseCW;
        newMeasurementCW = true;
    }
}

void IRAM_ATTR pwmCCW_ISR()
{
    uint32_t now = micros();
    int level = digitalRead(spindle_pwm_ccw_pin);
    if (level == HIGH)
    { // Rising edge
        if (lastRiseCCW != 0)
        {
            periodCCW = now - lastRiseCCW;
        }
        lastRiseCCW = now;
    }
    else
    { // Falling edge
        pulseWidthCCW = now - lastRiseCCW;
        newMeasurementCCW = true;
    }
}

// Passthrough ISRs for Motion Mode signals (kept minimal)
static void IRAM_ATTR handleEnableChangeInterrupt()
{
    digitalWrite(enablePinOut, digitalRead(enablePinIn));
}

static void IRAM_ATTR handleDirectionChangeInterrupt()
{
    digitalWrite(dirPinOut, digitalRead(dirPinIn));
}

static void IRAM_ATTR handleStepInterrupt()
{
    digitalWrite(stepPinOut, digitalRead(stepPinIn));
}

StepperController::StepperController()
{
    // Constructor: initial conditions can be set here if needed.
}

void StepperController::setup()
{
    LOGI("Settings Pins...");
    // Setup input pins from the MKS board
    pinMode(dirPinIn, INPUT);
    pinMode(enablePinIn, INPUT);
    pinMode(stepPinIn, INPUT);

    // Setup output pins to the stepper driver
    pinMode(dirPinOut, OUTPUT);
    pinMode(enablePinOut, OUTPUT);
    pinMode(stepPinOut, OUTPUT);

    // Setup PWM input pins and the spindle enable pin
    pinMode(spindle_pwm_cw_pin, INPUT);
    pinMode(spindle_pwm_ccw_pin, INPUT);
    pinMode(spindle_enable_pin, INPUT);

    loadSettings();
    LOGI("Settings loaded.");

    // Initialize FastAccelStepper engine and connect the stepper
    engine.init();
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
    // Use default driver selection (or specify DRIVER_DONT_CARE)
    stepper = engine.stepperConnectToPin(stepPinOut, DRIVER_DONT_CARE);
#else
    stepper = engine.stepperConnectToPin(stepPinOut);
#endif
    if (stepper)
    {
        stepper->setDirectionPin(dirPinOut);
        stepper->setEnablePin(enablePinOut);
        stepper->setAutoEnable(autoEnable);
        stepper->setAcceleration(acceleration);
        stepper->setLinearAcceleration(linearAcceleration);
        stepper->setJumpStart(jumpStart);
    }else{
        LOGE("Stepper not connected!");
    }
    LOGI("Stepper Engine setup completed.");

    // Attach passthrough interrupts for Motion Mode signals
    attachInterrupt(digitalPinToInterrupt(stepPinIn), handleStepInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(dirPinIn), handleDirectionChangeInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enablePinIn), handleEnableChangeInterrupt, CHANGE);
    LOGI("Motion mode interrupts attached.");

    // Attach interrupts for PWM measurement on both channels (CHANGE mode for both edges)
    attachInterrupt(digitalPinToInterrupt(spindle_pwm_cw_pin), pwmCW_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(spindle_pwm_ccw_pin), pwmCCW_ISR, CHANGE);
    LOGI("PWM measurement interrupts attached.");
}

void StepperController::loop()
{
    // If motion override is active, issue test motion commands and skip passthrough.
    if (motionOverride)
    {
        testMotionCommand();
        return;
    }

    // Normal operation: check spindle enable pin and act accordingly.
    if (digitalRead(spindle_enable_pin) == HIGH)
    {
        if (currentMode != SPINDLE_MODE)
        {
            switchToSpindleMode();
        }

        // Retrieve PWM measurements from both channels
        uint16_t pwmCWValue = 0;
        uint16_t pwmCCWValue = 0;
        uint32_t periodCWCopy = 0;
        uint32_t periodCCWCopy = 0;
        uint32_t pulseCWCopy = 0;
        uint32_t pulseCCWCopy = 0;
        bool newCW = false;
        bool newCCW = false;

        noInterrupts();
        newCW = newMeasurementCW;
        newCCW = newMeasurementCCW;
        if (newCW && periodCW > 0)
        {
            periodCWCopy = periodCW;
            pulseCWCopy = pulseWidthCW;
            newMeasurementCW = false;
        }
        if (newCCW && periodCCW > 0)
        {
            periodCCWCopy = periodCCW;
            pulseCCWCopy = pulseWidthCCW;
            newMeasurementCCW = false;
        }
        interrupts();

        // Calculate the duty cycle (10-bit value) for each channel.
        if (newCW)
        {
            float duty = (float)pulseCWCopy / periodCWCopy;
            pwmCWValue = (uint16_t)(duty * 1023);
        }
        if (newCCW)
        {
            float duty = (float)pulseCCWCopy / periodCCWCopy;
            pwmCCWValue = (uint16_t)(duty * 1023);
        }

        // Use a noise threshold to filter out spurious measurements.
        const uint16_t noiseThreshold = 10;
        bool cwActive = (pwmCWValue > noiseThreshold);
        bool ccwActive = (pwmCCWValue > noiseThreshold);

        // Choose the active channel based on which one has a valid measurement.
        if (cwActive || ccwActive)
        {
            if (cwActive && (!ccwActive || pwmCWValue >= pwmCCWValue))
            {
                digitalWrite(dirPinOut, HIGH); // Set direction for CW
                handleSpindleMode(pwmCWValue, true);
            }
            else if (ccwActive)
            {
                digitalWrite(dirPinOut, LOW); // Set direction for CCW
                handleSpindleMode(pwmCCWValue, false);
            }
        }
    }
    else
    {
        // If the spindle enable pin is LOW, revert to motion mode.
        if (currentMode != MOTION_MODE)
        {
            switchToMotionMode();
        }
        handleMotionMode();
    }
}

void StepperController::switchToMotionMode()
{
    currentMode = MOTION_MODE;
    if (stepper)
    {
        // Stop any continuous motion and disable outputs.
        stepper->stopMove();
        stepper->disableOutputs();
    }
    // Reattach the passthrough interrupts.
    attachInterrupt(digitalPinToInterrupt(stepPinIn), handleStepInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(dirPinIn), handleDirectionChangeInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enablePinIn), handleEnableChangeInterrupt, CHANGE);
    LOGI("Switched to Motion Mode");
}

void StepperController::handleMotionMode()
{
    // In motion mode, the passthrough is handled via the attached interrupts.
}

void StepperController::switchToSpindleMode()
{
    currentMode = SPINDLE_MODE;
    // Detach passthrough interrupts to avoid interference.
    detachInterrupt(digitalPinToInterrupt(stepPinIn));
    detachInterrupt(digitalPinToInterrupt(dirPinIn));
    detachInterrupt(digitalPinToInterrupt(enablePinIn));
    if (stepper)
    {
        stepper->enableOutputs();
    }
    LOGI("Switched to Spindle Mode");
}

void StepperController::handleSpindleMode(uint16_t pwmValue, bool cwDirection)
{
    // Map the 10-bit PWM value to an RPM value.
    unsigned long rpm = map(pwmValue, 0, 1023, 0, MAX_RPM);
    long stepsPerSecond = (rpm * stepsPerRevolution) / 60;
    setStepperSpeed(stepsPerSecond);

    // Issue the continuous-motion command.
    if (stepper)
    {
        if (cwDirection)
        {
            stepper->runForward();
        }
        else
        {
            stepper->runBackward();
        }
    }
}

void StepperController::setStepperSpeed(long stepsPerSecond)
{
    if (stepper)
    {
        stepper->setSpeedInHz(stepsPerSecond);
    }
}

// --- FastAccelStepper configuration setters ---

void StepperController::setAccelerationSetting(long accel)
{
    acceleration = accel;
    if (stepper)
    {
        stepper->setAcceleration(acceleration);
    }
    preferences.begin("stepper", false);
    preferences.putLong("accel", acceleration);
    preferences.end();
}

void StepperController::setLinearAccelerationSetting(uint32_t linearAcc)
{
    linearAcceleration = linearAcc;
    if (stepper)
    {
        stepper->setLinearAcceleration(linearAcceleration);
    }
    preferences.begin("stepper", false);
    preferences.putULong("linAcc", linearAcceleration);
    preferences.end();
}

void StepperController::setJumpStartSetting(uint32_t jumpStep)
{
    jumpStart = jumpStep;
    if (stepper)
    {
        stepper->setJumpStart(jumpStart);
    }
    preferences.begin("stepper", false);
    preferences.putULong("jump", jumpStart);
    preferences.end();
}

void StepperController::setAutoEnableSetting(bool autoEn)
{
    autoEnable = autoEn;
    if (stepper)
    {
        stepper->setAutoEnable(autoEnable);
    }
    preferences.begin("stepper", false);
    preferences.putBool("autoEn", autoEnable);
    preferences.end();
}

// --- Preferences Loading and Saving ---

void StepperController::loadSettings()
{
    preferences.begin("stepper", false);
    stepsPerRevolution = preferences.getUInt("stepsPerRev", 1600);
    acceleration = preferences.getLong("accel", 1000);
    linearAcceleration = preferences.getULong("linAcc", 0);
    jumpStart = preferences.getULong("jump", 0);
    autoEnable = preferences.getBool("autoEn", true);
    preferences.end();
}

void StepperController::saveSettings()
{
    preferences.begin("stepper", true);
    preferences.putUInt("stepsPerRev", stepsPerRevolution);
    preferences.putLong("accel", acceleration);
    preferences.putULong("linAcc", linearAcceleration);
    preferences.putULong("jump", jumpStart);
    preferences.putBool("autoEn", autoEnable);
    preferences.end();
}

// --- Motion Override Functions ---

void StepperController::enableMotionOverride()
{
    // Detach passthrough interrupts so that we can issue motion commands.
    detachInterrupt(digitalPinToInterrupt(stepPinIn));
    detachInterrupt(digitalPinToInterrupt(dirPinIn));
    detachInterrupt(digitalPinToInterrupt(enablePinIn));
    motionOverride = true;
    LOGI("Motion override enabled");
}

void StepperController::disableMotionOverride()
{
    motionOverride = false;
    // Reattach the passthrough interrupts for motion mode.
    attachInterrupt(digitalPinToInterrupt(stepPinIn), handleStepInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(dirPinIn), handleDirectionChangeInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enablePinIn), handleEnableChangeInterrupt, CHANGE);
    LOGI("Motion override disabled");
}

void StepperController::testMotionCommand()
{
    // For testing purposes, if the stepper is not already running, start a continuous forward move.
    if (stepper)
    {
        if (!stepper->isRunning())
        {
            int8_t ret = stepper->runForward();
            if (ret != 0)
            {
                LOGI("runForward failed: " + String(ret));
            }
            else
            {
                LOGI("Motion override: Running forward");
            }
        }
    }
}

// --- Web API Command Functions ---

// For spindle mode: set target RPM and direction.
void StepperController::webSetSpindleSpeed(unsigned long rpm, bool cwDirection)
{
    // Ensure we are in spindle mode.
    if (currentMode != SPINDLE_MODE)
    {
        switchToSpindleMode();
    }
    long stepsPerSecond = (rpm * stepsPerRevolution) / 60;
    setStepperSpeed(stepsPerSecond);
    if (stepper)
    {
        if (cwDirection)
        {
            stepper->runForward();
        }
        else
        {
            stepper->runBackward();
        }
        LOGI("Web API: Set Spindle Speed to " + String(rpm) + " RPM");
    }
}

// For motion mode: move to an absolute position.
void StepperController::webMotionMoveTo(int32_t position, bool blocking)
{
    // Ensure we are in motion mode.
    if (currentMode != MOTION_MODE)
    {
        switchToMotionMode();
    }
    if (stepper)
    {
        int8_t ret = stepper->moveTo(position, blocking);
        if (ret != 0)
        {
            LOGI("Web API: moveTo error: " + String(ret));
        }
        else
        {
            LOGI("Web API: Moving to position " + String(position));
        }
    }
}

// For motion mode: perform a relative move.
void StepperController::webMotionMove(int32_t steps, bool blocking)
{
    if (currentMode != MOTION_MODE)
    {
        switchToMotionMode();
    }
    if (stepper)
    {
        int8_t ret = stepper->move(steps, blocking);
        if (ret != 0)
        {
            LOGI("Web API: move error: " + String(ret));
        }
        else
        {
            LOGI("Web API: Moving " + String(steps) + " steps");
        }
    }
}

// For motion mode: stop any ongoing move.
void StepperController::webMotionStop()
{
    if (stepper)
    {
        stepper->stopMove();
        LOGI("Web API: Motion stop issued");
    }
}

String StepperController::getMode()
{
    if (currentMode == SPINDLE_MODE)
    {
        return "Spindle";
    }
    else
    {
        return "Motion";
    }
}

String StepperController::getSpindleInfo()
{
    // Assuming you have relevant spindle information to return
    // For example, you might return the current RPM or other status
    unsigned long rpm = getCurrentRPM(); // Implement this method as needed
    return "RPM: " + String(rpm);
}

unsigned long StepperController::getCurrentRPM()
{
    // Calculate the RPM based on the current stepper speed
    if (stepper)
    {
        long stepsPerMilliSecond = stepper->getCurrentSpeedInMilliHz();
        long stepsPerSecond = stepsPerMilliSecond / 1000; // Convert milliHz to Hz
        return (stepsPerSecond * 60) / stepsPerRevolution;
    }
    return 0;
}

int StepperController::getCurrentPosition()
{
    if (stepper)
    {
        return stepper->getCurrentPosition();
    }
    return 0;
}