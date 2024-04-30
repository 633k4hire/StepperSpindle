#include "StepperController.h"

StepperController::StepperController() {
    // Constructor might set up some initial conditions if necessary
}

void StepperController::setup() {
    pinMode(dirPinIn, INPUT);
    pinMode(enablePinIn, INPUT);
    pinMode(stepPinIn, INPUT);
    pinMode(pwmInPin, INPUT);

    pinMode(dirPinOut, OUTPUT);
    pinMode(enablePinOut, OUTPUT);
    pinMode(stepPinOut, OUTPUT);

    loadSettings();  // Load settings from preferences

    // Initialize the FastAccelStepper engine
    engine.init();
    //stepper = engine.stepperConnectToPin(stepPinOut);
    stepper = engine.stepperConnectToPin(stepPinOut, DRIVER_RMT); //use RMT
    if (stepper) {
        stepper->setDirectionPin(dirPinOut);
        stepper->setEnablePin(enablePinOut);
        stepper->setAutoEnable(true);
    }

    // Set up interrupt for Motion Mode pass-through
    attachInterrupt(digitalPinToInterrupt(stepPinIn), handleStepInterrupt, CHANGE);

    // Attach an interrupt to handle changes in direction
    digitalWrite(dirPinOut, digitalRead(dirPinIn)); // Set initial direction
    attachInterrupt(digitalPinToInterrupt(dirPinIn), handleDirectionChangeInterrupt, CHANGE);

    // Attach an interrupt to handle changes in enable
    digitalWrite(enablePinOut, digitalRead(enablePinIn)); // Set initial direction
    attachInterrupt(digitalPinToInterrupt(enablePinIn), handleEnableChangeInterrupt, CHANGE);

    // Initialize the pulse counter for more accurate PWM measurements
    initPulseCounter();
}

void StepperController::initPulseCounter() {
    pcnt_config_t pcnt_config;
    pcnt_config.pulse_gpio_num = pwmInPin;
    pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    pcnt_config.channel = pcntChannel;
    pcnt_config.unit = pcntUnit;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DIS;
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim = 10000;
    pcnt_config.counter_l_lim = -10000;
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_clear(pcntUnit);
    pcnt_counter_resume(pcntUnit);
}
void StepperController::loadSettings() {
    preferences.begin("stepper", false);
    stepsPerRevolution = preferences.getUInt("stepsPerRev", 1600);  // Default to 1600 steps
    preferences.end();
}

void StepperController::saveSettings() {
    preferences.begin("stepper", true);
    preferences.putUInt("stepsPerRev", stepsPerRevolution);
    preferences.end();
}

// Interrupt service routine for enable changes
static void handleEnableChangeInterrupt() {
    digitalWrite(enablePinOut, digitalRead(enablePinIn));
}

// Interrupt service routine for direction changes
static void handleDirectionChangeInterrupt() {
    digitalWrite(dirPinOut, digitalRead(dirPinIn));
}

// Interrupt service routine for step changes
static void handleStepInterrupt() { 
    // Immediately set the step output pin to the same state
    digitalWrite(stepPinOut, digitalRead(stepPinIn));
}
void StepperController::loop() {
    int pwmValue = analogRead(pwmInPin);  // Read PWM value to decide on the mode
    if (pwmValue > 0) {  // Check if there's a significant PWM signal
        if (currentMode != SPINDLE_MODE) {
            switchToSpindleMode();
        }
        handleSpindleMode(pwmValue);
    } else {
        if (currentMode != MOTION_MODE) {
            switchToMotionMode();
        }
        handleMotionMode();
    }
}

void StepperController::switchToMotionMode() {
    currentMode = MOTION_MODE;
    // Additional logic to switch to motion mode if necessary
}

void StepperController::handleMotionMode() {
    // Motion mode logic is mostly handled by interrupts, but we can add safety checks here if needed
}

void StepperController::switchToSpindleMode() {
    currentMode = SPINDLE_MODE;
    stepper->enableOutputs();  // Ensure stepper is enabled
}

void StepperController::handleSpindleMode(int pwmValue) {
    unsigned long rpm = map(pwmValue, 0, 1023, 0, 1200);  // Convert PWM signal to an RPM value
    // Calculate the desired speed in steps per second
    long stepsPerSecond = (rpm * stepsPerRevolution) / 60;
    setStepperSpeed(stepsPerSecond);  // Adjust the stepper's speed
}

void StepperController::setStepperSpeed(long stepsPerSecond) {
    // Define maximum steps per second based on maximum RPM
    const long maxStepsPerSecond = (MAX_RPM * stepsPerRevolution) / 60;

    // Limit the steps per second to the maximum allowable speed
    long limitedStepsPerSecond = min(stepsPerSecond, maxStepsPerSecond);

    if (stepper) {
        // Set speed for stepper in Hz (steps per second)
        stepper->setSpeedInHz(limitedStepsPerSecond);
    }
}
