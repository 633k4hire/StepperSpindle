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
    stepper = engine.stepperConnectToPin(stepPinOut);
    if (stepper) {
        stepper->setEnablePin(enablePinOut);
        stepper->setAutoEnable(true);
        stepper->setAcceleration(1000);  // Set a default acceleration
        stepper->setSpeed(1000);         // Set a default speed (steps per second)
    }

    // Set up interrupt for Motion Mode pass-through
    attachInterrupt(digitalPinToInterrupt(stepPinIn), handleStepInterrupt, RISING);

    // Initialize the pulse counter for more accurate PWM measurements
    initPulseCounter();
}
static void handleStepInterrupt(){
    digitalWrite(dirPinOut, digitalRead(dirPinIn));  // Copy direction from input to output
    digitalWrite(stepPinOut, HIGH);  // Trigger step pulse
    delayMicroseconds(5);  // Minimum pulse width
    digitalWrite(stepPinOut, LOW);
}

void StepperController::loadSettings() {
    preferences.begin("stepper", false);
    unsigned int stepsPerRev = preferences.getUInt("stepsPerRev", 1600);  // Default to 1600 steps
    if (stepper) {
        stepper->setStepsPerRevolution(stepsPerRev);
    }
    preferences.end();
}

void StepperController::saveSettings() {
    preferences.begin("stepper", true);
    if (stepper) {
        preferences.putUInt("stepsPerRev", stepper->getStepsPerRevolution());
    }
    preferences.end();
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

int16_t StepperController::readPWM() {
    int16_t count;
    pcnt_get_counter_value(pcntUnit, &count);
    return count;
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
    long stepsPerMinute = (rpm * stepper->getStepsPerRevolution()) / 60;
    stepper->setSpeed(stepsPerMinute);  // Set speed in steps per minute
}
