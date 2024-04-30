
// StepperController.cpp
#include "StepperController.h"

StepperController::StepperController() {
    // Constructor for the StepperController class
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

    pcnt_counter_pause(pcntUnit);
    pcnt_counter_clear(pcntUnit);
    pcnt_counter_resume(pcntUnit);
}

void StepperController::setup() {
    Serial.begin(115200);
    initPulseCounter();

    engine.init();
    stepper = engine.stepperConnectToPin(stepPinStepper);
    if (stepper) {
        stepper->setDirectionPin(dirPinStepper);
        stepper->setEnablePin(enablePinStepper);
        stepper->setAutoEnable(true);

        // Optional delay settings if needed
        stepper->setDelayToEnable(50);
        stepper->setDelayToDisable(1000);

        stepper->setSpeedInUs(1000);  // 1000 microseconds per step
        stepper->setAcceleration(100);
        stepper->move(1000);  // Move 1000 steps
    }
}

void StepperController::loop() {
    checkPwmAndAdjustStepper();
}

void StepperController::checkPwmAndAdjustStepper() {
    int16_t count = 0;
    pcnt_get_counter_value(pcntUnit, &count);
    pcnt_counter_clear(pcntUnit);

    if (count > 0) {  // If PWM signal detected
        int stepsPerSecond = map(count, 0, 10000, 0, 1000);
        stepper->setSpeedInHz(stepsPerSecond); // Assuming 1000 Hz as the highest mapping
    } else {
        // Optional: additional logic for handling no PWM input
    }
}
