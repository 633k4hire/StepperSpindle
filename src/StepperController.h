
// StepperController.h
#pragma once

#include <FastAccelStepper.h>
#include <driver/pcnt.h>

class StepperController {
public:
    StepperController();
    void setup();
    void loop();

private:
    const int dirPinStepper = 18;
    const int enablePinStepper = 26;
    const int stepPinStepper = 17;

    const int pwmInPin = 26; // PWM input pin using pulse counter

    FastAccelStepperEngine engine;
    FastAccelStepper* stepper = nullptr;

    static const pcnt_unit_t pcntUnit = PCNT_UNIT_0;
    static const pcnt_channel_t pcntChannel = PCNT_CHANNEL_0;
    void initPulseCounter();
    void checkPwmAndAdjustStepper();
};
