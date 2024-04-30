#include <Arduino.h>
#include "StepperController.h"

StepperController sc;

void setup() {
    sc.setup();  // Setup the stepper controller
}

void loop() {
    sc.loop();  // Continuously check and adjust stepper based on PWM input
}