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
        stepper->setDirectionPin(dirPinOut);
        stepper->setEnablePin(enablePinOut);
        stepper->setAutoEnable(true);
    }

    initRMT(); // Initialize the RMT module for PWM reading
}

void StepperController::initRMT() {
    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_CHANNEL_0;
    rmt_rx.gpio_num = static_cast<gpio_num_t>(pwmInPin);
    rmt_rx.clk_div = 80; // Assuming 80 MHz APB clock
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = 4000; // Set based on your PWM frequency

    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0); // Allocate memory for RX ring buffer
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

void StepperController::readRMT() {
    RingbufHandle_t rb = NULL;
    rmt_get_ringbuf_handle(RMT_CHANNEL_0, &rb);
    rmt_rx_start(RMT_CHANNEL_0, true);

    size_t rx_size = 0;
    rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
    if (item) {
        int high_duration = item->duration0; // High part of the PWM pulse
        int low_duration = item->duration1;  // Low part of the PWM pulse
        int total_duration = high_duration + low_duration;
        dutyCycle = (high_duration / (float)total_duration) * 100.0;

        // Use duty cycle or frequency for further processing
        // Serial.print("Duty Cycle: ");
        // Serial.print(dutyCycle);
        // Serial.println("%");

        vRingbufferReturnItem(rb, (void*) item);
    }
    rmt_rx_stop(RMT_CHANNEL_0);
}
void StepperController::loop() {
    readRMT(); // Read the PWM duty cycle

    // Process duty cycle to decide on the mode
    if (dutyCycle > 0) {  // Assuming duty_cycle is stored from readRMT
        if (currentMode != SPINDLE_MODE) {
            switchToSpindleMode();
        }
        handleSpindleMode(dutyCycle);
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

void StepperController::handleSpindleMode(float duty_cycle) {
    // Map duty cycle (0% to 100%) to RPM (0 to 1200 RPM)
    float maxRPM = 1200.0;  // Maximum RPM at 100% duty cycle
    float rpm = (duty_cycle / 100.0) * maxRPM;

    // Convert RPM to steps per minute
    long stepsPerMinute = static_cast<long>(rpm * stepsPerRevolution / 60);

    // Now set the speed in steps per minute
    if (stepper) {
        stepper->setSpeedInHz(stepsPerMinute / 60);  // FastAccelStepper uses Hz, convert steps/min to steps/sec
    }

    // Optionally, start the stepper if not already moving
    if (!stepper->isRunning()) {
        stepper->moveTo(stepper->getCurrentPosition() + stepsPerMinute); // Example to move a certain number of steps
    }
}

