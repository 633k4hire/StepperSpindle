#include "StepperController.h"
#define LOGI(x) Serial.println(x)



// Interrupt service routine for enable changes
static void handleEnableChangeInterrupt() {
    digitalWrite(enablePinOut, digitalRead(enablePinIn));
    LOGI("Enable pin state changed.");
}

// Interrupt service routine for direction changes
static void handleDirectionChangeInterrupt() {
    digitalWrite(dirPinOut, digitalRead(dirPinIn));
    LOGI("Direction pin state changed.");
}

// Interrupt service routine for step changes
static void handleStepInterrupt() { 
    digitalWrite(stepPinOut, digitalRead(stepPinIn));
    LOGI("Step pin state changed.");
}

StepperController::StepperController() {
    // Constructor might set up some initial conditions if necessary
}

void StepperController::setup() {
     Serial.begin(115200); 
     delay(500);
    Serial.println("Starting setup...");
    pinMode(dirPinIn, INPUT);
    pinMode(enablePinIn, INPUT);
    pinMode(stepPinIn, INPUT);
    pinMode(pwmInPin, INPUT);

    pinMode(dirPinOut, OUTPUT);
    pinMode(enablePinOut, OUTPUT);
    pinMode(stepPinOut, OUTPUT);
return;
    loadSettings();  // Load settings from preferences
    LOGI("Settings loaded.");

    // Initialize the FastAccelStepper engine
    engine.init();
    stepper = engine.stepperConnectToPin(stepPinOut); // Use RMT
    if (stepper) {
        stepper->setDirectionPin(dirPinOut);
        stepper->setEnablePin(enablePinOut);
        stepper->setAutoEnable(true);
    }
    LOGI("Stepper Engine setup completed.");

    // Set up interrupts for Motion Mode pass-through
    attachInterrupt(digitalPinToInterrupt(stepPinIn), handleStepInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(dirPinIn), handleDirectionChangeInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enablePinIn), handleEnableChangeInterrupt, CHANGE);
    LOGI("Interrupt setup completed.");
    // Initialize RMT for PWM measurement
    initRMT();
}

void StepperController::initRMT() {
    rmtConfig.rmt_mode = RMT_MODE_RX;
    rmtConfig.channel = RMT_CHANNEL_2;
    rmtConfig.gpio_num = (gpio_num_t)10;
    rmtConfig.mem_block_num = 1;
    rmtConfig.clk_div = 80;
    rmtConfig.rx_config.filter_en = true;
    rmtConfig.rx_config.filter_ticks_thresh = 100;
    rmtConfig.rx_config.idle_threshold = 0xFFFF;
    rmt_config(&rmtConfig);
     LOGI("Config of RMT Started.");
    
 
    rmt_rx_start(RMT_CHANNEL_2, true);
    LOGI("RMT setup completed.");
}

int16_t StepperController::readPWM() {
    RingbufHandle_t rb = NULL;
    size_t rx_size = 0;  // This will store the size of the received item.
    rmt_get_ringbuf_handle(RMT_CHANNEL_2, &rb);
    rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
    int16_t pwmValue = 0;
    if (item && rx_size > 0) {  // Ensure item is valid and size is correct
        uint32_t highDuration = item->duration0;  // High part duration
        uint32_t lowDuration = item->duration1;  // Low part duration
        uint32_t totalDuration = highDuration + lowDuration;

        if (totalDuration > 0) {  // Prevent division by zero
            float dutyCycle = (float)highDuration / totalDuration * 100.0f;  // Calculate duty cycle in percentage
            Serial.print("Read PWM: ");
            Serial.print(pwmValue);
            Serial.print(" (Duty Cycle: ");
            Serial.print(dutyCycle);
            Serial.println("%)");
            // Convert duty cycle to a 10-bit value, similar to an ADC read
            pwmValue = (int)(dutyCycle / 100.0f * 1023);
        }

        vRingbufferReturnItem(rb, (void*) item);  // Return the item to the ring buffer
    }
    return pwmValue;
}


void StepperController::loop() {
    return;
    Serial.print(".");
    int pwmValue = readPWM();
    if (pwmValue > 0) {
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
}

void StepperController::handleMotionMode() {
    // Motion mode logic is mostly handled by interrupts, but we can add safety checks here if needed
}

void StepperController::switchToSpindleMode() {
    currentMode = SPINDLE_MODE;
    stepper->enableOutputs();  // Ensure stepper is enabled
}

void StepperController::handleSpindleMode(int pwmValue) {
    unsigned long rpm = map(pwmValue, 0, 1023, 0, MAX_RPM);
    long stepsPerSecond = (rpm * stepsPerRevolution) / 60;
    setStepperSpeed(stepsPerSecond);
}

void StepperController::setStepperSpeed(long stepsPerSecond) {
    const long maxStepsPerSecond = (MAX_RPM * stepsPerRevolution) / 60;
    long limitedStepsPerSecond = min(stepsPerSecond, maxStepsPerSecond);
    if (stepper) {
        stepper->setSpeedInHz(limitedStepsPerSecond);
    }
}

void StepperController::loadSettings() {
    preferences.begin("stepper", false);
    stepsPerRevolution = preferences.getUInt("stepsPerRev", 1600);
    preferences.end();
}

void StepperController::saveSettings() {
    preferences.begin("stepper", true);
    preferences.putUInt("stepsPerRev", stepsPerRevolution);
    preferences.end();
}