#include <Arduino.h>
#include "StepperController.h"
#include "StepperApi.h"

// Global instances
StepperController sc;
StepperApi stepperApi(&sc);

unsigned long previousMillis = 0;
const long interval = 1000; // Interval at which to print status to Serial (1 second)

// WiFi credentials (update these with your network details)
const char *ssid = "YourNewNeighbors";
const char *password = "@tumamama13";

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Setup the stepper controller
    sc.setup();

    // Start the web interface (this serves the dashboard and REST endpoints)
    stepperApi.begin(ssid, password);
}

void loop()
{
    int stepPinState = digitalRead(stepPinIn);    
    Serial.println("STEP: " + String(stepPinState));
    delay(1000);
    // Run the stepper controller's loop for real-time control and PWM measurement
    // sc.loop();

    // // Print status to Serial every second for debugging purposes
    // unsigned long currentMillis = millis();
    // if (currentMillis - previousMillis >= interval) {
    //     previousMillis = currentMillis;
    //     String mode = sc.getMode();
    //     String spindleInfo = sc.getSpindleInfo();

    //     if (mode == "Spindle") {
    //         Serial.printf("Current Mode: %s, Spindle Info: %s\n", mode.c_str(), spindleInfo.c_str());
    //     } else {
    //         Serial.printf("Current Mode: %s\n", mode.c_str());
    //     }
    // }
}
