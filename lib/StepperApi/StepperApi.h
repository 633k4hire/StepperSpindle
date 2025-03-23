#ifndef STEPPERAPI_H
#define STEPPERAPI_H

#include <ESPAsyncWebServer.h>
#include "StepperController.h"

class StepperApi {
public:
    // Constructor receives a pointer reference to the StepperController
    StepperApi(StepperController* ctrl);
    // Begin WiFi and start the web server using provided credentials
    void begin(const char* ssid, const char* password);

private:
    AsyncWebServer server;
    StepperController* controller;
    bool apMode;  // Flag indicating if we're running in AP mode

    // Setup REST endpoints and page serving routes
    void setupRoutes();
};

#endif // STEPPERAPI_H
