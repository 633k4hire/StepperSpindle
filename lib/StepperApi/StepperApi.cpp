#include "StepperApi.h"
#include <WiFi.h>
#include <ESPmDNS.h>

// HTML served for the main dashboard page
const char index_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Stepper Controller Dashboard</title>
    <!-- Bootstrap Dark Theme (Bootswatch Darkly) -->
    <link href="https://cdn.jsdelivr.net/npm/bootswatch@5.2.3/dist/darkly/bootstrap.min.css" rel="stylesheet" crossorigin="anonymous">
    <!-- Chart.js -->
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  </head>
  <body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-dark">
      <a class="navbar-brand" href="#">Stepper Controller</a>
      <ul class="navbar-nav ms-auto">
         <li class="nav-item"><a class="nav-link" href="/wifi">WiFi Setup</a></li>
      </ul>
    </nav>
    <div class="container mt-4">
      <div id="alert" class="alert alert-danger" role="alert" style="display:none;"></div>
      <div class="row">
        <div class="col-md-6">
          <h3>Spindle Control</h3>
          <form id="spindleForm">
            <div class="mb-3">
              <label for="spindleRPM" class="form-label">RPM</label>
              <input type="number" class="form-control" id="spindleRPM" name="rpm" required>
            </div>
            <div class="mb-3">
              <label class="form-label">Direction</label><br>
              <div class="form-check form-check-inline">
                <input class="form-check-input" type="radio" name="direction" id="cw" value="cw" checked>
                <label class="form-check-label" for="cw">CW</label>
              </div>
              <div class="form-check form-check-inline">
                <input class="form-check-input" type="radio" name="direction" id="ccw" value="ccw">
                <label class="form-check-label" for="ccw">CCW</label>
              </div>
            </div>
            <button type="submit" class="btn btn-primary">Set Spindle Speed</button>
          </form>
        </div>
        <div class="col-md-6">
          <h3>Motion Control</h3>
          <form id="moveToForm" class="mb-3">
            <div class="mb-3">
              <label for="moveToPosition" class="form-label">Move To (Absolute Position)</label>
              <input type="number" class="form-control" id="moveToPosition" name="position" required>
            </div>
            <button type="submit" class="btn btn-primary">Move To</button>
          </form>
          <form id="moveForm" class="mb-3">
            <div class="mb-3">
              <label for="moveSteps" class="form-label">Move (Relative Steps)</label>
              <input type="number" class="form-control" id="moveSteps" name="steps" required>
            </div>
            <button type="submit" class="btn btn-primary">Move</button>
          </form>
          <button id="stopMotion" class="btn btn-danger">Stop Motion</button>
        </div>
      </div>
      <hr>
      <div class="row mt-4">
        <!-- Basic Status Section -->
        <div class="col-md-6">
          <h3>Status</h3>
          <p><strong>Mode:</strong> <span id="mode">-</span></p>
          <p><strong>Spindle Info:</strong> <span id="spindleInfo">-</span></p>
          <p><strong>RPM:</strong> <span id="rpm">0</span></p>
          <p><strong>Position:</strong> <span id="position">0</span></p>
        </div>
        <!-- RPM Chart -->
        <div class="col-md-6">
          <h3>RPM Chart</h3>
          <canvas id="rpmChart" width="400" height="300"></canvas>
        </div>
      </div>
      <hr>
      <!-- Detailed Stepper Info Section -->
      <div class="row mt-4">
        <div class="col-md-12">
          <h3>Stepper Detailed Information</h3>
          <table class="table table-dark table-striped">
            <tbody>
              <tr><th scope="row">Step Pin</th><td id="stepPin">-</td></tr>
              <tr><th scope="row">Direction Pin</th><td id="directionPin">-</td></tr>
              <tr><th scope="row">Direction (High Counts Up)</th><td id="directionHighCountsUp">-</td></tr>
              <tr><th scope="row">Enable Pin (High Active)</th><td id="enablePinHighActive">-</td></tr>
              <tr><th scope="row">Enable Pin (Low Active)</th><td id="enablePinLowActive">-</td></tr>
              <tr><th scope="row">Current Position</th><td id="currentPosition">-</td></tr>
              <tr><th scope="row">Running</th><td id="isRunning">-</td></tr>
              <tr><th scope="row">Max Speed (Hz)</th><td id="maxSpeedInHz">-</td></tr>
              <tr><th scope="row">Max Speed (mHz)</th><td id="maxSpeedInMilliHz">-</td></tr>
              <tr><th scope="row">Speed (µs)</th><td id="speedInUs">-</td></tr>
              <tr><th scope="row">Speed (Ticks)</th><td id="speedInTicks">-</td></tr>
              <tr><th scope="row">Current Speed (µs)</th><td id="currentSpeedInUs">-</td></tr>
              <tr><th scope="row">Current Speed (mHz)</th><td id="currentSpeedInMilliHz">-</td></tr>
              <tr><th scope="row">Acceleration</th><td id="acceleration">-</td></tr>
              <tr><th scope="row">Current Acceleration</th><td id="currentAcceleration">-</td></tr>
              <tr><th scope="row">Ramp State</th><td id="rampState">-</td></tr>
              <tr><th scope="row">Ramp Generator Active</th><td id="isRampGeneratorActive">-</td></tr>
              <tr><th scope="row">Queue Entries</th><td id="queueEntries">-</td></tr>
              <tr><th scope="row">Ticks in Queue</th><td id="ticksInQueue">-</td></tr>
              <tr><th scope="row">Position After Commands</th><td id="positionAfterCommandsCompleted">-</td></tr>
              <tr><th scope="row">Period After Commands (µs)</th><td id="periodInUsAfterCommandsCompleted">-</td></tr>
              <tr><th scope="row">Period After Commands (Ticks)</th><td id="periodInTicksAfterCommandsCompleted">-</td></tr>
            </tbody>
          </table>
        </div>
      </div>
    </div>
    <script>
      // RPM Chart Setup
      let rpmData = [];
      let labels = [];
      const ctx = document.getElementById('rpmChart').getContext('2d');
      const rpmChart = new Chart(ctx, {
        type: 'line',
        data: {
          labels: labels,
          datasets: [{
            label: 'RPM',
            data: rpmData,
            borderColor: 'rgba(75, 192, 192, 1)',
            fill: false
          }]
        },
        options: {
          scales: {
            x: { title: { display: true, text: 'Time (s)' } },
            y: { title: { display: true, text: 'RPM' } }
          }
        }
      });

      function showAlert(message) {
        const alertDiv = document.getElementById('alert');
        alertDiv.style.display = 'block';
        alertDiv.innerText = message;
        setTimeout(() => { alertDiv.style.display = 'none'; }, 5000);
      }

      async function updateStatus() {
        try {
          const response = await fetch('/api/status');
          if (!response.ok) { showAlert('Failed to fetch status'); return; }
          const data = await response.json();
          document.getElementById('mode').innerText = data.mode;
          document.getElementById('spindleInfo').innerText = data.spindleInfo;
          document.getElementById('rpm').innerText = data.rpm;
          document.getElementById('position').innerText = data.position || 0;
          const currentTime = new Date().toLocaleTimeString();
          labels.push(currentTime);
          rpmData.push(data.rpm);
          if (labels.length > 20) { labels.shift(); rpmData.shift(); }
          rpmChart.update();
        } catch (err) { showAlert('Error updating status'); }
      }
      setInterval(updateStatus, 1000);

      async function updateStepperInfo() {
        try {
          const response = await fetch('/api/stepper/info');
          if (!response.ok) { showAlert('Failed to fetch stepper info'); return; }
          const data = await response.json();
          document.getElementById('stepPin').innerText = data.stepPin;
          document.getElementById('directionPin').innerText = data.directionPin;
          document.getElementById('directionHighCountsUp').innerText = data.directionHighCountsUp;
          document.getElementById('enablePinHighActive').innerText = data.enablePinHighActive;
          document.getElementById('enablePinLowActive').innerText = data.enablePinLowActive;
          document.getElementById('currentPosition').innerText = data.currentPosition;
          document.getElementById('isRunning').innerText = data.isRunning;
          document.getElementById('maxSpeedInHz').innerText = data.maxSpeedInHz;
          document.getElementById('maxSpeedInMilliHz').innerText = data.maxSpeedInMilliHz;
          document.getElementById('speedInUs').innerText = data.speedInUs;
          document.getElementById('speedInTicks').innerText = data.speedInTicks;
          document.getElementById('currentSpeedInUs').innerText = data.currentSpeedInUs;
          document.getElementById('currentSpeedInMilliHz').innerText = data.currentSpeedInMilliHz;
          document.getElementById('acceleration').innerText = data.acceleration;
          document.getElementById('currentAcceleration').innerText = data.currentAcceleration;
          document.getElementById('rampState').innerText = data.rampState;
          document.getElementById('isRampGeneratorActive').innerText = data.isRampGeneratorActive;
          document.getElementById('queueEntries').innerText = data.queueEntries;
          document.getElementById('ticksInQueue').innerText = data.ticksInQueue;
          document.getElementById('positionAfterCommandsCompleted').innerText = data.positionAfterCommandsCompleted;
          document.getElementById('periodInUsAfterCommandsCompleted').innerText = data.periodInUsAfterCommandsCompleted;
          document.getElementById('periodInTicksAfterCommandsCompleted').innerText = data.periodInTicksAfterCommandsCompleted;
        } catch (err) { showAlert('Error updating stepper info'); }
      }
      setInterval(updateStepperInfo, 1000);

      // Form handlers for dashboard actions remain unchanged
      document.getElementById('spindleForm').addEventListener('submit', async (e) => {
        e.preventDefault();
        const formData = new FormData(e.target);
        const rpm = formData.get('rpm');
        const direction = formData.get('direction');
        try {
          const response = await fetch('/api/spindle', {
            method: 'POST',
            body: new URLSearchParams({ rpm, direction })
          });
          const result = await response.json();
          if(result.error) { showAlert(result.error); }
        } catch (err) { showAlert('Error sending spindle command'); }
      });
      document.getElementById('moveToForm').addEventListener('submit', async (e) => {
        e.preventDefault();
        const formData = new FormData(e.target);
        const position = formData.get('position');
        try {
          const response = await fetch('/api/motion/moveTo', {
            method: 'POST',
            body: new URLSearchParams({ position })
          });
          const result = await response.json();
          if(result.error) { showAlert(result.error); }
        } catch (err) { showAlert('Error sending moveTo command'); }
      });
      document.getElementById('moveForm').addEventListener('submit', async (e) => {
        e.preventDefault();
        const formData = new FormData(e.target);
        const steps = formData.get('steps');
        try {
          const response = await fetch('/api/motion/move', {
            method: 'POST',
            body: new URLSearchParams({ steps })
          });
          const result = await response.json();
          if(result.error) { showAlert(result.error); }
        } catch (err) { showAlert('Error sending move command'); }
      });
      document.getElementById('stopMotion').addEventListener('click', async () => {
        try {
          const response = await fetch('/api/motion/stop', { method: 'POST' });
          const result = await response.json();
          if(result.error) { showAlert(result.error); }
        } catch (err) { showAlert('Error sending stop command'); }
      });
    </script>
  </body>
</html>
)rawliteral";

// HTML for the WiFi Configuration page (used in AP mode or when requested)
const char wifi_config_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>WiFi Setup</title>
    <!-- Bootstrap Dark Theme -->
    <link href="https://cdn.jsdelivr.net/npm/bootswatch@5.2.3/dist/darkly/bootstrap.min.css" rel="stylesheet">
  </head>
  <body>
    <div class="container mt-4">
      <h3>WiFi Configuration</h3>
      <form id="wifiForm">
        <div class="mb-3">
          <label for="ssid" class="form-label">SSID</label>
          <input type="text" class="form-control" id="ssid" name="ssid" required>
        </div>
        <div class="mb-3">
          <label for="password" class="form-label">Password</label>
          <input type="password" class="form-control" id="password" name="password" required>
        </div>
        <button type="submit" class="btn btn-primary">Save</button>
      </form>
      <div id="message" class="mt-3"></div>
    </div>
    <script>
      document.getElementById('wifiForm').addEventListener('submit', async function(e) {
        e.preventDefault();
        const formData = new FormData(this);
        const ssid = formData.get('ssid');
        const password = formData.get('password');
        try {
          const response = await fetch('/api/wifi/save', {
            method: 'POST',
            body: new URLSearchParams({ ssid, password })
          });
          const result = await response.json();
          document.getElementById('message').innerText = result.message;
          if(result.status === "ok") { setTimeout(() => location.reload(), 3000); }
        } catch (err) {
          document.getElementById('message').innerText = "Error saving WiFi configuration";
        }
      });
    </script>
  </body>
</html>
)rawliteral";

//---------------------------------------------------------------------
// Constructor: initialize AsyncWebServer on port 80 and set apMode to false
//---------------------------------------------------------------------
StepperApi::StepperApi(StepperController* ctrl)
  : server(80), controller(ctrl), apMode(false) {
}

//---------------------------------------------------------------------
// Begin WiFi connection and start the web server, with AP and MDNS fallback
//---------------------------------------------------------------------
void StepperApi::begin(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    unsigned long startAttemptTime = millis();
    const unsigned long connectTimeout = 10000; // 10 seconds timeout
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < connectTimeout) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected. IP address:");
        Serial.println(WiFi.localIP());
        apMode = false;
    } else {
        Serial.println("\nFailed to connect to WiFi. Starting AP mode.");
        apMode = true;
        // Start AP mode using MAC address as SSID (no colons)
        String apSSID = WiFi.macAddress();
        apSSID.replace(":", "");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(apSSID.c_str());
        Serial.print("AP Mode started. SSID: ");
        Serial.println(apSSID);
    }
    // Start MDNS with domain as MAC address without colons + ".local"
    String mdnsName = WiFi.macAddress();
    mdnsName.replace(":", "");
    mdnsName += ".local";
    if (!MDNS.begin(mdnsName.c_str())) {
        Serial.println("Error setting up MDNS responder!");
    } else {
        Serial.print("mDNS responder started: ");
        Serial.println(mdnsName);
    }
    setupRoutes();
    server.begin();
    Serial.println("HTTP server started");
}

//---------------------------------------------------------------------
// Setup REST API endpoints and routes
//---------------------------------------------------------------------
void StepperApi::setupRoutes() {
    // Serve root: if in AP mode, show WiFi config page; else, show dashboard.
    server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (apMode) {
            request->send_P(200, "text/html", wifi_config_html);
        } else {
            request->send_P(200, "text/html", index_html);
        }
    });
    
    // Always serve the WiFi configuration page at /wifi
    server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", wifi_config_html);
    });
    
    // Endpoint to save WiFi credentials
    server.on("/api/wifi/save", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
            String newSSID = request->getParam("ssid", true)->value();
            String newPassword = request->getParam("password", true)->value();
            // Here you would typically save the credentials to persistent storage.
            Serial.println("Received new WiFi credentials:");
            Serial.println("SSID: " + newSSID);
            Serial.println("Password: " + newPassword);
            request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"WiFi configuration saved. Restarting...\"}");
            delay(1000);
            ESP.restart();
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing SSID or password\"}");
        }
    });
    
    // Status endpoint
    server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        String json = "{";
        json += "\"mode\":\"" + controller->getMode() + "\",";
        json += "\"spindleInfo\":\"" + controller->getSpindleInfo() + "\",";
        json += "\"rpm\":" + String(controller->getCurrentRPM()) + ",";
        json += "\"position\":" + String(controller->getCurrentPosition());
        json += "}";
        request->send(200, "application/json", json);
    });
    
    // Set spindle speed endpoint
    server.on("/api/spindle", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("rpm", true) && request->hasParam("direction", true)) {
            int rpm = request->getParam("rpm", true)->value().toInt();
            String direction = request->getParam("direction", true)->value();
            bool cw = (direction == "cw");
            controller->webSetSpindleSpeed(rpm, cw);
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing parameters\"}");
        }
    });
    
    // Motion moveTo endpoint (absolute position)
    server.on("/api/motion/moveTo", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("position", true)) {
            int position = request->getParam("position", true)->value().toInt();
            bool blocking = false;
            if (request->hasParam("blocking", true)) {
                blocking = (request->getParam("blocking", true)->value() == "true");
            }
            controller->webMotionMoveTo(position, blocking);
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing parameter: position\"}");
        }
    });
    
    // Motion move endpoint (relative steps)
    server.on("/api/motion/move", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("steps", true)) {
            int steps = request->getParam("steps", true)->value().toInt();
            bool blocking = false;
            if (request->hasParam("blocking", true)) {
                blocking = (request->getParam("blocking", true)->value() == "true");
            }
            controller->webMotionMove(steps, blocking);
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing parameter: steps\"}");
        }
    });
    
    // Motion stop endpoint
    server.on("/api/motion/stop", HTTP_POST, [this](AsyncWebServerRequest *request) {
        controller->webMotionStop();
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    
    // Load settings endpoint
    server.on("/api/settings/load", HTTP_GET, [this](AsyncWebServerRequest *request) {
        controller->loadSettings();
        String json = "{";
        json += "\"stepsPerRev\":" + String(controller->stepsPerRevolution) + ",";
        json += "\"accel\":" + String(controller->acceleration) + ",";
        json += "\"linAcc\":" + String(controller->linearAcceleration) + ",";
        json += "\"jump\":" + String(controller->jumpStart) + ",";
        json += "\"autoEn\":" + String(controller->autoEnable ? "true" : "false");
        json += "}";
        request->send(200, "application/json", json);
    });
    
    // Save settings endpoint
    server.on("/api/settings/save", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("stepsPerRev", true) &&
            request->hasParam("accel", true) &&
            request->hasParam("linAcc", true) &&
            request->hasParam("jump", true) &&
            request->hasParam("autoEn", true)) {
            controller->stepsPerRevolution = request->getParam("stepsPerRev", true)->value().toInt();
            controller->acceleration = request->getParam("accel", true)->value().toInt();
            controller->linearAcceleration = request->getParam("linAcc", true)->value().toInt();
            controller->jumpStart = request->getParam("jump", true)->value().toInt();
            controller->autoEnable = (request->getParam("autoEn", true)->value() == "true");
            controller->saveSettings();
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing parameters\"}");
        }
    });
    
    // Detailed stepper info endpoint
    server.on("/api/stepper/info", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (controller->stepper == nullptr) {
            request->send(500, "application/json", "{\"error\":\"Stepper not initialized\"}");
            return;
        }
        String json = "{";
        json += "\"stepPin\":" + String(controller->stepper->getStepPin()) + ",";
        json += "\"directionPin\":" + String(controller->stepper->getDirectionPin()) + ",";
        json += "\"directionHighCountsUp\":" + String(controller->stepper->directionPinHighCountsUp() ? "true" : "false") + ",";
        json += "\"enablePinHighActive\":" + String(controller->stepper->getEnablePinHighActive()) + ",";
        json += "\"enablePinLowActive\":" + String(controller->stepper->getEnablePinLowActive()) + ",";
        json += "\"currentPosition\":" + String(controller->stepper->getCurrentPosition()) + ",";
        json += "\"isRunning\":" + String(controller->stepper->isRunning() ? "true" : "false") + ",";
        json += "\"maxSpeedInUs\":" + String(controller->stepper->getMaxSpeedInUs()) + ",";
        json += "\"maxSpeedInTicks\":" + String(controller->stepper->getMaxSpeedInTicks()) + ",";
        json += "\"maxSpeedInHz\":" + String(controller->stepper->getMaxSpeedInHz()) + ",";
        json += "\"maxSpeedInMilliHz\":" + String(controller->stepper->getMaxSpeedInMilliHz()) + ",";
        json += "\"speedInUs\":" + String(controller->stepper->getSpeedInUs()) + ",";
        json += "\"speedInTicks\":" + String(controller->stepper->getSpeedInTicks()) + ",";
        json += "\"speedInMilliHz\":" + String(controller->stepper->getSpeedInMilliHz()) + ",";
        json += "\"currentSpeedInUs\":" + String(controller->stepper->getCurrentSpeedInUs(true)) + ",";
        json += "\"currentSpeedInMilliHz\":" + String(controller->stepper->getCurrentSpeedInMilliHz(true)) + ",";
        json += "\"acceleration\":" + String(controller->stepper->getAcceleration()) + ",";
        json += "\"currentAcceleration\":" + String(controller->stepper->getCurrentAcceleration()) + ",";
        json += "\"rampState\":" + String(controller->stepper->rampState()) + ",";
        json += "\"isRampGeneratorActive\":" + String(controller->stepper->isRampGeneratorActive() ? "true" : "false") + ",";
        json += "\"queueEntries\":" + String(controller->stepper->queueEntries()) + ",";
        json += "\"ticksInQueue\":" + String(controller->stepper->ticksInQueue()) + ",";
        json += "\"positionAfterCommandsCompleted\":" + String(controller->stepper->getPositionAfterCommandsCompleted()) + ",";
        json += "\"periodInUsAfterCommandsCompleted\":" + String(controller->stepper->getPeriodInUsAfterCommandsCompleted()) + ",";
        json += "\"periodInTicksAfterCommandsCompleted\":" + String(controller->stepper->getPeriodInTicksAfterCommandsCompleted());
        json += "}";
        request->send(200, "application/json", json);
    });
}
