
# StepperAPI

StepperAPI is a web-based API and dashboard for controlling and monitoring a stepper motor system using the FastAccelStepper library. The API provides endpoints to control spindle speed, execute motion commands, load/save settings, and retrieve detailed stepper status information. It also includes WiFi configuration support with an AP fallback and mDNS discovery.

## Overview

The StepperAPI class integrates with a [StepperController](#steppercontroller) to:
- Serve a responsive, dark-themed web dashboard (using Bootstrap and Chart.js) that displays:
  - Basic status (mode, spindle info, RPM, position, etc.)
  - A live RPM chart
  - Detailed stepper information (pins, speeds, acceleration, queue status, etc.)
- Provide REST API endpoints for:
  - Spindle and motion control commands
  - Loading and saving settings
  - Retrieving detailed stepper information
- Offer a dedicated WiFi configuration page when the board cannot connect to a network or when explicitly requested.

On startup, the board attempts to connect to WiFi using provided credentials. If the connection fails within a timeout period (10 seconds), the board falls back to AP mode using its MAC address (without colons) as the SSID. In both modes, mDNS is started so the board is accessible via `<MAC_ADDRESS_WITHOUT_COLONS>.local`.

## Prerequisites

- ESP32/ESP8266 board (e.g., Seeed Xiao ESP32C3)
- Arduino framework
- Libraries:
  - [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
  - [FastAccelStepper](https://github.com/gin66/FastAccelStepper)
  - [ESPmDNS](https://github.com/espressif/arduino-esp32/tree/master/libraries/ESPmDNS)

## Setup

1. **Hardware Setup:**  
   Wire your stepper motor, spindle, and required sensors according to your design and the [StepperController](#steppercontroller) documentation.

2. **Software Setup:**  
   - Configure your `platformio.ini` or Arduino IDE project with the required libraries.
   - Update the WiFi credentials in the main sketch.
   - Compile and upload the sketch to your board.

3. **Running the Application:**  
   - On startup, the board attempts to connect to your WiFi network.  
   - If successful, the dashboard will be available via the board’s IP address or via mDNS at `<MAC_ADDRESS_WITHOUT_COLONS>.local`.  
   - If the connection fails, the board starts as an access point (AP mode) with the SSID set to its MAC address (without colons). In AP mode, the root page displays the WiFi configuration interface.

## Endpoints

### Root Endpoint (`/`)
- **GET `/`**  
  **Behavior:**  
  - In **Station Mode** (WiFi connected), serves the main dashboard page.
  - In **AP Mode** (no WiFi connection), serves the WiFi configuration page.
- **Response:** HTML page.

### WiFi Setup Endpoints
- **GET `/wifi`**  
  **Description:** Always serves the WiFi configuration page regardless of mode.  
  **Response:** HTML page for entering new WiFi credentials.
  
- **POST `/api/wifi/save`**  
  **Description:** Accepts new WiFi credentials and restarts the board.  
  **Parameters:**  
  - `ssid` (string): The SSID of the new WiFi network.
  - `password` (string): The new WiFi network password.
  
  **Response:**  
  - **Success:**  
    ```json
    {
      "status": "ok",
      "message": "WiFi configuration saved. Restarting..."
    }
    ```  
  - **Error:**  
    ```json
    { "error": "Missing SSID or password" }
    ```

### Stepper Status and Control Endpoints

- **GET `/api/status`**  
  **Description:** Retrieves basic status information.  
  **Response JSON:**  
  ```json
  {
    "mode": "Motion",             // or "Spindle"
    "spindleInfo": "RPM: 123",
    "rpm": 123,
    "position": 456
  }


- **POST `/api/spindle`**  
  **Description:** Sets the spindle speed and direction (for spindle mode).  
  **Parameters:**  
  - `rpm` (integer): Target RPM.
  - `direction` (string): "cw" for clockwise or "ccw" for counterclockwise.
  
  **Response JSON:**  
  ```json
  { "status": "ok" }
  ```
  **Error Example:**  
  ```json
  { "error": "Missing parameters" }
  ```

- **POST `/api/motion/moveTo`**  
  **Description:** Moves the stepper to an absolute position (motion mode).  
  **Parameters:**  
  - `position` (integer): Absolute position target.
  - `blocking` (optional, string): "true" or "false" (default is false).
  
  **Response JSON:**  
  ```json
  { "status": "ok" }
  ```
  **Error Example:**  
  ```json
  { "error": "Missing parameter: position" }
  ```

- **POST `/api/motion/move`**  
  **Description:** Moves the stepper by a relative number of steps (motion mode).  
  **Parameters:**  
  - `steps` (integer): Number of steps to move.
  - `blocking` (optional, string): "true" or "false" (default is false).
  
  **Response JSON:**  
  ```json
  { "status": "ok" }
  ```
  **Error Example:**  
  ```json
  { "error": "Missing parameter: steps" }
  ```

- **POST `/api/motion/stop`**  
  **Description:** Stops any ongoing motion commands.  
  **Response JSON:**  
  ```json
  { "status": "ok" }
  ```

- **GET `/api/settings/load`**  
  **Description:** Loads and returns the stepper configuration settings.  
  **Response JSON:**  
  ```json
  {
    "stepsPerRev": 1600,
    "accel": 1000,
    "linAcc": 0,
    "jump": 0,
    "autoEn": "true"
  }
  ```

- **POST `/api/settings/save`**  
  **Description:** Saves new stepper configuration settings.  
  **Parameters:**  
  - `stepsPerRev` (integer): Steps per revolution.
  - `accel` (integer): Acceleration.
  - `linAcc` (integer): Linear acceleration steps.
  - `jump` (integer): Jump start steps.
  - `autoEn` (string): "true" or "false" for auto-enable.
  
  **Response JSON:**  
  ```json
  { "status": "ok" }
  ```
  **Error Example:**  
  ```json
  { "error": "Missing parameters" }
  ```

- **GET `/api/stepper/info`**  
  **Description:** Retrieves detailed stepper information from the FastAccelStepper API.  
  **Response JSON:**  
  ```json
  {
    "stepPin": 4,
    "directionPin": 6,
    "directionHighCountsUp": "true",
    "enablePinHighActive": 5,
    "enablePinLowActive": 0,
    "currentPosition": 1234,
    "isRunning": "true",
    "maxSpeedInUs": 200,
    "maxSpeedInTicks": 100,
    "maxSpeedInHz": 500,
    "maxSpeedInMilliHz": 500000,
    "speedInUs": 150,
    "speedInTicks": 75,
    "speedInMilliHz": 450000,
    "currentSpeedInUs": 145,
    "currentSpeedInMilliHz": 440000,
    "acceleration": 1000,
    "currentAcceleration": 900,
    "rampState": 2,
    "isRampGeneratorActive": "true",
    "queueEntries": 3,
    "ticksInQueue": 4000,
    "positionAfterCommandsCompleted": 1300,
    "periodInUsAfterCommandsCompleted": 160,
    "periodInTicksAfterCommandsCompleted": 80
  }
  ```
  *Note:* Actual values will depend on the stepper's current state and settings.

## Usage

1. **Dashboard Access:**  
   Once the board connects to WiFi (or starts in AP mode), access the web dashboard via:
   - Direct IP (in station mode)
   - mDNS name: `<MAC_ADDRESS_WITHOUT_COLONS>.local`  
   The dashboard provides controls for spindle and motion commands, status monitoring, and detailed stepper info.

2. **WiFi Reconfiguration:**  
   Click the "WiFi Setup" link in the dashboard or access `/wifi` directly to change network credentials. New credentials are saved via the `/api/wifi/save` endpoint, and the board will restart.

3. **API Integration:**  
   Developers can integrate or extend the REST endpoints in their own applications. All endpoints return JSON responses with status or error messages.

## Contributing

Feel free to fork this repository and contribute improvements or bug fixes. Pull requests are welcome!

## License

This project is provided under the MIT License.
```
