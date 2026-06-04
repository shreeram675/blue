# Blueprint Navigation System

Blueprint Navigation System is a Flask-based indoor navigation prototype for a smart wheelchair robot. It takes a labeled floor-plan image, detects room names with OCR, converts the plan into a robot-safe occupancy grid, plans a route with A*, and publishes movement commands to an ESP32 wheelchair controller through Firebase Realtime Database.

## Features

- Upload JPG/PNG blueprint images through a web dashboard.
- Detect room labels, dimensions, and orientation markers with EasyOCR.
- Convert floor plans into raw and inflated occupancy grids using OpenCV.
- Plan collision-aware routes with 8-directional A* and wall-clearance weighting.
- Convert paths into robot commands such as `F30`, `R90`, and `L45`.
- Send command queues to Firebase for ESP32 execution.
- Track command completion and robot pose in the UI.
- Support manual movement controls and browser voice destination commands.
- Include ESP32 firmware variants with and without ultrasonic obstacle detection.
- Generate research/report figures from local blueprint test cases.

## System Architecture

```text
Blueprint image
  -> EasyOCR room and dimension parsing
  -> OpenCV wall mask extraction
  -> Raw occupancy grid
  -> Safety-margin inflated grid
  -> A* path planner
  -> Motion command generation
  -> Firebase command queue
  -> ESP32 stepper-motor wheelchair
```

## Project Structure

```text
.
|-- app.py                              # Flask API and web server
|-- blueprint_parser.py                 # OCR room, dimension, orientation parser
|-- grid_generator.py                   # Wall extraction and occupancy grid generation
|-- astar.py                            # A* planner, smoothing, command conversion
|-- firebase_queue.py                   # Firebase Realtime Database queue publisher
|-- esp_log_receiver.py                 # UDP log receiver for ESP32 serial output
|-- templates/index.html                # Browser dashboard
|-- wheelchair_esp32/                   # Firmware with ultrasonic obstacle detection
|-- wheelchair_esp32_no_sensor/         # Firmware without ultrasonic sensor
|-- scripts/generate_paper_figures.py   # Generates paper/report figures
|-- paper_figures/                      # Generated metrics and visualization outputs
|-- test_blueprint*.jpg                 # Sample blueprint inputs
|-- grid_raw.jpg                        # Example raw grid overlay
|-- grid_inflated.jpg                   # Example inflated grid overlay
|-- path_result.jpg                     # Example path overlay
|-- GUIDE.md                            # Detailed hardware and run guide
|-- TODO.md                             # Working notes
`-- requirements.txt                    # Python dependencies
```

## Requirements

### Software

- Python 3.10+
- pip
- Chrome or Edge for browser voice commands
- Arduino IDE with ESP32 board support for firmware uploads

### Python Dependencies

Install the pinned dependencies:

```powershell
python -m pip install -r requirements.txt
```

EasyOCR installs PyTorch-related dependencies, so the first install can take a while.

### Hardware

- ESP32 DevKit
- Two 28BYJ-48 stepper motors
- Two ULN2003 motor drivers
- Optional HC-SR04 ultrasonic sensor
- External 5V supply for motors
- Shared ground between ESP32 and motor power supply

## Firebase Configuration

The server works without Firebase credentials, but hardware command publishing and pose sync require Firebase Realtime Database.

Create a `.env` or `.env.local` file in the project root:

```text
FIREBASE_CREDENTIALS=path-to-service-account.json
FIREBASE_DATABASE_URL=https://your-project-default-rtdb.region.firebasedatabase.app
```

Service-account JSON files and `.env` files are ignored by Git. Do not commit Firebase credentials or WiFi secrets.

## Run the Web App

Start the Flask server:

```powershell
python app.py
```

Open:

```text
http://localhost:5000
```

Basic workflow:

1. Upload a blueprint image.
2. Wait for room detection and grid generation.
3. Set the robot pose by clicking the map, or use the room-based pose API.
4. Select a detected room or use voice navigation.
5. Watch the route, command queue, Firebase status, and robot pose update.

## API Endpoints

| Method | Endpoint | Description |
| --- | --- | --- |
| `GET` | `/` | Web dashboard |
| `POST` | `/upload` | Upload and parse a blueprint |
| `GET` | `/is-free` | Check whether a grid cell is free |
| `POST` | `/set-pose` | Set robot pose by grid row/column/heading |
| `POST` | `/set-pose-room` | Set robot pose from a detected room name |
| `POST` | `/navigate` | Plan a path to a room and publish commands |
| `GET` | `/status` | Read current server, robot, grid, and queue state |
| `GET` | `/get-command` | Poll the next local command |
| `POST` | `/command-done` | Mark a command complete and advance pose |
| `GET` | `/grid.png` | Render current grid overlay |
| `GET` | `/path.png` | Render current path overlay |
| `GET` | `/debug_grid.png` | Render simplified grid debug image |
| `GET` | `/free-mask.png` | Render inflated free-space mask |
| `GET` | `/raw-mask.png` | Render raw free-space mask |
| `GET` | `/firebase-debug` | Inspect Firebase queue/status data |
| `POST` | `/manual-move` | Queue or apply manual movement commands |

## ESP32 Firmware

Two firmware builds are included:

- `wheelchair_esp32/wheelchair_esp32.ino`: uses HC-SR04 obstacle detection.
- `wheelchair_esp32_no_sensor/wheelchair_esp32_no_sensor.ino`: simpler test build without obstacle sensing and with UDP logging.

Before flashing:

1. Update WiFi SSID/password in the `.ino` file.
2. Update Firebase API key and database URL if needed.
3. Install Arduino libraries:
   - `Firebase ESP Client`
   - `AccelStepper`
4. Select `ESP32 Dev Module` and the correct COM port in Arduino IDE.

Default motor pins:

| Motor | IN1 | IN2 | IN3 | IN4 |
| --- | --- | --- | --- | --- |
| Left | GPIO 13 | GPIO 12 | GPIO 14 | GPIO 27 |
| Right | GPIO 26 | GPIO 23 | GPIO 25 | GPIO 32 |

Run the wireless log receiver for the no-sensor firmware:

```powershell
python esp_log_receiver.py
```

## Research Figures

The project includes a script for generating analysis figures and computed metrics:

```powershell
python scripts/generate_paper_figures.py
```

Outputs are written to `paper_figures/`, including architecture, processing stages, occupancy grid quality, A* performance, path length comparison, Firebase command timeline, and a robot accuracy template.

## Notes

- `GUIDE.md` contains detailed wiring, calibration, Firebase structure, and troubleshooting steps.
- The generated route respects the inflated grid when possible. If a route cannot be found, the backend can fall back to raw or doorway-opened grid planning and reports the fallback in the response.
- Firebase is optional for local UI experimentation, but required for ESP32 queue execution.
