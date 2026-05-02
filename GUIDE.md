# Blueprint-Nav Project — Run Guide

## What This Project Does
Upload a hospital blueprint image → AI detects rooms → Set robot start position →
Navigate to any room by clicking or voice → ESP32 wheelchair robot executes the path via Firebase.

---

## Requirements

### PC
- Python 3.10+
- Libraries: `pip install flask firebase-admin opencv-python numpy python-dotenv torch torchvision`
- Chrome or Edge browser (Voice commands require these)

### ESP32
- Arduino IDE with ESP32 board support installed
- Libraries (install via Arduino Library Manager):
  - `Firebase ESP Client` by Mobizt
  - `AccelStepper` by Mike McCauley

### Hardware
- ESP32 DevKit
- 2× 28BYJ-48 stepper motors + ULN2003 drivers
- HC-SR04 ultrasonic sensor (optional)
- Same WiFi network for PC and ESP32

---

## Project Files

```
Blueprint-nav/
├── app.py                          ← Flask server (main backend)
├── .env                            ← Firebase credentials
├── firebase_queue.py               ← Firebase publisher
├── astar.py                        ← Pathfinding
├── grid_generator.py               ← Blueprint → grid
├── templates/index.html            ← Web UI
├── esp_log_receiver.py             ← Wireless ESP32 log viewer
│
├── wheelchair_esp32/
│   └── wheelchair_esp32.ino        ← ESP32 firmware WITH ultrasonic
│
└── wheelchair_esp32_no_sensor/
    └── wheelchair_esp32_no_sensor.ino  ← ESP32 firmware WITHOUT ultrasonic
```

---

## Step 1 — Firebase Setup

1. Go to [Firebase Console](https://console.firebase.google.com)
2. Open project `id-project-b4d10`
3. Go to **Realtime Database** → Rules → set:
   ```json
   { "rules": { ".read": true, ".write": true } }
   ```
4. Make sure `.env` file exists with:
   ```
   FIREBASE_CREDENTIALS=id-project-b4d10-firebase-adminsdk-fbsvc-7734f12287.json
   FIREBASE_DATABASE_URL=https://id-project-b4d10-default-rtdb.asia-southeast1.firebasedatabase.app
   ```
5. Make sure the `.json` credentials file is in the project folder

---

## Step 2 — Start the Flask Server

Open PowerShell in the project folder:

```powershell
cd "C:\Users\shreeram\OneDrive\Desktop\rmsblue\Blueprint-nav"
python app.py
```

Server starts at `http://localhost:5000`

**To stop:** `Ctrl+C` in the terminal, or run `taskkill /F /IM python.exe`

**To restart:**
```powershell
taskkill /F /IM python.exe; Start-Sleep -Seconds 1; python app.py
```

---

## Step 3 — Flash ESP32

### Without ultrasonic sensor (recommended for testing):
1. Open `wheelchair_esp32_no_sensor/wheelchair_esp32_no_sensor.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module**
3. Select the correct COM port
4. Click **Upload**

### With ultrasonic sensor:
1. Open `wheelchair_esp32/wheelchair_esp32.ino`
2. Wire HC-SR04: TRIG → GPIO18, ECHO → GPIO19
3. Upload same way

### Motor wiring:
| Motor | IN1 | IN2 | IN3 | IN4 |
|-------|-----|-----|-----|-----|
| Left  | 13  | 12  | 14  | 27  |
| Right | 26  | 23  | 25  | 32  |

---

## Step 4 — Use the Web UI

1. Open `http://localhost:5000` in Chrome or Edge
2. **Upload Blueprint** — click Upload, select your floor plan image (JPG/PNG)
3. Wait for rooms to be detected (shown in the panel)
4. **Set Robot Position** — click "Set Position" then click on the map where the robot is
5. **Navigate** — click a room name from the list, or use voice command
6. Watch the command queue update as the robot executes

### Voice Commands
Click the microphone button and say:
- "Go to lab"
- "Take me to nurse station"
- "Navigate to ICU"
- "I want to go to pharmacy"

---

## Step 5 — View ESP32 Logs Wirelessly (no USB cable)

Make sure PC and ESP32 are on the same WiFi (`Shree`), then run:

```powershell
python esp_log_receiver.py
```

You will see all ESP32 Serial output live in the terminal.

---

## Step 6 — Manual Robot Control

In the web UI under **Manual Controls**:
- Arrow buttons → Forward / Backward / Left turn / Right turn (10 cm per press)
- STOP button → stops immediately and clears queue

---

## ESP32 — Detailed Wiring

### ULN2003 Motor Driver Connections

Each 28BYJ-48 motor comes with a ULN2003 driver board. Wire them as follows:

**Left Motor (ULN2003 board #1):**
```
ULN2003 IN1  →  ESP32 GPIO 13
ULN2003 IN2  →  ESP32 GPIO 12
ULN2003 IN3  →  ESP32 GPIO 14
ULN2003 IN4  →  ESP32 GPIO 27
ULN2003 VCC  →  5V (external supply or ESP32 5V pin)
ULN2003 GND  →  GND (shared with ESP32 GND)
Motor plug   →  the 5-pin white connector on the board
```

**Right Motor (ULN2003 board #2):**
```
ULN2003 IN1  →  ESP32 GPIO 26
ULN2003 IN2  →  ESP32 GPIO 23
ULN2003 IN3  →  ESP32 GPIO 25
ULN2003 IN4  →  ESP32 GPIO 32
ULN2003 VCC  →  5V
ULN2003 GND  →  GND
Motor plug   →  the 5-pin white connector on the board
```

> **Important:** Always share GND between ESP32 and the motor power supply. Floating GND causes erratic behavior.

### HC-SR04 Ultrasonic Sensor (optional)

The HC-SR04 runs on 5V but its ECHO pin outputs 5V, which can damage the ESP32's 3.3V GPIO.
Use a voltage divider on the ECHO line:

```
HC-SR04 VCC   →  5V
HC-SR04 GND   →  GND
HC-SR04 TRIG  →  ESP32 GPIO 18  (3.3V signal is enough to trigger)
HC-SR04 ECHO  →  1kΩ resistor → ESP32 GPIO 19
                              → 2kΩ resistor → GND
```

The 1kΩ + 2kΩ divider drops 5V → ~3.3V on the ECHO line.

Configure ECHO pin as plain `INPUT` (not `INPUT_PULLDOWN`):
```cpp
pinMode(ECHO_PIN, INPUT);   // correct
// NOT: pinMode(ECHO_PIN, INPUT_PULLDOWN);  — causes 0.6cm false readings
```

### Power Supply

The 28BYJ-48 draws ~160–250 mA per motor at 5V. Two motors = up to 500 mA.
- **Do not** power both motors from the ESP32's onboard 5V pin — it is limited to ~500 mA total and shared with the ESP32 itself.
- Use a dedicated 5V 1–2A power supply (phone charger with wires, or a 3× AA battery pack).
- Connect its GND to the ESP32 GND.

---

## ESP32 — Firebase Data Structure

The ESP32 reads commands from and writes status to Firebase Realtime Database.

### Queue path: `/robots/wheelchair_01/queue/`
```json
{
  "cmd_1": { "command": "F30",     "status": "DONE",        "sequence": 1 },
  "cmd_2": { "command": "R90",     "status": "IN_PROGRESS", "sequence": 2 },
  "cmd_3": { "command": "F45",     "status": "PENDING",     "sequence": 3 }
}
```
- `command` — motion code: `F`=forward cm, `B`=backward cm, `R`=right turn degrees, `L`=left turn degrees
- `status` — lifecycle: `PENDING` → `IN_PROGRESS` → `DONE` (or `BLOCKED` if obstacle detected)

### Status path: `/robots/wheelchair_01/status/`
```json
{
  "queue_status":   "IN_PROGRESS",
  "obstacle":       "CLEAR",
  "pose": { "heading": 90 },
  "last_heartbeat": 12345
}
```
- `queue_status` — mirrors the current command's status
- `obstacle` — `CLEAR` or `FRONT` (ultrasonic version only)
- `pose/heading` — robot's current compass heading in degrees (0=North, 90=East, 180=South, 270=West)
- `last_heartbeat` — `millis()/1000` at time of last Firebase write; use this to confirm ESP32 is alive

The Flask server polls `/robots/wheelchair_01/status/` every 1.5 seconds to sync the UI pose display.

---

## ESP32 — State Machine

The firmware operates as a simple state machine driven by Firebase:

```
IDLE ──(new PENDING cmd found)──► IN_PROGRESS ──(time elapsed)──► DONE ──► IDLE
                                        │
                                  (obstacle detected, F cmd only)
                                        ▼
                                    BLOCKED ──(obstacle clears)──► IN_PROGRESS (resumed)
                                        │
                                  (obstacle never clears)
                                        └── stays BLOCKED until obstacle removed
```

Key points:
- Only one command executes at a time — next command is not fetched until current is DONE
- Turns (R/L) are never blocked by the ultrasonic sensor — only forward (F) moves check for obstacles
- After an obstacle clears, the motor resumes for the **remaining** milliseconds (time already elapsed is subtracted)
- The ESP32 polls Firebase every **1200 ms** when idle; during motion it runs the motor loop every ~1 ms

---

## ESP32 — Serial Monitor / Wireless Log Output

Open Arduino IDE Serial Monitor at **115200 baud**, or run `python esp_log_receiver.py` for wireless.

### Normal startup output:
```
WiFi connected: 192.168.1.45
Firebase OK
wheelchair_01 ready
   MS_PER_CM=204.57  MS_PER_DEG=8.96
   Sensor: OFF  threshold=20 cm  confirm=2 readings
```

### Normal command execution:
```
▶ cmd_1: F30
   FWD  30.0 cm -> 6137 ms
DONE
▶ cmd_2: R90
   RIGHT 90 deg -> 806 ms
DONE
▶ cmd_3: F45
   FWD  45.0 cm -> 9206 ms
DONE
```

### With ultrasonic sensor (obstacle detected):
```
▶ cmd_1: F50
   FWD  50.0 cm -> 10229 ms
   Sensor: 45.2 cm
   Sensor: 18.3 cm
   ⚠ near 1/2
   Sensor: 16.1 cm
   ⚠ near 2/2
⛔ BLOCKED (16.1 cm)
   Sensor: 15.4 cm
   Sensor: 15.8 cm
   Sensor: 28.7 cm
✅ RESUMED — 6800 ms remaining
DONE
```

### Firebase connection failure:
```
WiFi connected: 192.168.1.45
Firebase FAILED: connection refused
```
→ Check Firebase API key and database URL in the `.ino` file match your project.

---

## ESP32 — How MS_PER_CM and MS_PER_DEG Are Calculated

These constants convert physical distances into motor run times:

```cpp
const float WHEEL_DIAMETER_CM = 4.9;
const float WHEEL_BASE_CM     = 8.0;
const float STEPS_PER_REV     = 4096.0;   // 28BYJ-48 in HALF4WIRE mode
const float RUN_SPEED         = 1300.0;   // steps per second

const float WHEEL_CIRC_CM = PI * WHEEL_DIAMETER_CM;          // circumference
const float MS_PER_CM     = (STEPS_PER_REV / WHEEL_CIRC_CM   // steps needed per cm
                             / RUN_SPEED)                     // ÷ steps/sec = seconds/cm
                            * 1000.0;                         // → ms/cm

const float MS_PER_DEG    = (WHEEL_BASE_CM * PI / 360.0)     // arc length per degree
                            * MS_PER_CM;                      // → ms/degree
```

**Example with defaults:**
- `WHEEL_CIRC_CM` = π × 4.9 = 15.39 cm
- `MS_PER_CM` = (4096 / 15.39 / 1300) × 1000 = **204.6 ms/cm**
- `MS_PER_DEG` = (8.0 × π / 360) × 204.6 = **14.3 ms/deg**

To move 30 cm → motor runs for 30 × 204.6 = **6138 ms**
To turn 90° → motor runs for 90 × 14.3 = **1287 ms**

Adjust `WHEEL_DIAMETER_CM` to fix straight-line accuracy.
Adjust `WHEEL_BASE_CM` to fix turn accuracy.

---

## Tuning Parameters

### ESP32 — Physical calibration (`wheelchair_esp32.ino` lines 42–47)
```cpp
const float WHEEL_DIAMETER_CM = 4.9;   // measure your actual wheel
const float WHEEL_BASE_CM     = 8.0;   // distance between left and right wheels
const float RUN_SPEED         = 1300.0; // lower = slower but more accurate
```

If robot overshoots turns → decrease `WHEEL_BASE_CM`
If robot undershoots distance → increase `WHEEL_DIAMETER_CM`

### ESP32 — Obstacle detection (`wheelchair_esp32.ino` lines 20–22)
```cpp
#define OBSTACLE_THRESHOLD_CM 20.0  // stop distance in cm
#define BLOCK_CONFIRM_COUNT   2     // readings needed to confirm obstacle
```

### Server — Robot size (`index.html` Robot Width field)
Set to actual robot width in cm. This controls how much the path avoids walls.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Server won't start | Check `.env` and credentials JSON file exist |
| No rooms detected | Try a clearer blueprint image with labeled rooms |
| ESP32 not connecting to WiFi | Check SSID/password in `.ino` file |
| Commands not reaching ESP32 | Open `http://localhost:5000/firebase-debug` to check Firebase queue |
| Robot moves wrong direction | Swap IN1/IN2 on one motor driver |
| Turns are inaccurate | Adjust `WHEEL_BASE_CM` in the `.ino` |
| Pose not updating after manual move | Ensure Firebase is enabled (check `/status` endpoint) |
| Ultrasonic reads 0.6 cm constantly | Change ECHO pin from `INPUT_PULLDOWN` to `INPUT` |
| ESP32 loops printing Firebase FAILED | API key or database URL is wrong in `.ino` |
| Motor vibrates but doesn't rotate | Wrong pin order — check IN1/IN3/IN2/IN4 order in AccelStepper constructor |
| Robot drifts left/right on straight moves | One motor spinning faster — lower `RUN_SPEED` or check power supply |
| Obstacle never detected | Verify HC-SR04 is wired to 5V not 3.3V; test with standalone sensor sketch |
| VS Code shows "cannot open WiFiUDP.h" | False positive — ESP32 Arduino core not in VS Code IntelliSense path; compiles fine in Arduino IDE |

---

## API Endpoints (for debugging)

| URL | Description |
|-----|-------------|
| `http://localhost:5000/status` | Current server state, pose, queue |
| `http://localhost:5000/firebase-debug` | Live Firebase queue contents |
| `http://localhost:5000/grid.png` | Current grid overlay image |

---

## Calibration Test Procedure

1. Flash `wheelchair_esp32_no_sensor.ino`
2. Run `python esp_log_receiver.py` on PC
3. In web UI: upload blueprint, set pose, send `F30` (30 cm forward)
4. Measure actual distance travelled
5. Adjust `WHEEL_DIAMETER_CM` proportionally:
   - Moved less than 30 cm → increase `WHEEL_DIAMETER_CM`
   - Moved more than 30 cm → decrease `WHEEL_DIAMETER_CM`
6. Repeat for a 90° turn using `R90`
7. Adjust `WHEEL_BASE_CM` the same way

### Quick formula:
```
new WHEEL_DIAMETER_CM = old × (actual_cm / commanded_cm)
new WHEEL_BASE_CM     = old × (actual_deg / commanded_deg)
```

Example: commanded 30 cm, moved 27 cm → new = 4.9 × (30/27) = **5.44**
