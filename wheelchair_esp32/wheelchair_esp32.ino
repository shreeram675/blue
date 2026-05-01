#include <WiFi.h>
#include <AccelStepper.h>
#include <Firebase_ESP_Client.h>

// ── WIFI ─────────────────────────────────────
#define WIFI_SSID       "Shree"
#define WIFI_PASSWORD   "!!!!!!!!"

// ── FIREBASE ─────────────────────────────────
#define FIREBASE_API_KEY      "AIzaSyAltzaCaE4QI8FWW0m9ZCW8mPdUer3QVCk"
#define FIREBASE_DATABASE_URL "https://id-project-b4d10-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define ROBOT_ID              "wheelchair_01"

// ── ULTRASONIC ───────────────────────────────
#define TRIG_PIN 18
#define ECHO_PIN 19

// Set false to disable obstacle detection entirely (useful for testing without sensor)
#define SENSOR_ENABLED        true
#define OBSTACLE_THRESHOLD_CM 5.0   // cm — increase if false positives persist
#define DIST_INTERVAL_MS      120    // ms between sensor pings (HC-SR04 needs ≥60ms)
#define BLOCK_CONFIRM_COUNT   3      // consecutive close readings required to trigger BLOCKED

// ── MOTOR PINS ───────────────────────────────
#define L_IN1 13
#define L_IN2 12
#define L_IN3 14
#define L_IN4 27

#define R_IN1 26
#define R_IN2 23
#define R_IN3 25
#define R_IN4 32

AccelStepper leftMotor (AccelStepper::HALF4WIRE, L_IN1, L_IN3, L_IN2, L_IN4);
AccelStepper rightMotor(AccelStepper::HALF4WIRE, R_IN1, R_IN3, R_IN2, R_IN4);

const float MAX_SPEED = 2000.0;
const float RUN_SPEED = 1300.0;

// ── PHYSICAL CONSTANTS ───────────────────────
const float WHEEL_DIAMETER_CM = 4.9;
const float WHEEL_BASE_CM     = 8.0;
const float STEPS_PER_REV     = 4096.0;
const float WHEEL_CIRC_CM     = PI * WHEEL_DIAMETER_CM;
const float MS_PER_CM         = (STEPS_PER_REV / WHEEL_CIRC_CM / RUN_SPEED) * 1000.0;
const float MS_PER_DEG        = (WHEEL_BASE_CM * PI / 360.0) * MS_PER_CM;

// ── FIREBASE ─────────────────────────────────
FirebaseData   fbdo;
FirebaseAuth   auth;
FirebaseConfig config;

// ── STATE ────────────────────────────────────
bool   motionActive = false;
bool   blocked      = false;

float         curLeftSpeed  = 0;
float         curRightSpeed = 0;
unsigned long motionStart   = 0;
unsigned long motionTotal   = 0;
unsigned long motionElapsed = 0;

String activeSeqKey  = "";
String activeCommand = "";

// Heading degrees CW from North: 0=N, 90=E, 180=S, 270=W
int headingDeg = 0;

unsigned long lastPollMs = 0;

// ── ULTRASONIC STATE ─────────────────────────
unsigned long lastDistMs  = 0;
float         lastDistCm  = 999.0;
int           nearCount   = 0;   // consecutive close readings
int           clearCount  = 0;   // consecutive clear readings (for resume debounce)

struct NextCmd { String key; int num; };

// ── PATH HELPERS ─────────────────────────────
String queuePath()          { return String("/robots/") + ROBOT_ID + "/queue"; }
String statusPath()         { return String("/robots/") + ROBOT_ID + "/status"; }
String itemPath(String key) { return queuePath() + "/" + key; }

// ── WIFI ─────────────────────────────────────
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println(" connected: " + WiFi.localIP().toString());
}

// ── FIREBASE ─────────────────────────────────
void connectFirebase() {
  config.api_key      = FIREBASE_API_KEY;
  config.database_url = FIREBASE_DATABASE_URL;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  if (!Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase anon auth failed: " +
                   String(config.signer.signupError.message.c_str()));
  }
}

// ── ULTRASONIC ───────────────────────────────
// Called at most every DIST_INTERVAL_MS to avoid starving motor loop.
// HC-SR04 echo line floats HIGH when disconnected → pulseIn returns immediately
// with a huge value, so lastDistCm stays 999 and no false BLOCKED fires.
float sampleDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // 20 ms timeout — shorter than before to reduce blocking time
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 20000);
  float d = dur * 0.034f / 2.0f;
  return (d < 0.5f || d > 400.0f) ? 999.0f : d;
}

// Returns true if a confirmed obstacle is present.
// Only samples once every DIST_INTERVAL_MS; requires BLOCK_CONFIRM_COUNT
// consecutive close readings so a single spurious ping can't trigger BLOCKED.
bool checkObstacle() {
#if !SENSOR_ENABLED
  return false;
#endif
  unsigned long now = millis();
  // Skip sensing for the first 200ms of a move — lets motors reach speed
  // before the 20ms pulseIn blockage can cause step-loss inaccuracy.
  if (now - motionStart < 200) return false;
  if (now - lastDistMs < DIST_INTERVAL_MS) {
    return (nearCount >= BLOCK_CONFIRM_COUNT);
  }
  lastDistMs = now;
  lastDistCm = sampleDistance();

  if (lastDistCm < OBSTACLE_THRESHOLD_CM) {
    nearCount++;
    clearCount = 0;
    Serial.printf("   Sensor: %.1f cm (near %d/%d)\n",
                  lastDistCm, nearCount, BLOCK_CONFIRM_COUNT);
  } else {
    clearCount++;
    nearCount = 0;
  }
  return (nearCount >= BLOCK_CONFIRM_COUNT);
}

// ── HEADING TRACKING ─────────────────────────
void applyTurn(char dir, float degrees) {
  int delta = (int)round(degrees);
  if (dir == 'R') headingDeg = ((headingDeg + delta) % 360 + 360) % 360;
  else            headingDeg = ((headingDeg - delta) % 360 + 360) % 360;
}

// ── FIREBASE STATUS ──────────────────────────
void updateFirebaseStatus(const char* qs, const char* obs) {
  if (!Firebase.ready()) return;
  FirebaseJson j;
  j.set("queue_status",   qs);
  j.set("obstacle",       obs);
  j.set("pose/heading",   headingDeg);
  j.set("last_heartbeat", (int)(millis() / 1000));
  Firebase.RTDB.updateNode(&fbdo, statusPath(), &j);
}

void setItemStatus(String key, const char* status) {
  if (!Firebase.ready()) return;
  FirebaseJson j;
  j.set("status", status);
  Firebase.RTDB.updateNode(&fbdo, itemPath(key), &j);
}

// ── MOTOR CONTROL ────────────────────────────
void stopMotors() {
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  leftMotor.runSpeed();
  rightMotor.runSpeed();
}

void startMotion(float leftSpd, float rightSpd, unsigned long durationMs) {
  curLeftSpeed  = leftSpd;
  curRightSpeed = rightSpd;
  motionTotal   = durationMs;
  motionElapsed = 0;
  motionStart   = millis();
  leftMotor.setSpeed(leftSpd);
  rightMotor.setSpeed(rightSpd);
  motionActive  = true;
  blocked       = false;
  nearCount     = 0;
  clearCount    = 0;
}

// ── COMMAND EXECUTION ────────────────────────
void executeCommand(String seqKey, String cmd) {
  cmd.trim(); cmd.toUpperCase();
  char  type  = cmd.charAt(0);
  float value = cmd.substring(1).toFloat();

  activeSeqKey  = seqKey;
  activeCommand = cmd;

  setItemStatus(seqKey, "IN_PROGRESS");
  updateFirebaseStatus("IN_PROGRESS", "CLEAR");
  Serial.printf("▶ %s: %s\n", seqKey.c_str(), cmd.c_str());

  unsigned long ms;

  if (type == 'F') {
    ms = (unsigned long)(value * MS_PER_CM);
    startMotion(RUN_SPEED, -RUN_SPEED, ms);   // left+, right- = forward
    Serial.printf("   FWD  %.1f cm → %lu ms\n", value, ms);
  }
  else if (type == 'B') {
    ms = (unsigned long)(value * MS_PER_CM);
    startMotion(-RUN_SPEED, RUN_SPEED, ms);
    Serial.printf("   BWD  %.1f cm → %lu ms\n", value, ms);
  }
  else if (type == 'R') {
    ms = (unsigned long)(value * MS_PER_DEG);
    startMotion(RUN_SPEED, RUN_SPEED, ms);    // left fwd, right same = right turn
    Serial.printf("   RIGHT %.0f deg → %lu ms\n", value, ms);
  }
  else if (type == 'L') {
    ms = (unsigned long)(value * MS_PER_DEG);
    startMotion(-RUN_SPEED, -RUN_SPEED, ms);  // left bwd, right same = left turn
    Serial.printf("   LEFT  %.0f deg → %lu ms\n", value, ms);
  }
  else {
    Serial.printf("   Unknown command: %s\n", cmd.c_str());
  }
}

// ── FIND NEXT PENDING ────────────────────────
NextCmd findNext(FirebaseJson* json) {
  size_t len = json->iteratorBegin();
  NextCmd best = {"", 999999};
  for (size_t i = 0; i < len; i++) {
    int type; String key, val;
    json->iteratorGet(i, type, key, val);
    FirebaseJsonData d;
    json->get(d, key + "/status");
    if (!d.success || d.stringValue != "PENDING") continue;
    int num = key.substring(4).toInt();  // "cmd_N" → N
    if (num > 0 && num < best.num) { best.num = num; best.key = key; }
  }
  json->iteratorEnd();
  return best;
}

// ── QUEUE POLLING ────────────────────────────
void pollQueue() {
  if (!Firebase.ready() || motionActive) return;
  if (millis() - lastPollMs < 1200) return;
  lastPollMs = millis();

  if (!Firebase.RTDB.getJSON(&fbdo, queuePath())) return;
  if (fbdo.dataType() != "json") return;

  FirebaseJson* json = fbdo.to<FirebaseJson*>();
  if (!json) return;

  NextCmd next = findNext(json);
  if (next.key == "") return;

  FirebaseJsonData d;
  json->get(d, next.key + "/command");
  if (!d.success) return;

  executeCommand(next.key, d.stringValue);
}

// ── MOTION LOOP ──────────────────────────────
void runMotion() {
  if (!motionActive) return;

  char cmdType = activeCommand.charAt(0);

  // Obstacle check — forward moves only, rate-limited and debounced
  if (cmdType == 'F') {
    bool obstacle = checkObstacle();

    if (obstacle && !blocked) {
      motionElapsed += millis() - motionStart;
      stopMotors();
      blocked = true;
      setItemStatus(activeSeqKey, "BLOCKED");
      updateFirebaseStatus("BLOCKED", "FRONT");
      Serial.printf("⛔ BLOCKED (dist=%.1f cm)\n", lastDistCm);
      return;
    }

    if (blocked) {
      if (!obstacle) {
        unsigned long remaining = motionTotal - motionElapsed;
        Serial.printf("✅ RESUMED — %lu ms remaining\n", remaining);
        startMotion(curLeftSpeed, curRightSpeed, remaining);
        setItemStatus(activeSeqKey, "IN_PROGRESS");
        updateFirebaseStatus("IN_PROGRESS", "CLEAR");
      } else {
        return;  // still blocked
      }
    }
  }

  leftMotor.runSpeed();
  rightMotor.runSpeed();

  if (millis() - motionStart >= motionTotal) {
    stopMotors();
    motionActive = false;

    float val = activeCommand.substring(1).toFloat();
    if (cmdType == 'R') applyTurn('R', val);
    if (cmdType == 'L') applyTurn('L', val);

    setItemStatus(activeSeqKey, "DONE");
    updateFirebaseStatus("DONE", "CLEAR");
    Serial.println("✅ DONE");
  }
}

// ── SETUP ────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLDOWN);  // PULLDOWN prevents floating-pin false triggers

  leftMotor.setMaxSpeed(MAX_SPEED);
  rightMotor.setMaxSpeed(MAX_SPEED);

  connectWiFi();
  connectFirebase();

  Serial.print("Firebase");
  unsigned long t = millis();
  while (!Firebase.ready() && millis() - t < 10000) {
    delay(300); Serial.print(".");
  }
  Serial.println(Firebase.ready() ? " OK" : " FAILED: " + String(fbdo.errorReason().c_str()));

  updateFirebaseStatus("IDLE", "CLEAR");
  Serial.println("wheelchair_01 ready");
  Serial.printf("   MS_PER_CM=%.2f  MS_PER_DEG=%.2f\n", MS_PER_CM, MS_PER_DEG);
  Serial.printf("   Sensor: %s  threshold=%.0f cm  confirm=%d readings\n",
                SENSOR_ENABLED ? "ON" : "OFF", OBSTACLE_THRESHOLD_CM, BLOCK_CONFIRM_COUNT);
}

// ── LOOP ─────────────────────────────────────
void loop() {
  runMotion();          // always first — motor timing is critical
  if (!motionActive) {
    pollQueue();
  }
}
