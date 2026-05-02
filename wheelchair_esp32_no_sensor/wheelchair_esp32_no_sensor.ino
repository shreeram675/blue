#include <WiFi.h>
#include <WiFiUDP.h>
#include <AccelStepper.h>
#include <Firebase_ESP_Client.h>

// ── WIFI ─────────────────────────────────────
#define WIFI_SSID       "Shree"
#define WIFI_PASSWORD   "!!!!!!!!"

// ── FIREBASE ─────────────────────────────────
#define FIREBASE_API_KEY      "AIzaSyAltzaCaE4QI8FWW0m9ZCW8mPdUer3QVCk"
#define FIREBASE_DATABASE_URL "https://id-project-b4d10-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define ROBOT_ID              "wheelchair_01"

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

float         curLeftSpeed  = 0;
float         curRightSpeed = 0;
unsigned long motionStart   = 0;
unsigned long motionTotal   = 0;

String activeSeqKey  = "";
String activeCommand = "";

int headingDeg = 0;

unsigned long lastPollMs = 0;

struct NextCmd { String key; int num; };

// ── UDP LOGGER ───────────────────────────────
WiFiUDP udpLog;
#define LOG_PORT 4444

void rlog(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  udpLog.beginPacket("255.255.255.255", LOG_PORT);
  udpLog.print(buf);
  udpLog.endPacket();
}

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
  udpLog.begin(LOG_PORT);
}

// ── FIREBASE ─────────────────────────────────
void connectFirebase() {
  config.api_key      = FIREBASE_API_KEY;
  config.database_url = FIREBASE_DATABASE_URL;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  if (!Firebase.signUp(&config, &auth, "", "")) {
    rlog("Firebase anon auth failed: %s\n", config.signer.signupError.message.c_str());
  }
}

// ── HEADING TRACKING ─────────────────────────
void applyTurn(char dir, float degrees) {
  int delta = (int)round(degrees);
  if (dir == 'R') headingDeg = ((headingDeg + delta) % 360 + 360) % 360;
  else            headingDeg = ((headingDeg - delta) % 360 + 360) % 360;
}

// ── FIREBASE STATUS ──────────────────────────
void updateFirebaseStatus(const char* qs) {
  if (!Firebase.ready()) return;
  FirebaseJson j;
  j.set("queue_status",   qs);
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
  motionStart   = millis();
  leftMotor.setSpeed(leftSpd);
  rightMotor.setSpeed(rightSpd);
  motionActive  = true;
}

// ── COMMAND EXECUTION ────────────────────────
void executeCommand(String seqKey, String cmd) {
  cmd.trim(); cmd.toUpperCase();
  char  type  = cmd.charAt(0);
  float value = cmd.substring(1).toFloat();

  activeSeqKey  = seqKey;
  activeCommand = cmd;

  setItemStatus(seqKey, "IN_PROGRESS");
  updateFirebaseStatus("IN_PROGRESS");
  rlog("▶ %s: %s\n", seqKey.c_str(), cmd.c_str());

  unsigned long ms;

  if (type == 'F') {
    ms = (unsigned long)(value * MS_PER_CM);
    startMotion(RUN_SPEED, -RUN_SPEED, ms);
    rlog("   FWD  %.1f cm -> %lu ms\n", value, ms);
  }
  else if (type == 'B') {
    ms = (unsigned long)(value * MS_PER_CM);
    startMotion(-RUN_SPEED, RUN_SPEED, ms);
    rlog("   BWD  %.1f cm -> %lu ms\n", value, ms);
  }
  else if (type == 'R') {
    ms = (unsigned long)(value * MS_PER_DEG);
    startMotion(RUN_SPEED, RUN_SPEED, ms);
    rlog("   RIGHT %.0f deg -> %lu ms\n", value, ms);
  }
  else if (type == 'L') {
    ms = (unsigned long)(value * MS_PER_DEG);
    startMotion(-RUN_SPEED, -RUN_SPEED, ms);
    rlog("   LEFT  %.0f deg -> %lu ms\n", value, ms);
  }
  else {
    rlog("   Unknown command: %s\n", cmd.c_str());
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
    int num = key.substring(4).toInt();
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

  leftMotor.runSpeed();
  rightMotor.runSpeed();

  if (millis() - motionStart >= motionTotal) {
    stopMotors();
    motionActive = false;

    char  cmdType = activeCommand.charAt(0);
    float val     = activeCommand.substring(1).toFloat();
    if (cmdType == 'R') applyTurn('R', val);
    if (cmdType == 'L') applyTurn('L', val);

    setItemStatus(activeSeqKey, "DONE");
    updateFirebaseStatus("DONE");
    rlog("DONE\n");
  }
}

// ── SETUP ────────────────────────────────────
void setup() {
  Serial.begin(115200);

  leftMotor.setMaxSpeed(MAX_SPEED);
  rightMotor.setMaxSpeed(MAX_SPEED);

  connectWiFi();
  connectFirebase();

  Serial.print("Firebase");
  unsigned long t = millis();
  while (!Firebase.ready() && millis() - t < 10000) {
    delay(300); Serial.print(".");
  }
  rlog(Firebase.ready() ? " OK\n" : " FAILED\n");

  updateFirebaseStatus("IDLE");
  rlog("wheelchair_01 ready\n");
  rlog("   MS_PER_CM=%.2f  MS_PER_DEG=%.2f\n", MS_PER_CM, MS_PER_DEG);
}

// ── LOOP ─────────────────────────────────────
void loop() {
  runMotion();
  if (!motionActive) {
    pollQueue();
  }
}
