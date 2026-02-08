
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <ctype.h>

// ===================== Bluetooth =====================
// BT TX -> Arduino D2 (RX)
// Arduino D3 (TX) -> BT RX (via divider)  <-- needed to SEND distance to phone
SoftwareSerial BT(2, 3); // RX, TX

// ===================== Ultrasonic (HC-SR04) =====================
#define US_TRIG_PIN 9
#define US_ECHO_PIN 10
#define DIST_REPORT_MS 5000UL

unsigned long lastDistReportMs = 0;

// ===================== Commands =====================
#define FORWARD   'F'
#define BACKWARD  'B'
#define LEFT      'L'
#define RIGHT     'R'
#define CIRCLE    'C'
#define CROSS     'X'
#define TRIANGLE  'T'
#define SQUARE    'S'
#define PAUSE     'P'

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ===================== Robot geometry / gait =====================
const float Z_STAND = 10.7f;

#define l1 6.5f
#define l2 7.0f

#define stepHeight     10.7f
#define stepClearance  2.2f   // keep same

// ===================== Channels (PCA9685) =====================
#define ANKLE_L_CH 0
#define KNEE_L_CH  1
#define HIP_L_CH   2

#define ANKLE_R_CH 4
#define KNEE_R_CH  5
#define HIP_R_CH   6

// ===================== Known-good init pose (RAW COUNTS) =====================
#define ANKLE_L0 185
#define KNEE_L0  460
#define HIP_L0   170

#define ANKLE_R0 205
#define KNEE_R0   80
#define HIP_R0   140

// ===================== Per-joint safety limits (RAW COUNTS) =====================
#define ANKLE_L_MIN 120
#define ANKLE_L_MAX 260
#define ANKLE_R_MIN 120
#define ANKLE_R_MAX 260

// ===================== TUNE THESE =====================
#define KNEE_CPD  2.1f
#define ANKLE_CPD 1.4f

#define HIP_CPD_L 1.50f
#define HIP_CPD_R 1.30f

#define STANCE_R_SCALE 0.90f
#define STANCE_L_SCALE 1.12f
#define SWING_R_SCALE  1.00f
#define SWING_L_SCALE  0.95f

// ===================== Prevent foot “crossing” =====================
#define SWING_MID_BIAS 0.8f

// ===================== RIGHT swing clearance (keep same) =====================
#define RIGHT_SWING_X_SEP       0.55f
#define RIGHT_SWING_EXTRA_LIFT  0.35f

// ===================== Turning =====================
// 0 = straight, + = turn right, - = turn left
#define TURN_GAIN 0.70f      // a bit stronger than before
#define TURN_BIAS 0.85f      // left/right bias magnitude
#define TURN_SPEED_BOOST 8   // ms faster when turning (higher frequency)

// ===================== Offsets computed at boot =====================
int HIP_L_OFFC, KNEE_L_OFFC, ANKLE_L_OFFC;
int HIP_R_OFFC, KNEE_R_OFFC, ANKLE_R_OFFC;

// ===================== Debug =====================
bool DEBUG = false;

float lastHipL, lastKneeL, lastAnkleL;
float lastHipR, lastKneeR, lastAnkleR;

int lastHL, lastKL, lastAL;
int lastHR, lastKR, lastAR;

float lastXl, lastZl, lastXr, lastZr;

// ===================== Motion control =====================
float stepLength = 2.5f;
int stepVelocity = 30;

enum Mode { MODE_STOP, MODE_FWD, MODE_LEFT, MODE_RIGHT };
volatile Mode mode = MODE_STOP;

// ===================== Helpers =====================
static inline int clampCount(int v) {
  if (v < 0) return 0;
  if (v > 4095) return 4095;
  return v;
}
static inline void setC(uint8_t ch, int offCount) {
  pwm.setPWM(ch, 0, clampCount(offCount));
}
static inline float clampAcos(float x) {
  return constrain(x, -1.0f, 1.0f);
}
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ===================== Ultrasonic =====================
float readDistanceCm() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  // timeout ~25ms (~4m), returns 0 on timeout
  unsigned long duration = pulseIn(US_ECHO_PIN, HIGH, 25000UL);
  if (duration == 0) return -1.0f;

  // HC-SR04: cm = us / 58
  return (float)duration / 58.0f;
}

void reportDistanceIfDue() {
  unsigned long now = millis();
  if (now - lastDistReportMs < DIST_REPORT_MS) return;
  lastDistReportMs = now;

  float cm = readDistanceCm();

  // Send to Bluetooth (phone terminal)
  BT.print("DIST_CM ");
  if (cm < 0) BT.println("-1");
  else BT.println(cm, 1);

  // Optional local debug
  if (DEBUG) {
    Serial.print("DIST_CM ");
    if (cm < 0) Serial.println("-1");
    else Serial.println(cm, 1);
  }
}

// ===================== Pose =====================
void setPose() {
  setC(ANKLE_L_CH, ANKLE_L0);
  setC(KNEE_L_CH,  KNEE_L0);
  setC(HIP_L_CH,   HIP_L0);

  setC(ANKLE_R_CH, ANKLE_R0);
  setC(KNEE_R_CH,  KNEE_R0);
  setC(HIP_R_CH,   HIP_R0);
}

void printDebug() {
  if (!DEBUG) return;

  Serial.print("R x="); Serial.print(lastXr, 2);
  Serial.print(" z=");  Serial.print(lastZr, 2);
  Serial.print(" | H="); Serial.print(lastHipR, 1);
  Serial.print(" K=");   Serial.print(lastKneeR, 1);
  Serial.print(" A=");   Serial.print(lastAnkleR, 1);

  Serial.print(" || L x="); Serial.print(lastXl, 2);
  Serial.print(" z=");      Serial.print(lastZl, 2);
  Serial.print(" | H=");    Serial.print(lastHipL, 1);
  Serial.print(" K=");      Serial.print(lastKneeL, 1);
  Serial.print(" A=");      Serial.println(lastAnkleL, 1);
}

// ===================== Servo update =====================
void updateServoPos(float hipDeg, float kneeDeg, float ankleDeg, char leg) {
  if (leg == 'l') {
    lastHipL = hipDeg; lastKneeL = kneeDeg; lastAnkleL = ankleDeg;

    lastHL = (int)round(HIP_L_OFFC   - HIP_CPD_L * hipDeg);
    lastKL = (int)round(KNEE_L_OFFC  - KNEE_CPD  * kneeDeg);
    lastAL = (int)round(ANKLE_L_OFFC - ANKLE_CPD * ankleDeg);

    lastAL = constrain(lastAL, ANKLE_L_MIN, ANKLE_L_MAX);

    setC(HIP_L_CH,   lastHL);
    setC(KNEE_L_CH,  lastKL);
    setC(ANKLE_L_CH, lastAL);
  } else {
    lastHipR = hipDeg; lastKneeR = kneeDeg; lastAnkleR = ankleDeg;

    lastHR = (int)round(HIP_R_OFFC   + HIP_CPD_R * hipDeg);
    lastKR = (int)round(KNEE_R_OFFC  + KNEE_CPD  * kneeDeg);
    lastAR = (int)round(ANKLE_R_OFFC + ANKLE_CPD * ankleDeg);

    lastAR = constrain(lastAR, ANKLE_R_MIN, ANKLE_R_MAX);

    setC(HIP_R_CH,   lastHR);
    setC(KNEE_R_CH,  lastKR);
    setC(ANKLE_R_CH, lastAR);
  }
}

// ===================== IK =====================
void pos(float x, float z, char leg) {
  if (leg == 'l') { lastXl = x; lastZl = z; }
  else            { lastXr = x; lastZr = z; }

  float hipRad2 = atan(x / z);
  float hipDeg2 = hipRad2 * (180.0f / PI);

  float z2 = z / cos(hipRad2);

  float c1 = (sq(l1) + sq(z2) - sq(l2)) / (2 * l1 * z2);
  float hipRad1 = acos(clampAcos(c1));
  float hipDeg1 = hipRad1 * (180.0f / PI);

  float c2 = (sq(l1) + sq(l2) - sq(z2)) / (2 * l1 * l2);
  float kneeRad  = PI - acos(clampAcos(c2));

  float c3 = (sq(l2) + sq(z2) - sq(l1)) / (2 * l2 * z2);
  float ankleRad = PI / 2 + hipRad2 - acos(clampAcos(c3));

  float hipDeg   = hipDeg1 + hipDeg2;
  float kneeDeg  = kneeRad  * (180.0f / PI);
  float ankleDeg = ankleRad * (180.0f / PI);

  if (isnan(hipDeg) || isnan(kneeDeg) || isnan(ankleDeg)) {
    if (DEBUG) Serial.println("IK NaN");
    return;
  }

  updateServoPos(hipDeg, kneeDeg, ankleDeg, leg);
}

// ===================== Bluetooth =====================
void applyCommand(char c) {
  c = (char)toupper((unsigned char)c);

  if (c == FORWARD || c == TRIANGLE) {
    mode = MODE_FWD;
  } else if (c == LEFT || c == SQUARE) {
    mode = MODE_LEFT;
  } else if (c == RIGHT || c == CIRCLE) {
    mode = MODE_RIGHT;
  } else if (c == BACKWARD || c == CROSS || c == PAUSE) {
    mode = MODE_STOP; // back stops
  } else if (c == 'D') {
    DEBUG = !DEBUG;
  } else if (c >= '0' && c <= '9') {
    int v = c - '0';
    stepVelocity = 70 - (v * 6);
    if (stepVelocity < 12) stepVelocity = 12;
  }
}

void pollBT() {
  while (BT.available()) {
    char c = BT.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    applyCommand(c);
  }
}

// ===================== Gait (EXACT forward movement + turn bias) =====================
// turnBias: 0=forward exact, + = right, - = left
void takeStepSame(float stepLen, int vel, float turnBias) {
  float b = clampf(turnBias, -1.0f, 1.0f);
  float mR = clampf(1.0f - TURN_GAIN * b, 0.20f, 2.00f);
  float mL = clampf(1.0f + TURN_GAIN * b, 0.20f, 2.00f);

  // Phase 1: RIGHT stance, LEFT swing (same math)
  for (float i = stepLen; i >= -stepLen; i -= 0.25f) {
    pollBT();
    reportDistanceIfDue();
    if (mode == MODE_STOP) return;

    float t = (stepLen - i) / (2.0f * stepLen);
    t = constrain(t, 0.0f, 1.0f);

    float swingBias = SWING_MID_BIAS * sin(PI * t);

    float xR = (i * STANCE_R_SCALE) * mR;
    float xL = (i * SWING_L_SCALE + swingBias) * mL;

    pos(-xR, stepHeight,                 'r');
    pos(+xL, stepHeight - stepClearance, 'l');

    printDebug();
    delay(vel);
  }
  delay(150);

  // Phase 2: LEFT stance, RIGHT swing (same math)
  for (float i = stepLen; i >= -stepLen; i -= 0.25f) {
    pollBT();
    reportDistanceIfDue();
    if (mode == MODE_STOP) return;

    float t = (stepLen - i) / (2.0f * stepLen);
    t = constrain(t, 0.0f, 1.0f);

    float swingBias = SWING_MID_BIAS * sin(PI * t);

    float sep  = RIGHT_SWING_X_SEP * sin(PI * t);
    float lift = RIGHT_SWING_EXTRA_LIFT * sin(PI * t);

    float xR = (i * SWING_R_SCALE + swingBias + sep) * mR;
    float xL = (i * STANCE_L_SCALE) * mL;

    float zR = (stepHeight - stepClearance) - lift;

    pos(+xR, zR,         'r');
    pos(-xL, stepHeight, 'l');

    printDebug();
    delay(vel);
  }
  delay(150);
}

// ===================== Init =====================
void initialize() {
  for (float i = 10.7f; i >= stepHeight; i -= 0.1f) {
    pos(0, i, 'l');
    pos(0, i, 'r');
    delay(10);
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  BT.begin(9600);

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  setPose();
  delay(2000);

  float hipRad2 = atan(0.0f / Z_STAND);
  float z2 = Z_STAND / cos(hipRad2);

  float c1 = (sq(l1) + sq(z2) - sq(l2)) / (2 * l1 * z2);
  float hipRad1 = acos(clampAcos(c1));

  float c2 = (sq(l1) + sq(l2) - sq(z2)) / (2 * l1 * l2);
  float kneeRad = PI - acos(clampAcos(c2));

  float c3 = (sq(l2) + sq(z2) - sq(l1)) / (2 * l2 * z2);
  float ankleRad = PI / 2 + hipRad2 - acos(clampAcos(c3));

  float hipDeg   = (hipRad1 + hipRad2) * (180.0f / PI);
  float kneeDeg  = kneeRad  * (180.0f / PI);
  float ankleDeg = ankleRad * (180.0f / PI);

  HIP_L_OFFC   = (int)round(HIP_L0   + HIP_CPD_L * hipDeg);
  KNEE_L_OFFC  = (int)round(KNEE_L0  + KNEE_CPD  * kneeDeg);
  ANKLE_L_OFFC = (int)round(ANKLE_L0 + ANKLE_CPD * ankleDeg);

  HIP_R_OFFC   = (int)round(HIP_R0   - HIP_CPD_R * hipDeg);
  KNEE_R_OFFC  = (int)round(KNEE_R0  - KNEE_CPD  * kneeDeg);
  ANKLE_R_OFFC = (int)round(ANKLE_R0 - ANKLE_CPD * ankleDeg);

  initialize();
  mode = MODE_STOP;
  setPose();

  lastDistReportMs = millis(); // start timer
  BT.println("READY");
}

// ===================== Loop =====================
void loop() {
  pollBT();
  reportDistanceIfDue();

  if (mode == MODE_STOP) {
    setPose();
    delay(20);
    return;
  }

  if (mode == MODE_FWD) {
    takeStepSame(stepLength, stepVelocity, 0.0f);
  } else if (mode == MODE_LEFT) {
    int v = stepVelocity - TURN_SPEED_BOOST;
    if (v < 12) v = 12;
    takeStepSame(stepLength, v, -TURN_BIAS);
  } else if (mode == MODE_RIGHT) {
    int v = stepVelocity - TURN_SPEED_BOOST;
    if (v < 12) v = 12;
    takeStepSame(stepLength, v, +TURN_BIAS);
  }
}