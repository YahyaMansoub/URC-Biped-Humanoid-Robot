#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ===================== Robot geometry / gait =====================
const float Z_STAND = 10.7f;

#define l1 6.5f
#define l2 7.0f

#define stepHeight     10.7f
#define stepClearance  2.5f

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
#define HIP_L0   120

#define ANKLE_R0 205
#define KNEE_R0   80
#define HIP_R0   140

// ===================== TUNE THESE (counts per degree) =====================
// Start smaller if it snaps; increase if it barely moves.
#define HIP_CPD   1.2f
#define KNEE_CPD  2.1f
#define ANKLE_CPD 2.1f

// ===================== Offsets computed at boot =====================
int HIP_L_OFFC, KNEE_L_OFFC, ANKLE_L_OFFC;
int HIP_R_OFFC, KNEE_R_OFFC, ANKLE_R_OFFC;

// ===================== Debug =====================
bool DEBUG = true;

float lastHipL, lastKneeL, lastAnkleL;
float lastHipR, lastKneeR, lastAnkleR;

int lastHL, lastKL, lastAL;
int lastHR, lastKR, lastAR;

float lastXl, lastZl, lastXr, lastZr;

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

void setPose() {
  setC(ANKLE_L_CH, ANKLE_L0);
  setC(KNEE_L_CH,  KNEE_L0);
  setC(HIP_L_CH,   HIP_L0);

  setC(ANKLE_R_CH, ANKLE_R0);
  setC(KNEE_R_CH,  KNEE_R0);
  setC(HIP_R_CH,   HIP_R0);
}

// ===================== Debug printer =====================
void printDebug() {
  if (!DEBUG) return;

  Serial.print("R x="); Serial.print(lastXr, 2);
  Serial.print(" z=");  Serial.print(lastZr, 2);
  Serial.print(" | H="); Serial.print(lastHipR, 1);
  Serial.print(" K=");   Serial.print(lastKneeR, 1);
  Serial.print(" A=");   Serial.print(lastAnkleR, 1);
  Serial.print(" || PWM R: ");
  Serial.print(lastHR); Serial.print(",");
  Serial.print(lastKR); Serial.print(",");
  Serial.print(lastAR);

  Serial.print(" || L x="); Serial.print(lastXl, 2);
  Serial.print(" z=");      Serial.print(lastZl, 2);
  Serial.print(" | H=");    Serial.print(lastHipL, 1);
  Serial.print(" K=");      Serial.print(lastKneeL, 1);
  Serial.print(" A=");      Serial.print(lastAnkleL, 1);
  Serial.print(" || PWM L: ");
  Serial.print(lastHL); Serial.print(",");
  Serial.print(lastKL); Serial.print(",");
  Serial.println(lastAL);
}

// ===================== Servo update (stores + sends) =====================
void updateServoPos(float hipDeg, float kneeDeg, float ankleDeg, char leg) {
  if (leg == 'l') {
    lastHipL = hipDeg; lastKneeL = kneeDeg; lastAnkleL = ankleDeg;

    lastHL = (int)round(HIP_L_OFFC   - HIP_CPD   * hipDeg);
    lastKL = (int)round(KNEE_L_OFFC  - KNEE_CPD  * kneeDeg);
    lastAL = (int)round(ANKLE_L_OFFC - ANKLE_CPD * ankleDeg);

    setC(HIP_L_CH,  lastHL);
    setC(KNEE_L_CH, lastKL);
    setC(ANKLE_L_CH,lastAL);
  } else {
    lastHipR = hipDeg; lastKneeR = kneeDeg; lastAnkleR = ankleDeg;

    lastHR = (int)round(HIP_R_OFFC   + HIP_CPD   * hipDeg);
    lastKR = (int)round(KNEE_R_OFFC  + KNEE_CPD  * kneeDeg);
    lastAR = (int)round(ANKLE_R_OFFC + ANKLE_CPD * ankleDeg);

    setC(HIP_R_CH,  lastHR);
    setC(KNEE_R_CH, lastKR);
    setC(ANKLE_R_CH,lastAR);
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
    Serial.println("IK NaN");
    return;
  }

  updateServoPos(hipDeg, kneeDeg, ankleDeg, leg);
}

// ===================== Gait =====================
void takeStep(float stepLength, int stepVelocity) {
  for (float i = stepLength; i >= -stepLength; i -= 0.25f) {
    pos(i,  stepHeight,                 'r');
    pos(-i, stepHeight - stepClearance, 'l');
    printDebug();
    delay(stepVelocity);
  }

  for (float i = stepLength; i >= -stepLength; i -= 0.5f) {
    pos(-i, stepHeight - stepClearance, 'r');
    pos(i,  stepHeight,                 'l');
    printDebug();
    delay(stepVelocity);
  }
}

void initialize() {
  for (float i = 10.7f; i >= stepHeight; i -= 0.1f) {
    pos(0, i, 'l');
    pos(0, i, 'r');
    delay(10);
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  setPose();
  delay(2000);

  // Compute IK angles at stand (x=0,z=Z_STAND)
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

  // Offsets so pos(0,Z_STAND) reproduces your init counts
  HIP_L_OFFC   = (int)round(HIP_L0   + HIP_CPD   * hipDeg);
  KNEE_L_OFFC  = (int)round(KNEE_L0  + KNEE_CPD  * kneeDeg);
  ANKLE_L_OFFC = (int)round(ANKLE_L0 + ANKLE_CPD * ankleDeg);

  HIP_R_OFFC   = (int)round(HIP_R0   - HIP_CPD   * hipDeg);
  KNEE_R_OFFC  = (int)round(KNEE_R0  - KNEE_CPD  * kneeDeg);
  ANKLE_R_OFFC = (int)round(ANKLE_R0 - ANKLE_CPD * ankleDeg);

  Serial.println("Start");
  Serial.print("OFF L H/K/A: "); Serial.print(HIP_L_OFFC); Serial.print(" ");
  Serial.print(KNEE_L_OFFC); Serial.print(" "); Serial.println(ANKLE_L_OFFC);
  Serial.print("OFF R H/K/A: "); Serial.print(HIP_R_OFFC); Serial.print(" ");
  Serial.print(KNEE_R_OFFC); Serial.print(" "); Serial.println(ANKLE_R_OFFC);

  initialize();
}

// ===================== Loop =====================
void loop() {
  // Start gentle; increase stepLength slowly later
  takeStep(2.5f, 30);
}

