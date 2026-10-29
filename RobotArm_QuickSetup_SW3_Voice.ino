#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>

#include <math.h>

#include <WiFi.h>

#include <WebServer.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Local Wi-Fi access point. Change password before use.

const char* WEB_SSID = "RobotArm-ESP32";

const char* WEB_PASSWORD = "RobotArm2026!";

WebServer webServer(80);

void processCommand(String cmd);

// ================= CAU HINH =================

#define SDA_PIN 21

#define SCL_PIN 22

#define SERVO_FREQ 50

const int N = 5;

const int servoCH[N] = {0, 1, 2, 3, 4};

const int GRIP_INDEX = 4;

const int POSE_COUNT = 5;

// Gia tri PWM tam thoi tu code cu

// Can hieu chinh theo servo thuc te

const int SERVOMIN = 110;

const int SERVOMAX = 500;

// Gioi han tam thoi, KHONG phai gioi han co khi da xac minh

float minAngle[N] = {0, 0, 0, 0, 0};

float maxAngle[N] = {180, 180, 180, 180, 180};

float currentAngle[N] = {0, 0, 0, 0, 0};

bool initialized[N] = {false, false, false, false, false};

float poses[POSE_COUNT][N];

bool poseSaved[POSE_COUNT] = {false, false, false, false, false};

const char* poseName[POSE_COUNT] = {

  "HOME", "PICK", "LIFT", "PLACE", "CUSTOM4"

};

// ================= KEP =================

float gripOpenAngle = 90;

float gripCloseAngle = 60;

bool openConfigured = false;

bool closeConfigured = false;

// ================= TOC DO =================

int setDuration = 1000;

int poseDuration = 2000;

int gripDuration = 800;

const unsigned long UPDATE_MS = 20;

// true = Quintic, false = Cubic

bool smoothMode = true;

bool stopRequested = false;

String inputLine = "";

// ================= PWM =================

int angleToPulse(int i, float angle) {

  angle = constrain(angle, minAngle[i], maxAngle[i]);

  // Dao chieu rieng servo CH1

  if (i == 1) {

    angle = 180.0f - angle;

  }

  float pulse = SERVOMIN +

      angle * (SERVOMAX - SERVOMIN) / 180.0f;

  return constrain((int)roundf(pulse), 0, 4095);

}

// ANG: gui khi goc lenh thay doi; gom nhieu kenh thanh 1 ban tin.
// Khong su dung bo hen gio de phat ANG khi robot dung yen.
bool angleDirty = false;
float lastAngleReported[N] = {NAN, NAN, NAN, NAN, NAN};
bool lastInitializedReported[N] = {false, false, false, false, false};
bool angleSnapshotSent = false;

void sendAnglesIfChanged(bool force = false) {
  bool changed = force || !angleSnapshotSent || angleDirty;
  if (!changed) return;
  bool trulyChanged = force || !angleSnapshotSent;
  for (int i = 0; i < N; ++i) {
    if (initialized[i] != lastInitializedReported[i]) trulyChanged = true;
    if (initialized[i] && (!lastInitializedReported[i] ||
        fabsf(currentAngle[i] - lastAngleReported[i]) >= 0.01f)) trulyChanged = true;
  }
  angleDirty = false;
  if (!trulyChanged) return;
  Serial.print("ANG");
  for (int i = 0; i < N; ++i) {
    Serial.print(',');
    if (initialized[i]) {
      Serial.print(currentAngle[i], 2);
      lastAngleReported[i] = currentAngle[i];
    } else {
      Serial.print("nan");
      lastAngleReported[i] = NAN;
    }
    lastInitializedReported[i] = initialized[i];
  }
  Serial.println();
  angleSnapshotSent = true;
}

void writeServo(int i, float angle) {

  if (i < 0 || i >= N) return;

  angle = constrain(angle, minAngle[i], maxAngle[i]);

  pwm.setPWM(servoCH[i], 0, angleToPulse(i, angle));

  if (!initialized[i] || fabsf(currentAngle[i] - angle) >= 0.01f) angleDirty = true;
  currentAngle[i] = angle;

}

// ================= NOI SUY =================

float interpolation(float t) {

  t = constrain(t, 0.0f, 1.0f);

  if (smoothMode) {

    // Quintic

    return t * t * t *

           (10.0f + t * (-15.0f + 6.0f * t));

  }

  // Cubic

  return t * t * (3.0f - 2.0f * t);

}

// ================= STOP =================

// STOP la dung bang phan mem,

// KHONG phai dung khan cap phan cung.

void pollStop() {

  while (Serial.available()) {

    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {

      inputLine.trim();

      inputLine.toUpperCase();

      if (inputLine == "STOP") {

        stopRequested = true;

      }

      inputLine = "";

    } else {

      if (inputLine.length() < 80) {

        inputLine += c;

      } else {

        inputLine = "";

      }

    }

  }

}

extern bool joyNeedsCenter;

// ================= CHUYEN DONG =================

bool moveServos(

  const float target[N],

  const bool active[N],

  int durationMs

) {

  joyNeedsCenter = true;

  float start[N];

  float finish[N];

  bool hasMovement = false;

  // Kiem tra tat ca kenh truoc khi chuyen dong

  for (int i = 0; i < N; i++) {

    start[i] = currentAngle[i];

    finish[i] = currentAngle[i];

    if (!active[i]) continue;

    if (!initialized[i]) {

      Serial.print("Chua SET CH");

      Serial.println(servoCH[i]);

      return false;

    }

    if (target[i] < minAngle[i] ||

        target[i] > maxAngle[i]) {

      Serial.print("Goc ngoai gioi han CH");

      Serial.println(servoCH[i]);

      return false;

    }

    finish[i] = target[i];

    if (fabsf(finish[i] - start[i]) > 0.01f) {

      hasMovement = true;

    }

  }

  if (!hasMovement) return true;

  if (durationMs < 1) durationMs = 1;

  stopRequested = false;

  unsigned long startTime = millis();
  unsigned long lastStreamChange = millis();

  while (true) {

    pollStop();

    if (stopRequested) {

      sendAnglesIfChanged(); // gui goc cuoi neu dang di chuyen thi dung
      Serial.println("DA NHAN STOP");

      return false;

    }

    unsigned long elapsed = millis() - startTime;

    float t = (float)elapsed / durationMs;

    if (t > 1.0f) t = 1.0f;

    float s = interpolation(t);

    for (int i = 0; i < N; i++) {

      if (!active[i]) continue;

      float angle = start[i] +

                    (finish[i] - start[i]) * s;

      writeServo(i, angle);

    }
    // Cap nhat PyBullet trong luc servo noi suy, toi da ~10 lan/giay,
    // CHI khi goc lenh da doi, khong gui khi dung yen.
    if (millis() - lastStreamChange >= 100 || t >= 1.0f) {
      sendAnglesIfChanged();
      lastStreamChange = millis();
    }

    if (t >= 1.0f) break;

    delay(UPDATE_MS);

  }

  for (int i = 0; i < N; i++) {

    if (active[i]) {

      writeServo(i, finish[i]);

    }

  }

  return true;

}

void invalidateYZAnchor();

extern bool geomCalibrated;

void geometryServoFK();

// ================= 3 JOYSTICK (ADC1 / Wi-Fi compatible) =================
// J1 X=>CH0, J1 Y=>CH1; J2 X=>CH2, J2 Y=>CH3; J3 Y=>CH4.
// Joystick is OFF at boot. Each joint needs a verified JLIMIT before enabling.
const int joyAnalogPins[5] = {34, 35, 32, 33, 39};
const int joyButtonPins[3] = {25, 26, 27};
bool voiceEnabled = false; // SW3 at GPIO27 toggles Python voice recognition
const int JOY_CENTER = 2048;
const int JOY_DEADZONE = 350;
const unsigned long JOY_PERIOD_MS = 25;
const unsigned long JOY_DEBOUNCE_MS = 55;
bool joyEnabled = false;
bool joyNeedsCenter = true;
bool joyFast = false;
bool joyLimitReady[N] = {false, false, false, false, false};
float joyMin[N] = {0, 0, 0, 0, 0};
float joyMax[N] = {180, 180, 180, 180, 180};
float joySlowDegS = 6.0f;  // JOYSPEED SLOW <deg/s>
float joyFastDegS = 18.0f; // JOYSPEED FAST <deg/s>
unsigned long joyLastTick = 0;
bool joyButtonStable[3] = {HIGH, HIGH, HIGH};
bool joyButtonRaw[3] = {HIGH, HIGH, HIGH};
unsigned long joyButtonChanged[3] = {0, 0, 0};

bool joyReady(bool verbose) {
  for (int i = 0; i < N; i++) {
    if (!initialized[i] || !joyLimitReady[i]) {
      if (verbose) {
        Serial.print("JOYSTICK: CH"); Serial.print(i);
        Serial.println(" requires SET and JLIMIT <ch> <min> <max>");
      }
      return false;
    }
    if (currentAngle[i] < joyMin[i] || currentAngle[i] > joyMax[i]) {
      if (verbose) {
        Serial.print("JOYSTICK: CH"); Serial.print(i);
        Serial.println(" currently outside JLIMIT; reposition safely with SET");
      }
      return false;
    }
  }
  return true;
}

void joystickEnable(bool enabled) {
  if (enabled && !joyReady(true)) {
    joyEnabled = false;
    return;
  }
  joyEnabled = enabled;
  joyNeedsCenter = true;
  joyLastTick = millis();
  Serial.println(joyEnabled ? "JOYSTICK ON: release ALL sticks to center before control" : "JOYSTICK OFF");
}

void joystickStatus() {
  Serial.print("JOYSTICK: "); Serial.println(joyEnabled ? "ON" : "OFF");
  Serial.print("JOYSTICK SPEED: ");
  Serial.print(joyFast ? "FAST (" : "SLOW (");
  Serial.print(joyFast ? joyFastDegS : joySlowDegS, 1);
  Serial.println(" deg/s)");
  Serial.printf("SLOW configured: %.1f deg/s | FAST configured: %.1f deg/s\n", joySlowDegS, joyFastDegS);
  for (int i = 0; i < N; i++) {
    Serial.print("JLIMIT CH"); Serial.print(i); Serial.print(": ");
    if (joyLimitReady[i]) { Serial.print(joyMin[i],1); Serial.print(".."); Serial.println(joyMax[i],1); }
    else Serial.println("NOT SET");
  }
}

void joystickTick() {
  unsigned long now = millis();
  // Poll switches also while joystick is disabled, to allow SW1 to enable it.
  for (int i = 0; i < 3; i++) {
    bool raw = digitalRead(joyButtonPins[i]);
    if (raw != joyButtonRaw[i]) {
      joyButtonRaw[i] = raw;
      joyButtonChanged[i] = now;
    }
    if ((now - joyButtonChanged[i]) >= JOY_DEBOUNCE_MS && raw != joyButtonStable[i]) {
      joyButtonStable[i] = raw;
      if (raw == LOW) {
        if (i == 0) joystickEnable(!joyEnabled);
        if (i == 1 && joyEnabled) {
          joyFast = !joyFast;
          Serial.println(joyFast ? "JOYSTICK FAST" : "JOYSTICK SLOW");
        }
        if (i == 2) {
          voiceEnabled = !voiceEnabled;
          Serial.println(voiceEnabled ? "VOICE ON" : "VOICE OFF");
        }
      }
    }
  }

  if (now - joyLastTick < JOY_PERIOD_MS) return;
  unsigned long elapsed = now - joyLastTick;
  joyLastTick = now;
  if (!joyEnabled) return;
  if (!joyReady(false)) {
    joystickEnable(false);
    Serial.println("JOYSTICK disabled: calibration/initialization no longer valid");
    return;
  }

  // Limit elapsed time to prevent an abrupt jump after a blocking Serial/Web move.
  float dt = fminf((float)elapsed / 1000.0f, 0.05f);
  int raw[5];
  for (int i = 0; i < N; i++) raw[i] = analogRead(joyAnalogPins[i]);
  bool centered = true;
  for (int i = 0; i < N; i++) {
    if (abs(raw[i] - JOY_CENTER) > JOY_DEADZONE) centered = false;
  }
  // Prevent motion when enabling with displaced sticks, or after AUTO/SET.
  if (joyNeedsCenter) {
    if (centered) { joyNeedsCenter = false; Serial.println("JOYSTICK READY"); }
    return;
  }

  const float maxRate = joyFast ? joyFastDegS : joySlowDegS;
  for (int i = 0; i < N; i++) {
    int delta = raw[i] - JOY_CENTER;
    int ad = abs(delta);
    if (ad <= JOY_DEADZONE) continue;
    float fraction = (float)(ad - JOY_DEADZONE) / (4095 - JOY_CENTER - JOY_DEADZONE);
    fraction = constrain(fraction, 0.0f, 1.0f);
    float step = (delta >= 0 ? 1.0f : -1.0f) * fraction * maxRate * dt;
    float next = constrain(currentAngle[i] + step, joyMin[i], joyMax[i]);
    next = constrain(next, minAngle[i], maxAngle[i]);
    if (fabsf(next - currentAngle[i]) > 0.0001f) {
      if (i == 0 || i == 1) invalidateYZAnchor();
      writeServo(i, next);
    }
  }
}


// ================= SET =================

int indexFromChannel(int ch) {

  for (int i = 0; i < N; i++) {

    if (servoCH[i] == ch) return i;

  }

  return -1;

}

void setServoAngle(int ch, float angle) {

  if (ch==0 || ch==1) invalidateYZAnchor();

  int i = indexFromChannel(ch);

  if (i < 0) {

    Serial.println("Kenh khong hop le. Dung 0..4.");

    return;

  }

  if (angle < minAngle[i] || angle > maxAngle[i]) {

    Serial.println("Goc ngoai gioi han.");

    return;

  }

  // Lan dau SET, khong biet goc thuc cua servo.

  // Chi gui lenh dich, khong noi suy.

  if (!initialized[i]) {

    initialized[i] = true;
    joyNeedsCenter = true;

    writeServo(i, angle);

    Serial.print("KHOI TAO CH");

    Serial.print(ch);

    Serial.print(" = ");

    Serial.println(angle, 1);

    return;

  }

  float target[N];

  bool active[N] = {false, false, false, false, false};

  for (int k = 0; k < N; k++) {

    target[k] = currentAngle[k];

  }

  target[i] = angle;

  active[i] = true;

  if (moveServos(target, active, setDuration)) {

    Serial.println("SET HOAN TAT");

    if (ch <= 2 && geomCalibrated && initialized[0] && initialized[1] && initialized[2]) geometryServoFK();

  }

}

// ================= GEOMETRIC MODEL =================

// Ideal equal-link 2-branch model, NOT verified linkage kinematics.

// Coordinate origin: CH2 axis at shoulder height (offset configurable).

// +Y at CH2=90, +X at CH2=180, +Z up.

// CH0/CH1 output remains geometric angles until linkage mapping verified.

const float LINK_MM = 125.0f;

float shoulderHeightMM = 0.0f; // shoulder height above CH2 origin; NOT measured

float radialOffsetMM = 0.0f;   // shoulder radial displacement; NOT measured

const float DEG = 180.0f / PI;

bool geometryIK(float x, float y, float z, float &base, float &a0, float &a1) {

  if (!isfinite(x)||!isfinite(y)||!isfinite(z)) return false;

  float radius=hypotf(x,y);

  float r=radius-radialOffsetMM;

  float zz=z-shoulderHeightMM;

  float h=hypotf(r,zz);

  if(h>2.0f*LINK_MM+0.0001f || h<0.0001f || radius<0.0001f) return false;

  base=90.0f+atan2f(x,y)*DEG;

  if(base<minAngle[2] || base>maxAngle[2]) return false;

  float phi=atan2f(zz,r);

  float theta=acosf(constrain(h/(2.0f*LINK_MM),-1.0f,1.0f));

  a0=(phi+theta)*DEG;

  a1=(phi-theta)*DEG;

  return true;

}

void geometryFK(float base,float a0,float a1) {

  if(!isfinite(base)||!isfinite(a0)||!isfinite(a1)) return;

  float u=a0/DEG,v=a1/DEG;

  float r=LINK_MM*(cosf(u)+cosf(v))+radialOffsetMM;

  float z=LINK_MM*(sinf(u)+sinf(v))+shoulderHeightMM;

  float t=(base-90.0f)/DEG;

  Serial.print("GEOMFK X=");Serial.print(r*sinf(t),3);

  Serial.print(" Y=");Serial.print(r*cosf(t),3);

  Serial.print(" Z=");Serial.println(z,3);

  Serial.println("IDEAL GEOMETRY ONLY; NOT ACTUAL SERVO FK");

}

void geometryCheck(float x,float y,float z) {

  float base,a0,a1;

  if(!geometryIK(x,y,z,base,a0,a1)) {

    Serial.println("GEOMIK: unreachable, singular, or base outside limits");return;

  }

  Serial.print("CH2 geometric = ");Serial.println(base,3);

  Serial.print("alpha0 geometric = ");Serial.println(a0,3);

  Serial.print("alpha1 geometric = ");Serial.println(a1,3);

  Serial.println("NO PWM. CH0/CH1 linkage-to-servo mapping UNVERIFIED.");

}

// ================= SERVO / GEOMETRY CALIBRATION =================

// Approximate local mapping only. It is NOT a solved closed-chain mechanism.

// At a physically measured reference TCP, CH0=130 and CH1=150.

// Local direction observations: increasing CH0 raises Z; increasing CH1 extends r.

// Angles in this section are command angles, not feedback.

bool geomCalibrated = false;

float geomRefA0 = 0.0f, geomRefA1 = 0.0f;

float geomRefServo0 = 130.0f, geomRefServo1 = 150.0f;

float geomRefX=0, geomRefY=0, geomRefZ=0;

bool geometryServoTargets(float x,float y,float z,float &s0,float &s1,float &s2) {

  if (!geomCalibrated) {

    Serial.println("GEOMETRY: first use GEOCAL X Y Z 130 150 at a measured reference TCP");

    return false;

  }

  float a0,a1;

  if(!geometryIK(x,y,z,s2,a0,a1)) return false;

  s0=geomRefServo0+(a0-geomRefA0);

  s1=geomRefServo1+(a1-geomRefA1);

  if(!isfinite(s0)||!isfinite(s1)||s0<minAngle[0]||s0>maxAngle[0]||

     s1<minAngle[1]||s1>maxAngle[1]||s2<minAngle[2]||s2>maxAngle[2]) {

    Serial.println("GEOMETRY: computed servo angles outside configured limits");

    return false;

  }

  return true;

}

void geometryCalibrate(float x,float y,float z,float s0,float s1) {

  float b,a0,a1;

  if(!geometryIK(x,y,z,b,a0,a1) || !isfinite(s0)||!isfinite(s1)||

     s0<minAngle[0]||s0>maxAngle[0]||s1<minAngle[1]||s1>maxAngle[1]) {

    Serial.println("GEOCAL: invalid reference or outside ideal workspace");return;

  }

  geomRefX=x;geomRefY=y;geomRefZ=z;

  geomRefA0=a0;geomRefA1=a1;

  geomRefServo0=s0;geomRefServo1=s1;

  geomCalibrated=true;

  Serial.println("GEOCAL saved in RAM. Approximate model only; motion DISARMED.");

  Serial.print("Reference geometric alpha0=");Serial.print(a0,3);

  Serial.print(" alpha1=");Serial.println(a1,3);

}

void geometryPreview(float x,float y,float z) {

  float s0,s1,s2;

  if(!geometryServoTargets(x,y,z,s0,s1,s2)) {

    Serial.println("GEOPREVIEW: cannot calculate target");return;

  }

  Serial.println("GEOPREVIEW: APPROXIMATE ONLY / NO PWM");

  Serial.print("CH0 command=");Serial.println(s0,3);

  Serial.print("CH1 command=");Serial.println(s1,3);

  Serial.print("CH2 command=");Serial.println(s2,3);

  Serial.println("WARNING: linkage geometry, TCP offset and collisions unverified");

}

void geometryMove(float x,float y,float z) {

  float s0,s1,s2;

  if(!geometryServoTargets(x,y,z,s0,s1,s2)) return;

  if(!initialized[0]||!initialized[1]||!initialized[2]) {

    Serial.println("GEOMOVE BLOCKED: SET CH0, CH1, CH2 first");return;

  }

  float target[N];bool active[N]={true,true,true,false,false};

  for(int i=0;i<N;i++) target[i]=currentAngle[i];

  target[0]=s0;target[1]=s1;target[2]=s2;

  Serial.println("GEOMOVE: sending commanded target (NO position feedback)");

  moveServos(target,active,2000);

}

void geometryServoFK() {

  if(!geomCalibrated||!initialized[0]||!initialized[1]||!initialized[2]) {

    Serial.println("GEOFKCMD: need GEOCAL and SET CH0/CH1/CH2");return;

  }

  float a0=geomRefA0+(currentAngle[0]-geomRefServo0);

  float a1=geomRefA1+(currentAngle[1]-geomRefServo1);

  Serial.println("GEOFKCMD: model estimate from commanded servo angles, not measured TCP");

  geometryFK(currentAngle[2],a0,a1);

}

// Test approximate XYZ for arbitrary servo command angles WITHOUT moving hardware.

void geometryTestXYZ(float ch0,float ch1,float ch2) {

  if (!geomCalibrated) {

    Serial.println("TESTXYZ: first calibrate with GEOCAL X Y Z 130 150");

    return;

  }

  if (!isfinite(ch0)||!isfinite(ch1)||!isfinite(ch2)||

      ch0<minAngle[0]||ch0>maxAngle[0]||

      ch1<minAngle[1]||ch1>maxAngle[1]||

      ch2<minAngle[2]||ch2>maxAngle[2]) {

    Serial.println("TESTXYZ: angle outside configured servo limits");return;

  }

  float a0=geomRefA0+(ch0-geomRefServo0);

  float a1=geomRefA1+(ch1-geomRefServo1);

  Serial.print("TESTXYZ CH0=");Serial.print(ch0,2);

  Serial.print(" CH1=");Serial.print(ch1,2);

  Serial.print(" CH2=");Serial.println(ch2,2);

  geometryFK(ch2,a0,a1);

  Serial.println("Estimated XYZ only; no servo movement, not measured position.");

}

// ================= SAVE =================

bool allInitialized() {

  for (int i = 0; i < N; i++) {

    if (!initialized[i]) return false;

  }

  return true;

}

void savePose(int p) {

  if (p < 0 || p >= POSE_COUNT) {

    Serial.println("Pose khong hop le.");

    return;

  }

  if (!allInitialized()) {

    Serial.println("Can SET du CH0, CH1, CH2, CH3, CH4.");

    return;

  }

  // Luu goc cua ca 5 servo

  for (int i = 0; i < N; i++) {

    poses[p][i] = currentAngle[i];

  }

  poseSaved[p] = true;

  // Hien thi thong tin pose vua luu

  Serial.println();

  Serial.println("================================");

  Serial.print("DA LUU THANH CONG: SAVE ");

  Serial.print(p);

  Serial.print(" - ");

  Serial.println(poseName[p]);

  Serial.println("--------------------------------");

  Serial.println("GOC CUA 5 SERVO:");

  for (int i = 0; i < N; i++) {

    Serial.print("CH");

    Serial.print(servoCH[i]);

    Serial.print(" = ");

    Serial.print(poses[p][i], 2);

    Serial.println(" do");

  }

  Serial.println("================================");

  Serial.println();

}

// ================= GO =================

bool moveToPose(int p, int durationMs, bool includeGrip) {

  invalidateYZAnchor();

  if (p < 0 || p >= POSE_COUNT || !poseSaved[p]) {

    Serial.println("Pose chua duoc luu.");

    return false;

  }

  bool active[N] = {true, true, true, true, true};

  if (!includeGrip) {

    active[GRIP_INDEX] = false;

  }

  Serial.print("DI CHUYEN DEN ");

  Serial.println(poseName[p]);

  return moveServos(poses[p], active, durationMs);

}

// ================= KEP =================

bool moveGripper(float angle) {

  if (!initialized[GRIP_INDEX]) {

    Serial.println("Can SET CH4 truoc.");

    return false;

  }

  float target[N];

  bool active[N] = {false, false, false, false, true};

  for (int i = 0; i < N; i++) {

    target[i] = currentAngle[i];

  }

  target[GRIP_INDEX] = angle;

  return moveServos(target, active, gripDuration);

}

bool openGripper() {

  if (!openConfigured) {

    Serial.println("Chua cau hinh GRIPOPEN.");

    return false;

  }

  Serial.println("MO KEP");

  return moveGripper(gripOpenAngle);

}

bool closeGripper() {

  if (!closeConfigured) {

    Serial.println("Chua cau hinh GRIPCLOSE.");

    return false;

  }

  Serial.println("DONG KEP");

  return moveGripper(gripCloseAngle);

}

// ================= AUTO =================

void autoPickPlace() {

  if (!allInitialized()) {

    Serial.println("AUTO: chua SET du 5 servo.");

    return;

  }

  for (int p = 0; p < POSE_COUNT; p++) {

    if (!poseSaved[p]) {

      Serial.println("AUTO: can SAVE du 0,1,2,3.");

      return;

    }

  }

  if (!openConfigured || !closeConfigured) {

    Serial.println(

      "AUTO: can GRIPOPEN va GRIPCLOSE."

    );

    return;

  }

  Serial.println("=== BAT DAU AUTO 5 SERVO ===");

  // 1. Mo kep

  if (!openGripper()) return;

  // 2. HOME: CH0, CH1, CH2, CH3

  if (!moveToPose(0, poseDuration, false)) return;

  // 3. PICK

  if (!moveToPose(1, poseDuration, false)) return;

  // 4. Dong kep

  if (!closeGripper()) return;

  delay(300);

  // 5. LIFT

  if (!moveToPose(2, poseDuration, false)) return;

  // 6. PLACE

  if (!moveToPose(3, poseDuration, false)) return;

  // 7. Mo kep

  if (!openGripper()) return;

  delay(300);

  // 8. HOME

  if (!moveToPose(0, poseDuration, false)) return;

  Serial.println("=== AUTO HOAN TAT ===");

}

// ================= AUTOXYZ: PICK & PLACE BY XYZ =================

// Coordinates describe the gripper TCP, not the object center.

// CH3 uses explicit PICK and PLACE wrist angles. CH4 opens/closes explicitly.

// Every waypoint is validated before any servo movement.

struct XYZPoint { float x, y, z; };

XYZPoint autoPickXYZ = {0,0,0};

XYZPoint autoPlaceXYZ = {0,0,0};

bool autoPickSet = false, autoPlaceSet = false;

float autoLiftMM = 0.0f; // user MUST configure a clearance, no default guess

float autoWristPick = 0.0f, autoWristPlace = 0.0f;

bool autoWristConfigured = false;

void printXYZPoint(const char* name, float x, float y, float z) {

  Serial.print(name); Serial.print(" X="); Serial.print(x,1);

  Serial.print(" Y="); Serial.print(y,1);

  Serial.print(" Z="); Serial.println(z,1);

}

bool autoXYZTargets(float goals[4][3]) {

  if (!geomCalibrated || !autoPickSet || !autoPlaceSet || autoLiftMM<=0) {

    Serial.println("AUTOXYZ: need GEOCAL, PICKXYZ, PLACEXYZ and XYZLIFT > 0");

    return false;

  }

  if (!isfinite(autoPickXYZ.z+autoLiftMM) || !isfinite(autoPlaceXYZ.z+autoLiftMM)) {

    Serial.println("AUTOXYZ: invalid approach heights"); return false;

  }

  XYZPoint wp[4] = {

    {autoPickXYZ.x,autoPickXYZ.y,autoPickXYZ.z+autoLiftMM},

    autoPickXYZ,

    {autoPlaceXYZ.x,autoPlaceXYZ.y,autoPlaceXYZ.z+autoLiftMM},

    autoPlaceXYZ

  };

  const char* names[4]={"PICK ABOVE", "PICK", "PLACE ABOVE", "PLACE"};

  for(int k=0;k<4;k++) {

    float a,b,c;

    if(!geometryServoTargets(wp[k].x,wp[k].y,wp[k].z,a,b,c)) {

      Serial.print("AUTOXYZ: unreachable waypoint "); Serial.println(names[k]);

      return false;

    }

    goals[k][0]=a; goals[k][1]=b; goals[k][2]=c;

    printXYZPoint(names[k],wp[k].x,wp[k].y,wp[k].z);

    Serial.print("  CH0=");Serial.print(a,2);

    Serial.print(" CH1=");Serial.print(b,2);

    Serial.print(" CH2=");Serial.println(c,2);

  }

  Serial.println("AUTOXYZ: waypoint angles valid; clearance/collision NOT verified");

  return true;

}

bool moveAutoXYZAngles(const float angles[3]) {

  float target[N];

  bool active[N]={true,true,true,false,false};

  for(int i=0;i<N;i++) target[i]=currentAngle[i];

  for(int i=0;i<3;i++) target[i]=angles[i];

  return moveServos(target,active,poseDuration);

}

// CH3 changes only while gripper is at an elevated waypoint.

bool moveAutoWrist(float angle) {

  float target[N]; bool active[N]={false,false,false,true,false};

  for(int i=0;i<N;i++) target[i]=currentAngle[i];

  target[3]=angle;

  return moveServos(target,active,poseDuration);

}

void autoXYZCycle() {

  float goals[4][3];

  if(!autoXYZTargets(goals)) return;

  if(!initialized[0]||!initialized[1]||!initialized[2]||!initialized[3]||!initialized[4]) {

    Serial.println("AUTOXYZ BLOCKED: SET CH0..CH4 first"); return;

  }

  if(!autoWristConfigured) {

    Serial.println("AUTOXYZ BLOCKED: configure AUTOCH3 pickAngle placeAngle first");return;

  }

  if(!openConfigured||!closeConfigured) {

    Serial.println("AUTOXYZ BLOCKED: configure AUTOCH4 openAngle closeAngle first");return;

  }

  if(autoWristPick<minAngle[3]||autoWristPick>maxAngle[3]||

     autoWristPlace<minAngle[3]||autoWristPlace>maxAngle[3]||

     gripOpenAngle<minAngle[4]||gripOpenAngle>maxAngle[4]||

     gripCloseAngle<minAngle[4]||gripCloseAngle>maxAngle[4]) {

    Serial.println("AUTOXYZ BLOCKED: CH3/CH4 outside angle limits");return;

  }

  Serial.println("AUTOXYZ START: CH3 pick/place; CH4 open/close");

  // Joint-interpolated motion: user must verify clearance and gripper orientation.

  Serial.println("AUTOXYZ 1/10: open CH4");

  if(!openGripper()) return;

  Serial.println("AUTOXYZ 2/10: move above PICK");

  if(!moveAutoXYZAngles(goals[0])) return;

  Serial.println("AUTOXYZ 3/10: CH3 pick angle");

  if(!moveAutoWrist(autoWristPick)) return;

  Serial.println("AUTOXYZ 4/10: descend to PICK");

  if(!moveAutoXYZAngles(goals[1])) return;

  Serial.println("AUTOXYZ 5/10: close CH4");

  if(!closeGripper()) return;

  Serial.println("AUTOXYZ 6/10: lift PICK");

  if(!moveAutoXYZAngles(goals[0])) return;

  Serial.println("AUTOXYZ 7/10: move above PLACE");

  if(!moveAutoXYZAngles(goals[2])) return;

  Serial.println("AUTOXYZ 8/10: CH3 place angle");

  if(!moveAutoWrist(autoWristPlace)) return;

  Serial.println("AUTOXYZ 9/10: descend to PLACE and open CH4");

  if(!moveAutoXYZAngles(goals[3])) return;

  if(!openGripper()) return;

  Serial.println("AUTOXYZ 10/10: retreat above PLACE");

  if(!moveAutoXYZAngles(goals[2])) return;

  Serial.println("AUTOXYZ DONE");

}

// Old Fusion point table, inverse triangulation, MOVEYZ and PATHYZ removed.

// Do not move motors using idealized IK until closed-loop linkage is solved.

void invalidateYZAnchor() {} // retained for legacy direct-servo motion calls

// ================= STATUS =================

void showStatus() {
  joystickStatus();

  Serial.println();

  Serial.println("===== STATUS =====");

  Serial.print("SMOOTH: ");

  Serial.println(smoothMode ? "ON (QUINTIC)" :

                              "OFF (CUBIC)");

  Serial.print("SET TIME: ");

  Serial.println(setDuration);

  Serial.print("GO/AUTO TIME: ");

  Serial.println(poseDuration);

  Serial.print("GRIP TIME: ");

  Serial.println(gripDuration);

  for (int i = 0; i < N; i++) {

    Serial.print("CH");

    Serial.print(servoCH[i]);

    Serial.print(": ");

    if (initialized[i]) {

      Serial.println(currentAngle[i], 2);

    } else {

      Serial.println("CHUA SET");

    }

  }

  for (int p = 0; p < POSE_COUNT; p++) {

    Serial.print("SAVE ");

    Serial.print(p);

    Serial.print(" (");

    Serial.print(poseName[p]);

    Serial.print("): ");

    Serial.println(poseSaved[p] ? "OK" : "CHUA LUU");

  }

  Serial.print("GRIPOPEN: ");

  if (openConfigured) {

    Serial.println(gripOpenAngle);

  } else {

    Serial.println("CHUA CAU HINH");

  }

  Serial.print("GRIPCLOSE: ");

  if (closeConfigured) {

    Serial.println(gripCloseAngle);

  } else {

    Serial.println("CHUA CAU HINH");

  }

  Serial.println("==================");

}

// ================= HELP =================

void showHelp() {
  Serial.println("JOYSTICK ON / JOYSTICK OFF / JOYSTICK STATUS");
  Serial.println("SW3 (GPIO27): toggle VOICE ON/OFF; PC must run arm_live.py");
  Serial.println("SETALL a0 a1 a2 a3 a4 : set all 5 angles in order (first set immediate)");
  Serial.println("JLIMITALL min0 max0 min1 max1 min2 max2 min3 max3 min4 max4");
  Serial.println("JLIMIT ch min max : verified physical bounds for each CH0..CH4");
  Serial.println("JOYSPEED SLOW deg_per_s : configure SLOW speed 1..25");
  Serial.println("JOYSPEED FAST deg_per_s : configure FAST speed 1..25");
  Serial.println("JOYSPEED deg_per_s : legacy alias to set FAST speed");
  Serial.println("SW1 toggle joystick; SW2 slow/fast; SW3 reserved");

  Serial.println("===== GEOMETRIC IK / FK (NO PWM) =====");

  Serial.println("GEOMIK 100 100 50   : target robot X Y Z mm");

  Serial.println("GEOMFK 135 45 -25  : base, alpha0, alpha1 deg");

  Serial.println("GEOOFFSET 0 0      : radial offset, shoulder height mm");

  Serial.println("LINKS               : show ideal model parameters");

  Serial.println("Direct control retained: SET ch angle, MOVE3, MOVE5");

  Serial.println("SAVE n, GO n, OPEN, CLOSE, AUTO, STATUS, STOP");

  Serial.println("GEOCAL X Y Z 130 150 : reference TCP measured at CH0=130 CH1=150");
  Serial.println("GEOCALNOW X Y Z     : calibrate at current commanded CH0/CH1 (SET CH0..2 first)");

  Serial.println("GEOPREVIEW X Y Z   : preview approximate CH0/CH1/CH2, NO PWM");

  Serial.println("GEOFKCMD            : approximate XYZ from current servo commands");

  Serial.println("TESTXYZ ch0 ch1 ch2 : estimated XYZ for SET angles, NO PWM");

  Serial.println("After SET CH0/CH1/CH2, estimated XYZ is printed automatically if calibrated.");

  Serial.println("GEOMOVE X Y Z       : no 3-degree cap; requires calibration");

  Serial.println("PICKXYZ x y z / PLACEXYZ x y z : gripper TCP targets mm");

  Serial.println("XYZLIFT mm          : explicit vertical approach clearance mm");

  Serial.println("AUTOCHECK           : preview 4 AUTOXYZ waypoints, NO PWM");

  Serial.println("AUTOCH3 pick place  : CH3 wrist angles at PICK and PLACE (degrees)");

  Serial.println("AUTOCH4 open close  : CH4 gripper angles (same as GRIPOPEN/GRIPCLOSE)");

  Serial.println("AUTOXYZ             : pick/place XYZ; operates CH0,CH1,CH2,CH3,CH4");

  Serial.println("AUTO (old)          : saved-pose routine unchanged");

  Serial.println("WARNING: closed-chain geometry not established; model is approximate.");

}

// ================= WEB UI (LOCAL ACCESS POINT) =================

// Web requests are handled between motions. The web STOP button is not

// a real-time emergency stop; use a physical power disconnect for emergencies.

const char WEB_PAGE[] PROGMEM = R"HTML(

<!doctype html><html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">

<title>Robot Arm 5 Servo</title><style>

:root{font-family:system-ui,Arial;background:#0c1423;color:#f1f5f9}body{max-width:720px;margin:auto;padding:15px}h1{font-size:23px}p,small{color:#a7b5c9;line-height:1.45}.card{background:#172438;border:1px solid #34465e;border-radius:14px;padding:15px;margin:12px 0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:12px}label{display:block;font-weight:600}input,button{box-sizing:border-box;width:100%;font-size:16px;border-radius:9px;padding:11px;margin-top:5px}input{background:#0c1423;border:1px solid #50647c;color:#fff}button{background:#0d9488;color:#fff;border:0;font-weight:700;cursor:pointer}.secondary{background:#40526c}.row{display:flex;gap:9px;flex-wrap:wrap}.row button{flex:1;min-width:100px}#msg{white-space:pre-wrap;color:#5eead4}

</style></head><body>

<h1>Robot Arm · 5 Servo</h1><p>DOIT ESP32 DevKit V1 · PCA9685 · Wi-Fi local</p>

<div class="card"><b>Trang thai</b><p id="state">Dang ket noi...</p><button class="secondary" onclick="refresh()">Lam moi</button><small>Goc hien thi la goc lenh, KHONG phai goc cam bien.</small></div>

<div class="card"><h3>Dieu khien tung servo</h3><p>Nhap goc trong gioi han co khi da kiem chung. Nut SET se dieu khien servo tuong ung.</p><div class="grid" id="servos"></div></div>

<div class="card"><h3>Di chuyen 5 servo dong thoi</h3><p>CH0–CH4 can duoc khoi tao truoc. Moi kenh duoc noi suy dong thoi.</p><label>Thoi gian chuyen dong (ms)<input id="dur" type="number" min="500" max="10000" value="3000"></label><button onclick="move5()">MOVE5 · ca 5 servo</button></div>

<div class="card"><h3>Luu / chay tu the</h3><p id="poses">Dang kiem tra tu the...</p><small>Can khoi tao du 5 servo truoc SAVE. SAVE luu goc hien tai, GO quay ve goc da luu.</small><div class="row"><button class="secondary" onclick="send('SAVE 0')">SAVE 0</button><button onclick="send('GO 0')">GO 0</button><button class="secondary" onclick="send('SAVE 1')">SAVE 1</button><button onclick="send('GO 1')">GO 1</button><button class="secondary" onclick="send('SAVE 2')">SAVE 2</button><button onclick="send('GO 2')">GO 2</button><button class="secondary" onclick="send('SAVE 3')">SAVE 3</button><button onclick="send('GO 3')">GO 3</button><button class="secondary" onclick="send('SAVE 4')">SAVE 4</button><button onclick="send('GO 4')">GO 4</button></div></div>

<div class="card"><h3>Dieu khien XYZ</h3><p>Toa do TCP tinh bang mm. GEOPREVIEW chi tinh goc; GEOMOVE se dieu khien CH0-CH2 theo mo hinh IK gan dung.</p>
<div class="grid"><label>X (mm)<input id="tx" type="number" step="any" value="90"></label><label>Y (mm)<input id="ty" type="number" step="any" value="90"></label><label>Z (mm)<input id="tz" type="number" step="any" value="60"></label></div>
<div class="row"><button class="secondary" onclick="previewXYZ()">Xem truoc goc</button><button onclick="moveXYZWeb()">GEOMOVE XYZ</button></div>
<p id="xyzResult">Chua tinh IK.</p>
<h4>Hieu chuan tham chieu</h4><p>GEOCALNOW dung goc CH0 va CH1 hien tai da SET; nhap X/Y/Z do thuc te tai tam kep. Can SET CH0..CH2 truoc, va toa do phai dung voi huong CH2 hien tai. Hieu chuan luu RAM (mat khi reset).</p>
<div class="grid"><label>X do (mm)<input id="cx" type="number" step="any" value="0"></label><label>Y do (mm)<input id="cy" type="number" step="any" placeholder="Do thuc te"></label><label>Z do (mm)<input id="cz" type="number" step="any" placeholder="Do thuc te"></label></div>
<div class="row"><button class="secondary" onclick="calibrateNowWeb()">GEOCALNOW (goc hien tai)</button><button class="secondary" onclick="calibrateWeb()">GEOCAL 130/150</button></div><small>Can GEOCAL truoc khi GEOMOVE hoac AUTOXYZ. Toa do la uoc tinh, khong phai cam bien vi tri.</small></div>
<div class="card"><h3>Dong hoc thuan - Goc sang XYZ</h3>
<div class="grid"><label>CH0 (do)<input id="f0" type="number" step="any" value="130"></label><label>CH1 (do)<input id="f1" type="number" step="any" value="150"></label><label>CH2 (do)<input id="f2" type="number" step="any" value="90"></label></div>
<button class="secondary" onclick="testXYZWeb()">TESTXYZ (khong quay servo)</button><p id="fkResult">Chua tinh FK.</p></div>
<div class="card"><h3>AUTOXYZ - Gap va tha theo toa do</h3>
<div class="grid"><label>Pick X<input id="px" type="number" step="any" value="90"></label><label>Pick Y<input id="py" type="number" step="any" value="90"></label><label>Pick Z<input id="pz" type="number" step="any" value="60"></label><label>Place X<input id="qx" type="number" step="any" value="100"></label><label>Place Y<input id="qy" type="number" step="any" value="100"></label><label>Place Z<input id="qz" type="number" step="any" value="80"></label><label>Nang (mm)<input id="lift" type="number" step="any" value="40"></label><label>CH3 gap<input id="w0" type="number" step="any" value="70"></label><label>CH3 tha<input id="w1" type="number" step="any" value="110"></label><label>CH4 mo<input id="g0" type="number" step="any" value="70"></label><label>CH4 dong<input id="g1" type="number" step="any" value="110"></label></div>
<div class="row"><button class="secondary" onclick="configAutoXYZ()">Luu AUTOXYZ</button><button class="secondary" onclick="send('AUTOCHECK')">AUTOCHECK</button><button onclick="send('AUTOXYZ')">Chay AUTOXYZ</button></div><small>Chay AUTOCHECK truoc; goc waypoint va ket qua chi tiet xem o Serial Monitor. Khong dam bao duong di tranh va cham.</small></div>
<div class="card"><h3>Hinh hoc IK va lenh Serial</h3><label>Lenh<input id="cmd" placeholder="GEOMIK 100 100 50 / GEOMFK 135 45 -25"></label><div class="row"><button onclick="sendManual()">Gui lenh</button><button class="secondary" onclick="send('STATUS')">STATUS</button><button class="secondary" onclick="send('LINKS')">LINKS</button></div><small>Ket qua chi tiet cua cac lenh hien tai xem trong Serial Monitor.</small></div>

<div class="card"><b>An toan</b><p>Khong su dung cac goc chua kiem chung. Trang web khong the dung khan cap trong khi mot lenh dang chay. Can cong tac ngat nguon servo rieng.</p></div><p id="msg" role="status"></p>

<script>

const root=document.getElementById('servos');

for(let i=0;i<5;i++){let el=document.createElement('div');el.innerHTML='<label>CH'+i+' <input id="a'+i+'" type="number" step="1" min="0" max="180" placeholder="Goc"></label><button class="secondary" onclick="setOne('+i+')">SET CH'+i+'</button>';root.appendChild(el)}

let busy=false;

async function refresh(){try{let r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error(r.status);let s=await r.json();document.getElementById('state').textContent=s.servos.map((v,i)=>'CH'+i+': '+(v.initialized?v.angle.toFixed(1)+'°':'chua SET')).join(' | ');document.getElementById('poses').textContent='Tu the: '+s.poses.map((v,i)=>'GO '+i+': '+(v?'DA LUU':'CHUA LUU')).join(' | ')}catch(e){document.getElementById('state').textContent='Khong ket noi duoc ESP32: '+e}}

function val(i){let t=document.getElementById('a'+i).value.trim(),v=Number(t);return t!==''&&Number.isFinite(v)&&v>=0&&v<=180?v:null}

function setOne(i){let v=val(i);if(v===null){alert('Nhap goc hop le CH'+i);return}send('SET '+i+' '+v)}

function move5(){let a=Array.from({length:5},(_,i)=>val(i));let d=Number(document.getElementById('dur').value);if(a.some(v=>v===null)||!Number.isInteger(d)||d<500||d>10000){alert('Nhap 5 goc 0..180 va thoi gian 500..10000 ms');return}send('MOVE5 '+a.join(' ')+' '+d)}

function sendManual(){send(document.getElementById('cmd').value)}
function num(id){const raw=document.getElementById(id).value.trim();const v=Number(raw);if(raw===''||!Number.isFinite(v))throw Error('Nhap so hop le: '+id);return v}
function xyzInput(){return [num('tx'),num('ty'),num('tz')]}
function previewXYZ(){try{const a=xyzInput();fetchGeometry('ik',a,'xyzResult')}catch(e){alert(e.message)}}
function testXYZWeb(){try{fetchGeometry('fk',[num('f0'),num('f1'),num('f2')],'fkResult')}catch(e){alert(e.message)}}
async function fetchGeometry(mode,a,id){let target=document.getElementById(id);target.textContent='Dang tinh...';try{let u='/api/geometry?mode='+encodeURIComponent(mode)+'&a='+encodeURIComponent(a[0])+'&b='+encodeURIComponent(a[1])+'&c='+encodeURIComponent(a[2]);let r=await fetch(u,{cache:'no-store'});let j=await r.json();if(!r.ok||!j.ok)throw Error(j.error||'Khong the tinh');target.textContent=mode==='ik'?('CH0 = '+j.ch0.toFixed(2)+'°, CH1 = '+j.ch1.toFixed(2)+'°, CH2 = '+j.ch2.toFixed(2)+'° (du kien)'):('X = '+j.x.toFixed(2)+', Y = '+j.y.toFixed(2)+', Z = '+j.z.toFixed(2)+' mm (uoc tinh)')}catch(e){target.textContent='Loi: '+e.message}}
function moveXYZWeb(){try{let a=xyzInput();send('GEOMOVE '+a.join(' '))}catch(e){alert(e.message)}}
function calibrateWeb(){try{send('GEOCAL '+[num('cx'),num('cy'),num('cz'),130,150].join(' '))}catch(e){alert(e.message)}}
function calibrateNowWeb(){try{send('GEOCALNOW '+[num('cx'),num('cy'),num('cz')].join(' '))}catch(e){alert(e.message)}}
async function configAutoXYZ(){try{let commands=['PICKXYZ '+[num('px'),num('py'),num('pz')].join(' '),'PLACEXYZ '+[num('qx'),num('qy'),num('qz')].join(' '),'XYZLIFT '+num('lift'),'AUTOCH3 '+[num('w0'),num('w1')].join(' '),'AUTOCH4 '+[num('g0'),num('g1')].join(' ')];for(const cmd of commands){if(!await send(cmd))return}document.getElementById('msg').textContent='Da gui 5 lenh cau hinh. Xem AUTOCHECK va Serial Monitor.'}catch(e){alert(e.message)}}


async function send(cmd){if(busy||!cmd.trim())return false;if(/^(SET|GO|MOVE5|MOVE3|GEOMOVE|AUTOXYZ|FPOSE|FSET|MOVEYZ|PATHYZ|OPEN|CLOSE|AUTO)\b/i.test(cmd)&&!confirm('Xac nhan chuyen dong: '+cmd+' ?'))return false;busy=true;document.getElementById('msg').textContent='Dang gui: '+cmd;try{let r=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({cmd})});document.getElementById('msg').textContent=await r.text();if(!r.ok)throw Error('HTTP '+r.status);await refresh();return true}catch(e){document.getElementById('msg').textContent='Loi ket noi: '+e;return false}finally{busy=false}}

refresh();setInterval(()=>{if(!busy)refresh()},5000)

</script></body></html>

)HTML";

void webStatus() {

  String json = "{\"servos\":[";

  for (int i = 0; i < N; i++) {

    if (i) json += ',';

    json += "{\"angle\":" + String(currentAngle[i], 2);

    json += ",\"initialized\":";

    json += initialized[i] ? "true" : "false";

    json += '}';

  }

  json += "],\"poses\":[";

  for (int p = 0; p < POSE_COUNT; p++) {

    if (p) json += ",";

    json += poseSaved[p] ? "true" : "false";

  }

  json += "]}";

  webServer.send(200, "application/json", json);

}

// Read-only Web preview: no PWM, uses the same approximate GEOCAL mapping.
void webGeometry() {
  String mode = webServer.arg("mode");
  if (!webServer.hasArg("a") || !webServer.hasArg("b") || !webServer.hasArg("c")) {
    webServer.send(400,"application/json","{\"ok\":false,\"error\":\"Missing values\"}"); return;
  }
  String sa=webServer.arg("a"), sb=webServer.arg("b"), sc=webServer.arg("c");
  char tail;
  float a,b,c;
  if (sscanf(sa.c_str(),"%f %c",&a,&tail)!=1 ||
      sscanf(sb.c_str(),"%f %c",&b,&tail)!=1 ||
      sscanf(sc.c_str(),"%f %c",&c,&tail)!=1 ||
      !isfinite(a)||!isfinite(b)||!isfinite(c)) {
    webServer.send(400,"application/json","{\"ok\":false,\"error\":\"Invalid numeric input\"}");return;
  }
  if (!geomCalibrated) {
    webServer.send(409,"application/json","{\"ok\":false,\"error\":\"Need GEOCAL first\"}");return;
  }
  if (mode=="ik") {
    float base,alpha0,alpha1;
    if (!geometryIK(a,b,c,base,alpha0,alpha1)) {
      webServer.send(422,"application/json","{\"ok\":false,\"error\":\"XYZ outside ideal workspace\"}");return;
    }
    float ch0=geomRefServo0+(alpha0-geomRefA0);
    float ch1=geomRefServo1+(alpha1-geomRefA1);
    if (!isfinite(ch0)||!isfinite(ch1)||ch0<minAngle[0]||ch0>maxAngle[0]||ch1<minAngle[1]||ch1>maxAngle[1]) {
      webServer.send(422,"application/json","{\"ok\":false,\"error\":\"Servo angles outside limits\"}");return;
    }
    String out="{\"ok\":true,\"ch0\":"+String(ch0,4)+",\"ch1\":"+String(ch1,4)+",\"ch2\":"+String(base,4)+"}";
    webServer.send(200,"application/json",out);return;
  }
  if (mode=="fk") {
    if(a<minAngle[0]||a>maxAngle[0]||b<minAngle[1]||b>maxAngle[1]||c<minAngle[2]||c>maxAngle[2]) {
      webServer.send(422,"application/json","{\"ok\":false,\"error\":\"Servo angles outside limits\"}");return;
    }
    float alpha0=(geomRefA0+a-geomRefServo0)/DEG;
    float alpha1=(geomRefA1+b-geomRefServo1)/DEG;
    float r=LINK_MM*(cosf(alpha0)+cosf(alpha1))+radialOffsetMM;
    float z=LINK_MM*(sinf(alpha0)+sinf(alpha1))+shoulderHeightMM;
    float t=(c-90.0f)/DEG;
    String out="{\"ok\":true,\"x\":"+String(r*sinf(t),4)+",\"y\":"+String(r*cosf(t),4)+",\"z\":"+String(z,4)+"}";
    webServer.send(200,"application/json",out);return;
  }
  webServer.send(400,"application/json","{\"ok\":false,\"error\":\"Unknown mode\"}");
}

void webCommand() {

  if (!webServer.hasArg("cmd")) {

    webServer.send(400, "text/plain; charset=utf-8", "Thiếu lệnh cmd");

    return;

  }

  String cmd = webServer.arg("cmd");

  cmd.trim();

  if (cmd.length() == 0 || cmd.length() > 80 || cmd.indexOf('\n') >= 0 || cmd.indexOf('\r') >= 0) {

    webServer.send(400, "text/plain; charset=utf-8", "Lệnh không hợp lệ");

    return;

  }

  int poseIndex = -1;

  if (sscanf(cmd.c_str(), "SAVE %d", &poseIndex) == 1 && cmd.startsWith("SAVE ")) {

    if (poseIndex < 0 || poseIndex >= POSE_COUNT) {

      webServer.send(400, "text/plain; charset=utf-8", "So tu the khong hop le (0..4)."); return;

    }

    if (!allInitialized()) {

      webServer.send(409, "text/plain; charset=utf-8", "SAVE THAT BAI: can SET du CH0..CH4 truoc khi luu."); return;

    }

    savePose(poseIndex);

    webServer.send(200, "text/plain; charset=utf-8", "DA LUU SAVE " + String(poseIndex) + ". Bay gio hay doi sang tu the khac roi bam GO.");

    return;

  }

  if (sscanf(cmd.c_str(), "GO %d", &poseIndex) == 1 && cmd.startsWith("GO ")) {

    if (poseIndex < 0 || poseIndex >= POSE_COUNT || !poseSaved[poseIndex]) {

      webServer.send(409, "text/plain; charset=utf-8", "GO THAT BAI: tu the chua duoc SAVE thanh cong."); return;

    }

    if (!allInitialized()) {

      webServer.send(409, "text/plain; charset=utf-8", "GO THAT BAI: can SET du CH0..CH4."); return;

    }

  }

  // Response is sent BEFORE running a potentially blocking motion.

  // Acknowledgement means only that the command was received, not completed.

  webServer.send(200, "text/plain; charset=utf-8", "Đã nhận lệnh: " + cmd + "\nTheo dõi kết quả chi tiết trên Serial Monitor. Không đồng nghĩa đã thực hiện thành công.");

  Serial.print("WEB RX: "); Serial.println(cmd);

  processCommand(cmd);

}

void setupWeb() {

  WiFi.mode(WIFI_AP);

  if (!WiFi.softAP(WEB_SSID, WEB_PASSWORD)) {

    Serial.println("WEB: Khong tao duoc Wi-Fi AP");

    return;

  }

  webServer.on("/", HTTP_GET, []() { webServer.send_P(200, "text/html; charset=utf-8", WEB_PAGE); });

  webServer.on("/api/status", HTTP_GET, webStatus);
  webServer.on("/api/geometry", HTTP_GET, webGeometry);

  webServer.on("/api/command", HTTP_POST, webCommand);

  webServer.onNotFound([]() { webServer.send(404, "text/plain", "Not found"); });

  webServer.begin();

  Serial.print("WEB SSID: "); Serial.println(WEB_SSID);

  Serial.print("WEB URL: http://"); Serial.println(WiFi.softAPIP());

}

bool moveThree(float a0, float a1, float a2, int durationMs) {

  if (durationMs < 500 || durationMs > 10000) {

    Serial.println("MOVE3: duration 500..10000 ms"); return false;

  }

  float target[N]; bool active[N];

  for (int i = 0; i < N; i++) { target[i] = currentAngle[i]; active[i] = (i < 3); }

  target[0] = a0; target[1] = a1; target[2] = a2;

  bool ok = moveServos(target, active, durationMs);

  // Any motion, including an interrupted one, may change the IK pose.

  invalidateYZAnchor();

  return ok;

}

bool moveFive(float a0, float a1, float a2, float a3, float a4, int durationMs) {

  if (durationMs < 500 || durationMs > 10000) {

    Serial.println("MOVE5: duration 500..10000 ms"); return false;

  }

  float target[N] = {a0, a1, a2, a3, a4};

  bool active[N] = {true, true, true, true, true};

  bool ok = moveServos(target, active, durationMs);

  invalidateYZAnchor();

  return ok;

}

// ================= LENH =================

void processCommand(String cmd) {

  cmd.trim();

  cmd.toUpperCase();

  if (cmd.length() == 0) return;
  // Doc snapshot khi can, khong can SET hay lam servo chuyen dong.
  if (cmd == "GETANGLES") { sendAnglesIfChanged(true); return; }

  int ch;

  float angle;

  int p;

  int value;

  float f0, f1;

  float y0,z0,y1,z1;

  float m0,m1,m2,m3,m4; int moveTime;

  // SETALL sends five validated commands in one line.
  // First initialization may immediately move servos: use verified safe angles.
  if (cmd.startsWith("SETALL ")) {
    float v[N]; char extra;
    int parsed = sscanf(cmd.c_str(), "SETALL %f %f %f %f %f %c",
                        &v[0], &v[1], &v[2], &v[3], &v[4], &extra);
    bool valid = (parsed == 5);
    for (int i = 0; i < N && valid; i++) {
      if (!isfinite(v[i]) || v[i] < minAngle[i] || v[i] > maxAngle[i]) valid = false;
      if (joyLimitReady[i] && (v[i] < joyMin[i] || v[i] > joyMax[i])) valid = false;
    }
    if (!valid) {
      Serial.println("SETALL: invalid angles or outside configured limits. No movement.");
    } else {
      joystickEnable(false);
      Serial.println("SETALL: starting CH0..CH4 sequentially; first SET is immediate, NO feedback.");
      for (int i = 0; i < N; i++) setServoAngle(i, v[i]);
      Serial.println("SETALL: commands sent. Verify physical pose before JOYSTICK ON.");
    }
  }
  else if (cmd.startsWith("JLIMITALL ")) {
    float lo[N], hi[N]; char extra;
    int parsed = sscanf(cmd.c_str(),
                        "JLIMITALL %f %f %f %f %f %f %f %f %f %f %c",
                        &lo[0], &hi[0], &lo[1], &hi[1], &lo[2], &hi[2],
                        &lo[3], &hi[3], &lo[4], &hi[4], &extra);
    bool valid = (parsed == 10);
    for (int i = 0; i < N && valid; i++) {
      if (!isfinite(lo[i]) || !isfinite(hi[i]) ||
          lo[i] < minAngle[i] || hi[i] > maxAngle[i] || lo[i] >= hi[i]) valid = false;
    }
    if (!valid) {
      Serial.println("JLIMITALL: invalid limits. Existing limits unchanged.");
    } else {
      joystickEnable(false);
      for (int i = 0; i < N; i++) {
        joyMin[i] = lo[i]; joyMax[i] = hi[i]; joyLimitReady[i] = true;
      }
      Serial.println("JLIMITALL: limits set for CH0..CH4. Check physical safety.");
      joystickStatus();
    }
  }
  else if (cmd == "JOYSTICK ON") {
    joystickEnable(true);
  }
  else if (cmd == "JOYSTICK OFF") {
    joystickEnable(false);
  }
  else if (cmd == "JOYSTICK STATUS") {
    joystickStatus();
  }
  else if (cmd.startsWith("JOYSPEED ")) {
    float requested; char extra;
    bool slowCommand = false;
    bool fastCommand = false;
    bool validSyntax = false;
    if (sscanf(cmd.c_str(), "JOYSPEED SLOW %f %c", &requested, &extra) == 1) {
      slowCommand = true;
      validSyntax = true;
    } else if (sscanf(cmd.c_str(), "JOYSPEED FAST %f %c", &requested, &extra) == 1) {
      fastCommand = true;
      validSyntax = true;
    } else if (sscanf(cmd.c_str(), "JOYSPEED %f %c", &requested, &extra) == 1) {
      fastCommand = true; // preserve previous JOYSPEED 15 syntax
      validSyntax = true;
    }
    if (validSyntax && isfinite(requested) && requested >= 1.0f && requested <= 25.0f) {
      if (slowCommand) joySlowDegS = requested;
      if (fastCommand) joyFastDegS = requested;
      Serial.printf("JOYSTICK speeds: SLOW=%.1f FAST=%.1f deg/s\n", joySlowDegS, joyFastDegS);
      if (joySlowDegS > joyFastDegS) Serial.println("NOTE: SLOW is currently faster than FAST");
    } else {
      Serial.println("Use JOYSPEED SLOW 6 or JOYSPEED FAST 18 (1..25 deg/s)");
    }
  }
  else if (cmd.startsWith("JLIMIT ")) {
    float lo, hi; int k; char extra;
    if (sscanf(cmd.c_str(), "JLIMIT %d %f %f %c", &k, &lo, &hi, &extra) == 3 &&
        k >= 0 && k < N && isfinite(lo) && isfinite(hi) &&
        lo >= minAngle[k] && hi <= maxAngle[k] && lo < hi) {
      joyMin[k] = lo; joyMax[k] = hi; joyLimitReady[k] = true;
      joystickEnable(false);
      Serial.printf("JLIMIT CH%d = %.1f..%.1f (verify mechanically)\n", k, lo, hi);
    } else {
      Serial.println("JLIMIT syntax: JLIMIT ch min max; use VERIFIED mechanical limits");
    }
  }
  else if (sscanf(cmd.c_str(), "MOVE5 %f %f %f %f %f %d", &m0,&m1,&m2,&m3,&m4,&moveTime)==6) {

    moveFive(m0,m1,m2,m3,m4,moveTime);

  }

  else if (sscanf(cmd.c_str(), "MOVE3 %f %f %f %d", &m0,&m1,&m2,&moveTime)==4) {

    moveThree(m0,m1,m2,moveTime);

  }

  else if (sscanf(cmd.c_str(), "GEOCALNOW %f %f %f", &m0,&m1,&m2)==3) {
    if (!initialized[0] || !initialized[1] || !initialized[2]) {
      Serial.println("GEOCALNOW: SET CH0, CH1, CH2 first");
    } else {
      Serial.print("GEOCALNOW: CH0="); Serial.print(currentAngle[0],2);
      Serial.print(" CH1="); Serial.print(currentAngle[1],2);
      Serial.print(" CH2="); Serial.println(currentAngle[2],2);
      geometryCalibrate(m0,m1,m2,currentAngle[0],currentAngle[1]);
    }
  }
  else if (sscanf(cmd.c_str(), "GEOCAL %f %f %f %f %f", &m0,&m1,&m2,&m3,&m4)==5) {

    geometryCalibrate(m0,m1,m2,m3,m4);

  }

  else if (sscanf(cmd.c_str(), "GEOPREVIEW %f %f %f", &m0,&m1,&m2)==3) {

    geometryPreview(m0,m1,m2);

  }

  else if (sscanf(cmd.c_str(), "GEOMOVE %f %f %f", &m0,&m1,&m2)==3) {

    geometryMove(m0,m1,m2);

  }

  else if (sscanf(cmd.c_str(), "PICKXYZ %f %f %f", &m0,&m1,&m2)==3) {

    if(isfinite(m0)&&isfinite(m1)&&isfinite(m2)) {

      autoPickXYZ={m0,m1,m2}; autoPickSet=true;

      printXYZPoint("PICKXYZ saved", autoPickXYZ.x, autoPickXYZ.y, autoPickXYZ.z);

    } else Serial.println("PICKXYZ: invalid XYZ");

  }

  else if (sscanf(cmd.c_str(), "PLACEXYZ %f %f %f", &m0,&m1,&m2)==3) {

    if(isfinite(m0)&&isfinite(m1)&&isfinite(m2)) {

      autoPlaceXYZ={m0,m1,m2}; autoPlaceSet=true;

      printXYZPoint("PLACEXYZ saved", autoPlaceXYZ.x, autoPlaceXYZ.y, autoPlaceXYZ.z);

    } else Serial.println("PLACEXYZ: invalid XYZ");

  }

  else if (sscanf(cmd.c_str(), "XYZLIFT %f", &m0)==1) {

    if(isfinite(m0)&&m0>0.0f&&m0<=150.0f) {

      autoLiftMM=m0; Serial.print("XYZLIFT mm = ");Serial.println(autoLiftMM,1);

    } else Serial.println("XYZLIFT: choose 0 < mm <= 150; check physical clearance");

  }

  else if (sscanf(cmd.c_str(), "AUTOCH3 %f %f", &m0,&m1)==2) {

    if(isfinite(m0)&&isfinite(m1)&&m0>=minAngle[3]&&m0<=maxAngle[3]&&

       m1>=minAngle[3]&&m1<=maxAngle[3]) {

      autoWristPick=m0;autoWristPlace=m1;autoWristConfigured=true;

      Serial.print("AUTOCH3 saved PICK=");Serial.print(m0,1);

      Serial.print(" PLACE=");Serial.println(m1,1);

    } else Serial.println("AUTOCH3: angles outside CH3 limits");

  }

  else if (sscanf(cmd.c_str(), "AUTOCH4 %f %f", &m0,&m1)==2) {

    if(isfinite(m0)&&isfinite(m1)&&m0>=minAngle[4]&&m0<=maxAngle[4]&&

       m1>=minAngle[4]&&m1<=maxAngle[4]) {

      gripOpenAngle=m0;gripCloseAngle=m1;

      openConfigured=true;closeConfigured=true;

      Serial.print("AUTOCH4 saved OPEN=");Serial.print(m0,1);

      Serial.print(" CLOSE=");Serial.println(m1,1);

    } else Serial.println("AUTOCH4: angles outside CH4 limits");

  }

  else if (cmd=="AUTOCHECK") {

    float goals[4][3];

    bool valid=autoXYZTargets(goals);

    Serial.print("AUTOCH3: ");

    if(autoWristConfigured) {Serial.print(autoWristPick,1);Serial.print(" / ");Serial.println(autoWristPlace,1);}

    else Serial.println("NOT CONFIGURED");

    Serial.print("AUTOCH4: ");

    if(openConfigured&&closeConfigured) {Serial.print(gripOpenAngle,1);Serial.print(" / ");Serial.println(gripCloseAngle,1);}

    else Serial.println("NOT CONFIGURED");

    if(valid) Serial.println("AUTOCHECK: mathematical endpoints only; path and clearances unverified");

  }

  else if (cmd=="AUTOXYZ") {

    autoXYZCycle();

  }

  else if (sscanf(cmd.c_str(), "TESTXYZ %f %f %f", &m0,&m1,&m2)==3) {

    geometryTestXYZ(m0,m1,m2);

  }

  else if (cmd=="GEOFKCMD") {

    geometryServoFK();

  }

  else if (sscanf(cmd.c_str(), "GEOMIK %f %f %f", &m0,&m1,&m2)==3) {

    geometryCheck(m0,m1,m2);

  }

  else if (sscanf(cmd.c_str(), "GEOMFK %f %f %f", &m0,&m1,&m2)==3) {

    geometryFK(m0,m1,m2);

  }

  else if (sscanf(cmd.c_str(), "GEOOFFSET %f %f", &m0,&m1)==2) {

    if(isfinite(m0)&&isfinite(m1)&&fabsf(m0)<=250&&fabsf(m1)<=250) {

      radialOffsetMM=m0; shoulderHeightMM=m1; geomCalibrated=false;

      Serial.println("Offsets updated (UNVERIFIED physical values)");

    } else Serial.println("Invalid offset");

  }

  else if (cmd=="LINKS") {

    Serial.print("L1=L2 (mm): ");Serial.println(LINK_MM,2);

    Serial.print("Radial offset (mm): ");Serial.println(radialOffsetMM,2);

    Serial.print("Shoulder height (mm): ");Serial.println(shoulderHeightMM,2);

    Serial.println("Local servo calibration: "); Serial.println(geomCalibrated ? "YES (approximate)" : "NO");

  }

  else if (cmd.startsWith("FPOSE") || cmd.startsWith("FSET") ||

           cmd.startsWith("FCHECK") || cmd.startsWith("IKTABLE") ||

           cmd.startsWith("CHECKYZ") || cmd.startsWith("MOVEYZ") ||

           cmd.startsWith("PATHYZ") || cmd.startsWith("PATHCHECK") ||

           cmd.startsWith("ANCHORYZ") || cmd.startsWith("CURRENTYZ") ||

           cmd.startsWith("INVERSEDEBUG")) {

    Serial.println("Old Fusion/table IK command removed. Use GEOMIK / GEOMFK (NO PWM).");

  }

  else   if (sscanf(cmd.c_str(), "SET %d %f", &ch, &angle) == 2) {

    setServoAngle(ch, angle);

  }

  else if (sscanf(cmd.c_str(), "SAVE %d", &p) == 1) {

    savePose(p);

  }

  else if (sscanf(cmd.c_str(), "GO %d", &p) == 1) {

    if (p >= 0 && p < POSE_COUNT) {

      moveToPose(p, poseDuration, true);

    } else {

      Serial.println("Pose khong hop le.");

    }

  }

  else if (sscanf(cmd.c_str(), "GRIPOPEN %f", &angle) == 1) {

    if (angle < minAngle[GRIP_INDEX] ||

        angle > maxAngle[GRIP_INDEX]) {

      Serial.println("Goc kep ngoai gioi han.");

      return;

    }

    gripOpenAngle = angle;

    openConfigured = true;

    Serial.println("DA CAU HINH GOC MO KEP");

  }

  else if (sscanf(cmd.c_str(), "GRIPCLOSE %f", &angle) == 1) {

    if (angle < minAngle[GRIP_INDEX] ||

        angle > maxAngle[GRIP_INDEX]) {

      Serial.println("Goc kep ngoai gioi han.");

      return;

    }

    gripCloseAngle = angle;

    closeConfigured = true;

    Serial.println("DA CAU HINH GOC DONG KEP");

  }

  else if (cmd == "OPEN") {

    openGripper();

  }

  else if (cmd == "CLOSE") {

    closeGripper();

  }

  else if (cmd == "AUTO") {

    autoPickPlace();

  }

  else if (sscanf(cmd.c_str(), "SPEED %d", &value) == 1) {

    if (value >= 500 && value <= 10000) {

      poseDuration = value;

      Serial.print("GO/AUTO TIME = ");

      Serial.println(poseDuration);

    } else {

      Serial.println("SPEED: 500..10000 ms");

    }

  }

  else if (sscanf(cmd.c_str(), "SETSPEED %d", &value) == 1) {

    if (value >= 500 && value <= 10000) {

      setDuration = value;

      Serial.print("SET TIME = ");

      Serial.println(setDuration);

    } else {

      Serial.println("SETSPEED: 500..10000 ms");

    }

  }

  else if (sscanf(cmd.c_str(), "GRIPSPEED %d", &value) == 1) {

    if (value >= 500 && value <= 10000) {

      gripDuration = value;

      Serial.print("GRIP TIME = ");

      Serial.println(gripDuration);

    } else {

      Serial.println("GRIPSPEED: 500..10000 ms");

    }

  }

  else if (cmd == "SMOOTH ON") {

    smoothMode = true;

    Serial.println("QUINTIC ON");

  }

  else if (cmd == "SMOOTH OFF") {

    smoothMode = false;

    Serial.println("CUBIC ON");

  }

  else if (cmd == "STATUS") {

    showStatus();

  }

  else if (cmd == "STOP") {

    Serial.println("Robot khong dang chuyen dong.");

  }

  else if (cmd == "HELP") {

    showHelp();

  }

  else {

    Serial.println("LENH KHONG HOP LE. NHAP HELP.");

  }

}

// ================= SETUP =================

void setup() {

  Serial.begin(115200);

  delay(350);

  Serial.println("\nBOOT: GEOMETRIC IK - SERIAL READY");

  Serial.println("Try HELP, LINKS, GEOMIK 100 100 50");

  // I2C is initialized after the serial banner so a hardware problem

  // cannot conceal the fact that the firmware has started.

  Serial.println("INIT: Wire.begin");

  Serial.flush();

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.setClock(100000);

  Wire.setTimeOut(50);

  Serial.println("INIT: pwm.begin");

  Serial.flush();

  pwm.begin();

  Serial.println("INIT: pwm.begin returned");

  pwm.setPWMFreq(SERVO_FREQ);

  delay(100);

  analogReadResolution(12);
  for (int i = 0; i < 3; i++) {
    pinMode(joyButtonPins[i], INPUT_PULLUP);
    joyButtonStable[i] = digitalRead(joyButtonPins[i]);
    joyButtonRaw[i] = joyButtonStable[i];
    joyButtonChanged[i] = millis();
  }
  joyLastTick = millis();
  Serial.println("3 JOYSTICK: OFF by default; SW3(GPIO27) toggles VOICE ON/OFF over Serial");

  // No automatic servo position commands at boot.

  setupWeb();

  Serial.println("READY: GEOMETRIC IK/FK (NO PWM) + DIRECT SERVO + WEB");

}

// ================= LOOP =================

// Accept CR, LF, CRLF, and a line ending at either character.

// Echo accepted commands to make serial debugging easier.

void loop() {

  webServer.handleClient();
  joystickTick();
  // Gui snapshot lan dau va khi joystick/Web/Serial da thay doi goc.
  sendAnglesIfChanged();

  while (Serial.available()) {

    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {

      if (inputLine.length() > 0) {

        String cmd = inputLine;

        inputLine = "";

        Serial.print("RX: ");

        Serial.println(cmd);

        processCommand(cmd);

      }

    } else if (c >= 32 && c <= 126) {

      if (inputLine.length() < 80) {

        inputLine += c;

      } else {

        inputLine = "";

        Serial.println("ERROR: COMMAND TOO LONG");

      }

    }

  }

  sendAnglesIfChanged(); // Bao cao 1 lan sau moi dot thay doi (gom 5 kenh).
}
