#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>
#include <string.h>
#include <ctype.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec2 { float x, y; };
struct Vec3 { float x, y, z; };

struct LegParams {
  float l1 = 22.9f;
  float l2 = 130.0f;
  float l3 = 138.01f;
  float d1 = 44.2f;
  float d2 = 5.97f;
  float baoMin = -200.0f;
  float baoMax = -90.0f;
  float LAB = 130.0f;
  float LBC = 140.0f;
  float LBD = 37.0f;
  float LED = 115.0f;
  float LAE = 43.0f;
  float LAF = 43.0f;
  float LEF = 67.07f;
  float dGx = -20.18f;
  float dGy = -29.05f;
  float LGH = 38.0f;
  float LHF = 40.0f;
  int lockFSign = -1;
  int lockESign = -1;
  int lockDSign = -1;
};

struct RawIK { float j1, j2, j3; bool valid; };
struct ServoOut { float hip, bao, hgo; bool valid; };

struct LegState {
  Vec3 pos{0,0,0};
  float hip = 0.0f;
  float bao = 0.0f;
  float abd = 0.0f;
  float lastHgo = 90.0f;
};

struct PoseSet { Vec3 FL; Vec3 FR; Vec3 RL; Vec3 RR; };

struct LateralConfig {
  float flStartY; float flEndY;
  float frStartY; float frEndY;
  float rlStartY; float rlEndY;
  float rrStartY; float rrEndY;
};

struct GaitRuntimeConfig {
  float hStep;
  float tCycle;
  float phaseOffset;
  float swingRatio;
};

enum GaitMode {
  GAIT_IDLE = 0,
  GAIT_STAMP,
  GAIT_FORWARD,
  GAIT_BACKWARD,
  GAIT_LEFT,
  GAIT_RIGHT,
  GAIT_SIDE_LEFT,
  GAIT_SIDE_RIGHT
};

class QuadrupedController {
public:
  void begin() {
    Serial.begin(115200);
    Serial.setRxBufferSize(512);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(50);

    initMPU();

    pca.begin();
    pca.setPWMFreq(SERVO_FREQ);
    delay(1500);

    // Khi bật nguồn: chỉ đưa servo về NEUTRAL/NATURAL = 90° + offset.
    neutralPose();
    bootNeutralMode = true;
    idleBaseValid = false;
    delay(500);

    Serial.println("[BOOT] QUADRUPED_GAIT_TUNABLE_FULL | NEUTRAL ONLY. Gui STAND de vao IK pose.");
    printHelp();
  }

  void update() {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r' || c == '\n') {
        uartLine.trim();
        if (uartLine.length() > 0) handleCommand(uartLine);
        uartLine = "";
      } else {
        if (uartLine.length() < 240) uartLine += c;
      }
    }

    mpuTick();
    camSafetyTick();
    gaitTick();
    standBalanceTick();
    balanceOffsetPrintTick();
    imuIk12SerialTick();
  }

private:
  static constexpr uint8_t I2C_SDA = 21;
  static constexpr uint8_t I2C_SCL = 22;
  static constexpr uint16_t SERVO_FREQ = 50;
  static constexpr uint16_t US_MIN = 600;
  static constexpr uint16_t US_MAX = 2400;

  static constexpr uint8_t FL_HGO_CH = 0;
  static constexpr uint8_t FL_BAO_CH = 1;
  static constexpr uint8_t FL_HIP_CH = 2;
  static constexpr uint8_t FR_HGO_CH = 3;
  static constexpr uint8_t FR_BAO_CH = 4;
  static constexpr uint8_t FR_HIP_CH = 5;
  static constexpr uint8_t RL_HGO_CH = 6;
  static constexpr uint8_t RL_BAO_CH = 7;
  static constexpr uint8_t RL_HIP_CH = 8;
  static constexpr uint8_t RR_HGO_CH = 9;
  static constexpr uint8_t RR_BAO_CH = 10;
  static constexpr uint8_t RR_HIP_CH = 11;

  static constexpr unsigned long GAIT_PERIOD_MS = 30;

  Adafruit_PWMServoDriver pca{0x40};
  MPU6050 mpu{0x68};
  String uartLine;

  LegParams params;
  LegState FLs, FRs, RLs, RRs;

  // ===== MPU CONFIG =====
  static constexpr int MPU_AVG_SAMPLES = 20;
  float mpuAlphaGait = 0.40f;
  float mpuAlphaStand = 0.10f;
  static constexpr unsigned long MPU_READ_PERIOD_MS = 30;
  static constexpr unsigned long MPU_PRINT_PERIOD_MS = 30;

  int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  int16_t axBuf[MPU_AVG_SAMPLES] = {0};
  int16_t ayBuf[MPU_AVG_SAMPLES] = {0};
  int16_t azBuf[MPU_AVG_SAMPLES] = {0};
  int mpuIndexBuf = 0;
  bool mpuFilled = false;
  bool mpuFirstRun = true;
  bool mpuOK = false;
  float axAvg=0, ayAvg=0, azAvg=0;
  float axLpf=0, ayLpf=0, azLpf=0;
  float axOffset=0, ayOffset=0, azOffset=0;
  float gxOffset=0, gyOffset=0, gzOffset=0;
  float rollDeg=0, pitchDeg=0, yawDeg=0;
  float gzDpsLpf = 0.0f;
  float gyroZAlpha = 0.20f;
  float yawDeadbandDps = 0.45f;
  float yawStillDps = 1.20f;
  unsigned long mpuLastReadMs = 0;
  unsigned long mpuLastPrintMs = 0;
  bool mpuPrintEnable = true;
  bool mpuPlotMode = true;

  // ===== SERVO OFFSETS =====
  float FL_HGO_OFF = 20.0f, FL_BAO_OFF = -10.0f, FL_HIP_OFF = -5.0f;
  float RL_HGO_OFF = -10.0f,  RL_BAO_OFF = -16.0f,  RL_HIP_OFF = 10.0f;
  float FR_HGO_OFF = 0.0f,  FR_BAO_OFF = 3.0f,  FR_HIP_OFF = 15.0f;
  float RR_HGO_OFF = -10.0f, RR_BAO_OFF = -0.0f, RR_HIP_OFF = -3.0f;

  // ===== POSES =====
  PoseSet standPose {
    {-210.0f, -55.0f, -10.0f}, {-210.0f, 55.0f, -10.0f},
    {-210.0f, -55.0f, 20.0f}, {-210.0f, 55.0f, 20.0f}
  };
  PoseSet sitPose {
    {-230.0f, -50.0f, 40.0f}, {-230.0f, 50.0f, 40.0f},
    {-120.0f, -100.0f, -10.0f}, {-120.0f, 100.0f, -10.0f}
  };
  PoseSet shakeSitPose {
    {-225.0f, -55.0f, 35.0f}, {-230.0f, 0.0f, 40.0f},
    {-120.0f, -100.0f, -10.0f}, {-120.0f, 100.0f, -10.0f}
  };
  PoseSet liePose {
    {-80.0f, -100.0f, -10.0f}, {-80.0f, 100.0f, -10.0f},
    {-80.0f, -100.0f, -10.0f}, {-80.0f, 100.0f, -10.0f}
  };
  PoseSet stampCenter {
    {-210.0f, -55.0f, 0.0f}, {-210.0f, 55.0f, 0.0f},
    {-210.0f, -55.0f, 20.0f}, {-210.0f, 55.0f, 20.0f}
  };
  PoseSet forwardCenter {
    {-197.0f, -55.0f, -0.0f}, {-210.0f, 55.0f, -10.0f},
    {-197.0f, -55.0f, 20.0f}, {-210.0f, 55.0f, 20.0f}
  };
  PoseSet backwardCenter {
    {-201.0f, -55.0f, -0.0f}, {-210.0f, 55.0f, -0.0f},
    {-201.0f, -55.0f, 20.0f}, {-210.0f, 55.0f, 20.0f}
  };
  PoseSet leftCenter {
    {-210.0f, -0.0f, 0.0f}, {-210.0f, 100.0f, 0.0f},
    {-210.0f, -0.0f, 20.0f}, {-210.0f, 100.0f, 20.0f}
  };
  PoseSet rightCenter {
    {-210.0f, -100.0f, 0.0f}, {-210.0f, 0.0f, 0.0f},
    {-210.0f, -100.0f, 20.0f}, {-210.0f, 0.0f, 20.0f}
  };
  PoseSet sideCenter {
    {-202.0f, -55.0f, -0.0f}, {-210.0f, 55.0f, -10.0f},
    {-202.0f, -55.0f, 20.0f}, {-210.0f, 55.0f, 20.0f}
  };

  LateralConfig leftLateral  {-70.0f, -0.0f,  40.0f,  110.0f, -0.0f, -70.0f,  110.0f,  40.0f};
  LateralConfig rightLateral {-40.0f, -110.0f, 70.0f,  0.0f, -110.0f, -40.0f, 0.0f,  70.0f};

  // ===== LEG AMPLITUDES =====
  float gaitMoveAmpFL = 35.0f, gaitMoveAmpFR = 40.0f, gaitMoveAmpRL = 40.0f, gaitMoveAmpRR = 40.0f;
  float gaitSideAmpFL = 40.0f, gaitSideAmpFR = 40.0f, gaitSideAmpRL = 30.0f, gaitSideAmpRR = 30.0f;
  bool gaitSideInvert = false;

  // ===== PER-GAIT RUNTIME CONFIG =====
  // hStep      : do nhac chan theo truc X khi swing.
  // tCycle     : chu ky buoc, giay.
  // phaseOffset: lech pha nhom B = FR+RL so voi nhom A = FL+RR, chuan 0..1.
  // swingRatio : ti le thoi gian swing, giu 0.05..0.49.
  GaitRuntimeConfig gaitFwdCfg   {100.0f, 0.70f, 0.50f, 0.35f};
  GaitRuntimeConfig gaitBwdCfg   {100.0f, 0.70f, 0.50f, 0.35f};
  GaitRuntimeConfig gaitLeftCfg  { 50.0f, 0.40f, 0.50f, 0.49f};
  GaitRuntimeConfig gaitRightCfg { 50.0f, 0.40f, 0.50f, 0.49f};
  GaitRuntimeConfig gaitSLCfg    { 50.0f, 0.70f, 0.50f, 0.35f};
  GaitRuntimeConfig gaitSRCfg    { 50.0f, 0.70f, 0.50f, 0.35f};

  // Legacy variables kept for old GUI/bridge commands.
  float gaitHStepFB = 100.0f;
  float gaitTCycleFB = 0.7f;
  float gaitSwingRatioFB = 0.35f;
  float gaitHStepLR = 50.0f;
  float gaitTCycleLR = 0.4f;
  float gaitHStepSide = 50.0f;
  float gaitTCycleSide = 0.7f;
  float gaitSwingRatioSide = 0.35f;
  float gaitSidePhaseOffset = 0.5f;

  float poseTransitionSec = 1.2f;
  GaitMode gaitMode = GAIT_IDLE;
  unsigned long gaitStartMs = 0;
  unsigned long gaitLastMs = 0;

  // ===== GAIT X HEIGHT OFFSET CONTROL =====
  float gaitXStep = 10.0f;
  float gaitXMin = -240.0f;
  float gaitXMax = -140.0f;

  // ===== SHAKE =====
  int shakeCount = 6;
  float shakeLowX = 40.0f;
  float shakeHighX = 120.0f;
  float shakeLowZ = -80.0f;
  float shakeHighZ = -120.0f;
  float shakeYOffset = 20.0f;
  float shakeStepSec = 0.28f;

  // ===== BALANCE PID =====
  bool balanceEnable = false;
  float balKp = 0.70f, balKi = 0.00f, balKd = 0.10f;
  float balMaxX = 80.0f;
  float balDeadbandDeg = 1.0f;
  float balResetErrDeg = 0.3f;
  float balIMax = 80.0f;
  float balRollIntegral = 0.0f, balPitchIntegral = 0.0f;
  float balLastRollErr = 0.0f, balLastPitchErr = 0.0f;
  float balLastRollOut = 0.0f, balLastPitchOut = 0.0f;
  unsigned long balLastPidMs = 0;

  bool standBalanceEnable = false;
  bool disablePidForSpecialPose = false;
  float standBalRollSetpointDeg = 0.0f;
  float standBalPitchSetpointDeg = 0.0f;
  float standBalKp = 1.4f, standBalKi = 4.30f, standBalKd = 0.02f;
  float standBalMaxX = 80.0f;
  float standBalDeadbandDeg = 0.5f;
  float standBalResetErrDeg = 0.2f;
  float standBalIMax = 80.0f;
  float standBalRollIntegral = 0.0f, standBalPitchIntegral = 0.0f;
  float standBalLastRollErr = 0.0f, standBalLastPitchErr = 0.0f;
  float standBalLastRollOut = 0.0f, standBalLastPitchOut = 0.0f;
  unsigned long standBalLastPidMs = 0;
  unsigned long standBalLastTickMs = 0;
  PoseSet idleBasePose;
  bool idleBaseValid = false;
  bool bootNeutralMode = true;

  bool balanceOffsetPrintEnable = false;
  unsigned long balanceOffsetLastPrintMs = 0;
  static constexpr unsigned long BAL_OFFSET_PRINT_PERIOD_MS = 100;
  float lastDxFL = 0.0f, lastDxFR = 0.0f, lastDxRL = 0.0f, lastDxRR = 0.0f;
  float lastRollPidOut = 0.0f, lastPitchPidOut = 0.0f;

  // ===== YAW GAIT PID =====
  bool yawGaitOnlyEnable = false;
  bool yawGaitInvertSide = true;
  float yawGaitSetpointDeg = 0.0f;
  float yawGaitKp = 4.0f, yawGaitKi = 0.0f, yawGaitKd = 0.10f;
  float yawGaitMaxAmp = 100.0f;
  float yawGaitDeadbandDeg = 5.0f;
  float yawGaitHoldDeg = 0.3f;
  float yawGaitIMax = 30.0f;
  float yawGaitIntegral = 0.0f, yawGaitLastErr = 0.0f, yawGaitLastOut = 0.0f;
  unsigned long yawGaitLastPidMs = 0;

  // ===== CAMERA PERSON TRACKING PID =====
  bool camGaitEnable = false;
  bool camRequireOffsetBeforeWalk = false;
  bool camGaitInvertSide = true;
  int camDetected = 0;
  float camOffsetX = 0.0f;
  int camBoxW = 0, camBoxH = 0, camCenterX = 0, camCenterY = 0;
  unsigned long camLastRxMs = 0;
  unsigned long camLastPrintMs = 0;
  float camTargetOffsetPx = 0.0f;
  float camKp = 0.10f, camKi = 0.00f, camKd = 0.04f;
  float camMaxAmp = 35.0f;
  float camMinAmp = 15.0f;
  float camDeadbandPx = 70.0f;
  float camHoldPx = 6.0f;
  float camIMax = 120.0f;
  float camIntegral = 0.0f, camLastErr = 0.0f, camLastOut = 0.0f;
  unsigned long camLastPidMs = 0;
  int camStopBoxH = 500;
  unsigned long camLostStopMs = 800;
  bool camAutoPaused = false;
  unsigned long camLastPausePrintMs = 0;
  bool camSeenSinceEnable = false;
  bool camAlignRotateEnable = true;
  bool camAlignRotateInvert = false;
  float camAlignRangePx = 200.0f;
  unsigned long camAlignLastPrintMs = 0;

  // ===== IK12 SERIAL OUTPUT =====
  bool ik12SerialEnable = true;
  unsigned long ik12LastPrintMs = 0;
  static constexpr unsigned long IK12_PRINT_PERIOD_MS = 30;

  // ===== BASIC MATH =====
  static float clampF(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
  static float r2d(float r) { return r * 180.0f / (float)M_PI; }
  static float d2r(float d) { return d * (float)M_PI / 180.0f; }
  static float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }
  static float smoothStep01(float t) {
    t = clampF(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
  }
  static Vec3 lerpVec3(Vec3 a, Vec3 b, float t) {
    t = clampF(t, 0.0f, 1.0f);
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
  }
  static Vec2 v2add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
  static Vec2 v2sub(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
  static Vec2 v2mul(Vec2 a, float k) { return {a.x * k, a.y * k}; }
  static float v2dot(Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }
  static float v2norm(Vec2 a) { return sqrtf(v2dot(a,a)); }
  static Vec2 v2unit(Vec2 a) { float n = v2norm(a); return (n < 1e-9f) ? Vec2{0,0} : Vec2{a.x/n, a.y/n}; }
  static float v2cross(Vec2 a, Vec2 b) { return a.x*b.y - a.y*b.x; }

  // ===== MPU =====
  float calcAvg16(int16_t *buf, int n) {
    long sum = 0;
    for (int i = 0; i < n; i++) sum += buf[i];
    return (float)sum / (float)n;
  }
  float getCurrentMpuAlpha() { return (gaitMode == GAIT_IDLE) ? mpuAlphaStand : mpuAlphaGait; }
  float mpuLpf(float input, float prev) {
    float a = clampF(getCurrentMpuAlpha(), 0.0f, 1.0f);
    return a * input + (1.0f - a) * prev;
  }
  void initMPU() {
    mpu.initialize();
    if (!mpu.testConnection()) {
      mpuOK = false;
      Serial.println("[MPU] LOI ket noi! Kiem tra dia chi 0x68, SDA=21, SCL=22.");
      return;
    }
    mpuOK = true;
    Serial.println("[MPU] OK 0x68");
    calibrateAccelNonFatal();
  }
  void calibrateAccelNonFatal() {
    if (!mpuOK) return;
    long sumAx = 0, sumAy = 0, sumAz = 0;
    long sumGx = 0, sumGy = 0, sumGz = 0;
    const int samples = 700;
    Serial.println("[MPU] Dang calibrate accel + gyro, dat robot nam yen...");
    for (int i = 0; i < samples; i++) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      sumAx += ax; sumAy += ay; sumAz += az;
      sumGx += gx; sumGy += gy; sumGz += gz;
      delay(3);
    }
    axOffset = (float)sumAx / samples;
    ayOffset = (float)sumAy / samples;
    azOffset = (float)sumAz / samples - 16384.0f;
    gxOffset = (float)sumGx / samples;
    gyOffset = (float)sumGy / samples;
    gzOffset = (float)sumGz / samples;
    yawDeg = 0.0f;
    gzDpsLpf = 0.0f;
    Serial.printf("[MPU] Calib xong | accOff ax=%.1f ay=%.1f az=%.1f | gyroOff gx=%.1f gy=%.1f gz=%.1f | yaw=0\n",
                  axOffset, ayOffset, azOffset, gxOffset, gyOffset, gzOffset);
  }
  void mpuTick() {
    if (!mpuOK) return;
    unsigned long now = millis();
    if (now - mpuLastReadMs < MPU_READ_PERIOD_MS) return;
    float dt = (mpuLastReadMs == 0) ? (MPU_READ_PERIOD_MS * 0.001f) : ((now - mpuLastReadMs) * 0.001f);
    dt = clampF(dt, 0.005f, 0.08f);
    mpuLastReadMs = now;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float axCorrected = (float)ax - axOffset;
    float ayCorrected = (float)ay - ayOffset;
    float azCorrected = (float)az - azOffset;

    axBuf[mpuIndexBuf] = (int16_t)axCorrected;
    ayBuf[mpuIndexBuf] = (int16_t)ayCorrected;
    azBuf[mpuIndexBuf] = (int16_t)azCorrected;
    mpuIndexBuf++;
    if (mpuIndexBuf >= MPU_AVG_SAMPLES) { mpuIndexBuf = 0; mpuFilled = true; }
    int n = mpuFilled ? MPU_AVG_SAMPLES : mpuIndexBuf;
    if (n <= 0) n = 1;
    axAvg = calcAvg16(axBuf, n);
    ayAvg = calcAvg16(ayBuf, n);
    azAvg = calcAvg16(azBuf, n);

    if (mpuFirstRun) {
      axLpf = axAvg; ayLpf = ayAvg; azLpf = azAvg; mpuFirstRun = false;
    } else {
      axLpf = mpuLpf(axAvg, axLpf);
      ayLpf = mpuLpf(ayAvg, ayLpf);
      azLpf = mpuLpf(azAvg, azLpf);
    }

    rollDeg  = atan2f(ayLpf, sqrtf(axLpf * axLpf + azLpf * azLpf)) * 180.0f / (float)M_PI;
    pitchDeg = atan2f(-axLpf, sqrtf(ayLpf * ayLpf + azLpf * azLpf)) * 180.0f / (float)M_PI;

    float gxDps = ((float)gx - gxOffset) / 131.0f;
    float gyDps = ((float)gy - gyOffset) / 131.0f;
    float gzDps = ((float)gz - gzOffset) / 131.0f;
    bool gyroStill = (fabsf(gxDps) < yawStillDps && fabsf(gyDps) < yawStillDps && fabsf(gzDps) < yawStillDps);
    if (fabsf(gzDps) < yawDeadbandDps || gyroStill) gzDps = 0.0f;
    float za = clampF(gyroZAlpha, 0.0f, 1.0f);
    gzDpsLpf = za * gzDps + (1.0f - za) * gzDpsLpf;
    yawDeg += gzDpsLpf * dt;
    yawDeg = wrap180(yawDeg);

    if (mpuPrintEnable && now - mpuLastPrintMs >= MPU_PRINT_PERIOD_MS) {
      mpuLastPrintMs = now;
      if (mpuPlotMode) Serial.printf("roll:%.2f pitch:%.2f yaw:%.2f max:20 min:-20\n", rollDeg, pitchDeg, yawDeg);
      else Serial.printf("[MPU] roll=%.2f pitch=%.2f yaw=%.2f gz=%.2f dps\n", rollDeg, pitchDeg, yawDeg, gzDpsLpf);
    }
  }

  // ===== SERVO =====
  uint16_t usToTick(uint16_t us) const {
    uint16_t t = (uint16_t)((4096.0f * us) / (1000000.0f / SERVO_FREQ) + 0.5f);
    return t > 4095 ? 4095 : t;
  }
  void setUS(uint8_t ch, uint16_t us) { pca.setPWM(ch, 0, usToTick(us)); }
  void setAngle(uint8_t ch, float ang) {
    ang = clampF(ang, 0.0f, 180.0f);
    uint16_t us = (uint16_t)lroundf(US_MIN + (US_MAX - US_MIN) * (ang / 180.0f));
    setUS(ch, us);
  }
  void applyFLServo(float hip, float bao, float hgo) {
    setAngle(FL_HIP_CH, clampF(90.0f + hip + FL_HIP_OFF, 0.0f, 180.0f));
    setAngle(FL_HGO_CH, clampF(hgo + FL_HGO_OFF, 0.0f, 180.0f));
    setAngle(FL_BAO_CH, clampF(bao + 180.0f + FL_BAO_OFF, 0.0f, 180.0f));
  }
  void applyRLServo(float hip, float bao, float hgo) {
    setAngle(RL_HIP_CH, clampF(90.0f - hip + RL_HIP_OFF, 0.0f, 180.0f));
    setAngle(RL_HGO_CH, clampF(hgo + RL_HGO_OFF, 0.0f, 180.0f));
    setAngle(RL_BAO_CH, clampF(bao + 180.0f + RL_BAO_OFF, 0.0f, 180.0f));
  }
  void applyFRServo(float hip, float bao, float hgo) {
    setAngle(FR_HIP_CH, clampF(90.0f + hip + FR_HIP_OFF, 0.0f, 180.0f));
    setAngle(FR_BAO_CH, clampF(bao + FR_BAO_OFF, 0.0f, 180.0f));
    setAngle(FR_HGO_CH, clampF(180.0f - hgo + FR_HGO_OFF, 0.0f, 180.0f));
  }
  void applyRRServo(float hip, float bao, float hgo) {
    setAngle(RR_HIP_CH, clampF(90.0f - hip + RR_HIP_OFF, 0.0f, 180.0f));
    setAngle(RR_BAO_CH, clampF(bao + RR_BAO_OFF, 0.0f, 180.0f));
    setAngle(RR_HGO_CH, clampF(180.0f - hgo + RR_HGO_OFF, 0.0f, 180.0f));
  }
  void neutralPose() {
    setAngle(FL_HGO_CH, clampF(90 + FL_HGO_OFF, 0, 180));
    setAngle(FL_BAO_CH, clampF(90 + FL_BAO_OFF, 0, 180));
    setAngle(FL_HIP_CH, clampF(90 + FL_HIP_OFF, 0, 180));
    setAngle(RL_HGO_CH, clampF(90 + RL_HGO_OFF, 0, 180));
    setAngle(RL_BAO_CH, clampF(90 + RL_BAO_OFF, 0, 180));
    setAngle(RL_HIP_CH, clampF(90 + RL_HIP_OFF, 0, 180));
    setAngle(FR_HGO_CH, clampF(90 + FR_HGO_OFF, 0, 180));
    setAngle(FR_BAO_CH, clampF(90 + FR_BAO_OFF, 0, 180));
    setAngle(FR_HIP_CH, clampF(90 + FR_HIP_OFF, 0, 180));
    setAngle(RR_HGO_CH, clampF(90 + RR_HGO_OFF, 0, 180));
    setAngle(RR_BAO_CH, clampF(90 + RR_BAO_OFF, 0, 180));
    setAngle(RR_HIP_CH, clampF(90 + RR_HIP_OFF, 0, 180));
  }

  // ===== IK SOLVER =====
  bool circCirc(Vec2 O1, float r1, Vec2 O2, float r2, Vec2 &P1, Vec2 &P2) {
    Vec2 dV = v2sub(O2, O1);
    float d = v2norm(dV);
    if (d < 1e-7f || d > r1 + r2 + 1e-5f || d < fabsf(r1 - r2) - 1e-5f) return false;
    float a = (r1*r1 - r2*r2 + d*d) / (2.0f*d);
    float h2 = r1*r1 - a*a;
    if (h2 < -1e-6f) return false;
    float h = sqrtf(fmaxf(0.0f, h2));
    Vec2 u = v2mul(dV, 1.0f / d);
    Vec2 Pm = v2add(O1, v2mul(u, a));
    Vec2 perp{-u.y, u.x};
    P1 = v2add(Pm, v2mul(perp, h));
    P2 = v2sub(Pm, v2mul(perp, h));
    return true;
  }
  Vec2 pickSign(Vec2 P1, Vec2 P2, Vec2 o, Vec2 r, int s) {
    float c1 = v2cross(r, v2sub(P1, o));
    float c2 = v2cross(r, v2sub(P2, o));
    if (s > 0) return (c1 >= c2) ? P1 : P2;
    return (c1 <= c2) ? P1 : P2;
  }
  bool forwardABD(float baoNeg, float hgo, float &abdOut) {
    Vec2 A{0,0}, G{params.dGx, params.dGy};
    if (params.LEF > 2.0f * params.LAE) return false;
    float phi = 2.0f * asinf(params.LEF / (2.0f * params.LAE));
    float theta = d2r(baoNeg + 90.0f);
    float gamma = d2r(hgo + 90.0f);
    Vec2 B = v2add(A, {params.LAB*cosf(theta), params.LAB*sinf(theta)});
    Vec2 H = v2add(G, {params.LGH*cosf(gamma), params.LGH*sinf(gamma)});
    Vec2 F1, F2; if (!circCirc(A, params.LAF, H, params.LHF, F1, F2)) return false;
    Vec2 F = pickSign(F1, F2, A, v2sub(H, A), params.lockFSign);
    float angF = atan2f(F.y, F.x);
    Vec2 E1 = v2add(A, {params.LAE*cosf(angF+phi), params.LAE*sinf(angF+phi)});
    Vec2 E2 = v2add(A, {params.LAE*cosf(angF-phi), params.LAE*sinf(angF-phi)});
    Vec2 E = pickSign(E1, E2, A, v2sub(F, A), params.lockESign);
    Vec2 D1, D2; if (!circCirc(E, params.LED, B, params.LBD, D1, D2)) return false;
    Vec2 D = pickSign(D1, D2, B, v2sub(E, B), params.lockDSign);
    Vec2 uBA = v2unit(v2sub(A, B));
    Vec2 uBD = v2unit(v2sub(D, B));
    if (v2norm(uBA) < 1e-6f || v2norm(uBD) < 1e-6f) return false;
    float c = clampF(v2dot(uBA, uBD), -1.0f, 1.0f);
    abdOut = -r2d(acosf(c));
    return true;
  }
  bool solveHGO(float baoNeg, float abdNeg, float &hgo) {
    const int N = 80, IT_MAX = 28;
    float bestH = 90.0f, bestAbs = 1e9f;
    float hPrev = 0.0f, ePrev = 0.0f;
    bool okPrev = false;
    float abd;
    if (forwardABD(baoNeg, 0.0f, abd)) {
      ePrev = wrap180(abd - abdNeg);
      bestAbs = fabsf(ePrev);
      bestH = 0.0f;
      okPrev = true;
    }
    bool found = false;
    float a=0,b=0,ea=0;
    for (int i=1;i<=N;i++) {
      float h = 180.0f * i / N;
      if (!forwardABD(baoNeg, h, abd)) { okPrev = false; continue; }
      float e = wrap180(abd - abdNeg);
      float ae = fabsf(e);
      if (ae < bestAbs) { bestAbs = ae; bestH = h; }
      if (okPrev && ((ePrev <= 0 && e >= 0) || (ePrev >= 0 && e <= 0))) {
        a = hPrev; ea = ePrev; b = h; found = true; break;
      }
      hPrev = h; ePrev = e; okPrev = true;
    }
    if (!found) { hgo = bestH; return bestAbs < 2.0f; }
    for (int i=0;i<IT_MAX;i++) {
      float m = 0.5f * (a + b);
      if (!forwardABD(baoNeg, m, abd)) break;
      float em = wrap180(abd - abdNeg);
      if ((ea <= 0 && em >= 0) || (ea >= 0 && em <= 0)) b = m;
      else { a = m; ea = em; }
    }
    hgo = 0.5f * (a + b);
    return true;
  }
  RawIK walk2IKEngineRaw(float Px, float Py, float Pz, bool isRight) {
    RawIK out{0,0,0,false};
    float d1Val = isRight ? -params.d1 : params.d1;
    float a1 = Px, b1 = -Py;
    float delta1 = a1*a1 + b1*b1 - d1Val*d1Val;
    if (delta1 < 0.0f) return out;

    float the1 = atan2f(d1Val, -sqrtf(delta1)) - atan2f(b1, a1);
    if (!isRight && Py >= 0.0f) the1 -= 2.0f * (float)M_PI;
    if ( isRight && Py <  0.0f) the1 += 2.0f * (float)M_PI;

    float c1 = cosf(the1), s1 = sinf(the1);
    float termR = Px*c1 + Py*s1;
    float termZ = Pz - params.l1;

    float a3 = -2.0f * params.l2 * params.d2;
    float b3 =  2.0f * params.l2 * params.l3;
    float dVal3 = termR*termR + termZ*termZ - params.l2*params.l2 - params.l3*params.l3 - params.d2*params.d2;
    float delta3 = a3*a3 + b3*b3 - dVal3*dVal3;
    if (delta3 < 0.0f) return out;

    float the3 = atan2f(dVal3, -sqrtf(delta3)) - atan2f(b3, a3);
    float s3 = sinf(the3), c3 = cosf(the3);

    float a2 = params.l2 + params.l3*c3 - params.d2*s3;
    float b2 = params.l3*s3 + params.d2*c3;
    float dVal2 = Pz - params.l1;
    float inside = fmaxf(0.0f, a2*a2 + b2*b2 - dVal2*dVal2);
    float the2 = atan2f(dVal2, -sqrtf(inside)) - atan2f(b2, a2);
    float resThe2 = (Pz <= params.l1) ? (the2 + 2.0f*(float)M_PI) : the2;

    out.j1 = the1;
    out.j2 = d2r(360.0f - r2d(resThe2)) - (float)M_PI;
    float th3Deg = r2d(the3);
    if (th3Deg < -180.0f) th3Deg += 360.0f;
    out.j3 = d2r(-th3Deg) + ((float)M_PI / 2.0f);
    out.valid = true;
    return out;
  }
  ServoOut calcLeftServoFromPos(LegState &leg, Vec3 P) {
    ServoOut out{0,0,0,false};
    RawIK raw = walk2IKEngineRaw(P.x, P.y, P.z, false);
    if (!raw.valid) return out;
    float j1Deg = wrap180(r2d(raw.j1));
    float j2Deg = wrap180(r2d(raw.j2));
    float j3Deg = wrap180(r2d(raw.j3));
    float baoMag = fabsf(180.0f - j2Deg);
    float bao = -baoMag;
    bao = clampF(bao, params.baoMin, params.baoMax);
    float abd = -fabsf(90.0f - j3Deg);
    float hgo; bool ok = solveHGO(bao, abd, hgo);
    if (!ok) hgo = leg.lastHgo;
    hgo = clampF(hgo, 0.0f, 180.0f);
    leg.hip = j1Deg; leg.bao = bao; leg.abd = abd; leg.lastHgo = hgo; leg.pos = P;
    out = {j1Deg, bao, hgo, true};
    return out;
  }
  ServoOut calcRightServoFromPos(LegState &leg, Vec3 P) {
    ServoOut out{0,0,0,false};
    RawIK raw = walk2IKEngineRaw(P.x, P.y, P.z, true);
    if (!raw.valid) return out;
    float j1Deg = wrap180(r2d(raw.j1));
    float j2Deg = wrap180(r2d(raw.j2));
    float j3Deg = wrap180(r2d(raw.j3));
    float bao = fabsf(180.0f - j2Deg);
    float abd = fabsf(90.0f - j3Deg);
    float hgo; bool ok = solveHGO(-bao, -abd, hgo);
    if (!ok) hgo = leg.lastHgo;
    hgo = clampF(hgo, 0.0f, 180.0f);
    leg.hip = j1Deg; leg.bao = bao; leg.abd = abd; leg.lastHgo = hgo; leg.pos = P;
    out = {j1Deg, bao, hgo, true};
    return out;
  }
  bool moveLeg(const char* legName, Vec3 P, bool verbose) {
    if (strcmp(legName, "FL") == 0) {
      ServoOut s = calcLeftServoFromPos(FLs, P); if (!s.valid) return false; applyFLServo(s.hip, s.bao, s.hgo);
      if (verbose) Serial.printf("[FL] P=(%.2f,%.2f,%.2f) -> hip=%.2f bao=%.2f hgo=%.2f\n", P.x,P.y,P.z,s.hip,s.bao,s.hgo);
      return true;
    }
    if (strcmp(legName, "FR") == 0) {
      ServoOut s = calcRightServoFromPos(FRs, P); if (!s.valid) return false; applyFRServo(s.hip, s.bao, s.hgo);
      if (verbose) Serial.printf("[FR] P=(%.2f,%.2f,%.2f) -> hip=%.2f bao=%.2f hgo=%.2f\n", P.x,P.y,P.z,s.hip,s.bao,s.hgo);
      return true;
    }
    if (strcmp(legName, "RL") == 0) {
      ServoOut s = calcLeftServoFromPos(RLs, P); if (!s.valid) return false; applyRLServo(s.hip, s.bao, s.hgo);
      if (verbose) Serial.printf("[RL] P=(%.2f,%.2f,%.2f) -> hip=%.2f bao=%.2f hgo=%.2f\n", P.x,P.y,P.z,s.hip,s.bao,s.hgo);
      return true;
    }
    if (strcmp(legName, "RR") == 0) {
      ServoOut s = calcRightServoFromPos(RRs, P); if (!s.valid) return false; applyRRServo(s.hip, s.bao, s.hgo);
      if (verbose) Serial.printf("[RR] P=(%.2f,%.2f,%.2f) -> hip=%.2f bao=%.2f hgo=%.2f\n", P.x,P.y,P.z,s.hip,s.bao,s.hgo);
      return true;
    }
    return false;
  }
  void movePoseSet(const PoseSet &ps, bool verbose) {
    bool ok1 = moveLeg("FL", ps.FL, verbose);
    bool ok2 = moveLeg("FR", ps.FR, verbose);
    bool ok3 = moveLeg("RL", ps.RL, verbose);
    bool ok4 = moveLeg("RR", ps.RR, verbose);
    if (ok1 && ok2 && ok3 && ok4) printIK12RawDegFromPoseSet(ps, false);
    else if (verbose) Serial.println("[IK] movePoseSet failed on at least one leg");
  }

  // ===== POSE / IK12 =====
  PoseSet getCurrentPoseSet() { return {FLs.pos, FRs.pos, RLs.pos, RRs.pos}; }
  void setIdleBasePose(const PoseSet &ps) { idleBasePose = ps; idleBaseValid = true; resetStandBalanceState(true); }
  void setIdleBaseFromCurrent() { setIdleBasePose(getCurrentPoseSet()); }
  PoseSet lerpPoseSet(const PoseSet &a, const PoseSet &b, float t) {
    t = smoothStep01(t);
    return {lerpVec3(a.FL, b.FL, t), lerpVec3(a.FR, b.FR, t), lerpVec3(a.RL, b.RL, t), lerpVec3(a.RR, b.RR, t)};
  }
  PoseSet lerpPoseSetLinear(const PoseSet &a, const PoseSet &b, float t) {
    t = clampF(t, 0.0f, 1.0f);
    return {lerpVec3(a.FL, b.FL, t), lerpVec3(a.FR, b.FR, t), lerpVec3(a.RL, b.RL, t), lerpVec3(a.RR, b.RR, t)};
  }
  void transitionPoseSet(const PoseSet &target, float durationSec, bool verbose=false) {
    stopGait(false);
    PoseSet start = getCurrentPoseSet();
    durationSec = clampF(durationSec, 0.05f, 10.0f);
    int steps = (int)ceilf((durationSec * 1000.0f) / (float)GAIT_PERIOD_MS);
    if (steps < 1) steps = 1;
    for (int i = 0; i <= steps; i++) {
      float t = (float)i / (float)steps;
      PoseSet p = lerpPoseSet(start, target, t);
      movePoseSet(p, verbose);
      delay(GAIT_PERIOD_MS);
    }
    movePoseSet(target, verbose);
    setIdleBasePose(target);
    bootNeutralMode = false;
  }
  void transitionPoseSetFromLinear(const PoseSet &start, const PoseSet &target, float durationSec, bool verbose=false) {
    durationSec = clampF(durationSec, 0.05f, 10.0f);
    int steps = (int)ceilf((durationSec * 1000.0f) / (float)GAIT_PERIOD_MS);
    if (steps < 1) steps = 1;
    for (int i = 0; i <= steps; i++) {
      float t = (float)i / (float)steps;
      PoseSet p = lerpPoseSetLinear(start, target, t);
      movePoseSet(p, verbose);
      delay(GAIT_PERIOD_MS);
    }
    movePoseSet(target, verbose);
    setIdleBasePose(target);
    bootNeutralMode = false;
  }
  PoseSet makeShakePose(bool highWave) {
    PoseSet p = shakeSitPose;
    p.FL.x = shakeSitPose.FL.x + (highWave ? shakeHighX : shakeLowX);
    p.FL.y = shakeSitPose.FL.y + shakeYOffset;
    p.FL.z = shakeSitPose.FL.z + (highWave ? shakeHighZ : shakeLowZ);
    return p;
  }
  void handshakeMode() {
    stopGait(false);
    transitionPoseSet(shakeSitPose, poseTransitionSec, false);
    delay(150);
    PoseSet handUp = makeShakePose(true);
    transitionPoseSetFromLinear(shakeSitPose, handUp, 0.45f, false);
    delay(120);
    PoseSet cur = handUp;
    for (int i = 0; i < shakeCount; i++) {
      PoseSet low = makeShakePose(false);
      transitionPoseSetFromLinear(cur, low, shakeStepSec, false); cur = low;
      PoseSet high = makeShakePose(true);
      transitionPoseSetFromLinear(cur, high, shakeStepSec, false); cur = high;
    }
    transitionPoseSetFromLinear(cur, shakeSitPose, 0.45f, false);
    Serial.println("[OK] HANDSHAKE LEFT / BAT_TAY_TRAI");
  }
  void printIK12RawDegFromPoseSet(const PoseSet &ps, bool force=false) {
    if (!ik12SerialEnable) return;
    unsigned long now = millis();
    if (!force && (now - ik12LastPrintMs < IK12_PRINT_PERIOD_MS)) return;
    ik12LastPrintMs = now;
    RawIK fl = walk2IKEngineRaw(ps.FL.x, ps.FL.y, ps.FL.z, false);
    RawIK fr = walk2IKEngineRaw(ps.FR.x, ps.FR.y, ps.FR.z, true);
    RawIK rl = walk2IKEngineRaw(ps.RL.x, ps.RL.y, ps.RL.z, false);
    RawIK rr = walk2IKEngineRaw(ps.RR.x, ps.RR.y, ps.RR.z, true);
    if (!(fl.valid && fr.valid && rl.valid && rr.valid)) return;
    Serial.printf("IK12,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      r2d(fl.j1), r2d(fl.j2), r2d(fl.j3),
      r2d(fr.j1), r2d(fr.j2), r2d(fr.j3),
      r2d(rl.j1), r2d(rl.j2), r2d(rl.j3),
      r2d(rr.j1), r2d(rr.j2), r2d(rr.j3));
  }
  void setIK12Serial(bool en) { ik12SerialEnable = en; Serial.printf("[IK12] serial output %s\n", ik12SerialEnable ? "ON" : "OFF"); }
  void imuIk12SerialTick() {
    if (!ik12SerialEnable) return;
    printIK12RawDegFromPoseSet(getCurrentPoseSet(), false);
  }

  // ===== GAIT CONFIG HELPERS =====
  float normPhase01(float p) {
    if (!isfinite(p)) return 0.5f;
    p = p - floorf(p);
    if (p < 0.0f) p += 1.0f;
    return p;
  }
  GaitRuntimeConfig sanitizeGaitCfg(GaitRuntimeConfig cfg) {
    cfg.hStep = clampF(cfg.hStep, -200.0f, 200.0f);
    cfg.tCycle = clampF(cfg.tCycle, 0.05f, 10.0f);
    cfg.phaseOffset = normPhase01(cfg.phaseOffset);
    cfg.swingRatio = clampF(cfg.swingRatio, 0.05f, 0.49f);
    return cfg;
  }
  const char* gaitModeToName(GaitMode mode) {
    switch (mode) {
      case GAIT_FORWARD: return "FWD";
      case GAIT_BACKWARD: return "BWD";
      case GAIT_LEFT: return "LEFT";
      case GAIT_RIGHT: return "RIGHT";
      case GAIT_SIDE_LEFT: return "SL";
      case GAIT_SIDE_RIGHT: return "SR";
      case GAIT_STAMP: return "STAMP";
      default: return "IDLE";
    }
  }
  bool parseGaitModeName(const char* raw, GaitMode &modeOut) {
    if (!raw) return false;
    char m[18] = {0};
    strncpy(m, raw, sizeof(m) - 1);
    for (int i = 0; m[i]; ++i) m[i] = (char)toupper((unsigned char)m[i]);
    if (!strcmp(m, "FWD") || !strcmp(m, "FORWARD")) { modeOut = GAIT_FORWARD; return true; }
    if (!strcmp(m, "BWD") || !strcmp(m, "BACKWARD")) { modeOut = GAIT_BACKWARD; return true; }
    if (!strcmp(m, "LEFT") || !strcmp(m, "L")) { modeOut = GAIT_LEFT; return true; }
    if (!strcmp(m, "RIGHT") || !strcmp(m, "R")) { modeOut = GAIT_RIGHT; return true; }
    if (!strcmp(m, "SL") || !strcmp(m, "SIDE_LEFT") || !strcmp(m, "STRAFE_LEFT")) { modeOut = GAIT_SIDE_LEFT; return true; }
    if (!strcmp(m, "SR") || !strcmp(m, "SIDE_RIGHT") || !strcmp(m, "STRAFE_RIGHT")) { modeOut = GAIT_SIDE_RIGHT; return true; }
    return false;
  }
  GaitRuntimeConfig* getGaitCfgPtr(GaitMode mode) {
    switch (mode) {
      case GAIT_FORWARD: return &gaitFwdCfg;
      case GAIT_BACKWARD: return &gaitBwdCfg;
      case GAIT_LEFT: return &gaitLeftCfg;
      case GAIT_RIGHT: return &gaitRightCfg;
      case GAIT_SIDE_LEFT: return &gaitSLCfg;
      case GAIT_SIDE_RIGHT: return &gaitSRCfg;
      default: return &gaitFwdCfg;
    }
  }
  GaitRuntimeConfig getGaitCfg(GaitMode mode) { return sanitizeGaitCfg(*getGaitCfgPtr(mode)); }
  void setGaitCfg(GaitMode mode, float hStep, float tCycle, float phase, float swing) {
    GaitRuntimeConfig *cfg = getGaitCfgPtr(mode);
    cfg->hStep = clampF(hStep, -200.0f, 200.0f);
    cfg->tCycle = clampF(tCycle, 0.05f, 10.0f);
    cfg->phaseOffset = normPhase01(phase);
    cfg->swingRatio = clampF(swing, 0.05f, 0.49f);
  }
  void printOneGaitCfg(GaitMode mode) {
    GaitRuntimeConfig cfg = getGaitCfg(mode);
    Serial.printf("GAIT,%s,%.2f,%.3f,%.3f,%.3f\n", gaitModeToName(mode), cfg.hStep, cfg.tCycle, cfg.phaseOffset, cfg.swingRatio);
  }
  void printGaitAck(GaitMode mode, const char* source) {
    GaitRuntimeConfig cfg = getGaitCfg(mode);
    Serial.printf("GAIT_ACK,%s,%s,%.2f,%.3f,%.3f,%.3f\n",
                  source ? source : "SET", gaitModeToName(mode),
                  cfg.hStep, cfg.tCycle, cfg.phaseOffset, cfg.swingRatio);
  }
  void showAllGaitRuntimeConfig() {
    Serial.println("===== PER GAIT CONFIG =====");
    Serial.println("Format: GAIT,MODE,hStep,tCycle,phaseOffset,swingRatio");
    printOneGaitCfg(GAIT_FORWARD);
    printOneGaitCfg(GAIT_BACKWARD);
    printOneGaitCfg(GAIT_LEFT);
    printOneGaitCfg(GAIT_RIGHT);
    printOneGaitCfg(GAIT_SIDE_LEFT);
    printOneGaitCfg(GAIT_SIDE_RIGHT);
  }
  void resetGaitRuntimeDefaults() {
    gaitFwdCfg   = {100.0f, 0.70f, 0.50f, 0.35f};
    gaitBwdCfg   = {100.0f, 0.70f, 0.50f, 0.35f};
    gaitLeftCfg  = { 50.0f, 0.40f, 0.50f, 0.49f};
    gaitRightCfg = { 50.0f, 0.40f, 0.50f, 0.49f};
    gaitSLCfg    = { 50.0f, 0.70f, 0.50f, 0.35f};
    gaitSRCfg    = { 50.0f, 0.70f, 0.50f, 0.35f};
    Serial.println("[OK] RESET_GAITS default per-gait config restored");
  }
  void resetGaitPhaseClockIfRunning(GaitMode changedMode) {
    if (gaitMode == changedMode) { gaitStartMs = millis(); gaitLastMs = 0; }
  }

  // ===== TRAJECTORY =====
  Vec3 trajectoryPlanningZOverlap(float phase, Vec3 centerP, float ampZ, float hStep, float swingRatio) {
    swingRatio = clampF(swingRatio, 0.05f, 0.49f);
    phase = phase - floorf(phase);
    float frontOffset = ampZ;
    float backOffset  = -ampZ;
    if (phase < swingRatio) {
      float tau = phase / swingRatio;
      float ss = smoothStep01(tau);
      float x = centerP.x + hStep * sinf((float)M_PI * ss);
      float z = centerP.z + backOffset + (frontOffset - backOffset) * ss;
      return {x, centerP.y, z};
    }
    float tau = (phase - swingRatio) / (1.0f - swingRatio);
    float ss = smoothStep01(tau);
    float x = centerP.x;
    float z = centerP.z + frontOffset + (backOffset - frontOffset) * ss;
    return {x, centerP.y, z};
  }
  Vec3 trajectoryPlanningYOverlap(float phase, Vec3 centerP, float ampY, float hStep, float swingRatio) {
    swingRatio = clampF(swingRatio, 0.05f, 0.49f);
    phase = phase - floorf(phase);
    float frontOffset = ampY;
    float backOffset  = -ampY;
    if (phase < swingRatio) {
      float tau = phase / swingRatio;
      float ss = smoothStep01(tau);
      float x = centerP.x + hStep * sinf((float)M_PI * ss);
      float y = centerP.y + backOffset + (frontOffset - backOffset) * ss;
      return {x, y, centerP.z};
    }
    float tau = (phase - swingRatio) / (1.0f - swingRatio);
    float ss = smoothStep01(tau);
    float x = centerP.x;
    float y = centerP.y + frontOffset + (backOffset - frontOffset) * ss;
    return {x, y, centerP.z};
  }
  Vec3 trajectoryPlanningYCustom(float t, Vec3 centerP, float yStart, float yEnd, float hStep, float tCycle, float swingRatio) {
    // LEFT/RIGHT xoay cũ nhưng giờ cũng dùng swingRatio thật.
    // phase < swingRatio  : chân nhấc, đi từ yStart -> yEnd, x cộng hStep.
    // phase >= swingRatio : chân chống đất, quét yEnd -> yStart, x giữ ở tâm.
    tCycle = clampF(tCycle, 0.05f, 10.0f);
    swingRatio = clampF(swingRatio, 0.05f, 0.49f);

    float phase = fmodf(t, tCycle) / tCycle;
    if (phase < 0.0f) phase += 1.0f;

    Vec3 pStart{centerP.x, yStart, centerP.z};
    Vec3 pEnd{centerP.x, yEnd, centerP.z};
    Vec3 d{pEnd.x - pStart.x, pEnd.y - pStart.y, pEnd.z - pStart.z};

    if (phase < swingRatio) {
      float tau = phase / swingRatio;
      float ss = smoothStep01(tau);
      Vec3 p{pStart.x + d.x * ss, pStart.y + d.y * ss, pStart.z + d.z * ss};
      p.x += hStep * sinf((float)M_PI * ss);
      return p;
    }

    float tau = (phase - swingRatio) / (1.0f - swingRatio);
    float ss = smoothStep01(tau);
    return {pEnd.x - d.x * ss, pEnd.y - d.y * ss, pEnd.z - d.z * ss};
  }
  const PoseSet& getCenterSet(GaitMode mode) {
    switch (mode) {
      case GAIT_FORWARD: return forwardCenter;
      case GAIT_BACKWARD: return backwardCenter;
      case GAIT_LEFT: return leftCenter;
      case GAIT_RIGHT: return rightCenter;
      case GAIT_SIDE_LEFT: return sideCenter;
      case GAIT_SIDE_RIGHT: return sideCenter;
      case GAIT_STAMP: return stampCenter;
      default: return stampCenter;
    }
  }
  float getLegAmp(GaitMode mode, const char* leg) {
    if (mode == GAIT_STAMP) return 0.0f;
    if (mode == GAIT_FORWARD) {
      if (!strcmp(leg, "FL")) return -gaitMoveAmpFL;
      if (!strcmp(leg, "FR")) return -gaitMoveAmpFR;
      if (!strcmp(leg, "RL")) return -gaitMoveAmpRL;
      return -gaitMoveAmpRR;
    }
    if (mode == GAIT_BACKWARD) {
      if (!strcmp(leg, "FL")) return gaitMoveAmpFL;
      if (!strcmp(leg, "FR")) return gaitMoveAmpFR;
      if (!strcmp(leg, "RL")) return gaitMoveAmpRL;
      return gaitMoveAmpRR;
    }
    return 0.0f;
  }
  float getSideLegAmp(GaitMode mode, const char* leg) {
    if (!(mode == GAIT_SIDE_LEFT || mode == GAIT_SIDE_RIGHT)) return 0.0f;
    float amp = gaitSideAmpRR;
    if (!strcmp(leg, "FL")) amp = gaitSideAmpFL;
    else if (!strcmp(leg, "FR")) amp = gaitSideAmpFR;
    else if (!strcmp(leg, "RL")) amp = gaitSideAmpRL;
    float sign = (mode == GAIT_SIDE_LEFT) ? -1.0f : 1.0f;
    if (gaitSideInvert) sign = -sign;
    return sign * amp;
  }
  float getBaseLegAmpAbs(const char* leg) {
    if (!strcmp(leg, "FL")) return fabsf(gaitMoveAmpFL);
    if (!strcmp(leg, "FR")) return fabsf(gaitMoveAmpFR);
    if (!strcmp(leg, "RL")) return fabsf(gaitMoveAmpRL);
    return fabsf(gaitMoveAmpRR);
  }
  bool isLeftLeg(const char* leg) { return (!strcmp(leg, "FL") || !strcmp(leg, "RL")); }
  float getLegAmpYawGait(GaitMode mode, const char* leg, float yawAmpDelta) {
    if (mode == GAIT_STAMP) return 0.0f;
    if (!(mode == GAIT_FORWARD || mode == GAIT_BACKWARD)) return getLegAmp(mode, leg);
    float sideSign = isLeftLeg(leg) ? 1.0f : -1.0f;
    if (yawGaitInvertSide) sideSign = -sideSign;
    float mag = getBaseLegAmpAbs(leg) + sideSign * yawAmpDelta;
    mag = clampF(mag, 0.0f, getBaseLegAmpAbs(leg) + yawGaitMaxAmp);
    if (mode == GAIT_FORWARD) return -mag;
    if (mode == GAIT_BACKWARD) return mag;
    return 0.0f;
  }
  float getLegAmpCamGait(GaitMode mode, const char* leg, float camAmpDelta) {
    if (mode == GAIT_STAMP) return 0.0f;
    if (!(mode == GAIT_FORWARD || mode == GAIT_BACKWARD)) return getLegAmp(mode, leg);
    float sideSign = isLeftLeg(leg) ? 1.0f : -1.0f;
    if (camGaitInvertSide) sideSign = -sideSign;
    float base = getBaseLegAmpAbs(leg);
    float mag = base + sideSign * camAmpDelta;
    float minAmp = clampF(camMinAmp, 0.0f, base + camMaxAmp);
    mag = clampF(mag, minAmp, base + camMaxAmp);
    if (mode == GAIT_FORWARD) return -mag;
    if (mode == GAIT_BACKWARD) return mag;
    return 0.0f;
  }

  void startGait(GaitMode mode, const char* name) {
    disablePidForSpecialPose = false;
    camAutoPaused = false;
    if ((mode == GAIT_FORWARD || mode == GAIT_BACKWARD) && (camGaitEnable || camRequireOffsetBeforeWalk)) {
      camSeenSinceEnable = false;
      camLastRxMs = 0;
      resetCamGaitState(true);
    }
    if (bootNeutralMode) {
      movePoseSet(getCenterSet(mode), false);
      setIdleBasePose(getCenterSet(mode));
    }
    bootNeutralMode = false;
    gaitMode = mode;
    gaitStartMs = millis();
    gaitLastMs = 0;
    resetWalkBalanceState(true);
    resetStandBalanceState(true);
    resetYawGaitState(true);
    resetCamGaitState(true);
    yawGaitSetpointDeg = yawDeg;
    GaitRuntimeConfig cfg = getGaitCfg(mode);
    Serial.printf("[GAIT] %s | hStep=%.2f | tCycle=%.3f | phase=%.3f | swing=%.3f\n", name, cfg.hStep, cfg.tCycle, cfg.phaseOffset, cfg.swingRatio);
  }
  void stopGait(bool toGaitCenter) {
    camAutoPaused = false;

    // Luu gait dang chay truoc khi dua ve IDLE.
    // Neu set gaitMode = IDLE truoc roi moi getCenterSet() thi se mat mode cu.
    GaitMode lastMode = gaitMode;

    gaitMode = GAIT_IDLE;
    gaitLastMs = 0;

    if (toGaitCenter) {
      PoseSet target = stampCenter;

      // Khi dang chay gait nao thi STOP se ve dung tam cua gait do.
      // Cac bien center nay da duoc cap nhat boi SET_POSE / SET_PLEG / X+ / X-.
      if (lastMode == GAIT_FORWARD ||
          lastMode == GAIT_BACKWARD ||
          lastMode == GAIT_LEFT ||
          lastMode == GAIT_RIGHT ||
          lastMode == GAIT_SIDE_LEFT ||
          lastMode == GAIT_SIDE_RIGHT ||
          lastMode == GAIT_STAMP) {
        target = getCenterSet(lastMode);
      }

      movePoseSet(target, false);
      setIdleBasePose(target);
      bootNeutralMode = false;

      Serial.printf("[OK] STOP_CENTER %s\n", gaitModeToName(lastMode));
    } else {
      setIdleBaseFromCurrent();
    }
  }
  void gaitTick() {
    if (gaitMode == GAIT_IDLE) return;

    if (camMustBlockCurrentGait()) {
      if (!camAutoPaused) {
        camAutoPaused = true;
        resetCamGaitState(true);
        movePoseSet(stampCenter, false);
        setIdleBasePose(stampCenter);
      }
      return;
    }
    if ((camGaitEnable || camRequireOffsetBeforeWalk) && camAutoPaused && (gaitMode == GAIT_FORWARD || gaitMode == GAIT_BACKWARD)) return;

    unsigned long now = millis();
    if (now - gaitLastMs < GAIT_PERIOD_MS) return;
    gaitLastMs = now;
    float elapsed = (now - gaitStartMs) * 0.001f;

    GaitMode effectiveMode = getCamAlignedEffectiveMode(gaitMode);
    const PoseSet &centers = getCenterSet(effectiveMode);
    GaitRuntimeConfig cfg = getGaitCfg(effectiveMode);

    Vec3 pfl, pfr, prl, prr;
    bool gFL = true, gFR = true, gRL = true, gRR = true;

    if (effectiveMode == GAIT_LEFT || effectiveMode == GAIT_RIGHT) {
      const LateralConfig &latCfg = (effectiveMode == GAIT_LEFT) ? leftLateral : rightLateral;
      float phaseA = fmodf(elapsed, cfg.tCycle) / cfg.tCycle;
      if (phaseA < 0.0f) phaseA += 1.0f;
      float phaseB = normPhase01(phaseA + cfg.phaseOffset);
      pfl = trajectoryPlanningYCustom(elapsed, centers.FL, latCfg.flStartY, latCfg.flEndY, cfg.hStep, cfg.tCycle, cfg.swingRatio);
      pfr = trajectoryPlanningYCustom(elapsed + cfg.tCycle * cfg.phaseOffset, centers.FR, latCfg.frStartY, latCfg.frEndY, cfg.hStep, cfg.tCycle, cfg.swingRatio);
      prl = trajectoryPlanningYCustom(elapsed + cfg.tCycle * cfg.phaseOffset, centers.RL, latCfg.rlStartY, latCfg.rlEndY, cfg.hStep, cfg.tCycle, cfg.swingRatio);
      prr = trajectoryPlanningYCustom(elapsed, centers.RR, latCfg.rrStartY, latCfg.rrEndY, cfg.hStep, cfg.tCycle, cfg.swingRatio);
      gFL = (phaseA >= cfg.swingRatio); gRR = (phaseA >= cfg.swingRatio);
      gFR = (phaseB >= cfg.swingRatio); gRL = (phaseB >= cfg.swingRatio);
    } else if (effectiveMode == GAIT_SIDE_LEFT || effectiveMode == GAIT_SIDE_RIGHT) {
      float phaseA = fmodf(elapsed, cfg.tCycle) / cfg.tCycle;
      if (phaseA < 0.0f) phaseA += 1.0f;
      float phaseB = normPhase01(phaseA + cfg.phaseOffset);
      float ampFL = getSideLegAmp(effectiveMode, "FL");
      float ampFR = getSideLegAmp(effectiveMode, "FR");
      float ampRL = getSideLegAmp(effectiveMode, "RL");
      float ampRR = getSideLegAmp(effectiveMode, "RR");
      pfl = trajectoryPlanningYOverlap(phaseA, centers.FL, ampFL, cfg.hStep, cfg.swingRatio);
      pfr = trajectoryPlanningYOverlap(phaseB, centers.FR, ampFR, cfg.hStep, cfg.swingRatio);
      prl = trajectoryPlanningYOverlap(phaseB, centers.RL, ampRL, cfg.hStep, cfg.swingRatio);
      prr = trajectoryPlanningYOverlap(phaseA, centers.RR, ampRR, cfg.hStep, cfg.swingRatio);
      gFL = (phaseA >= cfg.swingRatio); gRR = (phaseA >= cfg.swingRatio);
      gFR = (phaseB >= cfg.swingRatio); gRL = (phaseB >= cfg.swingRatio);
    } else {
      float phaseA = fmodf(elapsed, cfg.tCycle) / cfg.tCycle;
      if (phaseA < 0.0f) phaseA += 1.0f;
      float phaseB = normPhase01(phaseA + cfg.phaseOffset);

      float ampFL, ampFR, ampRL, ampRR;
      if (camGaitEnable) {
        float camAmpDelta = updateCamGaitPid();
        ampFL = getLegAmpCamGait(effectiveMode, "FL", camAmpDelta);
        ampFR = getLegAmpCamGait(effectiveMode, "FR", camAmpDelta);
        ampRL = getLegAmpCamGait(effectiveMode, "RL", camAmpDelta);
        ampRR = getLegAmpCamGait(effectiveMode, "RR", camAmpDelta);
      } else if (yawGaitOnlyEnable) {
        float yawAmpDelta = updateYawGaitPid();
        ampFL = getLegAmpYawGait(effectiveMode, "FL", yawAmpDelta);
        ampFR = getLegAmpYawGait(effectiveMode, "FR", yawAmpDelta);
        ampRL = getLegAmpYawGait(effectiveMode, "RL", yawAmpDelta);
        ampRR = getLegAmpYawGait(effectiveMode, "RR", yawAmpDelta);
      } else {
        ampFL = getLegAmp(effectiveMode, "FL");
        ampFR = getLegAmp(effectiveMode, "FR");
        ampRL = getLegAmp(effectiveMode, "RL");
        ampRR = getLegAmp(effectiveMode, "RR");
      }
      pfl = trajectoryPlanningZOverlap(phaseA, centers.FL, ampFL, cfg.hStep, cfg.swingRatio);
      pfr = trajectoryPlanningZOverlap(phaseB, centers.FR, ampFR, cfg.hStep, cfg.swingRatio);
      prl = trajectoryPlanningZOverlap(phaseB, centers.RL, ampRL, cfg.hStep, cfg.swingRatio);
      prr = trajectoryPlanningZOverlap(phaseA, centers.RR, ampRR, cfg.hStep, cfg.swingRatio);
      gFL = (phaseA >= cfg.swingRatio); gRR = (phaseA >= cfg.swingRatio);
      gFR = (phaseB >= cfg.swingRatio); gRL = (phaseB >= cfg.swingRatio);
    }

    if (!yawGaitOnlyEnable && !camGaitEnable) {
      Vec2 mix = updateBalancePidMix();
      saveBalanceMixOut(mix);
      pfl = applyBalanceMix(pfl, "FL", gFL, mix, balMaxX);
      pfr = applyBalanceMix(pfr, "FR", gFR, mix, balMaxX);
      prl = applyBalanceMix(prl, "RL", gRL, mix, balMaxX);
      prr = applyBalanceMix(prr, "RR", gRR, mix, balMaxX);
    } else {
      saveBalanceMixOut({0.0f, 0.0f});
      saveBalanceDx("FL", 0.0f); saveBalanceDx("FR", 0.0f); saveBalanceDx("RL", 0.0f); saveBalanceDx("RR", 0.0f);
    }

    PoseSet gaitPose{pfl, pfr, prl, prr};
    movePoseSet(gaitPose, false);
  }

  // ===== PID / BALANCE / CAMERA =====
  float calcSignedAxisPid(float err, float &integ, float &lastErr, float &lastOut,
                          float kp, float ki, float kd, float maxOut,
                          float deadbandDeg, float resetErrDeg, float iMax, float dt) {
    iMax = fabsf(iMax);
    if (iMax < 1e-6f) iMax = 1e-6f;
    if (fabsf(err) < resetErrDeg) { lastErr = err; return lastOut; }
    if (fabsf(err) < deadbandDeg) { lastErr = err; return lastOut; }
    float newInteg = integ + err * dt;
    if ((integ >= iMax && err > 0.0f) || (integ <= -iMax && err < 0.0f)) integ = clampF(integ, -iMax, iMax);
    else integ = clampF(newInteg, -iMax, iMax);
    float deriv = (err - lastErr) / dt;
    lastErr = err;
    float out = kp * err + ki * integ + kd * deriv;
    lastOut = clampF(out, -maxOut, maxOut);
    return lastOut;
  }
  void resetWalkBalanceState(bool clearOutput=true) {
    balRollIntegral = 0.0f; balPitchIntegral = 0.0f;
    balLastRollErr = 0.0f; balLastPitchErr = 0.0f;
    if (clearOutput) { balLastRollOut = 0.0f; balLastPitchOut = 0.0f; }
    balLastPidMs = 0;
  }
  void resetStandBalanceState(bool clearOutput=true) {
    standBalRollIntegral = 0.0f; standBalPitchIntegral = 0.0f;
    standBalLastRollErr = 0.0f; standBalLastPitchErr = 0.0f;
    if (clearOutput) { standBalLastRollOut = 0.0f; standBalLastPitchOut = 0.0f; }
    standBalLastPidMs = 0;
  }
  void resetYawGaitState(bool clearOutput=true) {
    yawGaitIntegral = 0.0f; yawGaitLastErr = 0.0f;
    if (clearOutput) yawGaitLastOut = 0.0f;
    yawGaitLastPidMs = 0;
  }
  void resetCamGaitState(bool clearOutput=true) {
    camIntegral = 0.0f; camLastErr = 0.0f;
    if (clearOutput) camLastOut = 0.0f;
    camLastPidMs = 0;
  }
  Vec2 updateBalancePidMix() {
    if (!balanceEnable || !mpuOK) return {0.0f, 0.0f};
    unsigned long now = millis();
    float dt = 0.03f;
    if (balLastPidMs != 0) dt = clampF((now - balLastPidMs) * 0.001f, 0.005f, 0.20f);
    balLastPidMs = now;
    float r = calcSignedAxisPid(rollDeg, balRollIntegral, balLastRollErr, balLastRollOut, balKp, balKi, balKd, balMaxX, balDeadbandDeg, balResetErrDeg, balIMax, dt);
    float p = calcSignedAxisPid(pitchDeg, balPitchIntegral, balLastPitchErr, balLastPitchOut, balKp, balKi, balKd, balMaxX, balDeadbandDeg, balResetErrDeg, balIMax, dt);
    return {r, p};
  }
  Vec2 updateStandBalancePidMix() {
    if (!standBalanceEnable || !mpuOK) return {0.0f, 0.0f};
    unsigned long now = millis();
    float dt = 0.03f;
    if (standBalLastPidMs != 0) dt = clampF((now - standBalLastPidMs) * 0.001f, 0.005f, 0.20f);
    standBalLastPidMs = now;
    float rollErr = rollDeg - standBalRollSetpointDeg;
    float pitchErr = pitchDeg - standBalPitchSetpointDeg;
    float r = calcSignedAxisPid(rollErr, standBalRollIntegral, standBalLastRollErr, standBalLastRollOut, standBalKp, standBalKi, standBalKd, standBalMaxX, standBalDeadbandDeg, standBalResetErrDeg, standBalIMax, dt);
    float p = calcSignedAxisPid(pitchErr, standBalPitchIntegral, standBalLastPitchErr, standBalLastPitchOut, standBalKp, standBalKi, standBalKd, standBalMaxX, standBalDeadbandDeg, standBalResetErrDeg, standBalIMax, dt);
    return {r, p};
  }
  void saveBalanceDx(const char* leg, float dx) {
    if (!strcmp(leg, "FL")) lastDxFL = dx;
    else if (!strcmp(leg, "FR")) lastDxFR = dx;
    else if (!strcmp(leg, "RL")) lastDxRL = dx;
    else if (!strcmp(leg, "RR")) lastDxRR = dx;
  }
  void saveBalanceMixOut(Vec2 mix) { lastRollPidOut = mix.x; lastPitchPidOut = mix.y; }
  Vec3 applyBalanceMix(Vec3 p, const char* leg, bool isGrounded, Vec2 mix, float maxX) {
    float dx = 0.0f;
    if (!isGrounded) { saveBalanceDx(leg, 0.0f); return p; }
    if (!strcmp(leg, "FL")) dx =  mix.x + mix.y;
    else if (!strcmp(leg, "FR")) dx =  mix.x - mix.y;
    else if (!strcmp(leg, "RL")) dx = -mix.x + mix.y;
    else if (!strcmp(leg, "RR")) dx = -mix.x - mix.y;
    dx = clampF(dx, -maxX, maxX);
    saveBalanceDx(leg, dx);
    p.x += dx;
    return p;
  }
  void standBalanceTick() {
    if (gaitMode != GAIT_IDLE) return;
    if (disablePidForSpecialPose) return;
    if (!standBalanceEnable || !mpuOK) return;
    if (!idleBaseValid) setIdleBaseFromCurrent();
    unsigned long now = millis();
    if (now - standBalLastTickMs < GAIT_PERIOD_MS) return;
    standBalLastTickMs = now;
    Vec2 mix = updateStandBalancePidMix();
    saveBalanceMixOut(mix);
    PoseSet ps = idleBasePose;
    ps.FL = applyBalanceMix(ps.FL, "FL", true, mix, standBalMaxX);
    ps.FR = applyBalanceMix(ps.FR, "FR", true, mix, standBalMaxX);
    ps.RL = applyBalanceMix(ps.RL, "RL", true, mix, standBalMaxX);
    ps.RR = applyBalanceMix(ps.RR, "RR", true, mix, standBalMaxX);
    movePoseSet(ps, false);
  }
  void balanceOffsetPrintTick() {
    if (!balanceOffsetPrintEnable) return;
    unsigned long now = millis();
    if (now - balanceOffsetLastPrintMs < BAL_OFFSET_PRINT_PERIOD_MS) return;
    balanceOffsetLastPrintMs = now;
    const char* modeName = (gaitMode == GAIT_IDLE) ? "STAND" : "GAIT";
    Serial.printf("[OFF][%s] FL=%+.2f FR=%+.2f RL=%+.2f RR=%+.2f | rPID=%+.2f pPID=%+.2f | roll=%+.2f spR=%+.2f pitch=%+.2f spP=%+.2f\n",
      modeName, lastDxFL, lastDxFR, lastDxRL, lastDxRR, lastRollPidOut, lastPitchPidOut,
      rollDeg, standBalRollSetpointDeg, pitchDeg, standBalPitchSetpointDeg);
  }
  float updateYawGaitPid() {
    if (!yawGaitOnlyEnable || !mpuOK || gaitMode == GAIT_IDLE) return 0.0f;
    unsigned long now = millis();
    float dt = 0.03f;
    if (yawGaitLastPidMs != 0) dt = clampF((now - yawGaitLastPidMs) * 0.001f, 0.005f, 0.20f);
    yawGaitLastPidMs = now;
    float err = wrap180(yawGaitSetpointDeg - yawDeg);
    return calcSignedAxisPid(err, yawGaitIntegral, yawGaitLastErr, yawGaitLastOut,
      yawGaitKp, yawGaitKi, yawGaitKd, yawGaitMaxAmp, yawGaitDeadbandDeg, yawGaitHoldDeg, yawGaitIMax, dt);
  }
  bool camDataFresh() { return camLastRxMs != 0 && (millis() - camLastRxMs) <= camLostStopMs; }
  bool camBoxTooBig() { return camDetected && camStopBoxH > 0 && camBoxH >= camStopBoxH; }
  bool camOkToMove() {
    if (!camSeenSinceEnable) return false;
    if (!camDataFresh()) return false;
    if (!camDetected) return false;
    if (camBoxTooBig()) return false;
    return true;
  }
  bool camMustBlockCurrentGait() {
    if (!(gaitMode == GAIT_FORWARD || gaitMode == GAIT_BACKWARD)) return false;
    if (!camRequireOffsetBeforeWalk && !camGaitEnable) return false;
    return !camOkToMove();
  }
  bool camOffsetInsideAlignRange() { return fabsf(camOffsetX - camTargetOffsetPx) <= camAlignRangePx; }
  GaitMode getCamAlignedEffectiveMode(GaitMode requestedMode) {
    if (!camAlignRotateEnable) return requestedMode;
    if (!camGaitEnable && !camRequireOffsetBeforeWalk) return requestedMode;
    if (!(requestedMode == GAIT_FORWARD || requestedMode == GAIT_BACKWARD)) return requestedMode;
    if (!camOkToMove()) return requestedMode;
    float err = camOffsetX - camTargetOffsetPx;
    if (fabsf(err) <= camAlignRangePx) { camAlignLastPrintMs = 0; return requestedMode; }
    GaitMode sideMode;
    if (err < -camAlignRangePx) sideMode = camAlignRotateInvert ? GAIT_SIDE_RIGHT : GAIT_SIDE_LEFT;
    else sideMode = camAlignRotateInvert ? GAIT_SIDE_LEFT : GAIT_SIDE_RIGHT;
    unsigned long now = millis();
    if (now - camAlignLastPrintMs >= 300) {
      camAlignLastPrintMs = now;
      Serial.printf("[CAMALIGN] off=%.0f outside +-%.0f -> %s until center\n", camOffsetX, camAlignRangePx, (sideMode == GAIT_SIDE_LEFT) ? "SL" : "SR");
    }
    return sideMode;
  }
  float updateCamGaitPid() {
    if (!camGaitEnable || gaitMode == GAIT_IDLE) return 0.0f;
    if (!camSeenSinceEnable) return 0.0f;
    if (!camOkToMove()) { resetCamGaitState(true); return 0.0f; }
    unsigned long now = millis();
    float dt = 0.03f;
    if (camLastPidMs != 0) dt = clampF((now - camLastPidMs) * 0.001f, 0.005f, 0.20f);
    camLastPidMs = now;
    float err = camTargetOffsetPx - camOffsetX;
    return calcSignedAxisPid(err, camIntegral, camLastErr, camLastOut,
      camKp, camKi, camKd, camMaxAmp, camDeadbandPx, camHoldPx, camIMax, dt);
  }
  bool parseCameraCsv(String raw) {
    int detected = 0, offset = 0, boxW = 0, boxH = 0, cx = 0, cy = 0;
    char dir[18] = {0};
    int n = sscanf(raw.c_str(), "%d,%d,%17[^,],%d,%d,%d,%d", &detected, &offset, dir, &boxW, &boxH, &cx, &cy);
    if (n < 5) return false;
    camDetected = detected;
    camOffsetX = (float)offset;
    camBoxW = boxW; camBoxH = boxH;
    camCenterX = (n >= 6) ? cx : 0;
    camCenterY = (n >= 7) ? cy : 0;
    camLastRxMs = millis();
    camSeenSinceEnable = true;
    unsigned long now = millis();
    if (now - camLastPrintMs >= 200) {
      camLastPrintMs = now;
      Serial.printf("[CAM] det=%d off=%.0f box=%dx%d center=%d,%d out=%.2f en=%d\n",
        camDetected, camOffsetX, camBoxW, camBoxH, camCenterX, camCenterY, camLastOut, camGaitEnable);
    }
    return true;
  }
  void camSafetyTick() {
    if (!(gaitMode == GAIT_FORWARD || gaitMode == GAIT_BACKWARD)) { camAutoPaused = false; return; }
    if (!camGaitEnable && !camRequireOffsetBeforeWalk) { camAutoPaused = false; return; }
    bool fresh = camDataFresh();
    bool okMove = camOkToMove();
    if (!okMove) {
      if (!camAutoPaused) {
        camAutoPaused = true;
        resetCamGaitState(true);
        movePoseSet(stampCenter, false);
        setIdleBasePose(stampCenter);
        if (!camSeenSinceEnable) Serial.println("[CAM] PAUSE_WAIT_NEW_OFFSET | waiting camera offset");
        else if (!fresh) Serial.println("[CAM] PAUSE_LOST_SERIAL_OR_CAMERA | waiting resume");
        else if (!camDetected) Serial.println("[CAM] PAUSE_NO_PERSON | waiting resume");
        else Serial.printf("[CAM] PAUSE_BOX_TOO_BIG boxH=%d >= maxH=%d | width ignored\n", camBoxH, camStopBoxH);
      } else {
        unsigned long now = millis();
        if (now - camLastPausePrintMs >= 1000) {
          camLastPausePrintMs = now;
          Serial.println("[CAM] STILL_PAUSED waiting valid camera data");
        }
      }
      return;
    }
    if (camAutoPaused) {
      camAutoPaused = false;
      resetCamGaitState(true);
      gaitStartMs = millis();
      gaitLastMs = 0;
      Serial.printf("[CAM] RESUME boxH=%d < maxH=%d | mode=%s\n", camBoxH, camStopBoxH, gaitModeToName(gaitMode));
    }
  }

  // ===== SHOW / X / POSE CONFIG =====
  void showPose() {
    Serial.println("===== CURRENT FOOT POS =====");
    Serial.printf("FL: x=%.2f y=%.2f z=%.2f\n", FLs.pos.x, FLs.pos.y, FLs.pos.z);
    Serial.printf("FR: x=%.2f y=%.2f z=%.2f\n", FRs.pos.x, FRs.pos.y, FRs.pos.z);
    Serial.printf("RL: x=%.2f y=%.2f z=%.2f\n", RLs.pos.x, RLs.pos.y, RLs.pos.z);
    Serial.printf("RR: x=%.2f y=%.2f z=%.2f\n", RRs.pos.x, RRs.pos.y, RRs.pos.z);
  }
  void showCalc() {
    Serial.println("===== CURRENT SERVO CALC =====");
    Serial.printf("FL: hip=%.2f bao=%.2f abd=%.2f hgo=%.2f\n", FLs.hip, FLs.bao, FLs.abd, FLs.lastHgo);
    Serial.printf("FR: hip=%.2f bao=%.2f abd=%.2f hgo=%.2f\n", FRs.hip, FRs.bao, FRs.abd, FRs.lastHgo);
    Serial.printf("RL: hip=%.2f bao=%.2f abd=%.2f hgo=%.2f\n", RLs.hip, RLs.bao, RLs.abd, RLs.lastHgo);
    Serial.printf("RR: hip=%.2f bao=%.2f abd=%.2f hgo=%.2f\n", RRs.hip, RRs.bao, RRs.abd, RRs.lastHgo);
  }
  void showGaitConfig() {
    Serial.println("===== GAIT CONFIG =====");
    showAllGaitRuntimeConfig();
    Serial.printf("Legacy FB : hStep=%.2f | tCycle=%.3f | swingRatio=%.3f\n", gaitHStepFB, gaitTCycleFB, gaitSwingRatioFB);
    Serial.printf("Legacy LR : hStep=%.2f | tCycle=%.3f\n", gaitHStepLR, gaitTCycleLR);
    Serial.printf("Legacy SIDE: hStep=%.2f | tCycle=%.3f | swingRatio=%.3f | phase=%.3f | inv=%d | amp FL=%.1f FR=%.1f RL=%.1f RR=%.1f\n",
      gaitHStepSide, gaitTCycleSide, gaitSwingRatioSide, gaitSidePhaseOffset, gaitSideInvert,
      gaitSideAmpFL, gaitSideAmpFR, gaitSideAmpRL, gaitSideAmpRR);
  }
  void showSideConfig() {
    Serial.println("===== SIDE Y CONFIG =====");
    Serial.printf("LEFT  FL: yStart=%.2f yEnd=%.2f | FR: yStart=%.2f yEnd=%.2f\n", leftLateral.flStartY, leftLateral.flEndY, leftLateral.frStartY, leftLateral.frEndY);
    Serial.printf("LEFT  RL: yStart=%.2f yEnd=%.2f | RR: yStart=%.2f yEnd=%.2f\n", leftLateral.rlStartY, leftLateral.rlEndY, leftLateral.rrStartY, leftLateral.rrEndY);
    Serial.printf("RIGHT FL: yStart=%.2f yEnd=%.2f | FR: yStart=%.2f yEnd=%.2f\n", rightLateral.flStartY, rightLateral.flEndY, rightLateral.frStartY, rightLateral.frEndY);
    Serial.printf("RIGHT RL: yStart=%.2f yEnd=%.2f | RR: yStart=%.2f yEnd=%.2f\n", rightLateral.rlStartY, rightLateral.rlEndY, rightLateral.rrStartY, rightLateral.rrEndY);
  }
  void shiftPoseSetX(PoseSet &ps, float dx) {
    ps.FL.x = clampF(ps.FL.x + dx, gaitXMin, gaitXMax);
    ps.FR.x = clampF(ps.FR.x + dx, gaitXMin, gaitXMax);
    ps.RL.x = clampF(ps.RL.x + dx, gaitXMin, gaitXMax);
    ps.RR.x = clampF(ps.RR.x + dx, gaitXMin, gaitXMax);
  }
  void printGaitXStatus(const char* tag) {
    Serial.printf("[XCTRL] %s | STAND X=%.1f/%.1f/%.1f/%.1f | STOP X=%.1f/%.1f/%.1f/%.1f | FWD X=%.1f/%.1f/%.1f/%.1f | BWD X=%.1f/%.1f/%.1f/%.1f | LEFT X=%.1f/%.1f/%.1f/%.1f | RIGHT X=%.1f/%.1f/%.1f/%.1f | SL/SR X=%.1f/%.1f/%.1f/%.1f | step=%.1f range=[%.1f,%.1f]\n",
      tag,
      standPose.FL.x, standPose.FR.x, standPose.RL.x, standPose.RR.x,
      stampCenter.FL.x, stampCenter.FR.x, stampCenter.RL.x, stampCenter.RR.x,
      forwardCenter.FL.x, forwardCenter.FR.x, forwardCenter.RL.x, forwardCenter.RR.x,
      backwardCenter.FL.x, backwardCenter.FR.x, backwardCenter.RL.x, backwardCenter.RR.x,
      leftCenter.FL.x, leftCenter.FR.x, leftCenter.RL.x, leftCenter.RR.x,
      rightCenter.FL.x, rightCenter.FR.x, rightCenter.RL.x, rightCenter.RR.x,
      sideCenter.FL.x, sideCenter.FR.x, sideCenter.RL.x, sideCenter.RR.x,
      gaitXStep, gaitXMin, gaitXMax);
  }
  void applyGaitXDelta(float dx, bool moveNow=true) {
    shiftPoseSetX(standPose, dx);
    shiftPoseSetX(stampCenter, dx);
    shiftPoseSetX(forwardCenter, dx);
    shiftPoseSetX(backwardCenter, dx);
    shiftPoseSetX(leftCenter, dx);
    shiftPoseSetX(rightCenter, dx);
    shiftPoseSetX(sideCenter, dx);
    if (moveNow) {
      PoseSet cur = getCurrentPoseSet();
      shiftPoseSetX(cur, dx);
      movePoseSet(cur, false);
      setIdleBasePose(cur);
      resetStandBalanceState(true);
      bootNeutralMode = false;
      if (gaitMode != GAIT_IDLE) { gaitStartMs = millis(); gaitLastMs = 0; }
      printIK12RawDegFromPoseSet(cur, true);
    }
    printGaitXStatus(dx >= 0 ? "X+" : "X-");
  }
  void resetGaitXToDefault() {
    standPose.FL.x = -197.0f; standPose.FR.x = -210.0f; standPose.RL.x = -197.0f; standPose.RR.x = -210.0f;
    stampCenter.FL.x = -210.0f; stampCenter.FR.x = -210.0f; stampCenter.RL.x = -210.0f; stampCenter.RR.x = -210.0f;
    forwardCenter.FL.x = -197.0f; forwardCenter.FR.x = -210.0f; forwardCenter.RL.x = -197.0f; forwardCenter.RR.x = -210.0f;
    backwardCenter.FL.x = -201.0f; backwardCenter.FR.x = -210.0f; backwardCenter.RL.x = -201.0f; backwardCenter.RR.x = -210.0f;
    leftCenter.FL.x = -210.0f; leftCenter.FR.x = -210.0f; leftCenter.RL.x = -210.0f; leftCenter.RR.x = -210.0f;
    rightCenter.FL.x = -210.0f; rightCenter.FR.x = -210.0f; rightCenter.RL.x = -210.0f; rightCenter.RR.x = -210.0f;
    sideCenter.FL.x = -202.0f; sideCenter.FR.x = -210.0f; sideCenter.RL.x = -202.0f; sideCenter.RR.x = -210.0f;
    if (gaitMode == GAIT_IDLE && !disablePidForSpecialPose) {
      standBalanceEnable = false;
      resetStandBalanceState(true);
      movePoseSet(stampCenter, false);
      setIdleBasePose(stampCenter);
      bootNeutralMode = false;
    }
    printGaitXStatus("X_RESET");
  }

  bool getPoseSetPtrByName(const char* name, PoseSet **out) {
    if (!name || !out) return false;
    if (!strcmp(name, "STAND")) { *out = &standPose; return true; }
    if (!strcmp(name, "SIT")) { *out = &sitPose; return true; }
    if (!strcmp(name, "LIE") || !strcmp(name, "NAM")) { *out = &liePose; return true; }
    if (!strcmp(name, "SHAKE") || !strcmp(name, "SHAKE_SIT") || !strcmp(name, "SHAKESIT")) { *out = &shakeSitPose; return true; }
    if (!strcmp(name, "STAMP") || !strcmp(name, "STOP")) { *out = &stampCenter; return true; }
    if (!strcmp(name, "FWD") || !strcmp(name, "FORWARD")) { *out = &forwardCenter; return true; }
    if (!strcmp(name, "BWD") || !strcmp(name, "BACKWARD")) { *out = &backwardCenter; return true; }
    if (!strcmp(name, "LEFT")) { *out = &leftCenter; return true; }
    if (!strcmp(name, "RIGHT")) { *out = &rightCenter; return true; }
    if (!strcmp(name, "SIDE") || !strcmp(name, "SL") || !strcmp(name, "SR") || !strcmp(name, "SIDE_LEFT") || !strcmp(name, "SIDE_RIGHT")) { *out = &sideCenter; return true; }
    return false;
  }
  void printPoseSetCsv(const char* name, const PoseSet &ps) {
    Serial.printf("POSE,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      name, ps.FL.x, ps.FL.y, ps.FL.z, ps.FR.x, ps.FR.y, ps.FR.z, ps.RL.x, ps.RL.y, ps.RL.z, ps.RR.x, ps.RR.y, ps.RR.z);
  }
  void printAllPoseSetsCsv() {
    printPoseSetCsv("STAND", standPose); printPoseSetCsv("SIT", sitPose); printPoseSetCsv("LIE", liePose);
    printPoseSetCsv("SHAKE", shakeSitPose); printPoseSetCsv("STAMP", stampCenter); printPoseSetCsv("FWD", forwardCenter);
    printPoseSetCsv("BWD", backwardCenter); printPoseSetCsv("LEFT", leftCenter); printPoseSetCsv("RIGHT", rightCenter); printPoseSetCsv("SIDE", sideCenter);
  }
  bool applyPoseByName(const char* name) {
    PoseSet *ps = nullptr;
    if (!getPoseSetPtrByName(name, &ps)) return false;
    stopGait(false);
    movePoseSet(*ps, false);
    setIdleBasePose(*ps);
    bootNeutralMode = false;
    printIK12RawDegFromPoseSet(*ps, true);
    Serial.printf("[OK] APPLY_POSE %s\n", name);
    return true;
  }
  void setLegInPose(PoseSet &ps, const char* leg, Vec3 p) {
    if (!strcmp(leg, "FL")) ps.FL = p;
    else if (!strcmp(leg, "FR")) ps.FR = p;
    else if (!strcmp(leg, "RL")) ps.RL = p;
    else if (!strcmp(leg, "RR")) ps.RR = p;
  }
  float getLegYInPose(const PoseSet &ps, const char* leg) {
    if (!strcmp(leg, "FL")) return ps.FL.y;
    if (!strcmp(leg, "FR")) return ps.FR.y;
    if (!strcmp(leg, "RL")) return ps.RL.y;
    if (!strcmp(leg, "RR")) return ps.RR.y;
    return 0.0f;
  }

  void shiftLateralLegY(LateralConfig &lat, const char* leg, float dy) {
    if (fabsf(dy) < 1e-6f) return;
    if (!strcmp(leg, "FL")) { lat.flStartY += dy; lat.flEndY += dy; }
    else if (!strcmp(leg, "FR")) { lat.frStartY += dy; lat.frEndY += dy; }
    else if (!strcmp(leg, "RL")) { lat.rlStartY += dy; lat.rlEndY += dy; }
    else if (!strcmp(leg, "RR")) { lat.rrStartY += dy; lat.rrEndY += dy; }
  }

  void syncLeftRightLateralByLeg(PoseSet *ps, const char* leg, float oldY, float newY) {
    float dy = newY - oldY;
    if (fabsf(dy) < 1e-6f) return;

    // LEFT/RIGHT dùng leftLateral/rightLateral để sinh quỹ đạo Y.
    // Khi GUI/Serial đổi Y bằng SET_POSE hoặc SET_PLEG, dời cả yStart/yEnd
    // theo đúng delta để chân thật sự chạy theo tọa độ Y mới.
    if (ps == &leftCenter) shiftLateralLegY(leftLateral, leg, dy);
    else if (ps == &rightCenter) shiftLateralLegY(rightLateral, leg, dy);
  }

  void syncLeftRightLateralByPoseSet(PoseSet *ps, const PoseSet &oldPs, const PoseSet &newPs) {
    syncLeftRightLateralByLeg(ps, "FL", oldPs.FL.y, newPs.FL.y);
    syncLeftRightLateralByLeg(ps, "FR", oldPs.FR.y, newPs.FR.y);
    syncLeftRightLateralByLeg(ps, "RL", oldPs.RL.y, newPs.RL.y);
    syncLeftRightLateralByLeg(ps, "RR", oldPs.RR.y, newPs.RR.y);
  }

  bool isLeftRightPosePtr(PoseSet *ps) {
    return (ps == &leftCenter || ps == &rightCenter);
  }

  bool handlePoseSerialCommand(const String &line) {
    char poseName[18] = {0};
    char legName[8] = {0};
    float flx, fly, flz, frx, fry, frz, rlx, rly, rlz, rrx, rry, rrz;
    float x, y, z;
    if (line == "SHOWPOSES" || line == "GET_POSES" || line == "DUMP_POSES") { printAllPoseSetsCsv(); return true; }
    if (sscanf(line.c_str(), "GET_POSE %17s", poseName) == 1 || sscanf(line.c_str(), "SHOW_POSE %17s", poseName) == 1) {
      PoseSet *ps = nullptr; if (!getPoseSetPtrByName(poseName, &ps)) { Serial.println("[ERR] GET_POSE unknown pose"); return true; }
      printPoseSetCsv(poseName, *ps); return true;
    }
    if (sscanf(line.c_str(), "APPLY_POSE %17s", poseName) == 1) {
      if (!applyPoseByName(poseName)) Serial.println("[ERR] APPLY_POSE unknown pose"); return true;
    }
    if (sscanf(line.c_str(), "SET_PLEG %17s %7s %f %f %f", poseName, legName, &x, &y, &z) == 5 ||
        sscanf(line.c_str(), "SET_POSE_LEG %17s %7s %f %f %f", poseName, legName, &x, &y, &z) == 5) {
      PoseSet *ps = nullptr; if (!getPoseSetPtrByName(poseName, &ps)) { Serial.println("[ERR] SET_PLEG unknown pose"); return true; }
      if (strcmp(legName, "FL") && strcmp(legName, "FR") && strcmp(legName, "RL") && strcmp(legName, "RR")) { Serial.println("[ERR] SET_PLEG unknown leg"); return true; }
      float oldY = getLegYInPose(*ps, legName);
      setLegInPose(*ps, legName, {x, y, z});
      syncLeftRightLateralByLeg(ps, legName, oldY, y);
      Serial.printf("[OK] SET_PLEG %s %s %.2f %.2f %.2f\n", poseName, legName, x, y, z);
      printPoseSetCsv(poseName, *ps);
      if (isLeftRightPosePtr(ps)) showSideConfig();
      return true;
    }
    if (sscanf(line.c_str(), "SET_POSE %17s %f %f %f %f %f %f %f %f %f %f %f %f",
      poseName, &flx, &fly, &flz, &frx, &fry, &frz, &rlx, &rly, &rlz, &rrx, &rry, &rrz) == 13) {
      PoseSet *ps = nullptr; if (!getPoseSetPtrByName(poseName, &ps)) { Serial.println("[ERR] SET_POSE unknown pose"); return true; }
      PoseSet oldPs = *ps;
      *ps = {{flx, fly, flz}, {frx, fry, frz}, {rlx, rly, rlz}, {rrx, rry, rrz}};
      syncLeftRightLateralByPoseSet(ps, oldPs, *ps);
      Serial.printf("[OK] SET_POSE %s\n", poseName);
      printPoseSetCsv(poseName, *ps);
      if (isLeftRightPosePtr(ps)) showSideConfig();
      return true;
    }
    return false;
  }

  // ===== HELP / COMMANDS =====
  void printHelp() {
    Serial.println("=== Quadruped gait tunable build ===");
    Serial.println("Move: STAND / NEUTRAL / HOME / SIT / LIE / SHAKE / FWD / BWD / LEFT / RIGHT / SL / SR / STOP");
    Serial.println("Gait tuning:");
    Serial.println("  SET_GAIT MODE hStep tCycle phase swingRatio    // MODE=FWD/BWD/LEFT/RIGHT/SL/SR");
    Serial.println("  SET_GAIT_HSTEP MODE v | SET_GAIT_TCYCLE MODE v | SET_GAIT_PHASE MODE v | SET_GAIT_SWING MODE v");
    Serial.println("  SHOW_GAITS | RESET_GAITS | SHOWGAIT");
    Serial.println("Legacy gait: SET_FB hStep tCycle, SET_SWING_FB ratio, SET_LR hStep tCycle, SET_SIDE hStep tCycle, SET_SWING_SIDE ratio, SET_PHASE_SIDE phase");
    Serial.println("Height X: X+ / X- / X_RESET / SET_X_STEP v / SET_X_LIMITS min max / SHOWX");
    Serial.println("Balance: BAL_ON/OFF/SHOWBAL, SBAL_ON/OFF/SHOWSBAL, SET_SBAL kp ki kd maxX deadband hold, SET_SBAL_SP roll pitch");
    Serial.println("Camera: CAMGAIT_ON/OFF, CAMLOCK_ON/OFF, SET_CAMGAIT kp ki kd maxAmp deadband hold iMax, SET_CAMSTOP maxBoxH lostMs");
    Serial.println("Pose config: SET_POSE, SET_PLEG, GET_POSE, SHOWPOSES, APPLY_POSE");
    Serial.println("LEFT/RIGHT Y: SET_POSE/SET_PLEG LEFT/RIGHT tu dong doi ca yStart/yEnd cua quy dao");
    Serial.println("IK serial: IK12_ON / IK12_OFF / IK12_NOW");
  }

  void handleCommand(String line) {
    line.trim();
    if (parseCameraCsv(line)) return;
    line.toUpperCase();
    if (handlePoseSerialCommand(line)) return;

    if (line == "HELP") { printHelp(); return; }
    if (line == "IK12_ON") { setIK12Serial(true); return; }
    if (line == "IK12_OFF") { setIK12Serial(false); return; }
    if (line == "IK12_NOW") { printIK12RawDegFromPoseSet(getCurrentPoseSet(), true); return; }

    if (line == "X+" || line == "X +" || line == "X_PLUS" || line == "HEIGHT_UP" || line == "CAO+" || line == "CAO +") { applyGaitXDelta(gaitXStep, true); return; }
    if (line == "X-" || line == "X -" || line == "X_MINUS" || line == "HEIGHT_DOWN" || line == "CAO-" || line == "CAO -") { applyGaitXDelta(-gaitXStep, true); return; }
    if (line == "X_RESET" || line == "HEIGHT_RESET" || line == "X_HOME") { resetGaitXToDefault(); return; }
    if (line == "SHOWX" || line == "SHOW_X") { printGaitXStatus("SHOWX"); return; }
    float one;
    if (sscanf(line.c_str(), "SET_X_STEP %f", &one) == 1) { gaitXStep = clampF(fabsf(one), 0.1f, 50.0f); Serial.printf("[OK] SET_X_STEP %.2f\n", gaitXStep); return; }
    float a, b;
    if (sscanf(line.c_str(), "SET_X_LIMITS %f %f", &a, &b) == 2) { gaitXMin = fminf(a,b); gaitXMax = fmaxf(a,b); Serial.printf("[OK] SET_X_LIMITS min=%.2f max=%.2f\n", gaitXMin, gaitXMax); return; }

    if (line == "STAND") {
      disablePidForSpecialPose = true;
      if (bootNeutralMode) { movePoseSet(standPose, false); setIdleBasePose(standPose); bootNeutralMode = false; }
      else transitionPoseSet(standPose, poseTransitionSec, false);
      disablePidForSpecialPose = false;
      resetStandBalanceState(true);
      Serial.println("[OK] STAND | Balance PID mac dinh OFF, muon can bang dung SBAL_ON");
      return;
    }
    if (line == "NEUTRAL" || line == "NATURAL" || line == "HOME" || line == "SERVO_NEUTRAL") {
      stopGait(false);
      disablePidForSpecialPose = true;
      neutralPose();
      bootNeutralMode = true;
      idleBaseValid = false;
      resetWalkBalanceState(true); resetStandBalanceState(true); resetYawGaitState(true); resetCamGaitState(true);
      saveBalanceMixOut({0.0f, 0.0f});
      saveBalanceDx("FL", 0.0f); saveBalanceDx("FR", 0.0f); saveBalanceDx("RL", 0.0f); saveBalanceDx("RR", 0.0f);
      Serial.println("[OK] NEUTRAL/NATURAL/HOME: servo = 90 + offset, chua vao IK standPose");
      return;
    }
    if (line == "SIT") { disablePidForSpecialPose = true; transitionPoseSet(sitPose, poseTransitionSec, false); Serial.println("[OK] SIT SMOOTH (PID Disabled)"); return; }
    if (line == "LIE" || line == "NAM") { disablePidForSpecialPose = true; transitionPoseSet(liePose, poseTransitionSec, false); Serial.println("[OK] LIE SMOOTH (PID Disabled)"); return; }
    if (line == "SHAKE" || line == "BAT_TAY" || line == "BAT_TAY_TRAI" || line == "SHAKE_FL") { disablePidForSpecialPose = true; handshakeMode(); return; }

    if (line == "STAMP") { startGait(GAIT_STAMP, "STAMP"); return; }
    if (line == "FWD" || line == "FORWARD") { startGait(GAIT_FORWARD, "FORWARD"); return; }
    if (line == "BWD" || line == "BACKWARD") { startGait(GAIT_BACKWARD, "BACKWARD"); return; }
    if (line == "LEFT") { startGait(GAIT_LEFT, "LEFT_ROTATE"); return; }
    if (line == "RIGHT") { startGait(GAIT_RIGHT, "RIGHT_ROTATE"); return; }
    if (line == "SIDE_LEFT" || line == "SL" || line == "STRAFE_LEFT") { startGait(GAIT_SIDE_LEFT, "SIDE_LEFT"); return; }
    if (line == "SIDE_RIGHT" || line == "SR" || line == "STRAFE_RIGHT") { startGait(GAIT_SIDE_RIGHT, "SIDE_RIGHT"); return; }
    if (line == "GAIT_STOP" || line == "STOP") { stopGait(true); Serial.println("[OK] STOP"); return; }

    if (line == "SHOWPOSE") { showPose(); return; }
    if (line == "SHOWCALC") { showCalc(); return; }
    if (line == "SHOWSIDE") { showSideConfig(); return; }
    if (line == "SHOWGAIT" || line == "SHOW_GAIT") { showGaitConfig(); return; }
    if (line == "SHOW_GAITS" || line == "SHOWGAITS" || line == "GAIT_SHOW") { showAllGaitRuntimeConfig(); return; }
    if (line == "RESET_GAITS" || line == "RESETGAITS" || line == "GAIT_RESET") { resetGaitRuntimeDefaults(); showAllGaitRuntimeConfig(); return; }

    char gaitNameCmd[18] = {0};
    float gh, gt, gp, gs;
    if (sscanf(line.c_str(), "SET_GAIT %17s %f %f %f %f", gaitNameCmd, &gh, &gt, &gp, &gs) == 5) {
      GaitMode gm; if (!parseGaitModeName(gaitNameCmd, gm)) { Serial.println("[ERR] SET_GAIT unknown mode"); return; }
      setGaitCfg(gm, gh, gt, gp, gs); resetGaitPhaseClockIfRunning(gm);
      Serial.printf("[OK] SET_GAIT %s hStep=%.2f tCycle=%.3f phase=%.3f swing=%.3f\n",
        gaitModeToName(gm), getGaitCfg(gm).hStep, getGaitCfg(gm).tCycle, getGaitCfg(gm).phaseOffset, getGaitCfg(gm).swingRatio);
      printGaitAck(gm, "SET_GAIT");
      return;
    }
    if (sscanf(line.c_str(), "SET_GAIT_HSTEP %17s %f", gaitNameCmd, &one) == 2) {
      GaitMode gm; if (!parseGaitModeName(gaitNameCmd, gm)) { Serial.println("[ERR] SET_GAIT_HSTEP unknown mode"); return; }
      GaitRuntimeConfig cfg = getGaitCfg(gm); setGaitCfg(gm, one, cfg.tCycle, cfg.phaseOffset, cfg.swingRatio); resetGaitPhaseClockIfRunning(gm); printOneGaitCfg(gm); printGaitAck(gm, "SET_GAIT_HSTEP"); return;
    }
    if (sscanf(line.c_str(), "SET_GAIT_TCYCLE %17s %f", gaitNameCmd, &one) == 2) {
      GaitMode gm; if (!parseGaitModeName(gaitNameCmd, gm)) { Serial.println("[ERR] SET_GAIT_TCYCLE unknown mode"); return; }
      GaitRuntimeConfig cfg = getGaitCfg(gm); setGaitCfg(gm, cfg.hStep, one, cfg.phaseOffset, cfg.swingRatio); resetGaitPhaseClockIfRunning(gm); printOneGaitCfg(gm); printGaitAck(gm, "SET_GAIT_TCYCLE"); return;
    }
    if (sscanf(line.c_str(), "SET_GAIT_PHASE %17s %f", gaitNameCmd, &one) == 2) {
      GaitMode gm; if (!parseGaitModeName(gaitNameCmd, gm)) { Serial.println("[ERR] SET_GAIT_PHASE unknown mode"); return; }
      GaitRuntimeConfig cfg = getGaitCfg(gm); setGaitCfg(gm, cfg.hStep, cfg.tCycle, one, cfg.swingRatio); resetGaitPhaseClockIfRunning(gm); printOneGaitCfg(gm); printGaitAck(gm, "SET_GAIT_PHASE"); return;
    }
    if (sscanf(line.c_str(), "SET_GAIT_SWING %17s %f", gaitNameCmd, &one) == 2) {
      GaitMode gm; if (!parseGaitModeName(gaitNameCmd, gm)) { Serial.println("[ERR] SET_GAIT_SWING unknown mode"); return; }
      GaitRuntimeConfig cfg = getGaitCfg(gm); setGaitCfg(gm, cfg.hStep, cfg.tCycle, cfg.phaseOffset, one); resetGaitPhaseClockIfRunning(gm); printOneGaitCfg(gm); printGaitAck(gm, "SET_GAIT_SWING"); return;
    }

    float hStepCmd, tCycleCmd, swingCmd, sidePhaseCmd;
    if (sscanf(line.c_str(), "SET_FB %f %f", &hStepCmd, &tCycleCmd) == 2) {
      gaitHStepFB = hStepCmd; gaitTCycleFB = clampF(tCycleCmd, 0.05f, 10.0f);
      setGaitCfg(GAIT_FORWARD, gaitHStepFB, gaitTCycleFB, getGaitCfg(GAIT_FORWARD).phaseOffset, gaitSwingRatioFB);
      setGaitCfg(GAIT_BACKWARD, gaitHStepFB, gaitTCycleFB, getGaitCfg(GAIT_BACKWARD).phaseOffset, gaitSwingRatioFB);
      Serial.printf("[OK] SET_FB legacy -> FWD/BWD hStep=%.2f tCycle=%.3f swingRatio=%.3f\n", gaitHStepFB, gaitTCycleFB, gaitSwingRatioFB); return;
    }
    if (sscanf(line.c_str(), "SET_SWING_FB %f", &swingCmd) == 1) {
      gaitSwingRatioFB = clampF(swingCmd, 0.05f, 0.49f);
      setGaitCfg(GAIT_FORWARD, getGaitCfg(GAIT_FORWARD).hStep, getGaitCfg(GAIT_FORWARD).tCycle, getGaitCfg(GAIT_FORWARD).phaseOffset, gaitSwingRatioFB);
      setGaitCfg(GAIT_BACKWARD, getGaitCfg(GAIT_BACKWARD).hStep, getGaitCfg(GAIT_BACKWARD).tCycle, getGaitCfg(GAIT_BACKWARD).phaseOffset, gaitSwingRatioFB);
      Serial.printf("[OK] SET_SWING_FB legacy -> FWD/BWD %.3f\n", gaitSwingRatioFB); return;
    }
    if (sscanf(line.c_str(), "SET_LR %f %f", &hStepCmd, &tCycleCmd) == 2) {
      gaitHStepLR = hStepCmd; gaitTCycleLR = clampF(tCycleCmd, 0.05f, 10.0f);
      setGaitCfg(GAIT_LEFT, gaitHStepLR, gaitTCycleLR, getGaitCfg(GAIT_LEFT).phaseOffset, getGaitCfg(GAIT_LEFT).swingRatio);
      setGaitCfg(GAIT_RIGHT, gaitHStepLR, gaitTCycleLR, getGaitCfg(GAIT_RIGHT).phaseOffset, getGaitCfg(GAIT_RIGHT).swingRatio);
      Serial.printf("[OK] SET_LR legacy -> LEFT/RIGHT hStep=%.2f tCycle=%.3f\n", gaitHStepLR, gaitTCycleLR); return;
    }
    if (sscanf(line.c_str(), "SET_SIDE %f %f", &hStepCmd, &tCycleCmd) == 2) {
      gaitHStepSide = hStepCmd; gaitTCycleSide = clampF(tCycleCmd, 0.05f, 10.0f);
      setGaitCfg(GAIT_SIDE_LEFT, gaitHStepSide, gaitTCycleSide, gaitSidePhaseOffset, gaitSwingRatioSide);
      setGaitCfg(GAIT_SIDE_RIGHT, gaitHStepSide, gaitTCycleSide, gaitSidePhaseOffset, gaitSwingRatioSide);
      Serial.printf("[OK] SET_SIDE legacy -> SL/SR hStep=%.2f tCycle=%.3f swingRatio=%.3f phase=%.3f\n", gaitHStepSide, gaitTCycleSide, gaitSwingRatioSide, gaitSidePhaseOffset); return;
    }
    if (sscanf(line.c_str(), "SET_SWING_SIDE %f", &swingCmd) == 1) {
      gaitSwingRatioSide = clampF(swingCmd, 0.05f, 0.49f);
      setGaitCfg(GAIT_SIDE_LEFT, getGaitCfg(GAIT_SIDE_LEFT).hStep, getGaitCfg(GAIT_SIDE_LEFT).tCycle, getGaitCfg(GAIT_SIDE_LEFT).phaseOffset, gaitSwingRatioSide);
      setGaitCfg(GAIT_SIDE_RIGHT, getGaitCfg(GAIT_SIDE_RIGHT).hStep, getGaitCfg(GAIT_SIDE_RIGHT).tCycle, getGaitCfg(GAIT_SIDE_RIGHT).phaseOffset, gaitSwingRatioSide);
      Serial.printf("[OK] SET_SWING_SIDE legacy -> SL/SR %.3f\n", gaitSwingRatioSide); return;
    }
    if (sscanf(line.c_str(), "SET_PHASE_SIDE %f", &sidePhaseCmd) == 1) {
      gaitSidePhaseOffset = normPhase01(sidePhaseCmd);
      setGaitCfg(GAIT_SIDE_LEFT, getGaitCfg(GAIT_SIDE_LEFT).hStep, getGaitCfg(GAIT_SIDE_LEFT).tCycle, gaitSidePhaseOffset, getGaitCfg(GAIT_SIDE_LEFT).swingRatio);
      setGaitCfg(GAIT_SIDE_RIGHT, getGaitCfg(GAIT_SIDE_RIGHT).hStep, getGaitCfg(GAIT_SIDE_RIGHT).tCycle, gaitSidePhaseOffset, getGaitCfg(GAIT_SIDE_RIGHT).swingRatio);
      Serial.printf("[OK] SET_PHASE_SIDE legacy -> SL/SR %.3f\n", gaitSidePhaseOffset); return;
    }

    float sideAmpOne, sideAmpFL, sideAmpFR, sideAmpRL, sideAmpRR;
    if (sscanf(line.c_str(), "SET_SIDE_AMP %f %f %f %f", &sideAmpFL, &sideAmpFR, &sideAmpRL, &sideAmpRR) == 4) {
      gaitSideAmpFL = fabsf(sideAmpFL); gaitSideAmpFR = fabsf(sideAmpFR); gaitSideAmpRL = fabsf(sideAmpRL); gaitSideAmpRR = fabsf(sideAmpRR);
      Serial.printf("[OK] SET_SIDE_AMP FL=%.2f FR=%.2f RL=%.2f RR=%.2f\n", gaitSideAmpFL, gaitSideAmpFR, gaitSideAmpRL, gaitSideAmpRR); return;
    }
    if (sscanf(line.c_str(), "SET_SIDE_AMP %f", &sideAmpOne) == 1) {
      sideAmpOne = fabsf(sideAmpOne); gaitSideAmpFL = sideAmpOne; gaitSideAmpFR = sideAmpOne; gaitSideAmpRL = sideAmpOne; gaitSideAmpRR = sideAmpOne;
      Serial.printf("[OK] SET_SIDE_AMP all=%.2f\n", sideAmpOne); return;
    }
    if (line == "SIDE_INV") { gaitSideInvert = true; Serial.println("[OK] SIDE_INV"); return; }
    if (line == "SIDE_NORMAL") { gaitSideInvert = false; Serial.println("[OK] SIDE_NORMAL"); return; }

    if (line == "MPUCALIB") { calibrateAccelNonFatal(); return; }
    if (line == "YAWZERO") { yawDeg = 0.0f; gzDpsLpf = 0.0f; Serial.println("[OK] YAWZERO"); return; }
    if (line == "MPUSHOW") { Serial.printf("[MPU] roll=%.2f pitch=%.2f yaw=%.2f | ax=%.1f ay=%.1f az=%.1f | gzLpf=%.2f dps\n", rollDeg, pitchDeg, yawDeg, axLpf, ayLpf, azLpf, gzDpsLpf); return; }
    if (line == "MPUPLOT_ON") { mpuPrintEnable = true; mpuPlotMode = true; balanceOffsetPrintEnable = false; Serial.println("[OK] MPUPLOT_ON"); return; }
    if (line == "MPUPLOT_OFF") { mpuPrintEnable = true; mpuPlotMode = false; Serial.println("[OK] MPUPLOT_OFF"); return; }
    if (line == "MPUPRINT_ON") { mpuPrintEnable = true; Serial.println("[OK] MPUPRINT_ON"); return; }
    if (line == "MPUPRINT_OFF") { mpuPrintEnable = false; Serial.println("[OK] MPUPRINT_OFF"); return; }

    float v1, v2, v3, v4, v5, v6, v7;
    if (sscanf(line.c_str(), "SET_MPU_ALPHA %f %f", &v1, &v2) == 2) { mpuAlphaGait = clampF(v1,0,1); mpuAlphaStand = clampF(v2,0,1); Serial.printf("[OK] SET_MPU_ALPHA gait=%.3f stand=%.3f\n", mpuAlphaGait, mpuAlphaStand); return; }
    if (sscanf(line.c_str(), "SET_YAW_FILTER %f %f %f", &v1, &v2, &v3) == 3) { gyroZAlpha = clampF(v1,0,1); yawDeadbandDps = fabsf(v2); yawStillDps = fabsf(v3); Serial.printf("[OK] SET_YAW_FILTER alpha=%.3f deadband=%.2f still=%.2f\n", gyroZAlpha, yawDeadbandDps, yawStillDps); return; }

    if (line == "BAL_ON") { balanceEnable = true; Serial.println("[OK] BAL_ON"); return; }
    if (line == "BAL_OFF") { balanceEnable = false; resetWalkBalanceState(true); Serial.println("[OK] BAL_OFF"); return; }
    if (line == "SHOWBAL") { Serial.printf("[BAL] en=%d kp=%.3f ki=%.3f kd=%.3f maxX=%.2f deadband=%.2f hold=%.2f iMax=%.2f roll=%.2f pitch=%.2f | OFF FL=%.2f FR=%.2f RL=%.2f RR=%.2f\n", balanceEnable, balKp, balKi, balKd, balMaxX, balDeadbandDeg, balResetErrDeg, balIMax, rollDeg, pitchDeg, lastDxFL, lastDxFR, lastDxRL, lastDxRR); return; }
    if (line == "SBAL_ON") { standBalanceEnable = true; setIdleBaseFromCurrent(); Serial.println("[OK] SBAL_ON"); return; }
    if (line == "SBAL_OFF") { standBalanceEnable = false; resetStandBalanceState(true); setIdleBaseFromCurrent(); Serial.println("[OK] SBAL_OFF"); return; }
    if (line == "SHOWSBAL") { Serial.printf("[SBAL] en=%d kp=%.3f ki=%.3f kd=%.3f maxX=%.2f deadband=%.2f hold=%.2f iMax=%.2f | roll=%.2f spR=%.2f | pitch=%.2f spP=%.2f | OFF FL=%.2f FR=%.2f RL=%.2f RR=%.2f\n", standBalanceEnable, standBalKp, standBalKi, standBalKd, standBalMaxX, standBalDeadbandDeg, standBalResetErrDeg, standBalIMax, rollDeg, standBalRollSetpointDeg, pitchDeg, standBalPitchSetpointDeg, lastDxFL, lastDxFR, lastDxRL, lastDxRR); return; }
    if (sscanf(line.c_str(), "SET_BAL %f %f %f %f %f %f", &v1, &v2, &v3, &v4, &v5, &v6) >= 5) {
      balKp = v1; balKi = v2; balKd = v3; balMaxX = clampF(v4,0,80); balDeadbandDeg = clampF(v5,0,20); balResetErrDeg = clampF(v6,0,balDeadbandDeg); resetWalkBalanceState(true); Serial.println("[OK] SET_BAL"); return;
    }
    int setSBalN = sscanf(line.c_str(), "SET_SBAL %f %f %f %f %f %f", &v1, &v2, &v3, &v4, &v5, &v6);
    if (setSBalN >= 3) {
      standBalKp = v1; standBalKi = v2; standBalKd = v3;
      if (setSBalN >= 4) standBalMaxX = clampF(v4,0,80);
      if (setSBalN >= 5) standBalDeadbandDeg = clampF(v5,0,20);
      if (setSBalN >= 6) standBalResetErrDeg = clampF(v6,0,standBalDeadbandDeg);
      resetStandBalanceState(true); setIdleBaseFromCurrent();
      Serial.printf("[OK] SET_SBAL kp=%.4f ki=%.4f kd=%.4f maxX=%.2f deadband=%.2f hold=%.2f\n", standBalKp, standBalKi, standBalKd, standBalMaxX, standBalDeadbandDeg, standBalResetErrDeg); return;
    }
    if (sscanf(line.c_str(), "SET_SBAL_SP %f %f", &v1, &v2) == 2) { standBalRollSetpointDeg = clampF(v1,-45,45); standBalPitchSetpointDeg = clampF(v2,-45,45); resetStandBalanceState(true); setIdleBaseFromCurrent(); Serial.printf("[OK] SET_SBAL_SP rollSP=%.2f pitchSP=%.2f\n", standBalRollSetpointDeg, standBalPitchSetpointDeg); return; }
    if (line == "SBAL_SP_ZERO" || line == "SET_SBAL_SP_ZERO") { standBalRollSetpointDeg = 0; standBalPitchSetpointDeg = 0; resetStandBalanceState(true); setIdleBaseFromCurrent(); Serial.println("[OK] SBAL_SP_ZERO"); return; }
    if (line == "OFFDBG_ON") { balanceOffsetPrintEnable = true; Serial.println("[OK] OFFDBG_ON"); return; }
    if (line == "OFFDBG_OFF") { balanceOffsetPrintEnable = false; Serial.println("[OK] OFFDBG_OFF"); return; }

    if (line == "YAWGAIT_ON") { yawGaitOnlyEnable = true; yawGaitSetpointDeg = yawDeg; resetYawGaitState(true); Serial.printf("[OK] YAWGAIT_ON setpoint=%.2f\n", yawGaitSetpointDeg); return; }
    if (line == "YAWGAIT_OFF") { yawGaitOnlyEnable = false; resetYawGaitState(true); Serial.println("[OK] YAWGAIT_OFF"); return; }
    if (sscanf(line.c_str(), "SET_YAWGAIT %f %f %f %f %f %f %f", &v1, &v2, &v3, &v4, &v5, &v6, &v7) >= 4) {
      yawGaitKp=v1; yawGaitKi=v2; yawGaitKd=v3; yawGaitMaxAmp=fabsf(v4); yawGaitDeadbandDeg=fabsf(v5); yawGaitHoldDeg=clampF(fabsf(v6),0,yawGaitDeadbandDeg); yawGaitIMax=fabsf(v7); resetYawGaitState(true); Serial.println("[OK] SET_YAWGAIT"); return;
    }
    if (line == "YAWGAIT_INV_SIDE") { yawGaitInvertSide = true; resetYawGaitState(true); Serial.println("[OK] YAWGAIT_INV_SIDE"); return; }
    if (line == "YAWGAIT_NORMAL_SIDE") { yawGaitInvertSide = false; resetYawGaitState(true); Serial.println("[OK] YAWGAIT_NORMAL_SIDE"); return; }
    if (line == "SHOWYAWGAIT") { Serial.printf("[YAWGAIT] en=%d invSide=%d set=%.2f yaw=%.2f out=%.2f kp=%.3f ki=%.3f kd=%.3f maxAmp=%.2f\n", yawGaitOnlyEnable, yawGaitInvertSide, yawGaitSetpointDeg, yawDeg, yawGaitLastOut, yawGaitKp, yawGaitKi, yawGaitKd, yawGaitMaxAmp); return; }

    if (line == "CAMGAIT_ON") { camGaitEnable = true; camAlignRotateEnable = true; yawGaitOnlyEnable = false; balanceEnable = false; camSeenSinceEnable = false; camLastRxMs = 0; camAlignLastPrintMs = 0; resetCamGaitState(true); Serial.printf("[OK] CAMGAIT_ON | outside +-%.0f -> SL/SR\n", camAlignRangePx); return; }
    if (line == "CAMGAIT_OFF") { camGaitEnable = false; camAutoPaused = false; camSeenSinceEnable = false; resetCamGaitState(true); Serial.println("[OK] CAMGAIT_OFF"); return; }
    if (line == "CAMLOCK_ON") { camRequireOffsetBeforeWalk = true; camGaitEnable = true; camSeenSinceEnable = false; camLastRxMs = 0; camAutoPaused = false; resetCamGaitState(true); Serial.println("[OK] CAMLOCK_ON"); return; }
    if (line == "CAMLOCK_OFF") { camRequireOffsetBeforeWalk = false; camAutoPaused = false; resetCamGaitState(true); Serial.println("[OK] CAMLOCK_OFF"); return; }
    if (line == "CAMGAIT_INV_SIDE") { camGaitInvertSide = true; resetCamGaitState(true); Serial.println("[OK] CAMGAIT_INV_SIDE"); return; }
    if (line == "CAMGAIT_NORMAL_SIDE") { camGaitInvertSide = false; resetCamGaitState(true); Serial.println("[OK] CAMGAIT_NORMAL_SIDE"); return; }
    if (line == "CAMALIGN_ON") { camAlignRotateEnable = true; camAlignLastPrintMs = 0; resetCamGaitState(true); Serial.println("[OK] CAMALIGN_ON"); return; }
    if (line == "CAMALIGN_OFF") { camAlignRotateEnable = false; camAlignLastPrintMs = 0; resetCamGaitState(true); Serial.println("[OK] CAMALIGN_OFF"); return; }
    if (line == "CAMALIGN_INV") { camAlignRotateInvert = true; camAlignLastPrintMs = 0; resetCamGaitState(true); Serial.println("[OK] CAMALIGN_INV"); return; }
    if (line == "CAMALIGN_NORMAL") { camAlignRotateInvert = false; camAlignLastPrintMs = 0; resetCamGaitState(true); Serial.println("[OK] CAMALIGN_NORMAL"); return; }
    if (sscanf(line.c_str(), "SET_CAMALIGN_RANGE %f", &one) == 1 || sscanf(line.c_str(), "SET_CAMALIGN %f", &one) == 1) { camAlignRangePx = fmaxf(1.0f, fabsf(one)); resetCamGaitState(true); Serial.printf("[OK] SET_CAMALIGN_RANGE +-%.0f px\n", camAlignRangePx); return; }
    int stopH=0, arg2=0, arg3=0;
    int camStopN = sscanf(line.c_str(), "SET_CAMSTOP %d %d %d", &stopH, &arg2, &arg3);
    if (camStopN >= 1) { camStopBoxH = stopH; if (camStopN >= 3) camLostStopMs = (unsigned long)max(100, arg3); else if (camStopN >= 2) camLostStopMs = (unsigned long)max(100, arg2); Serial.printf("[OK] SET_CAMSTOP maxBoxH=%d lostMs=%lu | width ignored\n", camStopBoxH, camLostStopMs); return; }
    if (sscanf(line.c_str(), "SET_CAM_TARGET %f", &one) == 1) { camTargetOffsetPx = one; resetCamGaitState(true); Serial.printf("[OK] SET_CAM_TARGET %.1f px\n", camTargetOffsetPx); return; }
    int camN = sscanf(line.c_str(), "SET_CAMGAIT %f %f %f %f %f %f %f", &v1, &v2, &v3, &v4, &v5, &v6, &v7);
    if (camN >= 4) { camKp=v1; camKi=v2; camKd=v3; camMaxAmp=fabsf(v4); if (camN>=5) camDeadbandPx=fabsf(v5); if (camN>=6) camHoldPx=clampF(fabsf(v6),0,camDeadbandPx); if (camN>=7) camIMax=fabsf(v7); resetCamGaitState(true); Serial.println("[OK] SET_CAMGAIT"); return; }
    if (sscanf(line.c_str(), "SET_CAMMINAMP %f", &one) == 1) { camMinAmp = clampF(fabsf(one),0,200); Serial.printf("[OK] SET_CAMMINAMP %.2f\n", camMinAmp); return; }
    if (line == "SHOWCAMGAIT") { Serial.printf("[CAMGAIT] en=%d lock=%d inv=%d seen=%d det=%d fresh=%d paused=%d offset=%.1f target=%.1f out=%.2f box=%dx%d stopH=%d lostMs=%lu kp=%.3f ki=%.3f kd=%.3f maxAmp=%.2f minAmp=%.2f\n", camGaitEnable, camRequireOffsetBeforeWalk, camGaitInvertSide, camSeenSinceEnable, camDetected, camDataFresh(), camAutoPaused, camOffsetX, camTargetOffsetPx, camLastOut, camBoxW, camBoxH, camStopBoxH, camLostStopMs, camKp, camKi, camKd, camMaxAmp, camMinAmp); return; }

    if (sscanf(line.c_str(), "SET_POSE_TIME %f", &one) == 1) { poseTransitionSec = clampF(one, 0.05f, 10.0f); Serial.printf("[OK] SET_POSE_TIME %.3f sec\n", poseTransitionSec); return; }

    float yStart, yEnd;
    if (sscanf(line.c_str(), "LEFT_FL_Y %f %f", &yStart, &yEnd) == 2) { leftLateral.flStartY=yStart; leftLateral.flEndY=yEnd; Serial.println("[OK] LEFT_FL_Y"); return; }
    if (sscanf(line.c_str(), "LEFT_FR_Y %f %f", &yStart, &yEnd) == 2) { leftLateral.frStartY=yStart; leftLateral.frEndY=yEnd; Serial.println("[OK] LEFT_FR_Y"); return; }
    if (sscanf(line.c_str(), "LEFT_RL_Y %f %f", &yStart, &yEnd) == 2) { leftLateral.rlStartY=yStart; leftLateral.rlEndY=yEnd; Serial.println("[OK] LEFT_RL_Y"); return; }
    if (sscanf(line.c_str(), "LEFT_RR_Y %f %f", &yStart, &yEnd) == 2) { leftLateral.rrStartY=yStart; leftLateral.rrEndY=yEnd; Serial.println("[OK] LEFT_RR_Y"); return; }
    if (sscanf(line.c_str(), "RIGHT_FL_Y %f %f", &yStart, &yEnd) == 2) { rightLateral.flStartY=yStart; rightLateral.flEndY=yEnd; Serial.println("[OK] RIGHT_FL_Y"); return; }
    if (sscanf(line.c_str(), "RIGHT_FR_Y %f %f", &yStart, &yEnd) == 2) { rightLateral.frStartY=yStart; rightLateral.frEndY=yEnd; Serial.println("[OK] RIGHT_FR_Y"); return; }
    if (sscanf(line.c_str(), "RIGHT_RL_Y %f %f", &yStart, &yEnd) == 2) { rightLateral.rlStartY=yStart; rightLateral.rlEndY=yEnd; Serial.println("[OK] RIGHT_RL_Y"); return; }
    if (sscanf(line.c_str(), "RIGHT_RR_Y %f %f", &yStart, &yEnd) == 2) { rightLateral.rrStartY=yStart; rightLateral.rrEndY=yEnd; Serial.println("[OK] RIGHT_RR_Y"); return; }

    char legName[8]; float x, y, z;
    if (sscanf(line.c_str(), "LEG %7s %f %f %f", legName, &x, &y, &z) == 4) {
      stopGait(false);
      if (moveLeg(legName, {x,y,z}, true)) { setIdleBaseFromCurrent(); Serial.println("[OK] LEG"); }
      else Serial.println("[ERR] IK FAIL");
      return;
    }

    Serial.println("[ERR] Unknown command. Type HELP.");
  }
};

QuadrupedController robot;

void setup() {
  robot.begin();
}

void loop() {
  robot.update();
}
