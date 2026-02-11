#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ================= PCA9685 =================
Adafruit_PWMServoDriver pca(0x40);

// ================= ESP32 I2C =================
#define I2C_SDA 21
#define I2C_SCL 22

// ================= Servo settings =================
#define SERVO_FREQ 50
#define SERVO1_CH 0   // HGO (0..180)  -> Servo1
#define SERVO2_CH 1   // BAO (âm)      -> Servo2 (map từ BAO)

// --------- Calib microsecond ----------
#define SERVO1_US_MIN 600
#define SERVO1_US_MAX 2400
#define SERVO2_US_MIN 600
#define SERVO2_US_MAX 2400

// ====== OFFSET: Servo1 xuất thêm so với tính toán ======
static constexpr float SERVO1_OFFSET_DEG = 0.0f; // muốn +5 thì set 5.0f

// ================= BAO giới hạn như MATLAB =================
static constexpr float BAO_MIN = -200.0f;
static constexpr float BAO_MAX = -90.0f;

// ================= LOCK NHÁNH (GIỐNG MATLAB) =================
static constexpr int LOCK_F_SIGN = -1; // cross((H-A),(F-A))
static constexpr int LOCK_E_SIGN = -1; // cross((F-A),(E-A))
static constexpr int LOCK_D_SIGN = -1; // cross((E-B),(D-B))

// ================= Cơ cấu (mm) =================
static constexpr float LAB = 130.0f;
static constexpr float LBC = 140.0f;
static constexpr float LBD = 37.0f;
static constexpr float LED = 115.0f;

static constexpr float LAE = 43.0f;
static constexpr float LAF = 43.0f;
static constexpr float LEF = 67.07f;

static constexpr float dGx = -20.18f;
static constexpr float dGy = -29.05f;

static constexpr float LGH = 29.0f;
static constexpr float LHF = 40.0f;

// ================= UART buffer =================
String uartLine;

// ================= AUTO SEQUENCE =================
// BẠN CHỐT: mỗi phần tử là { BAO , góc B }
struct StepPair { float bao; float betaB; };

static const StepPair kSeq[] = {
{  -121.9f,    107.9f}, // 1
{  -113.9f,     92.2f}, // 3
{  -112.0f,     85.7f}, // 5
{  -111.4f,     80.7f}, // 7
{  -111.6f,     76.8f}, // 9
{  -112.5f,     73.6f}, // 11
{  -114.0f,     71.1f}, // 13
{  -115.9f,     69.2f}, // 15
{  -118.4f,     67.9f}, // 17
{  -121.2f,     67.2f}, // 19
{  -124.3f,     67.1f}, // 21
{  -127.8f,     67.5f}, // 23
{  -131.4f,     68.5f}, // 25
{  -135.2f,     70.1f}, // 27
{  -139.1f,     72.3f}, // 29
{  -143.1f,     75.1f}, // 31
{  -147.2f,     78.7f}, // 33
{  -151.5f,     83.1f}, // 35
{  -155.9f,     88.7f}, // 37
{  -161.1f,     96.8f}, // 39
{  -166.0f,    107.9f}, // 41
{  -162.9f,    105.7f}, // 43
{  -159.8f,    103.8f}, // 45
{  -156.8f,    102.2f}, // 47
{  -153.8f,    100.8f}, // 49
{  -151.0f,     99.6f}, // 51
{  -148.2f,     98.6f}, // 53
{  -145.5f,     97.9f}, // 55
{  -142.9f,     97.4f}, // 57
{  -140.3f,     97.1f}, // 59
{  -137.9f,     97.1f}, // 61
{  -135.6f,     97.2f}, // 63
{  -133.5f,     97.6f}, // 65
{  -131.4f,     98.2f}, // 67
{  -129.5f,     99.1f}, // 69
{  -127.8f,    100.2f}, // 71
{  -126.2f,    101.5f}, // 73
{  -124.7f,    103.0f}, // 75
{  -123.4f,    104.8f}, // 77
{  -122.4f,    106.8f}, // 79
};
static constexpr int kSeqN = sizeof(kSeq) / sizeof(kSeq[0]); // = 40

// ===== AUTO RUN STATE =====
bool seqRunning = false;
unsigned long seqLastMs = 0;
static constexpr unsigned long SEQ_PERIOD_MS = 15; // tốc độ update
int seqDir = 1;  // chạy ngược (-1) hay xuôi (+1)
int seqIdx = 0;

bool segActive = false;

// nội suy mượt giữa 2 điểm {BAO,B}
static constexpr float STEP_BAO_DEG  = 20.0f;  // mượt hơn -> giảm giật
static constexpr float STEP_BETA_DEG = 20.0f;

float baoCmd=0, betaCmd=0;
float baoTarget=0, betaTarget=0;

// giữ HGO gần nhất để fallback khi solver bị "mất nghiệm"
float lastHgoGood = 90.0f;

// ================= Helpers PCA =================
static inline uint16_t usToTick(uint16_t us) {
  const float period_us = 1000000.0f / SERVO_FREQ; // 20000us
  uint16_t tick = (uint16_t)((4096.0f * us) / period_us + 0.5f);
  if (tick > 4095) tick = 4095;
  return tick;
}
static inline void setServoUs(uint8_t ch, uint16_t us) {
  pca.setPWM(ch, 0, usToTick(us));
}
static inline void setServoAngleCalib(uint8_t ch, float angle,
                                      uint16_t us_min, uint16_t us_max) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  uint16_t us = (uint16_t)lroundf(us_min + (us_max - us_min) * (angle / 180.0f));
  setServoUs(ch, us);
}

// ================= Math =================
struct Vec2 { float x, y; };

static inline Vec2 vadd(Vec2 a, Vec2 b){ return {a.x+b.x, a.y+b.y}; }
static inline Vec2 vsub(Vec2 a, Vec2 b){ return {a.x-b.x, a.y-b.y}; }
static inline Vec2 vmul(Vec2 a, float k){ return {a.x*k, a.y*k}; }
static inline float vdot(Vec2 a, Vec2 b){ return a.x*b.x + a.y*b.y; }
static inline float vnorm(Vec2 a){ return sqrtf(vdot(a,a)); }
static inline Vec2 vunit(Vec2 a){
  float n=vnorm(a);
  return (n<1e-9f)?Vec2{0,0}:Vec2{a.x/n,a.y/n};
}
static inline float cross2(Vec2 a, Vec2 b){ return a.x*b.y - a.y*b.x; }

static inline float wrapToPi(float a){
  a = fmodf(a + (float)M_PI, 2.0f*(float)M_PI);
  if (a < 0) a += 2.0f*(float)M_PI;
  return a - (float)M_PI;
}

static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
  if (fabsf(in_max - in_min) < 1e-9f) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t*(out_max - out_min);
}

static inline float stepToward(float cur, float target, float stepDeg){
  float d = target - cur;
  if (fabsf(d) <= stepDeg) return target;
  return cur + (d > 0 ? stepDeg : -stepDeg);
}

// circle-circle intersection
static bool circleCircle(Vec2 O1, float r1, Vec2 O2, float r2, Vec2 &P1, Vec2 &P2){
  Vec2 dV = vsub(O2,O1);
  float d = vnorm(dV);
  if (d < 1e-7f) return false;
  if (d > r1 + r2 + 1e-5f) return false;
  if (d < fabsf(r1 - r2) - 1e-5f) return false;

  float a = (r1*r1 - r2*r2 + d*d) / (2.0f*d);
  float h2 = r1*r1 - a*a;
  if (h2 < -1e-6f) return false;
  float h = sqrtf(fmaxf(0.0f, h2));

  Vec2 u = vmul(dV, 1.0f/d);
  Vec2 Pm = vadd(O1, vmul(u, a));
  Vec2 perp = {-u.y, u.x};

  P1 = vadd(Pm, vmul(perp, h));
  P2 = vsub(Pm, vmul(perp, h));
  return true;
}

// chọn nghiệm theo LOCK dấu cross (giống MATLAB)
static Vec2 pickByLockSign(Vec2 P1, Vec2 P2, Vec2 origin, Vec2 refVec, int lockSign){
  float c1 = cross2(refVec, vsub(P1, origin));
  float c2 = cross2(refVec, vsub(P2, origin));
  if (lockSign > 0) return (c1 >= c2) ? P1 : P2;  // ưu tiên dương
  else              return (c1 <= c2) ? P1 : P2;  // ưu tiên âm
}

// ================= Forward: Beta = f(BAO, HGO) =================
// (để solver dùng)
static bool forwardBeta(float baoUpDeg, float hgoDeg, float &betaOutDeg){
  Vec2 A{0,0};
  Vec2 G{dGx, dGy};

  float R = LAE;
  if (LEF > 2.0f*R) return false;
  float phi = 2.0f * asinf(LEF/(2.0f*R)); // rad

  // BAO: hệ +Y -> đổi sang chuẩn +X
  float theta = (baoUpDeg + 90.0f) * (float)M_PI/180.0f;

  // HGO: 0° Up(+Y) -> gammaStd đo từ +X
  float gammaStdDeg = hgoDeg + 90.0f;
  float gamma = gammaStdDeg * (float)M_PI/180.0f;

  Vec2 B = vadd(A, {LAB*cosf(theta), LAB*sinf(theta)});
  Vec2 H = vadd(G, {LGH*cosf(gamma), LGH*sinf(gamma)});

  Vec2 F1,F2;
  if(!circleCircle(A, LAF, H, LHF, F1, F2)) return false;
  Vec2 F = pickByLockSign(F1, F2, A, vsub(H,A), LOCK_F_SIGN);

  float angF = atan2f(F.y - A.y, F.x - A.x);
  Vec2 E1 = vadd(A, {LAE*cosf(angF + phi), LAE*sinf(angF + phi)});
  Vec2 E2 = vadd(A, {LAE*cosf(angF - phi), LAE*sinf(angF - phi)});
  Vec2 E  = pickByLockSign(E1, E2, A, vsub(F,A), LOCK_E_SIGN);

  Vec2 D1,D2;
  if(!circleCircle(E, LED, B, LBD, D1, D2)) return false;
  Vec2 D  = pickByLockSign(D1, D2, B, vsub(E,B), LOCK_D_SIGN);

  Vec2 uBC = vunit(vsub(B, D));
  if (vnorm(uBC) < 1e-6f) return false;

  float angBC = atan2f(uBC.y, uBC.x);
  float beta = wrapToPi(angBC - theta - (float)M_PI);
  betaOutDeg = beta * 180.0f/(float)M_PI;
  return true;
}

// ================= Inverse: nhập BAO + Beta(B) => tìm HGO =================
static bool solveHGO(float baoUpDeg, float betaTargetDeg, float &hgoSolvedDeg){
  const float hMin = 0.0f, hMax = 180.0f;
  const int   N = 80;
  const int   itMax = 28;

  auto normErr = [&](float e){
    while (e > 180) e -= 360;
    while (e < -180) e += 360;
    return e;
  };

  auto errAt = [&](float h, float &err)->bool{
    float b;
    if(!forwardBeta(baoUpDeg, h, b)) return false;
    err = normErr(b - betaTargetDeg);
    return true;
  };

  bool foundBracket = false;
  float bestH = 90.0f, bestAbs = 1e9f;

  float hPrev = hMin, ePrev = 0;
  bool okPrev = errAt(hPrev, ePrev);
  if(okPrev) { bestAbs = fabsf(ePrev); bestH = hPrev; }

  float a=0,b=0,ea=0,eb=0;

  for(int i=1;i<=N;i++){
    float h = hMin + (hMax - hMin) * (float)i / (float)N;
    float e;
    bool ok = errAt(h, e);

    if(ok){
      float ae = fabsf(e);
      if(ae < bestAbs){ bestAbs = ae; bestH = h; }

      if(okPrev){
        if((ePrev <= 0 && e >= 0) || (ePrev >= 0 && e <= 0)){
          a = hPrev; ea = ePrev;
          b = h;     eb = e;
          foundBracket = true;
          break;
        }
      }
      hPrev = h; ePrev = e; okPrev = true;
    }else{
      okPrev = false;
    }
  }

  if(!foundBracket){
    hgoSolvedDeg = bestH;
    return (bestAbs < 2.0f);
  }

  for(int it=0; it<itMax; it++){
    float m = 0.5f*(a+b);
    float em;
    if(!errAt(m, em)){
      m = 0.5f*(a+m);
      if(!errAt(m, em)) break;
    }
    if((ea <= 0 && em >= 0) || (ea >= 0 && em <= 0)){
      b = m; eb = em;
    }else{
      a = m; ea = em;
    }
  }

  hgoSolvedDeg = 0.5f*(a+b);
  return true;
}

// ================= Apply: input BAO & solved HGO =================
static void applyBaoHgo(float baoDeg, float hgoDeg){
  baoDeg = constrain(baoDeg, BAO_MIN, BAO_MAX);

  // Servo1 = HGO + offset
  float hgoOut = hgoDeg + SERVO1_OFFSET_DEG;
  hgoOut = constrain(hgoOut, 0.0f, 180.0f);
  setServoAngleCalib(SERVO1_CH, hgoOut, SERVO1_US_MIN, SERVO1_US_MAX);

  // Servo2 = BAO map [-200..-90] -> [90..180]
  float servo2 = mapFloat(baoDeg, BAO_MIN, BAO_MAX, 90.0f, 180.0f);
  servo2 = constrain(servo2, 0.0f, 180.0f);
  setServoAngleCalib(SERVO2_CH, servo2, SERVO2_US_MIN, SERVO2_US_MAX);
}

// ===== Manual/Solve helper =====
static void solveAndApply(float baoDeg, float betaDeg, const char* tag){
  baoDeg = constrain(baoDeg, BAO_MIN, BAO_MAX);

  float hgo;
  bool ok = solveHGO(baoDeg, betaDeg, hgo);
  hgo = constrain(hgo, 0.0f, 180.0f);

  // fallback: nếu solver “NEAR/mất nhịp” thì giữ HGO cũ cho ổn định
  if (!ok) {
    // nếu lệch quá nhiều thường do vùng mất nghiệm -> giữ last
    hgo = lastHgoGood;
  } else {
    lastHgoGood = hgo;
  }

  applyBaoHgo(baoDeg, hgo);

  Serial.printf("[%s] BAO=%.2f  B=%.2f  -> HGO=%.2f  (ok=%d)  (servo1 out=%.2f)\n",
                tag, baoDeg, betaDeg, hgo, ok ? 1 : 0,
                constrain(hgo + SERVO1_OFFSET_DEG, 0.0f, 180.0f));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);
  delay(30);

  // ===== an toàn phần cứng: boot lên đưa cả 2 servo về 90° =====
  setServoAngleCalib(SERVO1_CH, 90.0f, SERVO1_US_MIN, SERVO1_US_MAX);
  setServoAngleCalib(SERVO2_CH, 180.0f, SERVO2_US_MIN, SERVO2_US_MAX);
  delay(300);

  // nếu bạn dùng lastHgoGood:
  lastHgoGood = 90.0f;

  // auto start index
  seqIdx = kSeqN - 1;

  Serial.println("Nhap lenh:");
  Serial.println("  C            -> RUN/STOP auto (auto solve BAO,B -> HGO)");
  Serial.println("  <BAO> <B>     -> manual solve BAO,B -> HGO");
}


void loop() {
  // =========================
  // 1) UART: chỉ còn C và manual BAO,B
  // =========================
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      uartLine.trim();
      if (uartLine.length() == 0) return;

      // Toggle RUN/STOP
      if (uartLine.equalsIgnoreCase("C")) {
        seqRunning = !seqRunning;
        seqLastMs = millis();
        segActive = false;
        Serial.printf("[AUTO] %s\n", seqRunning ? "START" : "STOP");
        uartLine = "";
        return;
      }

      // Manual: <BAO> <B>
      float baoDeg, betaDeg;
      if (sscanf(uartLine.c_str(), "%f %f", &baoDeg, &betaDeg) == 2) {
        seqRunning = false; // manual thì tắt auto
        segActive  = false;
        solveAndApply(baoDeg, betaDeg, "MAN");
      } else {
        Serial.println("Sai dinh dang! Dung: C  hoac: <BAO> <B>  VD: -129.9 58.3");
      }

      uartLine = "";
      return;
    }

    uartLine += c;
  }

  // =========================
  // 2) AUTO: chạy bảng {BAO,B} và luôn solve ra HGO
  // =========================
  if (!seqRunning) return;

  unsigned long now = millis();
  if (now - seqLastMs < SEQ_PERIOD_MS) return;
  seqLastMs = now;

  // Setup đoạn mới
  if (!segActive) {
    const StepPair &st0 = kSeq[seqIdx];
    baoCmd  = st0.bao;
    betaCmd = st0.betaB;

    int nextIdx = seqIdx + seqDir;
    if (nextIdx < 0) nextIdx = kSeqN - 1;
    if (nextIdx >= kSeqN) nextIdx = 0;

    const StepPair &st1 = kSeq[nextIdx];
    baoTarget  = st1.bao;
    betaTarget = st1.betaB;

    segActive = true;

    solveAndApply(baoCmd, betaCmd, "AUTO");
    // Serial.printf("[AUTO] seg %d->%d | BAO %.1f->%.1f | B %.1f->%.1f\n",
    //               seqIdx, nextIdx, baoCmd, baoTarget, betaCmd, betaTarget);
    return;
  }

  // Nội suy BAO & B mượt
  baoCmd  = stepToward(baoCmd,  baoTarget,  STEP_BAO_DEG);
  betaCmd = stepToward(betaCmd, betaTarget, STEP_BETA_DEG);

  solveAndApply(baoCmd, betaCmd, "AUTO");

  // Tới target -> advance index
  if (baoCmd == baoTarget && betaCmd == betaTarget) {
    int nextIdx = seqIdx + seqDir;
    if (nextIdx < 0) nextIdx = kSeqN - 1;
    if (nextIdx >= kSeqN) nextIdx = 0;

    seqIdx = nextIdx;
    segActive = false;
  }
}
