#include "motorDrive.h"
#include <math.h>

// ========================= State (definitions) =========================
String rx_line;
bool estop = false;
uint32_t last_cmd_ms = 0;

float tgt_LF = 0.f, tgt_LR = 0.f, tgt_RF = 0.f, tgt_RR = 0.f;
float out_LF = 0.f, out_LR = 0.f, out_RF = 0.f, out_RR = 0.f;

// ========================= Utils =========================
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi ? hi : x);
}

static void setupPwmPin(int pin, int ch) {
  pinMode(pin, OUTPUT);
  ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, 0);
}

static void writeSignedPWM(int ch_fwd, int ch_rev, float pwm_norm, bool invert = false) {
  if (invert) pwm_norm = -pwm_norm;
  pwm_norm = clampf(pwm_norm, -1.0f, 1.0f);
  const int duty = (int)lroundf(fabsf(pwm_norm) * PWM_MAX_DUTY);

  if (pwm_norm >= 0.f) {
    ledcWrite(ch_fwd, duty);
    ledcWrite(ch_rev, 0);
  } else {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_rev, duty);
  }
}

static void applyMotors(float LF, float LR, float RF, float RR) {
  // E-Stop บังคับเป็นศูนย์
  if (estop) { LF = LR = RF = RR = 0.f; }

  // Slew-rate limit
  float dLF = clampf(LF - out_LF, -RAMP_STEP, RAMP_STEP);
  float dLR = clampf(LR - out_LR, -RAMP_STEP, RAMP_STEP);
  float dRF = clampf(RF - out_RF, -RAMP_STEP, RAMP_STEP);
  float dRR = clampf(RR - out_RR, -RAMP_STEP, RAMP_STEP);

  out_LF = clampf(out_LF + dLF, -1.f, 1.f);
  out_LR = clampf(out_LR + dLR, -1.f, 1.f);
  out_RF = clampf(out_RF + dRF, -1.f, 1.f);
  out_RR = clampf(out_RR + dRR, -1.f, 1.f);

  // เขียนไปที่มอเตอร์
  writeSignedPWM(CH_LF_IN1, CH_LF_IN2, out_LF, INVERT_LF);
  writeSignedPWM(CH_LR_IN1, CH_LR_IN2, out_LR, INVERT_LR);
  writeSignedPWM(CH_RF_IN1, CH_RF_IN2, out_RF, INVERT_RF);
  writeSignedPWM(CH_RR_IN1, CH_RR_IN2, out_RR, INVERT_RR);
}

// =============== Command Parsing (Serial) =================
static bool startsWith(const String &s, const char *p) {
  size_t n = strlen(p);
  return s.length() >= (int)n && s.substring(0, n).equalsIgnoreCase(p);
}

static bool parseFloatAfterEquals(const String &s, const char *key, float &out) {
  int idx = s.indexOf(key);
  if (idx < 0) return false;
  idx += strlen(key);
  int end = idx;
  while (end < (int)s.length() && !isWhitespace(s[end])) end++;
  String num = s.substring(idx, end);
  out = num.toFloat();
  return true;
}

// แปลง V,W เป็นความเร็วเชิงเส้นล้อซ้าย/ขวา แล้วแมปเป็น PWM [-1..1]
void cmdVW_to_targets(float V_mps, float W_radps) {
  // ความเร็วเชิงเส้นล้อซ้าย/ขวา (m/s)
  float vL = V_mps - (W_radps * (WHEEL_SEP * 0.5f));
  float vR = V_mps + (W_radps * (WHEEL_SEP * 0.5f));

  // แปลงเป็นความเร็วเชิงมุมล้อ (rad/s)
  float omegaL = vL / WHEEL_RADIUS;
  float omegaR = vR / WHEEL_RADIUS;

  // แมปเป็น PWM normalized โดยใช้ OMEGA_WHEEL_FULL (คำนวณจาก MAX_LINEAR_UNITS/WHEEL_RADIUS)
  float pwmL = clampf(omegaL / OMEGA_WHEEL_FULL, -1.f, 1.f);
  float pwmR = clampf(omegaR / OMEGA_WHEEL_FULL, -1.f, 1.f);

  // กระจายค่าเดียวกันไปล้อหน้า/หลังของแต่ละฝั่ง
  tgt_LF = pwmL;
  tgt_LR = pwmL;
  tgt_RF = pwmR;
  tgt_RR = pwmR;
}

static void handleLine(String line) {
  line.trim();
  if (line.isEmpty()) return;

  if (startsWith(line, "ESTOP")) {
    int sp = line.indexOf(' ');
    int val = 1;
    if (sp >= 0) {
      String tail = line.substring(sp + 1);
      tail.trim();
      val = tail.toInt();
    }
    estop = (val != 0);
    if (estop) tgt_LF = tgt_LR = tgt_RF = tgt_RR = 0.f;
    Serial.printf("ACK ESTOP=%d\n", estop ? 1 : 0);
    return;
  }

  if (startsWith(line, "VW")) {
    float V = 0.f, W = 0.f;
    bool okV = parseFloatAfterEquals(line, "V=", V);
    bool okW = parseFloatAfterEquals(line, "W=", W);
    if (okV && okW) {
      last_cmd_ms = millis();
      cmdVW_to_targets(V, W);
      Serial.printf("ACK VW V=%.3f W=%.3f -> L=%.2f R=%.2f\n", V, W, tgt_LF, tgt_RF);
    } else {
      Serial.println("ERR VW FORMAT (need: VW V=<m/s> W=<rad/s>)");
    }
    return;
  }

  if (startsWith(line, "P")) {
    // หมายเหตุ: ต้องเช็ค "PW4" ก่อน "P" เสมอ ไม่งั้นชนกัน
    if (startsWith(line, "PW4")) {
      float lf=0.f, lr=0.f, rf=0.f, rr=0.f;
      bool ok1 = parseFloatAfterEquals(line, "LF=", lf);
      bool ok2 = parseFloatAfterEquals(line, "LR=", lr);
      bool ok3 = parseFloatAfterEquals(line, "RF=", rf);
      bool ok4 = parseFloatAfterEquals(line, "RR=", rr);
      if (ok1 && ok2 && ok3 && ok4) {
        last_cmd_ms = millis();
        tgt_LF = clampf(lf, -1.f, 1.f);
        tgt_LR = clampf(lr, -1.f, 1.f);
        tgt_RF = clampf(rf, -1.f, 1.f);
        tgt_RR = clampf(rr, -1.f, 1.f);
        Serial.printf("ACK PW4 LF=%.2f LR=%.2f RF=%.2f RR=%.2f\n", tgt_LF, tgt_LR, tgt_RF, tgt_RR);
      } else {
        Serial.println("ERR PW4 FORMAT (need: PW4 LF= LR= RF= RR=)");
      }
    } else {
      float L=0.f, R=0.f;
      bool okL = parseFloatAfterEquals(line, "L=", L);
      bool okR = parseFloatAfterEquals(line, "R=", R);
      if (okL && okR) {
        last_cmd_ms = millis();
        L = clampf(L, -1.f, 1.f);
        R = clampf(R, -1.f, 1.f);
        tgt_LF = tgt_LR = L;
        tgt_RF = tgt_RR = R;
        Serial.printf("ACK P L=%.2f R=%.2f\n", L, R);
      } else {
        Serial.println("ERR P FORMAT (need: P L=<..> R=<..> or PW4 ...)");
      }
    }
    return;
  }

  Serial.println("ERR UNKNOWN CMD");
}

// ========================= Public API =========================
void motorDrive_begin() {
  // ตั้งค่า PWM สำหรับทุกขา
  setupPwmPin(LF_IN1, CH_LF_IN1);
  setupPwmPin(LF_IN2, CH_LF_IN2);
  setupPwmPin(LR_IN1, CH_LR_IN1);
  setupPwmPin(LR_IN2, CH_LR_IN2);
  setupPwmPin(RF_IN1, CH_RF_IN1);
  setupPwmPin(RF_IN2, CH_RF_IN2);
  setupPwmPin(RR_IN1, CH_RR_IN1);
  setupPwmPin(RR_IN2, CH_RR_IN2);

  // เริ่มจากหยุดนิ่ง
  applyMotors(0, 0, 0, 0);
  last_cmd_ms = millis();

  Serial.println("READY 4WD MOTOR-ONLY @115200");
  Serial.println("Commands: ");
  Serial.println("  VW V=<m/s> W=<rad/s>");
  Serial.println("  P L=<-1..1> R=<-1..1>");
  Serial.println("  PW4 LF=<..> LR=<..> RF=<..> RR=<..>");
  Serial.println("  ESTOP 0|1");
}

void motorDrive_handleSerialOnce() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (rx_line.length() > 0) {
        handleLine(rx_line);
        rx_line = "";
      }
    } else {
      if (rx_line.length() < 240) rx_line += c;
    }
  }
}

void motorDrive_update() {
  // Watchdog: ค่อยๆ ลดถ้าไม่มีคำสั่งใหม่
  uint32_t now = millis();
  if (!estop && (now - last_cmd_ms > CMD_TIMEOUT_MS)) {
    tgt_LF *= IDLE_DECAY;
    tgt_LR *= IDLE_DECAY;
    tgt_RF *= IDLE_DECAY;
    tgt_RR *= IDLE_DECAY;
    // ใกล้ศูนย์แล้วปิดเลยเพื่อไม่ให้คราง
    if (fabsf(tgt_LF) < 0.02f) tgt_LF = 0.f;
    if (fabsf(tgt_LR) < 0.02f) tgt_LR = 0.f;
    if (fabsf(tgt_RF) < 0.02f) tgt_RF = 0.f;
    if (fabsf(tgt_RR) < 0.02f) tgt_RR = 0.f;
  }

  // ส่งค่าไปมอเตอร์
  applyMotors(tgt_LF, tgt_LR, tgt_RF, tgt_RR);
}
