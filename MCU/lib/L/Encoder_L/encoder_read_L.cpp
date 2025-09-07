#include "encoder_read_L.h"
#include <math.h>

// ถ้า M_PI ไม่ถูก define (บาง compiler) → กำหนดเอง
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

String cutline = "===============================================================================";
String underline = "_______________________________________________________________________________";

//กดปุ่มเพื่อเริ่มกำหนด จุดเริ่มต้น เพื่อหาว่าครบ 0.15 m นับจากจุดเริ่มต้นตอนไหน
//หมุนไปข้างหน้าระยะ 15 cm(0.15 m) ต้องหมุนทั้งหมด 0.15/(2*Pi*(0.127/2)) = 0.375967 รอบ

// ======================= Constructor =======================
// กำหนดขา A/B ของล้อหลัง (rear) และล้อหน้า (front)
// พร้อมกำหนดค่า PPR (pulses per revolution) ของ encoder (output shaft)
DualEncoderReader::DualEncoderReader(int rearA, int rearB, int frontA, int frontB,
                                     float pulses_per_rev_output)
: rA_(rearA), rB_(rearB), fA_(frontA), fB_(frontB),
  ppr_out_(pulses_per_rev_output) {}


// ======================= begin() =======================
// เรียกครั้งเดียวใน setup() เพื่อเตรียม encoder
void DualEncoderReader::begin(bool enable_internal_pullups) {
  // ใช้ internal pull-up resistor ถ้าเลือก true
  ESP32Encoder::useInternalWeakPullResistors = enable_internal_pullups ? UP : DOWN;

  // attachHalfQuad → ใช้โหมด half-quadrature (นับขาขึ้น+ลงของช่อง A เทียบกับ B)
  encR_.attachHalfQuad(rA_, rB_);
  encF_.attachHalfQuad(fA_, fB_);
  encR_.clearCount();
  encF_.clearCount();

  // เก็บค่า count เริ่มต้น
  lastR_ = encR_.getCount();
  lastF_ = encF_.getCount();

  // ค่าเริ่มต้นของตำแหน่งและความเร็ว
  posR_rad_ = posF_rad_ = 0.f;
  velR_rad_s_ = velF_rad_s_ = 0.f;

  // reset ระยะรวมสะสม
  total_dist_R_m_ = 0.f;   
  total_dist_F_m_ = 0.f;

  // เก็บ timestamp ไว้ใช้คำนวณ dt
  last_ts_ms_ = millis();
}


// ======================= reset() =======================
// รีเซต encoder + ค่าต่าง ๆ กลับเป็น 0
void DualEncoderReader::reset() {
  encR_.clearCount();
  encF_.clearCount();
  lastR_ = 0;
  lastF_ = 0;
  posR_rad_ = posF_rad_ = 0.f;
  velR_rad_s_ = velF_rad_s_ = 0.f;
  total_dist_R_m_ = 0.f;
  total_dist_F_m_ = 0.f;
  last_ts_ms_ = millis();
}


// ======================= update() =======================
// เรียกบ่อย ๆ ใน loop() → อัปเดตตำแหน่ง, ความเร็ว, ระยะสะสม
void DualEncoderReader::update() {
  // อ่านค่าพัลส์ปัจจุบันจาก encoder
  long curR = encR_.getCount() * invR_;
  long curF = encF_.getCount() * invF_;

  // ความเปลี่ยนแปลงจากรอบก่อน (Δcount)
  long dR = curR - lastR_;
  long dF = curF - lastF_;
  lastR_  = curR;
  lastF_  = curF;

  // เวลาที่ผ่านไปตั้งแต่รอบก่อน (s)
  uint32_t now = millis();
  float dt = (now - last_ts_ms_) / 1000.0f;
  if (dt <= 0.f) dt = 1e-3f; // กันหารศูนย์
  last_ts_ms_ = now;

  const float ppr = (ppr_out_ <= 0.f ? 1.f : ppr_out_); // ป้องกันหารศูนย์
  const float two_pi = 2.0f * (float)M_PI;
  const float C = circumferenceM(); // เส้นรอบวงล้อ (m)

  // ============ ระยะทางรวม (สะสมตลอด) ============
  // (Δcount / PPR) * C → ระยะทางที่เพิ่มขึ้นในรอบนี้
  // fabsf() → นับเป็นระยะบวกเสมอ (absolute distance)
  total_dist_R_m_ += fabsf(((float)dR / ppr) * C);
  total_dist_F_m_ += fabsf(((float)dF / ppr) * C);

  // ============ ตำแหน่งเชิงมุม [rad] ============
  // (count / PPR) * 2π
  posR_rad_ = ((float)curR / ppr) * two_pi;
  posF_rad_ = ((float)curF / ppr) * two_pi;

  // ============ ความเร็วเชิงมุม [rad/s] ============
  // ((Δcount / PPR) / dt) * 2π
  float revR_dt = ((float)dR / ppr) / dt;
  float revF_dt = ((float)dF / ppr) / dt;
  velR_rad_s_ = revR_dt * two_pi;
  velF_rad_s_ = revF_dt * two_pi;
}


// ======================= printFB() =======================
// แสดงค่าความเร็ว + ระยะในรอบปัจจุบัน (รีเซ็ตทุกครั้งที่ครบ 1 รอบ)
// และพิมพ์ “TOTAL” ไว้ด้านบน
void DualEncoderReader::printFB(Stream& s) const {
  // --- พิมพ์ระยะรวมสะสม (ไม่รีเซ็ต) ---
  s.println(cutline);
  s.printf("TOTAL   | Total_R=%.6f Total_F=%.6f\n",
           total_dist_R_m_, total_dist_F_m_);

  // --- Dist_R/Dist_F = ระยะในรอบปัจจุบัน ---
  // ไม่อ่าน getCount() ที่นี่ (เมธอดนี้เป็น const) ใช้มุมที่คำนวณจาก update() แทน
  const float two_pi = 2.0f * (float)M_PI;
  const float C      = circumferenceM();

  // จำนวนรอบตั้งแต่เริ่ม (อาจเป็นลบได้หากหมุนย้อน)
  float revR = posR_rad_ / two_pi;
  float revF = posF_rad_ / two_pi;

  // ส่วนเศษของจำนวนรอบในช่วง [0,1)
  float fracR = revR - floorf(revR);
  float fracF = revF - floorf(revF);
  // (หมายเหตุ: ถ้า rev เป็นลบ floorf จะพา frac ไปอยู่ [0,1) ให้อยู่แล้ว)

  // ระยะในรอบปัจจุบัน = ส่วนเศษของรอบ * เส้นรอบวง
  float distR_cycle = fracR * C;
  float distF_cycle = fracF * C;
  s.println(underline);
  // แสดง FB_ENC
  s.printf("FB_ENC  | Vel_R=%.6f Vel_F=%.6f Dist_R=%.6f     Dist_F=%.6f\n",
           velR_rad_s_, velF_rad_s_, distR_cycle, distF_cycle);
}


// ======================= circumferenceM() =======================
// เส้นรอบวงล้อ (เมตร) = 2πR
float DualEncoderReader::circumferenceM() const {
  return 2.0f * (float)M_PI * wheel_radius_m_;
}


// ======================= remainingToNextRevRearM() =======================
// คำนวณระยะที่เหลือจนครบรอบถัดไป (ล้อหลัง)
float DualEncoderReader::remainingToNextRevRearM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posR_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f; // กันเศษทศนิยมเล็ก ๆ
  return remain_rev * C;
}


// ======================= remainingToNextRevFrontM() =======================
// คำนวณระยะที่เหลือจนครบรอบถัดไป (ล้อหน้า)
float DualEncoderReader::remainingToNextRevFrontM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posF_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f;
  return remain_rev * C;
}


// ======================= printFB2() =======================
// แสดง C = เส้นรอบวงล้อ และระยะที่เหลือ RL/RR
void DualEncoderReader::printFB2(Stream& s) const {
  s.printf("FB_ENC2 | C=%.6f                    DistLeft_R=%.6f DistLeft_F=%.6f\n",
           circumferenceM(),
           remainingToNextRevRearM(),
           remainingToNextRevFrontM());
  s.println(cutline);
}
