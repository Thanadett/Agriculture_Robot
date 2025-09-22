#include "Logger.h"

void Logger::begin(bool enable_default){ enabled_ = enable_default; Serial.println("# t,v_sp,w_sp,vL,vR,pwmL,pwmR,yaw_deg"); }

void Logger::logCSV(float t, float vsp, float wsp, float vL, float vR, float pwmL, float pwmR, float yawDeg){
  if(!enabled_) return;
  Serial.printf("%.6f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f\n", t, vsp, wsp, vL, vR, pwmL, pwmR, yawDeg);
}
