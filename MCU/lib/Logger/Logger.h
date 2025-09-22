#pragma once
#include <Arduino.h>
#include "config.h"

class Logger {
public:
  void begin(bool enable_default);
  void toggle(){ enabled_ = !enabled_; }
  bool enabled() const { return enabled_; }
  void logCSV(float t, float vsp, float wsp, float vL, float vR, float pwmL, float pwmR, float yawDeg);

private:
  bool enabled_ = false;
};
