#pragma once
#include <Arduino.h>
#include "Wheel.h"
#include "config.h"

class Encoder
{
public:
  void begin();
  int32_t readTicks(Wheel w) const;
  int32_t deltaTicks(Wheel w);

private:
  static void isrFL_A();
  static void isrFL_B();
  static void isrRL_A();
  static void isrRL_B();
  static void isrFR_A();
  static void isrFR_B();
  static void isrRR_A();
  static void isrRR_B();

  static volatile int32_t ticks_[4];
};
