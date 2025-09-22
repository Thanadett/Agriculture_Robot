#include "Encoder.h"
#include "config.h"
#include "Wheel.h"

static constexpr int FL_A = ENC_FL_A, FL_B = ENC_FL_B;
static constexpr int RL_A = ENC_RL_A, RL_B = ENC_RL_B;
static constexpr int FR_A = ENC_FR_A, FR_B = ENC_FR_B;
static constexpr int RR_A = ENC_RR_A, RR_B = ENC_RR_B;

volatile int32_t Encoder::ticks_[4] = {0, 0, 0, 0};

static inline void handleQuad(volatile int32_t &t, int pinA, int pinB)
{
  // Fast 2-bit state method (minimal): read B to determine direction on A edge
  if (digitalRead(pinA))
    t += (digitalRead(pinB) ? -1 : +1);
  else
    t += (digitalRead(pinB) ? +1 : -1);
}

void Encoder::isrFL_A() { handleQuad(ticks_[0], FL_A, FL_B); }
void Encoder::isrFL_B() { handleQuad(ticks_[0], FL_A, FL_B); }
void Encoder::isrRL_A() { handleQuad(ticks_[1], RL_A, RL_B); }
void Encoder::isrRL_B() { handleQuad(ticks_[1], RL_A, RL_B); }
void Encoder::isrFR_A() { handleQuad(ticks_[2], FR_A, FR_B); }
void Encoder::isrFR_B() { handleQuad(ticks_[2], FR_A, FR_B); }
void Encoder::isrRR_A() { handleQuad(ticks_[3], RR_A, RR_B); }
void Encoder::isrRR_B() { handleQuad(ticks_[3], RR_A, RR_B); }

void Encoder::begin()
{
  pinMode(FL_A, INPUT_PULLUP);
  pinMode(FL_B, INPUT_PULLUP);
  pinMode(RL_A, INPUT_PULLUP);
  pinMode(RL_B, INPUT_PULLUP);
  pinMode(FR_A, INPUT_PULLUP);
  pinMode(FR_B, INPUT_PULLUP);
  pinMode(RR_A, INPUT_PULLUP);
  pinMode(RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_A), isrFL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_B), isrFL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_A), isrRL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_B), isrRL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_A), isrFR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_B), isrFR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_A), isrRR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_B), isrRR_B, CHANGE);
}

int32_t Encoder::readTicks(Wheel w) const { return ticks_[(int)w]; }
int32_t Encoder::deltaTicks(Wheel w)
{
  noInterrupts();
  int i = (int)w;
  int32_t cur = ticks_[i];
  static int32_t last[4] = {0, 0, 0, 0};
  int32_t d = cur - last[i];
  last[i] = cur;
  interrupts();
  return d;
}
