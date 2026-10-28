#include <Arduino.h>
#include "IR Sensor.h"

// ---------- Pin assignments (change to match your wiring) ----------
const int SEL_A = A2; // decoder select A (port 3, S1)
const int SEL_B = A3; // decoder select B (port 3, S2)
const int IR_IN = A1; // IR detector voltage (port 4, S1)

// ---------- Timing / averaging ----------
const int SETTLE_US = 1500; // detector needs ~1 ms to settle; add margin
const int N_SAMPLES = 5;    // average to reduce noise

// ---------- Decoder control ----------
// Y0 (B=0, A=0) -> LOW -> L293D -> emitter ON
// Any other output selected -> Y0 HIGH -> emitter OFF
void emitterOn()
{
  digitalWrite(SEL_A, LOW);
  digitalWrite(SEL_B, LOW);
}
void emitterOff()
{
  digitalWrite(SEL_A, HIGH);
  digitalWrite(SEL_B, HIGH);
} // selects Y3

void setupIR()
{
  pinMode(SEL_A, OUTPUT);
  pinMode(SEL_B, OUTPUT);
  emitterOff();
}

int readAveraged()
{
  long sum = 0;
  for (int i = 0; i < N_SAMPLES; i++)
    sum += analogRead(IR_IN);
  return sum / N_SAMPLES;
}

// Return the averaged raw detector reading, with the emitter on.
int readIRRaw()
{
  emitterOn();
  delayMicroseconds(SETTLE_US);
  int reading = readAveraged();
  emitterOff();
  return reading;
}
