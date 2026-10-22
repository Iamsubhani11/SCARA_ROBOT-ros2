#pragma once
#include "Arduino.h"
// Kinematic mock: constant-acceleration-free motion at max speed (good enough for protocol tests).
class AccelStepper {
 public:
  enum { DRIVER = 1 };
  static AccelStepper *all[8]; static int n;
  double truePos;      // physical position in steps (unknown to the firmware before homing)
  long offset = 0;     // currentPosition = truePos + offset
  long target = 0; double maxSpeed = 1000, speed = 0, lastT = 0;
  AccelStepper(int, int, int) { truePos = 300 + 200 * n; all[n++] = this; }
  long currentPosition() { return std::lround(truePos) + offset; }
  void setCurrentPosition(long p) { offset = p - std::lround(truePos); target = p; }
  void moveTo(long p) { target = p; }
  void setMaxSpeed(double v) { maxSpeed = v; }
  void setAcceleration(double) {}
  void setSpeed(double v) { speed = v; }
  long distanceToGo() { return target - currentPosition(); }
  void stop() { target = currentPosition(); }
  void step(double dt, double v) { truePos += v * dt; }
  bool run() {
    g_time_us += 12.5;   // virtual time passes while the firmware spins
    double dt = (g_time_us - lastT) * 1e-6; lastT = g_time_us;
    long d = distanceToGo(); if (!d) return false;
    double mv = maxSpeed * dt; if (mv > std::fabs((double)d)) mv = std::fabs((double)d);
    truePos += d > 0 ? mv : -mv; return true;
  }
  bool runSpeed() { g_time_us += 12.5; double dt = (g_time_us - lastT) * 1e-6; lastT = g_time_us; truePos += speed * dt; return true; }
};
