// Runs the firmware on the PC: commands are read from stdin as "<time_ms> <line>".
#include "Arduino.h"
#include "AccelStepper.h"
#include "Servo.h"
double g_time_us = 0;
MockSerial Serial;
AccelStepper *AccelStepper::all[8]; int AccelStepper::n = 0;
// physical limit switches at these true positions (steps), in the homing direction
static const int SWITCH_PIN[4] = {11, 10, 9, A3};
static const double SWITCH_AT[4] = {-1500, -2000, -500, 20000};
static const int DIR[4] = {-1, -1, -1, +1};
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int analogRead(uint8_t) { return 512; }
int digitalRead(uint8_t pin) {
  for (int i = 0; i < 4; i++)
    if (pin == SWITCH_PIN[i]) {
      double p = AccelStepper::all[i]->truePos;
      return (DIR[i] < 0 ? p <= SWITCH_AT[i] : p >= SWITCH_AT[i]) ? HIGH : LOW;
    }
  return LOW;
}
#include "../../firmware/scara_twin/scara_twin.ino"
int main() {
  char buf[256];
  std::deque<std::pair<double, std::string>> script;
  while (fgets(buf, sizeof buf, stdin)) {
    double t; int off;
    if (sscanf(buf, "%lf %n", &t, &off) == 1) script.push_back({t, std::string(buf + off)});
  }
  setup();
  // script times are delays (ms) after the previous command was delivered
  double next = script.empty() ? 0 : script.front().first, last = 0;
  while (!script.empty() || g_time_us / 1000.0 < last + 3000) {
    if (!script.empty() && g_time_us / 1000.0 >= next) {
      for (char c : script.front().second) Serial.in.push_back(c);
      script.pop_front();
      last = g_time_us / 1000.0;
      if (!script.empty()) next = last + script.front().first;
    }
    loop();
    g_time_us += 50;   // 20 kHz loop
  }
}
