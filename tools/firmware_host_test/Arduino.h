// Minimal host-side Arduino mock so the twin firmware can be compiled and run on a PC.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <deque>
typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define F(x) x
extern double g_time_us;
inline unsigned long millis() { return (unsigned long)(g_time_us / 1000.0); }
inline unsigned long micros() { return (unsigned long)g_time_us; }
void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
int analogRead(uint8_t);
inline long lround_(double v) { return std::lround(v); }
struct MockSerial {
  std::deque<char> in;
  void begin(long) {}
  int available() { return (int)in.size(); }
  int read() { char c = in.front(); in.pop_front(); return c; }
  void print(const char *s) { fputs(s, stdout); }
  void print(char c) { fputc(c, stdout); }
  void print(int v) { printf("%d", v); }
  void print(unsigned v) { printf("%u", v); }
  void print(long v) { printf("%ld", v); }
  void print(double v, int d = 2) { printf("%.*f", d, v); }
  void println(const char *s) { printf("%s\n", s); }
  void println(int v) { printf("%d\n", v); }
  void println() { printf("\n"); }
};
extern MockSerial Serial;
