#pragma once
class Servo { public: int v = -1; void attach(int, int, int) {} void write(int a) { v = a; } };
