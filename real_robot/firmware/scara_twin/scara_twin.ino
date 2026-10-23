/*
 * SCARA digital-twin firmware
 * ---------------------------------------------------------------------------
 * For the HowToMechatronics "SCARA robot" build:
 *   Arduino UNO + CNC Shield V3 + 4x A4988 stepper drivers (1/4 step) + 4x NEMA 17,
 *   4 normally-closed limit switches, MG996R servo gripper.
 * Pins, conversion constants, speeds and the homing numbers are the ones of the original
 * HowToMechatronics sketch (wiring: docs/wiring.svg); only the serial protocol is new.
 * Power: 12 V >= 4 A (6 A recommended) into the CNC shield for the steppers; the MG996R
 * needs its own 5-6 V >= 2.5 A supply (e.g. a buck converter from the 12 V), NOT the
 * Arduino 5 V pin - connect all grounds together.
 *
 * It speaks a small text protocol over USB (115200 baud, one command per
 * line) with the ROS 2 / MuJoCo digital twin (real_robot/scara_twin_bridge.py,
 * mujoco/mujoco_twin.py):
 *
 *   PC -> Arduino  (same buttons as the Mechamind "SCARA control panel")
 *     M <j1_deg> <j2_deg> <j3_deg> <z_mm> <grip_pct>   move to a joint target
 *     H                                                 zero here: "the arm is at the ready pose now"
 *     F                                                 free motors (drivers off - move the arm by hand)
 *     L                                                 lock motors (drivers on)
 *     C <0|1>                                           conveyor relay off / on (A1, or D10 when built without switches)
 *     K                                                 home on the 4 limit switches (the original homing() routine)
 *     G <deg>                                           gripper servo straight to an angle (find YOUR closed angle)
 *     V <steps_per_s> <steps_per_s2>                    max speed / acceleration
 *     S                                                 stop where you are
 *     Z, E <0|1>                                        old names of H and F / L
 *     ?                                                 print one status line now
 *
 *   Arduino -> PC (every 50 ms)
 *     P <j1_deg> <j2_deg> <j3_deg> <z_mm> <grip_pct> <moving> <homed> <motors_on> <conveyor_on>
 *   and messages:   I <text>  (info)    W <text>  (warning)    E <text> (error)
 *
 * Joint conventions (same numbers as the HowToMechatronics GUI):
 *   j1 = theta1, j2 = theta2, j3 = phi (gripper rotation), z = height in mm
 *   (0 = bottom, 170 = top limit switch), grip_pct 0 = open .. 100 = closed.
 * The bridge converts these to the URDF joints (column, forearm, wrist,
 * shoulder, fingers) with the offsets / signs in config/twin_hardware.yaml.
 *
 * Start-up: send K - the robot homes on its 4 limit switches exactly like the original
 * sketch (Z up, then J3, J2, J1) and parks straight out with Z at 100 mm.  The original
 * homes by itself in setup(); here the PC sends K, so nothing moves before you ask.
 * Built without switches?  Set USE_LIMIT_SWITCHES 0, put the arm straight out with Z at
 * 150 mm by hand and send H ("zero here") instead.
 *
 * TEACH mode (like the MechaMind potentiometer twin): set TEACH_WITH_POTS to 1,
 * wire 4 potentiometers (5V / GND / wiper) to A1, A2, A4, A5 and the robot
 * follows the knobs while the twin in Gazebo / RViz / MuJoCo follows the robot.
 *
 * Libraries: AccelStepper (Mike McCauley) and Servo - both from the Library
 * Manager.  Board: Arduino UNO.
 *
 * The motion part is written for this project; every pin, conversion constant,
 * speed and homing number below is the one of the HowToMechatronics sketch.  Two
 * things are NOT in the original and you set them yourself: the conveyor relay
 * (new, on a free pin) and the servo's closed angle (depends on your gripper / cube).
 */
#include <AccelStepper.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

// ----------------------------------------------------------------- options
#ifndef TEACH_WITH_POTS
#define TEACH_WITH_POTS 0          // 1 = robot follows potentiometers on A1 A2 A4 A5
#endif
#ifndef USE_LIMIT_SWITCHES
#define USE_LIMIT_SWITCHES 1       // the HowToMechatronics build has 4 switches (0 = none: relay on D10, zero with H)
#endif
#if TEACH_WITH_POTS && USE_LIMIT_SWITCHES
#error "TEACH_WITH_POTS uses A1 for a knob and USE_LIMIT_SWITCHES moves the conveyor relay to A1 - pick one"
#endif
#define BAUD 115200
#define STATUS_PERIOD_MS 50        // 20 Hz position feedback

// ----------------------------------------------------------------- pins (CNC shield V3)
// Same pins as the HowToMechatronics SCARA sketch ("SCARA Robot - Arduino and Processing Codes"):
//   stepper1 (J1, theta1)  AccelStepper(1, 2, 5)    X slot
//   stepper2 (J2, theta2)  AccelStepper(1, 3, 6)    Y slot
//   stepper3 (J3, phi)     AccelStepper(1, 4, 7)    Z slot
//   stepper4 (Z axis)      AccelStepper(1, 12, 13)  A slot (A-axis jumpers set to D12 / D13)
//   limitSwitch1 = 11 (Z+/Z- header), limitSwitch2 = 10 (Y), limitSwitch3 = 9 (X), limitSwitch4 = A3 (CoolEn)
//   gripperServo.attach(A0, 600, 2500)  (A0 = "Abort" pin)
#if USE_LIMIT_SWITCHES
const uint8_t CONVEYOR_RELAY_PIN = A1;   // pin 10 belongs to limitSwitch2 then -> relay on "Hold" (A1)
#else
const uint8_t CONVEYOR_RELAY_PIN = 10;   // Y end-stop header, like the Mechamind build (relay module IN)
#endif
const bool RELAY_ACTIVE_LOW = false;      // most blue relay boards switch on with LOW: set true for those
const uint8_t LIMIT_J1 = 11;       // limitSwitch1
const uint8_t LIMIT_J2 = 10;       // limitSwitch2
const uint8_t LIMIT_J3 = 9;        // limitSwitch3
const uint8_t LIMIT_Z = A3;        // limitSwitch4
const uint8_t ENABLE_PIN = 8;      // CNC shield EN, LOW = drivers enabled
const uint8_t SERVO_PIN = A0;
const uint8_t POT_PINS[4] = {A1, A2, A4, A5};   // teach mode: j1, j2, j3, z

AccelStepper stJ1(AccelStepper::DRIVER, 2, 5);
AccelStepper stJ2(AccelStepper::DRIVER, 3, 6);
AccelStepper stJ3(AccelStepper::DRIVER, 4, 7);
AccelStepper stZ(AccelStepper::DRIVER, 12, 13);
AccelStepper *const AX[4] = {&stJ1, &stJ2, &stJ3, &stZ};
Servo gripper;

// ----------------------------------------------------------------- CALIBRATION
// Conversion constants of the HowToMechatronics sketch (A4988 at quarter step):
//   theta1AngleToSteps = 44.444444   (J1 20:1 belt reduction)
//   theta2AngleToSteps = 35.555555   (J2 16:1)
//   phiAngleToSteps    = 10          (J3 4:1 - the original sketch uses 10, keep it)
//   zDistanceToSteps   = 100         (8 mm lead screw)
// There are NO encoders: a joint's position is only the count of steps sent, so these must
// match the mechanics (change them only if you changed pulleys or micro-stepping).
const float STEPS_PER_UNIT[4] = {44.444444, 35.555555, 10.0, 100.0};
// homing() of the original sketch, in steps: speed while seeking the switch, the position
// set when it closes, and where the axis goes afterwards.  Order: Z, J3, J2, J1.
const float HOME_SEEK_SPEED[4] = {-1200, -1300, -1100, 1500};
const long HOME_SWITCH_STEPS[4] = {-3955, -5420, -1662, 17000};   // = -89.0 deg, -152.4 deg, -166.2 deg, 170 mm
const long HOME_PARK_STEPS[4] = {0, 0, 0, 10000};                 // arm straight, Z at 100 mm
// soft limits (deg, deg, deg, mm) - never commanded outside these
const float LIM_LO[4] = {-89.0, -150.0, -162.0, 0.0};
const float LIM_HI[4] = {266.0, 150.0, 162.0, 170.0};
// "Zero here" pose (Mechamind-style start without switches): arm straight, Z = 150 mm
const float READY[4] = {0.0, 0.0, 0.0, 150.0};
// gripper servo: 180 = open (the original sketch starts with data[6] = 180).  The original
// GUI sends "slider + 50" for the gripper, so the closed angle is whatever squeezes YOUR cube:
// close the gripper on a cube from the GUI and put the angle where it just holds it here.
const int SERVO_OPEN_DEG = 180;
const int SERVO_CLOSED_DEG = 80;
const float MAX_SPEED = 4000, ACCEL = 2000;      // setMaxSpeed(4000), setAcceleration(2000) as in the original
const unsigned long HOMING_TIMEOUT_MS = 30000;

// ----------------------------------------------------------------- state
bool homed = false;
bool enabled = true;
bool conveyorOn = false;
float gripPct = 0.0;
char line[72];
uint8_t lineLen = 0;
unsigned long lastStatus = 0;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static long toSteps(uint8_t i, float v) { return lround(v * STEPS_PER_UNIT[i]); }

static float toUnits(uint8_t i) { return AX[i]->currentPosition() / STEPS_PER_UNIT[i]; }

static void setGripper(float pct) {
  gripPct = clampf(pct, 0, 100);
  gripper.write(SERVO_OPEN_DEG + (int)((SERVO_CLOSED_DEG - SERVO_OPEN_DEG) * gripPct / 100.0));
}

static void moveTo(const float target[4]) {
  for (uint8_t i = 0; i < 4; i++) AX[i]->moveTo(toSteps(i, clampf(target[i], LIM_LO[i], LIM_HI[i])));
}

static bool anyMoving() {
  for (uint8_t i = 0; i < 4; i++)
    if (AX[i]->distanceToGo() != 0) return true;
  return false;
}

static void runAll() {
  for (uint8_t i = 0; i < 4; i++) AX[i]->run();
}

static void sendStatus() {
  Serial.print('P');
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(' ');
    Serial.print(toUnits(i), 2);
  }
  Serial.print(' ');
  Serial.print(gripPct, 1);
  Serial.print(' ');
  Serial.print(anyMoving() ? 1 : 0);
  Serial.print(' ');
  Serial.print(homed ? 1 : 0);
  Serial.print(' ');
  Serial.print(enabled ? 1 : 0);
  Serial.print(' ');
  Serial.println(conveyorOn ? 1 : 0);
}

static void setConveyor(bool on) {
  conveyorOn = on;
  digitalWrite(CONVEYOR_RELAY_PIN, (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

static void zeroHere() {
  for (uint8_t i = 0; i < 4; i++) AX[i]->setCurrentPosition(toSteps(i, READY[i]));
  homed = true;
  Serial.println(F("I ZERO SET HERE"));
}

static void setSpeed(float v, float a) {
  for (uint8_t i = 0; i < 4; i++) {
    AX[i]->setMaxSpeed(v);
    AX[i]->setAcceleration(a);
  }
}

static void enableMotors(bool on) {
  enabled = on;
  digitalWrite(ENABLE_PIN, on ? LOW : HIGH);
}

#if USE_LIMIT_SWITCHES
// Home one axis like the original homing(): run at the seek speed until the switch opens
// (normally-closed switch to GND with INPUT_PULLUP: HIGH = pressed), set the known position,
// then go to the park position.
static bool homeAxis(uint8_t i, uint8_t pin) {
  unsigned long t0 = millis();
  AX[i]->setSpeed(HOME_SEEK_SPEED[i]);
  while (digitalRead(pin) != HIGH) {
    AX[i]->runSpeed();
    if (millis() - t0 > HOMING_TIMEOUT_MS) return false;
  }
  AX[i]->setCurrentPosition(HOME_SWITCH_STEPS[i]);
  AX[i]->moveTo(HOME_PARK_STEPS[i]);
  while (AX[i]->distanceToGo() != 0) AX[i]->run();
  return true;
}

static void homeAll() {
  Serial.println(F("I HOMING"));
  homed = false;
  enableMotors(true);
  const uint8_t order[4] = {3, 2, 1, 0};          // Z, J3, J2, J1 - same order as the original
  const uint8_t pins[4] = {LIMIT_J1, LIMIT_J2, LIMIT_J3, LIMIT_Z};
  for (uint8_t k = 0; k < 4; k++) {
    uint8_t i = order[k];
    if (!homeAxis(i, pins[i])) {
      Serial.print(F("E homing timeout on axis "));
      Serial.println(i + 1);
      for (uint8_t j = 0; j < 4; j++) AX[j]->moveTo(AX[j]->currentPosition());
      return;
    }
  }
  homed = true;
  Serial.println(F("I HOMED"));
}
#endif

// ----------------------------------------------------------------- commands
static void handleLine(char *s) {
  char *cmd = strtok(s, " \t\r");
  if (!cmd) return;
  switch (cmd[0]) {
    case 'M': case 'm': {
      float v[5];
      for (uint8_t i = 0; i < 5; i++) {
        char *tok = strtok(NULL, " \t\r");
        if (!tok) { Serial.println(F("E M needs 5 numbers")); return; }
        v[i] = atof(tok);
      }
#if TEACH_WITH_POTS
      (void)v;
      Serial.println(F("W teach mode: M ignored"));
#else
      if (!homed) { Serial.println(F("W not homed - send K (limit switches) or H (zero here)")); return; }
      if (!enabled) { Serial.println(F("W motors are free - send L to lock them")); return; }
      moveTo(v);
      setGripper(v[4]);
#endif
      break;
    }
    case 'H': case 'h': case 'Z': case 'z': zeroHere(); break;
    case 'K': case 'k':
#if USE_LIMIT_SWITCHES
      homeAll();
#else
      Serial.println(F("W limit switches disabled (USE_LIMIT_SWITCHES 0) - use H at the ready pose"));
#endif
      break;
    case 'F': case 'f':
      enableMotors(false);
      Serial.println(F("I MOTORS FREE"));
      break;
    case 'L': case 'l':
      enableMotors(true);
      for (uint8_t i = 0; i < 4; i++) AX[i]->moveTo(AX[i]->currentPosition());   // hold where it is
      Serial.println(F("I MOTORS LOCKED"));
      break;
    case 'C': case 'c': {
      char *a = strtok(NULL, " \t\r");
      setConveyor(a && atoi(a) != 0);
      Serial.println(conveyorOn ? F("I CONVEYOR ON") : F("I CONVEYOR OFF"));
      break;
    }
    case 'V': case 'v': {
      char *a = strtok(NULL, " \t\r");
      char *b = strtok(NULL, " \t\r");
      if (a && b) setSpeed(atof(a), atof(b));
      break;
    }
    case 'S': case 's':
      for (uint8_t i = 0; i < 4; i++) AX[i]->stop();
      break;
    case 'E': case 'e': {
      char *a = strtok(NULL, " \t\r");
      enableMotors(a && atoi(a) != 0);
      break;
    }
    case 'G': case 'g': {           // calibration: servo straight to an angle
      char *a = strtok(NULL, " \t\r");
      if (a) {
        int deg = atoi(a);
        deg = deg < 0 ? 0 : (deg > 180 ? 180 : deg);
        gripper.write(deg);
        Serial.print(F("I SERVO "));
        Serial.println(deg);
      }
      break;
    }
    case '?': sendStatus(); break;
    default:
      Serial.print(F("E unknown command "));
      Serial.println(cmd);
  }
}

static void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[lineLen] = 0;
      handleLine(line);
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

#if TEACH_WITH_POTS
const uint8_t FILTER = 16;
int potBuf[4][FILTER];
long potSum[4];
uint8_t potIdx = 0;
unsigned long lastPot = 0;

static void initPots() {
  for (uint8_t i = 0; i < 4; i++) {
    int raw = analogRead(POT_PINS[i]);
    potSum[i] = (long)raw * FILTER;
    for (uint8_t k = 0; k < FILTER; k++) potBuf[i][k] = raw;
  }
}

static void followPots() {
  if (millis() - lastPot < 20) return;
  lastPot = millis();
  float target[4];
  for (uint8_t i = 0; i < 4; i++) {
    int raw = analogRead(POT_PINS[i]);
    potSum[i] += raw - potBuf[i][potIdx];
    potBuf[i][potIdx] = raw;
    float avg = potSum[i] / (float)FILTER;
    target[i] = LIM_LO[i] + (LIM_HI[i] - LIM_LO[i]) * avg / 1023.0;
  }
  potIdx = (potIdx + 1) % FILTER;
  if (homed) moveTo(target);
}
#endif

// ----------------------------------------------------------------- main
void setup() {
  Serial.begin(BAUD);
#if USE_LIMIT_SWITCHES
  pinMode(LIMIT_J1, INPUT_PULLUP);
  pinMode(LIMIT_J2, INPUT_PULLUP);
  pinMode(LIMIT_J3, INPUT_PULLUP);
  pinMode(LIMIT_Z, INPUT_PULLUP);
#endif
  pinMode(CONVEYOR_RELAY_PIN, OUTPUT);
  setConveyor(false);
  pinMode(ENABLE_PIN, OUTPUT);
  enableMotors(true);
  setSpeed(MAX_SPEED, ACCEL);
  gripper.attach(SERVO_PIN, 600, 2500);
  setGripper(0);
#if USE_LIMIT_SWITCHES
  Serial.println(F("I SCARA twin firmware ready - send K to home on the limit switches"));
#else
  Serial.println(F("I SCARA twin firmware ready - put the arm at the ready pose and send H"));
#endif
#if TEACH_WITH_POTS
  initPots();
#if USE_LIMIT_SWITCHES
  homeAll();                       // teach mode homes on power-up
#else
  zeroHere();
#endif
#endif
}

void loop() {
  readSerial();
#if TEACH_WITH_POTS
  followPots();
#endif
  runAll();
  if (millis() - lastStatus >= STATUS_PERIOD_MS) {
    lastStatus = millis();
    sendStatus();
  }
}
