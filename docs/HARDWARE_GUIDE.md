# Building and connecting the real robot – step by step

This is the short, practical version for the day you build the robot. The full details are in the
main README (sections 10–12). Everything here matches the firmware in
`real_robot/firmware/scara_twin/scara_twin.ino` – if this page and the firmware ever disagree, the
firmware wins.

The robot is the HowToMechatronics SCARA (Arduino UNO + CNC Shield V3 + four A4988 drivers + four
NEMA 17 steppers + one MG996R servo). The conveyor cell around it is the one from the Mechamind
Labs videos: a small belt with a DC motor switched by a relay, and one USB webcam on a stand.

## 1. Choose your build (this decides two pins)

| | **A – with limit switches** (HowToMechatronics) | **B – without switches** (Mechamind style) |
|---|---|---|
| How the robot finds zero | homes on 4 switches (`K`, *Home switches*) | you put the arm straight out, Z at 150 mm, and press *Zero here* (`H`) |
| Conveyor relay IN | **A1** (the "Hold" pin) | **D10** (the Y end-stop header) |
| Firmware setting | `#define USE_LIMIT_SWITCHES 1` (default) | `#define USE_LIMIT_SWITCHES 0` |
| Bring-up | `python3 real_robot/scara_bringup.py --port /dev/ttyACM0` | add `--no-switches` |

Do not wire the relay to D10 in build A – D10 is the J2 limit switch there, and homing would go
wrong. Build B has no J2 switch, so D10 is free for the relay (that is why the Mechamind panel says
"relay on the Y+ endstop header (pin 10)").

## 2. Wiring

```
                 12 V 6 A supply ──► CNC shield screw terminal (+ / -)
                                     │
   Arduino UNO ── CNC Shield V3 ─────┼─ X slot  A4988 ─► J1 base motor      (step D2, dir D5)
        │                            ├─ Y slot  A4988 ─► J2 elbow motor     (step D3, dir D6)
        │ USB to the PC              ├─ Z slot  A4988 ─► J3 gripper turn    (step D4, dir D7)
        │                            └─ A slot  A4988 ─► Z lead-screw motor (step D12, dir D13 – set the A-axis jumpers)
        │
        ├─ A0 ("Abort")  ─► servo signal (orange)     servo + / - from a 5–6 V buck converter, NOT the Arduino 5 V
        ├─ D11 / D10 / D9 / A3 ─► limit switches J1 / J2 / J3 / Z (build A; each switch between the pin and GND)
        ├─ A1 (build A) or D10 (build B) ─► relay module IN     relay VCC ─► 5 V, GND ─► GND
        └─ D8 = drivers enable (already on the shield)
   Relay contacts (COM / NO) switch the belt motor's own supply.
   ALL grounds together: 12 V supply, buck converter, relay board, Arduino.
```

| part | pin | note |
|---|---|---|
| J1 base (θ1) | X slot, step D2, dir D5 | 20:1 belts, 44.444 steps/° |
| J2 elbow (θ2) | Y slot, step D3, dir D6 | 16:1 belts, 35.556 steps/° |
| J3 gripper turn | Z slot, step D4, dir D7 | 4:1 belt, 10 steps/° |
| Z axis | A slot, step D12, dir D13 | 8 mm lead screw, 100 steps/mm – A-axis jumpers on D12/D13 |
| limit switches (build A) | J1 D11, J2 D10, J3 D9, Z A3 | normally closed, to GND |
| gripper servo | A0, 600–2500 µs | 180° = open |
| conveyor relay | A1 (build A) / D10 (build B) | `RELAY_ACTIVE_LOW true` for boards that switch on LOW |

Before the first power-up:

1. A4988s the right way round: the driver's **EN** pin on the shield's **EN** mark (a driver put in
   backwards burns out at power-up). Turn their current limit low – you raise it later only as far
   as needed.
2. Micro-stepping: quarter step (the jumpers under each driver as in the HowToMechatronics article) –
   the steps/° numbers above assume it.
3. Servo on its own 5–6 V. An MG996R squeezing a cube can pull more than 2 A – on the Arduino's 5 V
   pin that resets the board.
4. No jumper from EN to GND on the shield, otherwise *Motors free* cannot switch the drivers off.

## 3. Flash the firmware

1. Arduino IDE → Library Manager → **AccelStepper** (Mike McCauley). Servo is built in.
2. Open `real_robot/firmware/scara_twin/scara_twin.ino`. Build B: change `USE_LIMIT_SWITCHES` to `0`.
3. Board **Arduino UNO**, port, **Upload**.
4. Serial Monitor, 115200 baud, *Newline*: you see `I SCARA twin firmware ready`. Close the Serial
   Monitor again – only one program can use the port.

On Linux, once: `sudo usermod -aG dialout $USER`, then log out and in. Clone boards with a CH340
chip show up as `/dev/ttyUSB0` instead of `/dev/ttyACM0`.

## 4. The bring-up check (once, and after every wiring change)

```bash
python3 real_robot/scara_bringup.py --port /dev/ttyACM0            # build A
python3 real_robot/scara_bringup.py --port /dev/ttyACM0 --no-switches   # build B
python3 real_robot/scara_bringup.py --fake --yes                   # see what it does, no robot needed
```

It checks one thing at a time and says in plain words what to fix:

1. the Arduino answers (port, permission, firmware),
2. homing (or zero here),
3. every joint moves a little and you say if it went the right way – a wrong way is saved and every
   program (app, website, webcam cell, digital twin) uses it from then on,
4. gripper open / close,
5. the relay switches the belt,
6. the digital twin: a target sent the way the app sends it comes back as the same joint values,
   and you confirm the real arm looks like the 3-D one.

The robot has no encoders – the Arduino only counts the steps it sent. A motor that skips or does
not turn cannot be seen by any program, so the check asks you to watch.

One more value is yours to set: **`SERVO_CLOSED_DEG`** in the firmware. Put a cube between the
fingers, send `G 120`, `G 100`, `G 90` … in the Serial Monitor and use the angle where it just holds
the cube firmly.

## 5. Drive it and make it a digital twin

```bash
python3 server/scara_twin_server.py --port /dev/ttyACM0                # MuJoCo + the real robot
python3 server/scara_twin_server.py --sim gazebo --world command --port /dev/ttyACM0   # Gazebo + the real robot
```

Open http://localhost:8080 (or the phone app → the PC's address, port 9090):

1. **Robot** tab → *Connect robot* → *Home switches* (build A) or *Zero here* (build B).
2. At the top choose **Twin**: the real robot now copies the simulation – joysticks, sliders, IK,
   the drawn paths, Teach programs and the conveyor cell all move both.
3. The sliders and the IK stay inside the real robot's range (column −89…266°, elbow ±150°, wrist
   ±162°, Z 0…170 mm), so the simulation never goes where the real arm cannot follow.
4. If the USB cable is pulled or the Arduino hangs, the app says **NOT ANSWERING** within 1.5 s and the
   twin stops sending; plug it back in and it carries on.

## 6. The camera cell (Mechamind Part 25)

The real cell uses one ordinary webcam on a stand looking at the pick station – no depth camera.

```bash
python3 real_robot/scara_camera_setup.py                          # train colours + the belt ruler (once)
python3 real_robot/scara_hw_cell.py --port /dev/ttyACM0 --teach --home   # teach pick / drop / ready poses (once)
python3 server/scara_twin_server.py --sim hardware --port /dev/ttyACM0 --mode command   # run it from the app
```

Section 11 of the README explains each key. Before it starts, the cell checks that the camera is
trained, the poses are taught and the robot is homed, and tells you what is missing.

## 7. When something is wrong

| you see | do this |
|---|---|
| a motor hums but does not turn | raise that A4988's current a little; check the motor cable in the right header |
| a joint goes the wrong way | the bring-up check fixes it; or *Direction check → Opposite* in the app |
| the arm is a little off after a crash | it lost steps (no encoders) – home again (`K` / *Home switches*) |
| the Arduino resets when the gripper closes | the servo is on the Arduino's 5 V – give it its own supply |
| the belt runs when it should stop | relay board switches on LOW: `RELAY_ACTIVE_LOW true` |
| homing never ends | a limit switch is wired normally-open or not to GND; press it and send `?` |
| "Permission denied" on /dev/ttyACM0 | `sudo usermod -aG dialout $USER`, log out and in |
