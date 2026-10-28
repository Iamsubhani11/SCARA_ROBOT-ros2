# SCARA Digital Twin – phone app and browser GUI (v3.3)

`SCARA_Digital_Twin_v3.apk` – same name and package as before (`com.iamsubhani.scaradigitaltwin`),
rebuilt from scratch. It drives the MuJoCo or Gazebo simulation on your PC, the real robot, or both
at once (digital twin) – and with **Try offline** the robot, the table and the conveyor cell run on the
phone itself, no PC needed.

The same page is the PC GUI: `python3 server/scara_twin_server.py` serves it at http://localhost:8080
and connects it by itself. New in v3: the **FK / IK** tab, **voice** commands, **hand control** with the
front camera, exact-position sliders, the keyboard, the activity log, Gazebo world switching, and the
real webcam cell drawn as it stands on the table (`--sim hardware`). New in 3.3: **MediaPipe hand
tracking** (offline, one or two hands, pinch), **Draw a path**, **programs** you can export and import,
**belt tracking** and a **demo for the review**.

## Install

Version 3.3 installs over 3.x – no need to uninstall. It is bigger than before (11 MB, was 1.2 MB) because
MediaPipe's program and hand model are inside it, so hand tracking works without internet.

Uninstall the old app (v1.0 / v1.1) first – the new APK is signed with a different key and Android
refuses to install it on top. Then open the APK on the phone and allow "install unknown apps" for
your file manager. The first start shows a short guide (the **?** button brings it back).

## Connect to the PC

1. On the PC start the twin server – it is the app's partner for everything:
   ```bash
   python3 server/scara_twin_server.py                         # MuJoCo (table; switch to the cell from the app)
   python3 server/scara_twin_server.py --world command         # MuJoCo, starting in the command cell
   python3 server/scara_twin_server.py --sim gazebo            # the Gazebo cell you launched (any mode)
   python3 server/scara_twin_server.py --port /dev/ttyACM0     # + the real robot
   python3 server/scara_twin_server.py --sim hardware --port /dev/ttyACM0 --mode command   # real camera cell
   ```
   It prints `phone app: ws://192.168.x.x:9090`.
2. Phone and PC on the same Wi-Fi. Tap the status pill at the top of the app, type the PC address and
   port 9090, tap **Connect**. The app remembers the address and reconnects by itself next time.
3. If nothing happens: PC firewall – `sudo ufw allow 9090`.

No PC at hand: **Try offline** on the same sheet.

## The screen

| part | what it does |
|---|---|
| **?** | the how-to guide |
| status pill | Connected / Offline / Connect – tap it to connect, change the PC or go offline |
| **STOP** | freezes the arm and the conveyor at once, from any tab. It latches (yellow-black): tap again to release |
| **Simulation / Real robot / Twin** | what your sticks and buttons move: the simulated arm, the real one, or both (the real robot copies the simulation) |
| 3-D view | drag to turn, pinch to zoom; **Front**, **Top**, **Follow** (the view follows the gripper); the small window is the gripper camera (**Arm camera** hides it). The blue ring on the table is how far the arm reaches |
| joint readout | column, height, elbow, wrist, gripper. In Twin mode the real robot's values are under them |

## Manual tab – drive the arm

* **Joysticks / Sliders**: the switch at the top of the tab. Sliders give one exact slider per joint
  and one for the gripper (the PC page shows the sliders under the joysticks).
* **Joints**: left stick = column (←→) and height (↑↓), right stick = elbow (←→) and wrist (↑↓).
* **XY + height**: the left stick moves the gripper over the table in straight lines – push up and it
  goes away from you *as you see it in the 3-D view* (turn the view and the directions turn with it).
  Right stick: height (↑↓) and wrist turn (←→). This is the easy way to pick things up.
* **Tilt**: drive with the phone's tilt instead of the left stick (hold the phone the way you like
  when you switch it on – that is "level"). Tap again to switch it off.
* **Open / Close / Home**, and the **Gripper** slider to close it part way.
* **Speed** slider (10–100 %).
* **Safe / Normal / Off**: Safe is slow with all guards, Normal is faster, Off has no limits at all.
* **Tap to move a puck**: tap a puck in the 3-D view, then tap the spot it should go to – the robot
  picks it up and puts it there by itself.

**Picking by hand:** gripper open → bring it over the puck → go down. Once it is within about 4 cm
the gripper centres itself over the puck (pick assist). Go all the way down, **Close**, lift, move,
lower, **Open**. If a finger would land on the puck the arm stops above it and says so – move it a
little and go down again. The phone buzzes when it grabs.

## Auto tab – the robot works by itself

* **Table / Sorting / Command** chooses the world. Offline it switches the phone's own simulation;
  connected to MuJoCo the PC switches worlds (the viewer window reopens). Gazebo runs the world you
  launched it with (`mode:=sort` / `mode:=command`).
* **Start / Stop** run and stop the conveyor cell.
* **Sorting**: every colour goes to its own bin.
* **Command**: one bin. Tap a colour – the robot keeps picking that colour until you tap another one.
  **Any colour** picks everything, **Stop picking** lets everything pass.
* **Build a sequence**: tap colours in the order you want, then **Run sequence** – one of each, in that order.
* **Command box**: type, or dictate with the microphone on your keyboard: `red`, `3 green`,
  `red then blue then green`, `any`, `none`, `start`, `stop`.
* **Follow belt** (command cell): the robot picks a wanted puck from the moving belt instead of
  stopping it – it follows the puck with the Jacobian while it goes down, closes and lifts. The belt
  only pauses while the arm is at the bin. In the sorting cell it keeps stopping the belt (every puck
  is wanted there and they come too close together). It works offline and with MuJoCo; with Gazebo
  it is switched off for now (the app says so) and the belt stops for every pick.
* **Demo for the review**: the table (draws a star, moves a puck), the sorting cell and the command
  cell with belt tracking, one after another. Tap again or STOP to end it.
* The chips show what is being picked, the sequence still to come, and the totals (R G B Y).
* Offline only: **Add a puck**, **Reset**, and picks per minute.

## Teach tab

Move the arm, **Save this pose**, repeat. **Play** runs the poses in order (with the gripper as saved),
**Loop** repeats them until **Stop**. Each pose has **Go** and **Delete**. Offline the poses are kept on
the phone; connected, they are kept on the PC (`~/.scara_teach_points.json`).

**Programs**: *Save as program* keeps the poses under a name, *Load* brings them back, *Export file*
downloads a `.json` file (in the phone app: saved on the PC in `~/scara_programs/`, or shown as text
to copy when offline), *Import file* reads one back (the phone app takes pasted text).

## FK / IK tab

**Forward kinematics** updates as the arm moves: X, Y, pad height, gripper yaw, reach, elbow side and
manipulability (|sin θ2|, low = close to the straight-arm singularity), in the **Robot** frame (origin on
the column axis, x along the arm at θ1 = 0) or the **World** frame of the simulator.
**Inverse kinematics**: type X, Y, height and yaw – or **Tap the table**, or **Use the gripper's pose** –
and both elbow solutions appear with every joint value. A joint past its limit is red and that
solution cannot be sent; **Go** on the other one moves the arm there. The area the gripper can reach
(joint limits included) is shaded on the table.

**Draw a path**: *Draw on the table* and draw with a finger, or pick a circle, square, star or figure 8.
Red parts are out of reach. *Run path* goes to the start and moves the gripper along the path at its
present height (inverse kinematics for every point, 4 mm apart); *Loop* repeats it.

**Motion graph**: the four joints over the last 10 s. **Record**, move the arm, **Stop recording**,
**Save CSV** – a table of time, joints and gripper position for the graphs in a report (the website
downloads it; the phone app saves it on the PC in `~/scara_recordings/`). *Show the gripper's path*
draws where the gripper went in the 3-D view.

## Voice and hand control

**Voice**: the microphone at the top or *Speak* in the Auto tab. The app uses the phone's speech
recogniser – offline when offline speech is installed (Google app ▸ Settings ▸ Voice). Say "red then
blue", "3 green", "any", "stop picking", "start", "stop", "open", "close", "home", "sorting", "command",
"table", "emergency stop", "resume".

**Gesture** (button under the 3-D view, or in the Manual tab): allow the camera (the app asks for it
the first time and uses the front camera). **MediaPipe** finds your hand and draws its 21 points:

* palm in the middle circle = still; move it = the **left joystick**;
* **fist** closes the gripper, **open hand** opens it;
* **pinch** (thumb + index) and move = the **right joystick** (in *XY + height*: up / down = height);
* **two hands**: the one on the right of the picture is the right joystick.

A hand starts driving only after it has been in its circle once. If MediaPipe cannot start, the
simple **colour tracker** takes over (hold your open palm in the yellow square for 2 s); the ✋ button
switches between them, and the window always says which one runs – or why the camera did not start
(permission, plain http from another computer, camera busy, too dark).

## Robot tab (needs the PC)

Serial port → **Connect robot** (or **Software robot** to try it without hardware) → **Home
switches** (or **Zero here** on a robot without limit switches). **Motors free / Motors on**. In
camera-cell mode (`--sim hardware`) the buttons that save the pick, upstream, drop and parking poses
appear here too, with a tick for every pose already saved.

**Direction check** (first time with the real robot): home it, choose *Twin*, press **Nudge** next to a
joint – the real and the 3-D arm move a little. Other way round? **Opposite** fixes and saves it.

## Offline mode, honestly

The phone's simulator is a simple one: it follows the real geometry, speed limits and pick heights,
but it is not a physics engine (pucks don't push each other, they drop straight down). For the real
thing use MuJoCo or Gazebo on the PC – the controls are exactly the same.

## Changing the app

The app is the web page in `app/ui` (HTML + JavaScript + three.js) inside a small Android WebView shell
(`app/android`). Try changes in Chrome on the PC before building the APK:

```bash
python3 tools/build_app.py --meshes-only          # refresh app/ui/meshes.js + scene.js from the package
python3 -m http.server -d app/ui 8000            # open http://localhost:8000 (phone size: F12 > device toolbar)
python3 tools/build_app.py                        # build app/SCARA_Digital_Twin_v3.apk (needs Java + apktool + signer)
```

With a plain `rosbridge_server` (`digital_twin.launch.py rosbridge:=true`) instead of the twin server,
the sticks, gripper, conveyor and colours work through ROS topics; teach, robot and safety need the
twin server.
