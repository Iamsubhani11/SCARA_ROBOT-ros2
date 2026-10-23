# MuJoCo

Everything that runs the cell in MuJoCo. Start from the repository folder (`~/Scara_robot`).

| program | what it does |
|---|---|
| `launch_mujoco.py` | the table (default), `--auto` sorting cell, `--command` command cell, `--gui` + the app/website (`--open` opens the browser) |
| `scara_mujoco_teleop.py` | drive the arm with the keyboard |
| `scara_mujoco_joystick.py` | drive it with an Xbox pad |
| `mujoco_twin.py` | MuJoCo follows (or drives) the real robot, no ROS |
| `scara_mujoco_ros.py` | MuJoCo with the same ROS 2 topics as Gazebo (used by `launch/mujoco_ros.launch.py`) |
| `models/` | the three scenes: `scara_scene_mujoco.xml` (sorting), `scara_command_mujoco.xml`, `scara_manual_mujoco.xml` (table) |

```bash
python3 mujoco/launch_mujoco.py --auto --show
```

The scenes are generated from `common/scara_scene.py` by `python3 tools/build_scene.py` – change the
numbers there, not the XML.
