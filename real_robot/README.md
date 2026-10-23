# The real robot

| what | |
|---|---|
| `firmware/scara_twin/` | the Arduino sketch (UNO + CNC Shield V3, HowToMechatronics pins and numbers) |
| `scara_camera_setup.py` | train the webcam once: colours (click on cubes) and the belt ruler |
| `scara_hw_cell.py` | the autonomous webcam cell: teach the poses (`--teach`), then sort or take commands |
| `scara_cam2d.py` | the webcam detector used by the cell |
| `scara_twin_bridge.py` | ROS 2 node: the real robot as a ghost in RViz (`launch/digital_twin.launch.py`) |

```bash
python3 real_robot/scara_camera_setup.py
python3 real_robot/scara_hw_cell.py --port /dev/ttyACM0 --teach
python3 server/scara_twin_server.py --sim hardware --port /dev/ttyACM0     # the cell in the app / website
```

No robot yet? Add `--fake` anywhere for a software robot that speaks the same serial protocol.
