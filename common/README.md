# Shared code

Used by MuJoCo, Gazebo, the real robot and the server alike – one place for every number and rule.

| module | |
|---|---|
| `scara_scene.py` | every dimension: robot, table, belt, pucks, bins, cameras (change the cell here) |
| `scara_kinematics.py` | forward / inverse kinematics with the real link lengths (`python3 common/scara_kinematics.py` tests it) |
| `scara_safety.py` | the safety mode in front of every manual control |
| `scara_sorter_core.py` | sorting, command mode, sequences, stop-and-pick |
| `scara_feeder.py` | puts pucks on the belt in the simulations |
| `scara_vision.py` | colour + depth puck detection for the simulated cameras |
| `scara_voice.py` | wake word, speech to text, command parsing |
| `scara_twin_protocol.py` | the serial protocol of the Arduino, and the software robot (`--fake`) |
| `scara_models.py`, `scara_joy_core.py` | spawnable models, joystick mapping |
