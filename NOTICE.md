# Notices and credits

**SCARA Robot – ROS 2 digital twin** © 2026 Shaik Subhani
([@Iamsubhani11](https://github.com/Iamsubhani11)). The code, robot description, worlds,
firmware, phone app and documentation in this repository are released under the
[MIT License](LICENSE), unless a file says otherwise.

## Mechanical design

The robot is the open SCARA design published by Dejan Nedelkovski on
[HowToMechatronics](https://howtomechatronics.com/projects/scara-robot-how-to-build-your-own-arduino-based-robot/).
The meshes in `meshes/` were made from that design to match the printed robot, and the
firmware keeps the pin map and motion constants of the original Arduino sketch. Thanks to
the author for publishing it – please visit the article for the original files and their terms.

## Third-party software shipped in this repository

| Component | Where | Licence |
|---|---|---|
| three.js r160 (`three.min.js`, `OrbitControls.js`) | `app/ui/lib/` | MIT – `app/ui/lib/LICENSE-three.txt` |
| Barlow and Barlow Semi Condensed fonts | `app/ui/fonts/` | SIL Open Font License 1.1 – `app/ui/fonts/OFL-Barlow.txt` |

## Used, not shipped

ROS 2, Gazebo, MuJoCo, OpenCV, NumPy, MediaPipe, AccelStepper and the Python packages in
`requirements.txt` are installed separately and keep their own licences.
