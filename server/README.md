# Twin server

`scara_twin_server.py` connects everything: it runs (or follows) the simulation, talks to the real
robot over USB, serves the **website** at http://localhost:8080 and the **phone app** on port 9090.

```bash
python3 server/scara_twin_server.py                              # MuJoCo, switch worlds in the app
python3 server/scara_twin_server.py --sim gazebo --world sort     # starts Gazebo itself
python3 server/scara_twin_server.py --port /dev/ttyACM0          # + the real robot (Twin mode in the app)
python3 server/scara_twin_server.py --fake                       # + a software robot, no hardware
python3 server/scara_twin_server.py --sim hardware --port /dev/ttyACM0   # the real webcam cell
```

The browser opens only with `--open`; otherwise go to the address it prints.
