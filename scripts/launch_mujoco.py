#!/usr/bin/env python3

import os
import sys
import time
import argparse
import mujoco
import mujoco.viewer

def get_scene_xml_path():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xml_path = os.path.join(pkg_dir, 'urdf', 'scara_scene_mujoco.xml')
    if not os.path.exists(xml_path):
        xml_path = '/home/hemanthros/scara_description_ws/urdf/scara_scene_mujoco.xml'
    return xml_path

def main():
    parser = argparse.ArgumentParser(description="SCARA MuJoCo Simulation Launcher")
    parser.add_argument('--auto', action='store_true', help='Run automated pick and place sequence')
    args, unknown = parser.parse_known_args()

    xml_path = get_scene_xml_path()
    print(f"Loading MuJoCo model from: {xml_path}")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print("\n=======================================================")
    print("=== SCARA MuJoCo Simulation Backend Started ===")
    print("=======================================================")

    if not args.auto:
        print("\n>>> MANUAL PICKING CONTROL MODE ACTIVE <<<")
        print("1. Expand the 'Control' tab on the right sidebar in the MuJoCo window.")
        print("2. Use the interactive sliders to move the SCARA robot joints manually:")
        print("   - act_column      : Rotate base column left / right (-2.0 to 2.0 rad)")
        print("   - act_shoulder    : Move Z-axis UP / DOWN (-0.15 to 0.02 m)")
        print("   - act_forearm     : Rotate forearm arm angle (-2.0 to 2.0 rad)")
        print("   - act_wrist       : Rotate gripper wrist angle")
        print("   - act_left_finger : Close / Open left gripper finger")
        print("   - act_right_finger: Close / Open right gripper finger")
        print("=======================================================\n")
    else:
        print("\n>>> AUTOMATED PICK AND PLACE MODE ACTIVE <<<\n")

    # Initial Pose
    data.ctrl[0] = 0.0    # column
    data.ctrl[1] = 0.0    # shoulder Z-height
    data.ctrl[2] = 0.0    # forearm
    data.ctrl[3] = 0.0    # wrist
    data.ctrl[4] = -0.05  # left finger open
    data.ctrl[5] = -0.05  # right finger open

    STEPS = [
        ("Step 1: Home Position & Open Gripper", [0.0, 0.0, 0.0, 0.0, -0.05, -0.05], 2.5),
        ("Step 2: Traverse Arm over Yellow Puck on Conveyor", [-0.32, 0.0, 0.75, 0.0, -0.05, -0.05], 2.5),
        ("Step 3: Descend Z-Axis to Yellow Puck", [-0.32, -0.14, 0.75, 0.0, -0.05, -0.05], 2.0),
        ("Step 4: Grasp Yellow Puck", [-0.32, -0.14, 0.75, 0.0, 0.0, 0.0], 2.0),
        ("Step 5: Lift Yellow Puck off Conveyor Track", [-0.32, 0.0, 0.75, 0.0, 0.0, 0.0], 2.0),
        ("Step 6: Swing Arm across Wooden Table Platform", [0.785, 0.0, 0.85, 0.0, 0.0, 0.0], 3.0),
        ("Step 7: Descend Z-Axis to Table Surface", [0.785, -0.10, 0.85, 0.0, 0.0, 0.0], 2.0),
        ("Step 8: Release Gripper", [0.785, -0.10, 0.85, 0.0, -0.05, -0.05], 2.0),
        ("Step 9: Retract Z-Axis", [0.785, 0.0, 0.85, 0.0, -0.05, -0.05], 2.0),
        ("Step 10: Return Home Position", [0.0, 0.0, 0.0, 0.0, -0.05, -0.05], 2.5),
    ]

    current_step_idx = 0
    step_start_time = time.time()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            now = time.time()

            if args.auto:
                step_name, target_ctrl, step_dur = STEPS[current_step_idx]
                elapsed_in_step = now - step_start_time

                if elapsed_in_step >= step_dur:
                    print(f"[Completed] {step_name}")
                    current_step_idx = (current_step_idx + 1) % len(STEPS)
                    step_start_time = now
                    step_name, target_ctrl, step_dur = STEPS[current_step_idx]
                    print(f"[Starting] {step_name}")

                for i in range(model.nu):
                    data.ctrl[i] = target_ctrl[i]

            # Step physics simulation forward
            mujoco.mj_step(model, data)

            # Sync GUI viewer state
            viewer.sync()

            # Maintain physics rate
            time.sleep(model.opt.timestep)

if __name__ == '__main__':
    main()
