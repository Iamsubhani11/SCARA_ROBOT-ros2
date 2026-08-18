#!/usr/bin/env python3

import os
import subprocess
import time

# ============================================================
# SCARA WORKSPACE
# ============================================================

WORKSPACE = os.path.expanduser("~/scara_robot")

URDF_FILE = os.path.join(
    WORKSPACE,
    "urdf",
    "SCARA URDF.urdf"
)

MESH_DIR = os.path.join(
    WORKSPACE,
    "meshes"
)

TEMP_URDF = "/tmp/scara_gazebo.urdf"
TEMP_SDF = "/tmp/scara_gazebo.sdf"

# ============================================================
# CHECK FILES
# ============================================================

print("======================================")
print(" SCARA Gazebo Sim 8 Launcher")
print("======================================")

print("URDF:", URDF_FILE)
print("Meshes:", MESH_DIR)

if not os.path.isfile(URDF_FILE):
    print("ERROR: URDF not found!")
    exit(1)

if not os.path.isdir(MESH_DIR):
    print("ERROR: meshes directory not found!")
    exit(1)

print("URDF found.")
print("Meshes found.")

# ============================================================
# READ URDF
# ============================================================

with open(URDF_FILE, "r") as f:
    urdf = f.read()

# ============================================================
# CHANGE package:// mesh references to absolute file paths
# ============================================================

urdf = urdf.replace(
    "package://SCARA Robot/meshes/",
    "file://" + MESH_DIR + "/"
)

urdf = urdf.replace(
    "package://scara_robot/meshes/",
    "file://" + MESH_DIR + "/"
)

urdf = urdf.replace(
    "package://scara_description/meshes/",
    "file://" + MESH_DIR + "/"
)

# Save temporary URDF
with open(TEMP_URDF, "w") as f:
    f.write(urdf)

print("Temporary URDF:", TEMP_URDF)

# ============================================================
# URDF -> SDF
# ============================================================

print("\nConverting URDF to SDF...")

result = subprocess.run(
    ["gz", "sdf", "-p", TEMP_URDF],
    capture_output=True,
    text=True
)

if result.returncode != 0:
    print("ERROR converting URDF to SDF")
    print(result.stderr)
    exit(1)

with open(TEMP_SDF, "w") as f:
    f.write(result.stdout)

print("SDF created:", TEMP_SDF)

if "<model" not in result.stdout:
    print("ERROR: No model found in generated SDF")
    exit(1)

print("SDF model detected.")

# ============================================================
# GAZEBO RESOURCE PATH
# ============================================================

env = os.environ.copy()

env["GZ_SIM_RESOURCE_PATH"] = (
    MESH_DIR + ":" +
    env.get("GZ_SIM_RESOURCE_PATH", "")
)

# ============================================================
# START GAZEBO
# ============================================================

print("\nStarting Gazebo Sim 8...")

gazebo = subprocess.Popen(
    [
        "gz",
        "sim",
        "-v",
        "4",
        "empty.sdf"
    ],
    env=env
)

print("Waiting for Gazebo...")
time.sleep(6)

# ============================================================
# SPAWN SCARA
# ============================================================

print("\nSpawning SCARA...")

spawn_request = (
    'sdf_filename: "' +
    TEMP_SDF +
    '", name: "scara"'
)

spawn = subprocess.run(
    [
        "gz",
        "service",
        "-s",
        "/world/empty/create",
        "--reqtype",
        "gz.msgs.EntityFactory",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "5000",
        "--req",
        spawn_request
    ],
    env=env,
    capture_output=True,
    text=True
)

print("\n======================================")
print(" SPAWN RESULT")
print("======================================")

print(spawn.stdout)

if spawn.stderr:
    print(spawn.stderr)

print("======================================")
