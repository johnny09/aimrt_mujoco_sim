#!/usr/bin/env python3
"""
UR5 Robot Visualization Demo for Foxglove

This demo loads a URDF file and visualizes the UR5 robot in Foxglove Studio.
It publishes:
1. Robot description (URDF) to /robot_description
2. Joint states to /joint_states
3. TF transforms to /tf

Usage:
    1. Install foxglove Python SDK: pip install foxglove-sdk
    2. Run this script: python ur5_visualization_demo.py
    3. The script will generate fixed URDF files in the urdf/ directory
    4. Open Foxglove Studio (https://studio.foxglove.dev or desktop app)
    5. IMPORTANT: Manually load the URDF file in Foxglove:
       - File -> Open -> Select: urdf/ur5_robot_fixed_relative.urdf
       - Make sure to open from the correct directory (foxglove/test/)
    6. Connect to the local server (default: ws://localhost:8765)
    7. Add panels:
       - "3D" panel to visualize the robot (URDF should already be loaded)
       - "Plot" panel to view joint states from /joint_states topic
       - "Raw Messages" panel to view /robot_description (for reference)

Note:
    - The robot will animate with sinusoidal joint movements for demonstration.
    - Mesh files use relative paths, so you MUST manually load the URDF file
      (loading from /robot_description topic won't work for mesh files).
    - The script generates two versions:
      * ur5_robot_fixed_relative.urdf - Use this one (relative paths)
      * ur5_robot_fixed_absolute.urdf - Absolute paths (may not work in browser)
"""

import logging
import math
import re
import time
from pathlib import Path

import foxglove
from foxglove import Channel
from foxglove.schemas import (
    FrameTransform,
    FrameTransforms,
    Quaternion,
)

# UR5 joint names (in order from base to end effector)
UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# UR5 link names (for TF transforms)
UR5_LINKS = [
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "ee_link",
]


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to a rotation quaternion."""
    roll, pitch, yaw = roll * 0.5, pitch * 0.5, yaw * 0.5

    sin_r, cos_r = math.sin(roll), math.cos(roll)
    sin_p, cos_p = math.sin(pitch), math.cos(pitch)
    sin_y, cos_y = math.sin(yaw), math.cos(yaw)

    w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
    x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
    y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
    z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y

    return Quaternion(x=x, y=y, z=z, w=w)


def find_actual_mesh_file(mesh_dir: Path, filename: str) -> str:
    """Find the actual mesh file, handling case-insensitive and underscore matching."""
    if not mesh_dir.exists():
        return filename

    # Try exact match first
    if (mesh_dir / filename).exists():
        return filename

    # Try case-insensitive match
    filename_lower = filename.lower()
    for actual_file in mesh_dir.iterdir():
        if actual_file.name.lower() == filename_lower:
            logging.debug(
                f"Found mesh file (case mismatch): {actual_file.name} -> {filename}"
            )
            return actual_file.name

    # Try matching with underscore variations (e.g., upper_arm.stl -> upperarm.stl)
    # Remove underscores and try again
    filename_no_underscore = filename_lower.replace("_", "")
    for actual_file in mesh_dir.iterdir():
        actual_name_lower = actual_file.name.lower()
        # Try: remove underscores from both
        if actual_name_lower.replace("_", "") == filename_no_underscore:
            logging.debug(
                f"Found mesh file (underscore mismatch): {actual_file.name} -> {filename}"
            )
            return actual_file.name
        # Try: add underscores (less common but possible)
        if actual_name_lower == filename_no_underscore:
            logging.debug(
                f"Found mesh file (underscore mismatch): {actual_file.name} -> {filename}"
            )
            return actual_file.name

    # If not found, return original (might be in a different location)
    logging.warning(f"Mesh file not found: {mesh_dir / filename}")
    return filename


def load_urdf(urdf_path: str, use_absolute_paths: bool = False) -> str:
    """Load URDF file content and fix mesh paths for Foxglove.

    Args:
        urdf_path: Path to the URDF file
        use_absolute_paths: If True, use absolute paths for mesh files.
                           If False, use relative paths from URDF location.

    Returns:
        URDF content with fixed mesh paths
    """
    urdf_path_obj = Path(urdf_path).resolve()
    urdf_dir = urdf_path_obj.parent

    with open(urdf_path, "r") as f:
        urdf_content = f.read()

    # Convert package:// paths to relative or absolute paths
    # Pattern to match package:// paths and relative paths
    package_pattern = r'(package://ur_description/|ur_description/)([^"]+)'

    mesh_files_found = 0
    mesh_files_missing = 0

    def replace_mesh_path(match):
        nonlocal mesh_files_found, mesh_files_missing
        prefix = match.group(1)
        relative_path = match.group(2)

        # Split path into directory and filename
        path_parts = relative_path.split("/")
        mesh_filename = path_parts[-1]
        mesh_subdir = "/".join(path_parts[:-1])

        # Construct paths
        if mesh_subdir:
            mesh_dir = urdf_dir / "ur_description" / mesh_subdir
        else:
            mesh_dir = urdf_dir / "ur_description"

        # Find actual filename (handle case sensitivity)
        actual_filename = find_actual_mesh_file(mesh_dir, mesh_filename)
        actual_mesh_path = mesh_dir / actual_filename

        if actual_mesh_path.exists():
            mesh_files_found += 1
            if use_absolute_paths:
                # Use absolute path with forward slashes
                # Foxglove should be able to resolve absolute paths
                abs_path = str(actual_mesh_path.resolve()).replace("\\", "/")
                fixed_path = abs_path
            else:
                # Use relative path from URDF file location
                # URDF file is in urdf/ directory, mesh files are in urdf/ur_description/
                # So relative path should work when loading from urdf/ directory
                # Try multiple formats for compatibility
                if mesh_subdir:
                    # Format 1: Simple relative path (most common)
                    fixed_path = f"ur_description/{mesh_subdir}/{actual_filename}"
                    # Alternative: relative to current directory when URDF is loaded
                    # fixed_path = f"./ur_description/{mesh_subdir}/{actual_filename}"
                else:
                    fixed_path = f"ur_description/{actual_filename}"
                fixed_path = fixed_path.replace("\\", "/")
        else:
            mesh_files_missing += 1
            # Still return a path, even if file doesn't exist
            if mesh_subdir:
                fixed_path = f"ur_description/{mesh_subdir}/{actual_filename}"
            else:
                fixed_path = f"ur_description/{actual_filename}"
            fixed_path = fixed_path.replace("\\", "/")

        return fixed_path

    # Replace all package:// and relative paths
    urdf_content = re.sub(package_pattern, replace_mesh_path, urdf_content)

    # Also handle any paths that might use backslashes
    urdf_content = urdf_content.replace("\\", "/")

    # Debug: show a sample of replaced paths
    sample_matches = re.findall(r'filename="([^"]+)"', urdf_content)
    if sample_matches:
        logging.debug(f"Sample mesh paths after replacement: {sample_matches[:3]}")

    logging.info(
        f"Fixed mesh paths in URDF: {mesh_files_found} found, "
        f"{mesh_files_missing} missing. Using {'absolute' if use_absolute_paths else 'relative'} paths."
    )

    if mesh_files_missing > 0:
        logging.warning(
            f"{mesh_files_missing} mesh files not found. "
            "Make sure mesh files are accessible from Foxglove."
        )

    return urdf_content


def main():
    # Set logging level
    foxglove.set_log_level(logging.INFO)

    # Get URDF file path
    script_dir = Path(__file__).parent
    urdf_path = script_dir / "urdf" / "ur5_robot.urdf"

    if not urdf_path.exists():
        logging.error(f"URDF file not found: {urdf_path}")
        return

    # Load URDF content and fix mesh paths
    # Use absolute paths - Foxglove desktop app can access them
    # Browser version may have limitations, but absolute paths are most reliable
    urdf_content = load_urdf(str(urdf_path), use_absolute_paths=True)
    logging.info(f"Loaded URDF from: {urdf_path}")

    # Save the fixed URDF file
    fixed_urdf_path = script_dir / "urdf" / "ur5_robot_fixed.urdf"
    with open(fixed_urdf_path, "w") as f:
        f.write(urdf_content)
    logging.info(f"Saved fixed URDF to: {fixed_urdf_path}")

    logging.info(
        "\n" + "=" * 70 + "\n"
        "IMPORTANT: For mesh files to load correctly in Foxglove:\n"
        "OPTION 1 (Recommended): Use Foxglove Desktop App\n"
        "  1. Download and install Foxglove Desktop App (not browser version)\n"
        "  2. File -> Open -> Select: " + str(fixed_urdf_path) + "\n"
        "  3. Desktop app can access local file system and load mesh files\n\n"
        "OPTION 2: Use Browser Version with Manual Setup\n"
        "  1. Open Foxglove Studio in browser\n"
        "  2. File -> Open -> Select the fixed URDF file\n"
        "  3. If mesh files don't load, you may need to serve files via HTTP\n"
        "     or use a local web server\n\n"
        "NOTE: Loading URDF from /robot_description topic may not work\n"
        "      for mesh files due to browser security restrictions.\n"
        "=" * 70
    )

    # Create channels
    # Robot description channel (string message)
    # ROS std_msgs/String format: {"data": "string_content"}
    robot_desc_channel = Channel(
        topic="/robot_description",
        message_encoding="json",
        schema={
            "type": "object",
            "properties": {
                "data": {"type": "string"},
            },
            "required": ["data"],
        },
    )

    # Joint state channel (ROS sensor_msgs/JointState compatible format)
    joint_state_channel = Channel(
        topic="/joint_states",
        message_encoding="json",
        schema={
            "type": "object",
            "properties": {
                "header": {
                    "type": "object",
                    "properties": {
                        "stamp": {
                            "type": "object",
                            "properties": {
                                "sec": {"type": "integer"},
                                "nanosec": {"type": "integer"},
                            },
                        },
                        "frame_id": {"type": "string"},
                    },
                },
                "name": {"type": "array", "items": {"type": "string"}},
                "position": {"type": "array", "items": {"type": "number"}},
                "velocity": {"type": "array", "items": {"type": "number"}},
                "effort": {"type": "array", "items": {"type": "number"}},
            },
        },
    )

    # Start Foxglove server
    server = foxglove.start_server()
    logging.info("Foxglove server started. Connect to it from Foxglove Studio.")
    logging.info("Waiting for clients to connect...")

    # Wait a bit for clients to connect
    time.sleep(2.0)

    # Publish robot description immediately
    logging.info("Publishing robot description (initial)...")
    robot_desc_msg = {"data": urdf_content}
    robot_desc_channel.log(robot_desc_msg)
    logging.info(
        f"Published robot description to /robot_description "
        f"({len(urdf_content)} bytes)"
    )

    # Main loop: publish joint states and TF transforms
    counter = 0
    last_robot_desc_publish = time.time()  # Initialize with current time
    robot_desc_publish_interval = 5.0  # Publish robot description every 5 seconds

    try:
        while True:
            counter += 1
            now = time.time()

            # Publish robot description periodically so new clients can receive it
            if now - last_robot_desc_publish >= robot_desc_publish_interval:
                logging.info("Publishing robot description...")
                # ROS std_msgs/String format: {"data": "string_content"}
                robot_desc_msg = {"data": urdf_content}
                robot_desc_channel.log(robot_desc_msg)
                logging.info(
                    f"Published robot description to /robot_description "
                    f"({len(urdf_content)} bytes, {len(urdf_content.split(chr(10)))} lines)"
                )
                last_robot_desc_publish = now

            # Generate sinusoidal joint positions for demonstration
            # Each joint has a different phase and frequency
            joint_positions = []
            joint_velocities = []
            joint_efforts = []

            for i, joint_name in enumerate(UR5_JOINT_NAMES):
                # Different frequency and phase for each joint
                frequency = 0.1 + i * 0.05  # 0.1 to 0.35 Hz
                phase = i * math.pi / 3  # Different phase for each joint
                amplitude = math.pi / 4  # 45 degrees amplitude

                position = amplitude * math.sin(2 * math.pi * frequency * now + phase)
                velocity = (
                    amplitude
                    * 2
                    * math.pi
                    * frequency
                    * math.cos(2 * math.pi * frequency * now + phase)
                )
                effort = 0.0  # No effort for this demo

                joint_positions.append(position)
                joint_velocities.append(velocity)
                joint_efforts.append(effort)

            # Publish joint states (ROS sensor_msgs/JointState format)
            joint_state_msg = {
                "header": {
                    "stamp": {
                        "sec": int(now),
                        "nanosec": int((now % 1) * 1e9),
                    },
                    "frame_id": "base_link",
                },
                "name": UR5_JOINT_NAMES,
                "position": joint_positions,
                "velocity": joint_velocities,
                "effort": joint_efforts,
            }
            joint_state_channel.log(joint_state_msg)

            # Publish TF transforms
            # For simplicity, we'll publish a transform from world to base_link
            # In a real scenario, you would compute all transforms based on joint positions
            # using forward kinematics
            transforms = [
                FrameTransform(
                    parent_frame_id="world",
                    child_frame_id="base_link",
                    rotation=euler_to_quaternion(roll=0, pitch=0, yaw=0),
                ),
            ]
            foxglove.log("/tf", FrameTransforms(transforms=transforms))

            # Sleep to maintain ~30 Hz update rate
            time.sleep(1.0 / 30.0)

    except KeyboardInterrupt:
        logging.info("Shutting down...")
        server.stop()


if __name__ == "__main__":
    main()
