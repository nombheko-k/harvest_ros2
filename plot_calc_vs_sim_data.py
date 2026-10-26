#!/usr/bin/env python3
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# ROS 2 Core Python utilities for MCAP/Storage Deserialization
try:
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("[!] Error: This script requires a sourced ROS 2 environment.")
    print("    Please execute: source /opt/ros/<distro>/setup.bash")
    sys.exit(1)

def extract_kinematic_comparison(bag_path, target_joint="elbow_joint", 
                                 ik_topic="/arm_controller/joint_trajectory", 
                                 fk_topic="/joint_states"):
    """
    Parses both the IK calculated command topic and the FK simulation telemetry topic 
    from the MCAP file, then aligns their timelines for comparison.
    """
    if not bag_path or not os.path.exists(bag_path):
        print(f"[!] Error: Provided path does not exist: {bag_path}")
        sys.exit(1)

    # Configure native ROS 2 MCAP Reader engine
    storage_id = "mcap" if bag_path.endswith(".mcap") else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[!] Failed to open bag file: {e}")
        sys.exit(1)

    # Compile the active topic/type schema registry inside the bag file
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Verify both essential comparison tracks exist
    for topic in [ik_topic, fk_topic]:
        if topic not in type_map:
            print(f"[!] Error: Target comparison topic '{topic}' not found in this bag.")
            sys.exit(1)

    # Pre-load the Python message interface classes dynamically
    IK_MsgClass = get_message(type_map[ik_topic])
    FK_MsgClass = get_message(type_map[fk_topic])

    # Temporary storage lists
    t_ik, pos_ik = [], []
    t_fk, pos_fk = [], []

    # Sequence iteration loop across binary storage tracks
    while reader.has_next():
        topic, data, ts = reader.read_next()
        timestamp_sec = ts / 1e9

        # Track 1: Parse Calculated IK Positions
        if topic == ik_topic:
            msg = deserialize_message(data, IK_MsgClass)
            # Handle JointTrajectory array messages (common for IK nodes)
            if hasattr(msg, 'joint_names') and target_joint in msg.joint_names:
                j_idx = msg.joint_names.index(target_joint)
                for pt in msg.points:
                    # Trajectory points contain an execution duration offset
                    t_offset = pt.time_from_start.sec + (pt.time_from_start.nanosec / 1e9)
                    t_ik.append(timestamp_sec + t_offset)
                    pos_ik.append(pt.positions[j_idx])
            # Fallback if your node outputs standard JointState updates instead
            elif hasattr(msg, 'name') and target_joint in msg.name:
                t_ik.append(timestamp_sec)
                pos_ik.append(msg.position[msg.name.index(target_joint)])

        # Track 2: Parse Measured FK Simulation Positions
        elif topic == fk_topic:
            msg = deserialize_message(data, FK_MsgClass)
            if target_joint in msg.name:
                t_fk.append(timestamp_sec)
                pos_fk.append(msg.position[msg.name.index(target_joint)])

    if not pos_ik or not pos_fk:
        print(f"[!] Error: Could not find telemetry for '{target_joint}' in one or both topics.")
        sys.exit(1)

    # Convert to Numpy Arrays
    t_ik, pos_ik = np.array(t_ik), np.array(pos_ik)
    t_fk, pos_fk = np.array(t_fk), np.array(pos_fk)

    # Normalize timelines relative to the earliest record to start perfectly at 0.0s
    t_start = min(t_ik[0], t_fk[0])
    t_ik -= t_start
    t_fk -= t_start

    # Sort data arrays chronologically (necessary for multi-threaded node recordings)
    sort_ik = np.argsort(t_ik)
    t_ik, pos_ik = t_ik[sort_ik], pos_ik[sort_ik]
    sort_fk = np.argsort(t_fk)
    t_fk, pos_fk = t_fk[sort_fk], pos_fk[sort_fk]

    # ALIGN TIMELINES: Interpolate the calculated IK data onto the Gazebo simulation (FK) timeline
    pos_ik_aligned = np.interp(t_fk, t_ik, pos_ik)

    return t_fk, pos_ik_aligned, pos_fk

def main():
    if len(sys.argv) < 2:
        print("[!] Usage: python3 script.py /path/to/bag.mcap [joint_name] [ik_topic] [fk_topic]")
        sys.exit(1)

    bag_path = sys.argv[1]
    target_joint = sys.argv[2] if len(sys.argv) > 2 else "elbow_joint"
    ik_topic = sys.argv[3] if len(sys.argv) > 3 else "/arm_controller/joint_trajectory"
    fk_topic = sys.argv[4] if len(sys.argv) > 4 else "/joint_states"

    # Extract and align tracking data
    t, pos_ik, pos_fk = extract_kinematic_comparison(bag_path, target_joint, ik_topic, fk_topic)
    
    # Calculate exact kinematic delta (IK Calculation vs FK Gazebo Reality)
    kinematic_error = pos_ik - pos_fk

    # KINEMATIC ERRORS: GRAPH GENERATION

    fig, (ax_main, ax_err) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, gridspec_kw={'height_ratios': [2, 1]})
    fig.suptitle(f"Kinematic Discrepancy Analysis: {target_joint}", fontsize=14, fontweight='bold')

    # Top Plot: Positional Traces
    ax_main.plot(t, pos_ik, 'b-', linewidth=2.0, label="Calculated Position (IK Node Goal)")
    ax_main.plot(t, pos_fk, 'r--', linewidth=2.0, label="Simulation Position (FK Gazebo Read)")
    ax_main.set_ylabel("Joint Position [rad]", fontsize=11, fontweight='bold')
    ax_main.grid(True, linestyle='--', alpha=0.6)
    ax_main.legend(loc="best", fontsize=10)
    ax_main.set_title("IK Command vs. FK Simulated Feedback Profile", fontsize=11, fontstyle='italic')

    # Bottom Plot: Absolute Kinematic Tracking Error Window
    ax_err.plot(t, kinematic_error, 'm-', linewidth=1.8, label="Kinematic Error (IK - FK)")
    ax_err.axhline(0, color='black', linestyle=':', alpha=0.5)
    ax_err.set_ylabel("Error Delta [rad]", fontsize=11, fontweight='bold')
    ax_err.set_xlabel("Time [seconds]", fontsize=11, fontweight='bold')
    ax_err.grid(True, linestyle='--', alpha=0.6)
    ax_err.legend(loc="upper right", fontsize=10)

    plt.tight_layout()
    plot_name = f"{target_joint}_kinematic_error.png"
    plt.savefig(plot_name, dpi=300)
    print(f"[+] Saved analysis plot to: {plot_name}")
    
    plt.show()

if __name__ == "__main__":
    main()
