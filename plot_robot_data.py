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

def extract_multi_target_data(bag_path, target_joint):
    """
    Parses changing commands and actual joint states from the MCAP/db3 bag file,
    tracking multiple consecutive target goals mapped onto Simulation Time.
    """
    # FIX 1: Automatically expand '~' shortcuts if passed as raw strings
    bag_path = os.path.expanduser(bag_path)

    # FIX 2: Dynamic extension checking whether it's a directory or a direct file path
    if bag_path.endswith(".mcap") or bag_path.endswith(".db3"):
        storage_id = "mcap" if bag_path.endswith(".mcap") else "sqlite3"
    else:
        # If a directory is provided, look for internal files to detect format
        storage_id = "mcap"
        for file in os.listdir(bag_path):
            if file.endswith(".db3"):
                storage_id = "sqlite3"
                break

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[!] Failed to open bag file path '{bag_path}': {e}")
        sys.exit(1)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    cmd_topic = "/arm_controller/joint_trajectory"
    fb_topic = "/joint_states"

    for topic in [cmd_topic, fb_topic]:
        if topic not in type_map:
            print(f"[!] Error: Target topic '{topic}' not found in this bag.")
            sys.exit(1)

    Cmd_Class = get_message(type_map[cmd_topic])
    Fb_Class = get_message(type_map[fb_topic])

    # Dynamic target state trackers
    raw_cmd_t, raw_cmd_p = [], []
    raw_fb_t, raw_fb_p = [], []

    while reader.has_next():
        topic, data, ts = reader.read_next()
        
        # Track 1: Captured Trajectory Commands (Changing Targets)
        if topic == cmd_topic:
            msg = deserialize_message(data, Cmd_Class)
            if target_joint in msg.joint_names:
                j_idx = msg.joint_names.index(target_joint)
                
                # Extract simulation stamp from the message header if available
                if hasattr(msg, 'header') and msg.header.stamp.sec != 0:
                    base_sim_time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
                else:
                    base_sim_time = ts / 1e9 # Fallback to recording time
                
                for pt in msg.points:
                    t_offset = pt.time_from_start.sec + (pt.time_from_start.nanosec / 1e9)
                    raw_cmd_t.append(base_sim_time + t_offset)
                    raw_cmd_p.append(pt.positions[j_idx])

        # Track 2: Measured Feedback (Simulation State Timeline)
        elif topic == fb_topic:
            msg = deserialize_message(data, Fb_Class)
            if target_joint in msg.name:
                j_idx = msg.name.index(target_joint)
                if hasattr(msg, 'header') and msg.header.stamp.sec != 0:
                    sim_time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
                else:
                    sim_time = ts / 1e9
                raw_fb_t.append(sim_time)
                raw_fb_p.append(msg.position[j_idx])

    if not raw_cmd_p or not raw_fb_p:
        print(f"[!] Error: Missing data vectors for axis '{target_joint}'.")
        sys.exit(1)

    # Convert to arrays and sort by simulation timeline
    cmd_t, cmd_p = np.array(raw_cmd_t), np.array(raw_cmd_p)
    fb_t, fb_p = np.array(raw_fb_t), np.array(raw_fb_p)

    idx_c = np.argsort(cmd_t)
    cmd_t, cmd_p = cmd_t[idx_c], cmd_p[idx_c]
    
    idx_f = np.argsort(fb_t)
    fb_t, fb_p = fb_t[idx_f], fb_p[idx_f]

    # Shift timeline origin relative to the start of the simulation data
    t_start = fb_t[0]
    cmd_t -= t_start
    fb_t -= t_start

    # Align commanding profile onto the high-rate Gazebo physics simulation clock
    # Uses nearest-neighbor/previous hold interpolation to simulate step target updates
    cmd_p_aligned = np.interp(fb_t, cmd_t, cmd_p)

    # Derive Ideal Commanded Motion Kinematics over Sim Time
    dt = np.diff(fb_t, prepend=-0.005)
    dt[dt <= 0] = 0.005

    vel_cmd = np.gradient(cmd_p_aligned, dt)
    acc_cmd = np.gradient(vel_cmd, dt)
    jerk_cmd = np.gradient(acc_cmd, dt)

    return fb_t, cmd_p_aligned, vel_cmd, acc_cmd, jerk_cmd, fb_p

def generate_joint_plots(bag_path, target_joint):
    print(f"[*] Extracting sim-time profile for joint: {target_joint}...")
    t, p_cmd, v_cmd, a_cmd, j_cmd, p_meas = extract_multi_target_data(bag_path, target_joint)
    
    error = p_cmd - p_meas

    # GRAPH 1: DYNAMIC S-CURVE COMMAND TRAJECTORIES
    fig1, axs = plt.subplots(4, 1, figsize=(11, 8), sharex=True)
    fig1.suptitle(f"Multi-Target Kinematic Profile Commands: {target_joint}", fontsize=13, fontweight='bold')

    axs[0].plot(t, p_cmd, 'b-', linewidth=2.0, label="Command Position s(t)")
    axs[0].set_ylabel("Pos [rad or m]", fontsize=9, fontweight='bold')
    axs[0].grid(True, linestyle='--', alpha=0.5)
    axs[0].legend(loc="upper right")

    axs[1].plot(t, v_cmd, 'g-', linewidth=1.8, label="Derived Velocity v(t)")
    axs[1].set_ylabel("Vel [rad/s]", fontsize=9, fontweight='bold')
    axs[1].grid(True, linestyle='--', alpha=0.5)
    axs[1].legend(loc="upper right")

    axs[2].plot(t, a_cmd, 'r-', linewidth=1.5, label="Derived Acc a(t)")
    axs[2].set_ylabel("Acc [rad/s²]", fontsize=9, fontweight='bold')
    axs[2].grid(True, linestyle='--', alpha=0.5)
    axs[2].legend(loc="upper right")

    axs[3].plot(t, np.clip(j_cmd, -15, 15), 'm-', linewidth=1.2, label="Derived Jerk j(t)")
    axs[3].set_ylabel("Jerk [rad/s³]", fontsize=9, fontweight='bold')
    axs[3].set_xlabel("Simulation Time [seconds]", fontsize=10, fontweight='bold')
    axs[3].grid(True, linestyle='--', alpha=0.5)
    axs[3].legend(loc="upper right")

    plt.tight_layout()
    plot1_out = f"{target_joint}_trajectory_profile.png"
    plt.savefig(f"~/ros2_ws/src/harvest_ros2/{plot1_out}", dpi=300)
    plt.close()

    # GRAPH 2: CLOSED-LOOP PERFORMANCE & COUPLING ERROR OVER TIME
    fig2, (ax_main, ax_err) = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True, gridspec_kw={'height_ratios': [2, 1]})
    fig2.suptitle(f"Dynamic Closed-Loop PID Performance: {target_joint}", fontsize=13, fontweight='bold')

    ax_main.plot(t, p_cmd, 'b-', linewidth=2.0, label="Dynamic Command Reference")
    ax_main.plot(t, p_meas, 'r--', linewidth=1.5, label="Gazebo Physics Telemetry")
    ax_main.set_ylabel("Joint Position", fontsize=10, fontweight='bold')
    ax_main.grid(True, linestyle='--', alpha=0.5)
    ax_main.legend(loc="best")

    ax_err.plot(t, error, 'k-', linewidth=1.5, label="Dynamic Tracking Error (e)")
    ax_err.axhline(0, color='red', linestyle=':', alpha=0.6)
    ax_err.set_ylabel("Error Delta", fontsize=10, fontweight='bold')
    ax_err.set_xlabel("Simulation Time [seconds]", fontsize=10, fontweight='bold')
    ax_err.grid(True, linestyle='--', alpha=0.5)
    ax_err.legend(loc="upper right")

    plt.tight_layout()
    plot2_out = f"{target_joint}_pid_performance.png"
    plt.savefig(f"~/ros2_ws/src/harvest_ros2/{plot2_out}", dpi=300)
    plt.close()
    
    print(f"[+] Saved analysis files for {target_joint}: \n    -> {plot1_out}\n    -> {plot2_out}")

def main():
    if len(sys.argv) < 2:
        print("[!] Usage: python3 dynamic_plotter.py /path/to/bag.mcap [optional_single_joint]")
        sys.exit(1)

    bag_path = sys.argv[1]
    
    # All active arm joints present in the crop clearing simulation
    all_joints = ["rail_joint", "base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"]
    
    if len(sys.argv) > 2:
        target_joints = [sys.argv[2]]
    else:
        target_joints = all_joints

    for joint in target_joints:
        try:
            generate_joint_plots(bag_path, joint)
        except Exception as e:
            print(f"[!] Skipped or failed processing for joint {joint}: {e}")

if __name__ == "__main__":
    main()
