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

def extract_joint_data(bag_path, target_joint):
    """
    Parses changing commands and actual joint states from the MCAP/db3 bag file,
    tracking multiple consecutive target goals mapped onto Simulation Time.
    """
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

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    cmd_topic = "/arm_controller/joint_trajectory"
    fb_topic = "/joint_states"

    Cmd_Class = get_message(type_map[cmd_topic])
    Fb_Class = get_message(type_map[fb_topic])

    raw_cmd_t, raw_cmd_p = [], []
    raw_fb_t, raw_fb_p = [], []

    while reader.has_next():
        topic, data, ts = reader.read_next()
        
        if topic == cmd_topic:
            msg = deserialize_message(data, Cmd_Class)
            if target_joint in msg.joint_names:
                j_idx = msg.joint_names.index(target_joint)
                if hasattr(msg, 'header') and msg.header.stamp.sec != 0:
                    base_sim_time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
                else:
                    base_sim_time = ts / 1e9
                
                for pt in msg.points:
                    t_offset = pt.time_from_start.sec + (pt.time_from_start.nanosec / 1e9)
                    raw_cmd_t.append(base_sim_time + t_offset)
                    raw_cmd_p.append(pt.positions[j_idx])

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
        raise ValueError(f"Missing data vectors for axis '{target_joint}'")

    cmd_t, cmd_p = np.array(raw_cmd_t), np.array(raw_cmd_p)
    fb_t, fb_p = np.array(raw_fb_t), np.array(raw_fb_p)

    idx_c = np.argsort(cmd_t)
    cmd_t, cmd_p = cmd_t[idx_c], cmd_p[idx_c]
    
    idx_f = np.argsort(fb_t)
    fb_t, fb_p = fb_t[idx_f], fb_p[idx_f]

    t_start = fb_t[0]
    cmd_t -= t_start
    fb_t -= t_start

    cmd_p_aligned = np.interp(fb_t, cmd_t, cmd_p)

    dt = np.diff(fb_t, prepend=-0.005)
    dt[dt <= 0] = 0.005

    vel_cmd = np.gradient(cmd_p_aligned, dt)
    acc_cmd = np.gradient(vel_cmd, dt)
    jerk_cmd = np.gradient(acc_cmd, dt)

    return fb_t, cmd_p_aligned, vel_cmd, acc_cmd, jerk_cmd, fb_p

def main():
    if len(sys.argv) < 2:
        print("[!] Usage: python3 plot_robot_data.py /path/to/bag.mcap")
        sys.exit(1)

    # Expand '~' to your full home directory path and create the folder if needed
    output_dir = os.path.expanduser("~/ros2_ws/src/harvest_ros2")
    os.makedirs(output_dir, exist_ok=True)

    bag_path = sys.argv[1]
    all_joints = ["rail_joint", "base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"]
    
    # Grid layouts: 3 rows, 2 columns for the 6 joints
    fig_kin, axs_kin = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    fig_pid, axs_pid = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    
    fig_kin.suptitle("Unified S-Curve Command Profiles Across All Joints (Sim Time)", fontsize=16, fontweight='bold')
    fig_pid.suptitle("Unified Closed-Loop PID Performance & Tracking Errors (Sim Time)", fontsize=16, fontweight='bold')
    
    axs_kin = axs_kin.flatten()
    axs_pid = axs_pid.flatten()

    for idx, joint in enumerate(all_joints):
        print(f"[*] Extracting simulation data for dashboard axis: {joint}...")
        try:
            t, p_cmd, v_cmd, a_cmd, j_cmd, p_meas = extract_joint_data(bag_path, joint)
            error = p_cmd - p_meas
            
            # DASHBOARD 1: KINEMATICS COMMAND PROFILE
            ax_k = axs_kin[idx]
            ax_k.set_title(f"Axis: {joint}", fontweight='bold', fontsize=12)
            ax_k.plot(t, p_cmd, 'b-', label='Pos [rad/m]', linewidth=1.5)
            ax_k.plot(t, v_cmd, 'g-', label='Vel [rad/s]', linewidth=1.0, alpha=0.8)
            ax_k.plot(t, a_cmd, 'r-', label='Acc [rad/s²]', linewidth=1.0, alpha=0.7)
            ax_k.grid(True, linestyle='--', alpha=0.5)
            if idx >= 4:
                ax_k.set_xlabel("Simulation Time [s]", fontsize=10)
            if idx % 2 == 0:
                ax_k.set_ylabel("Kinematic State Units", fontsize=10)
            ax_k.legend(loc="upper right", fontsize=8)

            # DASHBOARD 2: PID PERFORMANCE & TRACKING ERROR 
            ax_p = axs_pid[idx]
            ax_p.set_title(f"Axis: {joint}", fontweight='bold', fontsize=12)
            ax_p.plot(t, p_cmd, 'b-', label='Command (IK)', linewidth=1.8)
            ax_p.plot(t, p_meas, 'r--', label='Measured (Gazebo)', linewidth=1.5)
            
            # Instantiating secondary twin y-axis to display error profile clearly
            ax_p_err = ax_p.twinx()
            ax_p_err.plot(t, error, 'k-', label='Error (e)', linewidth=1.0, alpha=0.5)
            ax_p_err.axhline(0, color='gray', linestyle=':', alpha=0.5)
            
            ax_p.grid(True, linestyle='--', alpha=0.5)
            if idx >= 4:
                ax_p.set_xlabel("Simulation Time [s]", fontsize=10)
            if idx % 2 == 0:
                ax_p.set_ylabel("Position [rad or m]", fontsize=10)
            if idx % 2 != 0:
                ax_p_err.set_ylabel("Error Delta [rad or m]", color='dimgray', fontsize=10)
                
            # Merging legends across twin handles safely
            lines, labels = ax_p.get_legend_handles_labels()
            lines2, labels2 = ax_p_err.get_legend_handles_labels()
            ax_p.legend(lines + lines2, labels + labels2, loc="lower right", fontsize=8)

        except Exception as e:
            print(f"[!] Skipped or failed processing for joint {joint}: {e}")

    # Layout compilation adjustments
    fig_kin.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig_pid.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    fig_kin.savefig(os.path.join(output_dir, "unified_kinematic_commands.png"), dpi=300)
    fig_pid.savefig(os.path.join(output_dir, "unified_pid_performance.png"), dpi=300)
    
    print("\n[+] Unified layout rendering complete:")
    print("    -> saved: unified_kinematic_commands.png")
    print("    -> saved: unified_pid_performance.png")

if __name__ == "__main__":
    main()
