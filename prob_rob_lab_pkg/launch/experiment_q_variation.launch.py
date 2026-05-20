"""
Experiment: Process Noise (Q) Variation
========================================
Runs three EKF nodes in parallel with different q_scale values so the effect
of process noise confidence can be compared in real time.

  q_scale=0.1  → high model trust   → /ekf_q_low/pose
  q_scale=1.0  → balanced           → /ekf_q_mid/pose
  q_scale=10.0 → low model trust    → /ekf_q_high/pose
"""

from launch import LaunchDescription
from launch_ros.actions import Node

BASE = dict(motion_model='differential_drive', r_x=0.01, r_y=0.01, r_th=0.001)
COMMON = dict(output='screen')


def generate_launch_description():

    ekf_low = Node(
        package='prob_rob_lab_pkg', executable='ekf_node', name='ekf_q_low',
        parameters=[{**BASE, 'q_scale': 0.1}],
        remappings=[('/ekf/pose', '/ekf_q_low/pose')],
        **COMMON)

    ekf_mid = Node(
        package='prob_rob_lab_pkg', executable='ekf_node', name='ekf_q_mid',
        parameters=[{**BASE, 'q_scale': 1.0}],
        remappings=[('/ekf/pose', '/ekf_q_mid/pose')],
        **COMMON)

    ekf_high = Node(
        package='prob_rob_lab_pkg', executable='ekf_node', name='ekf_q_high',
        parameters=[{**BASE, 'q_scale': 10.0}],
        remappings=[('/ekf/pose', '/ekf_q_high/pose')],
        **COMMON)

    landmark = Node(
        package='turtlebot4_estimation', executable='landmark_node',
        name='landmark_node', **COMMON)

    return LaunchDescription([ekf_low, ekf_mid, ekf_high, landmark])
