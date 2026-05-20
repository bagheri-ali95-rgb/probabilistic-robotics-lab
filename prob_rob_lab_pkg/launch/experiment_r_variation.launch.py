"""
Experiment: Measurement Noise (R) Variation
============================================
Runs three EKF nodes in parallel with different R (measurement noise) scales
to show the effect of sensor trust on estimation.

  R × 0.1  → high sensor trust   → /ekf_r_low/pose
  R × 1.0  → balanced            → /ekf_r_mid/pose
  R × 10.0 → low sensor trust    → /ekf_r_high/pose
"""

from launch import LaunchDescription
from launch_ros.actions import Node

BASE = dict(motion_model='differential_drive', q_scale=1.0)
COMMON = dict(output='screen')


def generate_launch_description():

    ekf_r_low = Node(
        package='prob_rob_lab_pkg', executable='ekf_node', name='ekf_r_low',
        parameters=[{**BASE, 'r_x': 0.001, 'r_y': 0.001, 'r_th': 0.0001}],
        remappings=[('/ekf/pose', '/ekf_r_low/pose')],
        **COMMON)

    ekf_r_mid = Node(
        package='prob_rob_lab_pkg', executable='ekf_node', name='ekf_r_mid',
        parameters=[{**BASE, 'r_x': 0.01,  'r_y': 0.01,  'r_th': 0.001}],
        remappings=[('/ekf/pose', '/ekf_r_mid/pose')],
        **COMMON)

    ekf_r_high = Node(
        package='prob_rob_lab_pkg', executable='ekf_node', name='ekf_r_high',
        parameters=[{**BASE, 'r_x': 0.1,   'r_y': 0.1,   'r_th': 0.01}],
        remappings=[('/ekf/pose', '/ekf_r_high/pose')],
        **COMMON)

    landmark = Node(
        package='turtlebot4_estimation', executable='landmark_node',
        name='landmark_node', **COMMON)

    return LaunchDescription([ekf_r_low, ekf_r_mid, ekf_r_high, landmark])
