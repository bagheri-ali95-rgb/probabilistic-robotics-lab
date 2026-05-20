"""
Experiment: Choose different Motion Models
==========================================
Runs KF, EKF, and PF simultaneously with BOTH the Simple (Euler) model and
the Differential-Drive (Exact-Arc) model so their trajectories can be compared
side-by-side in RViz2.

Output topics
-------------
  Simple model  : /kf_simple/pose  /ekf_simple/pose  /pf_simple/pose
  Diff-drive    : /kf_diff/pose    /ekf_diff/pose    /pf_diff/pose

Two evaluator instances write separate CSV files:
  ~/eval_simple_<timestamp>.csv
  ~/eval_diff_<timestamp>.csv
"""

from launch import LaunchDescription
from launch_ros.actions import Node

COMMON = dict(output='screen')

SIMPLE_PARAMS = dict(motion_model='simple',            q_scale=1.0,
                     r_x=0.01, r_y=0.01, r_th=0.001)
DIFF_PARAMS   = dict(motion_model='differential_drive', q_scale=1.0,
                     r_x=0.01, r_y=0.01, r_th=0.001)
PF_EXTRA      = dict(num_particles=500, resample_threshold=0.5)


def generate_launch_description():

    # ── Simple motion model ──────────────────────────────────────────
    kf_simple = Node(
        package='prob_rob_lab_pkg', executable='kf_node', name='kf_simple',
        parameters=[SIMPLE_PARAMS],
        remappings=[('/kf/pose', '/kf_simple/pose')],
        **COMMON)

    ekf_simple = Node(
        package='prob_rob_lab_pkg', executable='ekf_node', name='ekf_simple',
        parameters=[SIMPLE_PARAMS],
        remappings=[('/ekf/pose', '/ekf_simple/pose')],
        **COMMON)

    pf_simple = Node(
        package='prob_rob_lab_pkg', executable='pf_node', name='pf_simple',
        parameters=[{**SIMPLE_PARAMS, **PF_EXTRA}],
        remappings=[('/pf/pose', '/pf_simple/pose'),
                    ('/pf/particles', '/pf_simple/particles')],
        **COMMON)

    # ── Differential-Drive model ─────────────────────────────────────
    kf_diff = Node(
        package='prob_rob_lab_pkg', executable='kf_node', name='kf_diff',
        parameters=[DIFF_PARAMS],
        remappings=[('/kf/pose', '/kf_diff/pose')],
        **COMMON)

    ekf_diff = Node(
        package='prob_rob_lab_pkg', executable='ekf_node', name='ekf_diff',
        parameters=[DIFF_PARAMS],
        remappings=[('/ekf/pose', '/ekf_diff/pose')],
        **COMMON)

    pf_diff = Node(
        package='prob_rob_lab_pkg', executable='pf_node', name='pf_diff',
        parameters=[{**DIFF_PARAMS, **PF_EXTRA}],
        remappings=[('/pf/pose', '/pf_diff/pose'),
                    ('/pf/particles', '/pf_diff/particles')],
        **COMMON)

    # ── Landmark detection ───────────────────────────────────────────
    landmark = Node(
        package='turtlebot4_estimation', executable='landmark_node',
        name='landmark_node', **COMMON)

    # ── Evaluator – Simple ───────────────────────────────────────────
    eval_simple = Node(
        package='turtlebot4_estimation', executable='evaluator_node',
        name='evaluator_simple',
        parameters=[{
            'kf_topic':  '/kf_simple/pose',
            'ekf_topic': '/ekf_simple/pose',
            'pf_topic':  '/pf_simple/pose',
            'label':     'simple',
        }],
        **COMMON)

    # ── Evaluator – Diff-drive ───────────────────────────────────────
    eval_diff = Node(
        package='turtlebot4_estimation', executable='evaluator_node',
        name='evaluator_diff',
        parameters=[{
            'kf_topic':  '/kf_diff/pose',
            'ekf_topic': '/ekf_diff/pose',
            'pf_topic':  '/pf_diff/pose',
            'label':     'diff',
        }],
        **COMMON)

    return LaunchDescription([
        kf_simple, ekf_simple, pf_simple,
        kf_diff,   ekf_diff,   pf_diff,
        landmark,
        eval_simple, eval_diff,
    ])
