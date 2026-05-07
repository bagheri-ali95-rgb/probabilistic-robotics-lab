from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    motion_model_arg = DeclareLaunchArgument(
        'motion_model',
        default_value='differential_drive',
        description='Motion model: simple or differential_drive'
    )
    motion_model = LaunchConfiguration('motion_model')

    kf_node = Node(
        package='prob_rob_lab_pkg',
        executable='kf_node',
        name='kf_node',
        parameters=[{
            'motion_model': motion_model,
            'q_scale': 1.0,
            'r_x': 0.01, 'r_y': 0.01, 'r_th': 0.001,
        }],
        output='screen',
    )

    ekf_node = Node(
        package='prob_rob_lab_pkg',
        executable='ekf_node',
        name='ekf_node',
        parameters=[{
            'motion_model': motion_model,
            'q_scale': 1.0,
            'r_x': 0.01, 'r_y': 0.01, 'r_th': 0.001,
        }],
        output='screen',
    )

    pf_node = Node(
        package='prob_rob_lab_pkg',
        executable='pf_node',
        name='pf_node',
        parameters=[{
            'motion_model': motion_model,
            'num_particles': 500,
            'resample_threshold': 0.5,
            'r_x': 0.01, 'r_y': 0.01, 'r_th': 0.001,
        }],
        output='screen',
    )

    return LaunchDescription([
        motion_model_arg,
        kf_node,
        ekf_node,
        pf_node,
    ])
