from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    talker_node = Node(
        package='prob_rob_lab_pkg',
        executable='talker',
        name='talker'
    )

    listener_node = Node(
        package='prob_rob_lab_pkg',
        executable='listener',
        name='listener'
    )

    return LaunchDescription([
        talker_node,
        listener_node
    ])