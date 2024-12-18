import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    friend_pose = Node(
        package    = "edabot_communication",
        executable = "friend_pose"
    )

    navigation = Node(
        package    = "edabot_communication",
        executable = "navigate_to_goal_action_client"
    )

    return LaunchDescription([
        # friend_pose,
        navigation,
    ])