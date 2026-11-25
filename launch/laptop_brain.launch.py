"""Laptop 'Brain' Launch File for TrainerBot Rehab.

Starts on the Laptop:
  1. rosbridge_websocket (port 9090 for native Node-RED Dashboard)
  2. vision_pose_node (MediaPipe Pose perception + anonymized joint angles + preview)
  3. rehab_coach_node (exercise catalog, safety-first game loop, victory chime)
  4. gemini_live_node (gemini-3.8-live voice coaching & routine suggestion)
  5. speaker_node (plays chimes, TTS & Gemini voice on Laptop speakers by default
     until MAX98357A arrives; set enable_laptop_speaker:=false when Pi speaker is active)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_laptop_speaker_arg = DeclareLaunchArgument(
        "enable_laptop_speaker", default_value="true"
    )
    alsa_speaker_arg = DeclareLaunchArgument("alsa_speaker", default_value="default")

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090, "address": "0.0.0.0"}],
    )

    vision_pose_node = Node(
        package="trainerbot_rehab",
        executable="vision_pose_node",
        name="vision_pose_node",
        output="screen",
    )

    rehab_coach_node = Node(
        package="trainerbot_rehab",
        executable="rehab_coach_node",
        name="rehab_coach_node",
        output="screen",
    )

    gemini_live_node = Node(
        package="trainerbot_rehab",
        executable="gemini_live_node",
        name="gemini_live_node",
        output="screen",
    )

    laptop_speaker_node = Node(
        package="trainerbot_rehab",
        executable="speaker_node",
        name="laptop_speaker_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_laptop_speaker")),
        parameters=[{
            "alsa_device": LaunchConfiguration("alsa_speaker"),
            "speaker_mode": "alsa",
            "pcm_sample_rate": 24000,
        }],
    )

    return LaunchDescription([
        enable_laptop_speaker_arg,
        alsa_speaker_arg,
        rosbridge_node,
        vision_pose_node,
        rehab_coach_node,
        gemini_live_node,
        laptop_speaker_node,
    ])
