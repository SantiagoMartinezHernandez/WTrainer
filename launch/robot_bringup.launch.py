"""TurtleBot3 Raspberry Pi Bringup Launch File (Hardware I/O + On-Robot PID).

Starts directly on the TurtleBot3 Pi:
  1. turtlebot3_bringup/robot.launch.py (OpenCR wheels + LiDAR)
  2. usb_cam camera publisher (/image_raw/compressed)
  3. robot_pid_controller_node (local PID follower, LiDAR 1m safety, fail-safe watchdog)
  4. smooth servo_node (/trainerbot/camera_tilt)
  5. speaker_node (driverless ALSA 3.5mm/PAM8403 audio for chimes, TTS & Gemini PCM)
  6. mic_node (INMP441 I2S 16 kHz PCM capture)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    servo_type_arg = DeclareLaunchArgument("servo_type", default_value="pwm")
    gpio_pin_arg = DeclareLaunchArgument("gpio_pin", default_value="12")
    alsa_mic_arg = DeclareLaunchArgument("alsa_mic", default_value="plughw:0,0")
    alsa_speaker_arg = DeclareLaunchArgument("alsa_speaker", default_value="default")
    enable_speaker_arg = DeclareLaunchArgument("enable_speaker", default_value="false")
    speaker_mode_arg = DeclareLaunchArgument("speaker_mode", default_value="alsa")
    speaker_pwm_pin_arg = DeclareLaunchArgument("speaker_pwm_pin", default_value="13")
    start_following_arg = DeclareLaunchArgument("start_following", default_value="false")
    target_distance_arg = DeclareLaunchArgument("target_distance", default_value="1.05")

    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_bringup"),
                "launch",
                "robot.launch.py",
            )
        )
    )

    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[{
            "pixel_format": "mjpeg2rgb",
            "image_width": 640,
            "image_height": 480,
            "framerate": 15.0,
        }],
    )

    robot_pid_controller_node = Node(
        package="trainerbot_rehab",
        executable="robot_pid_controller_node",
        name="robot_pid_controller_node",
        output="screen",
        parameters=[{
            "start_following": LaunchConfiguration("start_following"),
            "target_distance": LaunchConfiguration("target_distance"),
        }],
    )

    servo_node = Node(
        package="trainerbot_rehab",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[{
            "servo_type": LaunchConfiguration("servo_type"),
            "gpio_pin": LaunchConfiguration("gpio_pin"),
            "max_deg_per_sec": 45.0,
        }],
    )

    speaker_node = Node(
        package="trainerbot_rehab",
        executable="speaker_node",
        name="speaker_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_speaker")),
        parameters=[{
            "alsa_device": LaunchConfiguration("alsa_speaker"),
            "speaker_mode": LaunchConfiguration("speaker_mode"),
            "pwm_gpio_pin": LaunchConfiguration("speaker_pwm_pin"),
            "pcm_sample_rate": 24000,
        }],
    )

    mic_node = Node(
        package="trainerbot_rehab",
        executable="mic_node",
        name="mic_node",
        output="screen",
        parameters=[{
            "alsa_device": LaunchConfiguration("alsa_mic"),
            "sample_rate": 16000,
        }],
    )

    return LaunchDescription([
        servo_type_arg,
        gpio_pin_arg,
        alsa_mic_arg,
        alsa_speaker_arg,
        enable_speaker_arg,
        speaker_mode_arg,
        speaker_pwm_pin_arg,
        start_following_arg,
        target_distance_arg,
        turtlebot_launch,
        usb_cam_node,
        robot_pid_controller_node,
        servo_node,
        speaker_node,
        mic_node,
    ])
