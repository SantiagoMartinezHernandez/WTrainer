"""All-on-Bot Standalone Launch File for 4 GB Raspberry Pi TurtleBot3.

Runs the ENTIRE WTrainer stack directly on the Raspberry Pi (no Laptop ROS nodes
required except the user's browser opening Node-RED on http://<ROBOT_IP>:1880/dashboard/page1):
  1. turtlebot3_bringup/robot.launch.py (OpenCR wheels + LiDAR)
  2. usb_cam_node_exe (/image_raw/compressed)
  3. robot_pid_controller_node (20 Hz PID, bidirectional LiDAR distance, Safety Grid)
  4. servo_node (50 Hz smooth PWM/Dynamixel tilt + startup sweep)
  5. mic_node (INMP441 I2S PCM capture + software gain)
  6. speaker_node (ALSA / 3.5 mm / MAX98357A speaker on Pi)
  7. vision_pose_node (MediaPipe Pose on Pi)
  8. rehab_coach_node (biomechanical anti-cheat state machine)
  9. gemini_live_node (online Gemini Live or offline Vosk STT fallback)
 10. rosbridge_websocket (port 9090) + rosapi_node (for Pi-hosted Node-RED)
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
    alsa_mic_arg = DeclareLaunchArgument("alsa_mic", default_value="auto")
    mic_start_active_arg = DeclareLaunchArgument("mic_start_active", default_value="false")
    alsa_speaker_arg = DeclareLaunchArgument("alsa_speaker", default_value="default")
    enable_speaker_arg = DeclareLaunchArgument("enable_speaker", default_value="true")
    speaker_mode_arg = DeclareLaunchArgument("speaker_mode", default_value="alsa")
    speaker_pwm_pin_arg = DeclareLaunchArgument("speaker_pwm_pin", default_value="13")
    start_following_arg = DeclareLaunchArgument("start_following", default_value="false")
    target_distance_arg = DeclareLaunchArgument("target_distance", default_value="1.40")
    camera_reversed_arg = DeclareLaunchArgument("camera_reversed", default_value="true")

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
            "camera_reversed": LaunchConfiguration("camera_reversed"),
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
            "max_deg_per_sec": 30.0,
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
            "start_active": LaunchConfiguration("mic_start_active"),
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

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090, "address": "0.0.0.0"}],
    )

    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    return LaunchDescription([
        servo_type_arg,
        gpio_pin_arg,
        alsa_mic_arg,
        mic_start_active_arg,
        alsa_speaker_arg,
        enable_speaker_arg,
        speaker_mode_arg,
        speaker_pwm_pin_arg,
        start_following_arg,
        target_distance_arg,
        camera_reversed_arg,
        turtlebot_launch,
        usb_cam_node,
        robot_pid_controller_node,
        servo_node,
        mic_node,
        speaker_node,
        vision_pose_node,
        rehab_coach_node,
        gemini_live_node,
        rosbridge_node,
        rosapi_node,
    ])
