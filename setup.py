import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'trainerbot_rehab'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['*.task'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.json'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santi',
    maintainer_email='andres.martinezhernandez@unifr.ch',
    description='Universal human-framing rehabilitation TurtleBot3 package.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_pid_controller_node = trainerbot_rehab.robot.robot_pid_controller_node:main',
            'servo_node = trainerbot_rehab.robot.servo_node:main',
            'speaker_node = trainerbot_rehab.robot.speaker_node:main',
            'mic_node = trainerbot_rehab.robot.mic_node:main',
            'vision_pose_node = trainerbot_rehab.laptop.vision_pose_node:main',
            'rehab_coach_node = trainerbot_rehab.laptop.rehab_coach_node:main',
            'gemini_live_node = trainerbot_rehab.laptop.gemini_live_node:main',
        ],
    },
)
