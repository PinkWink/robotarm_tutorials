from setuptools import find_packages, setup

package_name = 'robot_arm_tutorials'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/tutorial_demo.launch.py',
            'launch/servo_keyboard.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/sample_waypoints.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pw',
    maintainer_email='user@example.com',
    description='MoveIt2 Python tutorials for 6-DOF robot arm (ROS2 Jazzy)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ex01_joint_state_reader = robot_arm_tutorials.ex01_joint_state_reader:main',
            'ex02_named_pose = robot_arm_tutorials.ex02_named_pose:main',
            'ex03_joint_goal = robot_arm_tutorials.ex03_joint_goal:main',
            'ex04_pose_goal = robot_arm_tutorials.ex04_pose_goal:main',
            'ex05_cartesian_path = robot_arm_tutorials.ex05_cartesian_path:main',
            'ex06_gripper_control = robot_arm_tutorials.ex06_gripper_control:main',
            'ex07_pick_and_place = robot_arm_tutorials.ex07_pick_and_place:main',
            'ex08_constraints = robot_arm_tutorials.ex08_constraints:main',
            'ex09_collision_objects = robot_arm_tutorials.ex09_collision_objects:main',
            'ex10_multi_planner = robot_arm_tutorials.ex10_multi_planner:main',
            'ex11_keyboard_servo = robot_arm_tutorials.ex11_keyboard_servo:main',
            'ex12_waypoint_follow = robot_arm_tutorials.ex12_waypoint_follow:main',
            'ex13_circular_path = robot_arm_tutorials.ex13_circular_path:main',
            'ex14_circular_servo = robot_arm_tutorials.ex14_circular_servo:main',
            'ex15_keyboard_ik = robot_arm_tutorials.ex15_keyboard_ik:main',
        ],
    },
)
