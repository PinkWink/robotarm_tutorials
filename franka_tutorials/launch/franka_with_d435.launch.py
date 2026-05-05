"""Attach a D435 depth camera to the FR3 end-effector via a URDF wrapper
that does NOT modify the franka_description / franka_fr3_moveit_config
packages. The D435 link, optical frames, and Gazebo camera sensor are all
defined in franka_tutorials/urdf/fr3_gazebo_with_d435.urdf.xacro.

This launch:
  1) Includes franka_tutorials/launch/franka_gazebo_moveit.launch.py with
     urdf_file:=fr3_gazebo_with_d435.urdf.xacro so the robot loaded into
     Gazebo + MoveIt is the FR3 + D435 wrapper.
  2) Bridges the simulated RGB image from Gazebo to ROS via ros_gz_bridge:
        /d435/color/image_raw  (sensor_msgs/Image)
        /d435/color/camera_info (sensor_msgs/CameraInfo)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # World file with the Sensors system loaded so the camera actually
    # publishes on Gazebo transport. Ships under franka_tutorials/worlds.
    import os
    from ament_index_python.packages import get_package_share_directory
    world_path = os.path.join(
        get_package_share_directory('franka_tutorials'),
        'worlds', 'empty_with_sensors.sdf',
    )

    franka_gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_tutorials'),
                'launch', 'franka_gazebo_moveit.launch.py',
            ])
        ),
        launch_arguments={
            'urdf_file': 'fr3_gazebo_with_d435.urdf.xacro',
            'gz_args': f'-r {world_path}',
        }.items(),
    )

    # Bridge the camera image and info from Gazebo to ROS.
    # The Gazebo topic name comes from <topic>d435/color/image_raw</topic>
    # in fr3_gazebo_with_d435.urdf.xacro; Gazebo also publishes a sibling
    # camera_info under .../camera_info automatically.
    d435_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='d435_bridge',
        arguments=[
            '/d435/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d435/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    return LaunchDescription([
        franka_gazebo_moveit,
        d435_bridge,
    ])
