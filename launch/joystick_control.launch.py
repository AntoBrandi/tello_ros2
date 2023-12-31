import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        emulate_tty=True,
        parameters=[os.path.join(get_package_share_directory("tello_ros2"), "config", "joy_teleop.yaml")],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        emulate_tty=True,
        parameters=[os.path.join(get_package_share_directory("tello_ros2"), "config", "joy_config.yaml")]
    )
    
    tello_ros2 = Node(
        package="tello_ros2",
        executable="tello_ros2_node",
        name="tello_ros2_node",
        emulate_tty=True,
    )
    
    return LaunchDescription([
        joy_node,
        joy_teleop,
        tello_ros2,
    ])