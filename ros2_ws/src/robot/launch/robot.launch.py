from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_gps = LaunchConfiguration("enable_gps")

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="false",
            description="Launch the RPLIDAR C1 node alongside the robot node.",
        ),
        DeclareLaunchArgument(
            "enable_gps",
            default_value="true",
            description="Launch the robot_gps node alongside the robot node.",
        ),
        Node(
            package="bridge",
            executable="bridge",
            name="bridge",
            output="screen",
        ),
        Node(
            package="rplidar_ros",
            executable="rplidar_c1_node",
            name="rplidar_node",
            output="screen",
            condition=IfCondition(enable_lidar),
            parameters=[{
                "channel_type": "serial",
                "serial_port": "/dev/rplidar",
                "serial_baudrate": 460800,
                "frame_id": "laser_frame",
                "angle_compensate": True,
                "scan_mode": "Standard",
            }],
        ),
        Node(
            package="robot",
            executable="robot",
            name="robot",
            output="screen",
        ),
        Node(
            package="sensors",
            executable="robot_gps",
            name="robot_gps",
            output="screen",
            condition=IfCondition(enable_gps),
        ),
    ])
