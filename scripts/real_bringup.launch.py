"""
M1013 실제 로봇 bringup launch 파일

실행:
  ros2 launch scripts/real_bringup.launch.py host:=<ROBOT_IP> model:=m1013
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ns    = LaunchConfiguration("ns")
    host  = LaunchConfiguration("host")
    port  = LaunchConfiguration("port")
    model = LaunchConfiguration("model")

    return LaunchDescription([
        DeclareLaunchArgument("ns",    default_value="dsr01",     description="ROS2 namespace"),
        DeclareLaunchArgument("host",  default_value="192.168.1.100", description="실제 로봇 IP"),
        DeclareLaunchArgument("port",  default_value="12345",     description="DRCF port"),
        DeclareLaunchArgument("model", default_value="m1013",     description="robot model"),

        GroupAction([
            PushRosNamespace(ns),

            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[{"use_sim_time": False}],
                remappings=[("~/robot_description", "/robot_description")],
                output="screen",
            ),

            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
            ),

            # mode:=real 로 실제 로봇에 연결
            Node(
                package="dsr_hardware2",
                executable="dsr_hardware2_node",
                parameters=[{
                    "mode":  "real",
                    "host":  host,
                    "port":  port,
                    "model": model,
                }],
                output="screen",
            ),
        ]),
    ])
