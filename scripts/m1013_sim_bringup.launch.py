"""
M1013 시뮬레이션 bringup launch 파일
Isaac Sim을 뷰어로 사용 (rviz 제외)

실행:
  ros2 launch scripts/m1013_sim_bringup.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ns        = LaunchConfiguration("ns")
    mode      = LaunchConfiguration("mode")
    host      = LaunchConfiguration("host")
    port      = LaunchConfiguration("port")
    model     = LaunchConfiguration("model")

    return LaunchDescription([
        # ── 인자 선언 ──
        DeclareLaunchArgument("ns",    default_value="dsr01",       description="ROS2 namespace"),
        DeclareLaunchArgument("mode",  default_value="virtual",     description="virtual or real"),
        DeclareLaunchArgument("host",  default_value="127.0.0.1",   description="DRCF IP"),
        DeclareLaunchArgument("port",  default_value="12345",       description="DRCF port"),
        DeclareLaunchArgument("model", default_value="m1013",       description="robot model"),

        GroupAction([
            PushRosNamespace(ns),

            # ── ros2_control_node ──
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[{
                    "robot_description": "",
                    "use_sim_time": False,
                }],
                remappings=[("~/robot_description", "/robot_description")],
                output="screen",
            ),

            # ── robot_state_publisher ──
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
            ),

            # ── dsr_hardware2 드라이버 ──
            Node(
                package="dsr_hardware2",
                executable="dsr_hardware2_node",
                parameters=[{
                    "mode":  mode,
                    "host":  host,
                    "port":  port,
                    "model": model,
                }],
                output="screen",
            ),
        ]),
    ])
