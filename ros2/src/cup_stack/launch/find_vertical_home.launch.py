"""Launch find_vertical_home with MoveItPy parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("cup_stack"), "config", "moveit_py.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_x", default_value="nan",
                                  description="목표 x (기본: 현재 x 유지)"),
            DeclareLaunchArgument("target_y", default_value="nan",
                                  description="목표 y (기본: 현재 y 유지)"),
            DeclareLaunchArgument("target_z", default_value="nan",
                                  description="목표 z (기본: 현재 z 유지)"),
            Node(
                package="cup_stack",
                executable="find_vertical_home",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "target_x": LaunchConfiguration("target_x"),
                        "target_y": LaunchConfiguration("target_y"),
                        "target_z": LaunchConfiguration("target_z"),
                    },
                ],
            ),
        ]
    )
