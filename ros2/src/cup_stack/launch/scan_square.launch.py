"""Launch the 4-corner square scan task with MoveItPy parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Run scan_square under the robot namespace so its MoveItPy controller
    # manager binds to /<ns>/dsr_moveit_controller. Must match the bringup
    # namespace (default dsr01); otherwise every move ABORTs with "Action
    # client not connected to action server".
    namespace = LaunchConfiguration("name")

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
            DeclareLaunchArgument(
                "name",
                default_value="dsr01",
                description="Robot namespace; must match the bringup",
            ),
            Node(
                package="cup_stack",
                executable="scan_square",
                namespace=namespace,
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                ],
            ),
        ]
    )
