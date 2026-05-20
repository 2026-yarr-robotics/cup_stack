"""Launch the CupStack Skill API server with MoveItPy params.

Launch arguments:
  - ``host`` (default ``0.0.0.0``), ``port`` (default ``8765``)
  - ``move_home`` (default ``false``): move the arm to HOME before
    the server starts accepting requests.
  - ``cup_grip_z_offset`` (default ``0.10`` m): vertical distance
    from cup-top centre to the gripper grip point — calibrate to
    the actual cup geometry.
  - ``pick_z_base`` (default ``0.323`` m): gripper Z when picking
    the top of a 1-cup nested source stack. Used by /skill/pick when
    the caller supplies ``nested_count`` instead of an explicit Z.
  - ``nest_inc`` (default ``0.012`` m): rise per additional nested
    cup. ``pick_z = pick_z_base + (nested_count - 1) * nest_inc``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Build the launch description for the skill API server."""

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
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("move_home", default_value="false"),
            DeclareLaunchArgument(
                "cup_grip_z_offset", default_value="0.10"
            ),
            DeclareLaunchArgument("pick_z_base", default_value="0.323"),
            DeclareLaunchArgument("nest_inc", default_value="0.012"),
            Node(
                package="cup_stack",
                executable="skill_api_server",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "move_home": LaunchConfiguration("move_home"),
                        "cup_grip_z_offset": LaunchConfiguration(
                            "cup_grip_z_offset"
                        ),
                        "pick_z_base": LaunchConfiguration("pick_z_base"),
                        "nest_inc": LaunchConfiguration("nest_inc"),
                    },
                ],
            ),
        ]
    )
