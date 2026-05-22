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


# Bringup (dsr_bringup2_rviz.launch.py) places controller_manager and all
# controllers under /dsr01. MoveIt's moveit_simple_controller_manager creates
# its action client using the configured controller name as a *relative* name
# (`<controller>/follow_joint_trajectory`), so the only way to make it bind
# to /dsr01/dsr_moveit_controller/follow_joint_trajectory is to put the
# parent ROS node itself under /dsr01.  Then relative name resolution does
# the right thing without launch-level remappings (which don't propagate to
# the controller_manager's internal node).
#
# launch_ros' `namespace=` only applies `-r __ns:=/dsr01` to the primary
# rclpy node; MoveItPy spins up its own rclcpp nodes (the controller
# manager among them) which ignore that arg and land at root. Setting
# ROS_NAMESPACE in the process env forces every rclcpp node created in
# this process under /dsr01, so the simple_controller_manager's action
# client resolves to /dsr01/dsr_moveit_controller/follow_joint_trajectory
# and actually binds to the bringup_real action server.
ROBOT_NAMESPACE = "/dsr01"


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

    # dsr_bringup2_rviz.launch.py spawns only dsr_controller2 (Doosan SDK-direct,
    # claims no ros2_control command interfaces). MoveIt needs a standard
    # FollowJointTrajectory controller, so activate dsr_moveit_controller against
    # the namespaced controller_manager.  Doosan's own demo.launch.py spawns
    # both controllers side-by-side, so coexistence is the intended config.
    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dsr_moveit_controller",
            "--controller-manager",
            "/dsr01/controller_manager",
        ],
        output="screen",
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
            dsr_moveit_controller_spawner,
            Node(
                package="cup_stack",
                executable="skill_api_server",
                namespace=ROBOT_NAMESPACE,
                output="screen",
                additional_env={"ROS_NAMESPACE": ROBOT_NAMESPACE},
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
