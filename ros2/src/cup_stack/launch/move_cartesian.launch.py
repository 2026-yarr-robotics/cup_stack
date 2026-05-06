"""Launch the move_cartesian node with MoveItPy parameters."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
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

    # Static transform: link_6 → camera_link (hand-in-eye calibration, T_gripper2camera^{-1})
    # Translation in meters, quaternion in xyzw order
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.038400", "0.058850", "-0.005715",
            "0.002446", "-0.003002", "0.999861", "0.016209",
            "link_6", "camera_link",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            Node(
                package="cup_stack",
                executable="move_cartesian",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                ],
            ),
            camera_tf,
        ]
    )
