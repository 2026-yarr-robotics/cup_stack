#!/usr/bin/env python3
"""Compute joint angles for perfectly vertical EE at current XY position.

기본: 현재 xyz 유지
--target-x 0.611  : 목표 x (기본: 현재 x 유지)
--target-y -0.237 : 목표 y (기본: 현재 y 유지)
--target-z 0.468  : 목표 z (기본: 현재 z 유지)
"""

import argparse
import math
import threading

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-x", type=float, default=None,
                        help="목표 x (기본: 현재 x 유지)")
    parser.add_argument("--target-y", type=float, default=None,
                        help="목표 y (기본: 현재 y 유지)")
    parser.add_argument("--target-z", type=float, default=None,
                        help="목표 z 높이 (기본: 현재 z 유지)")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args or None)
    node = Node("find_vertical_home")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot = MoveItPy(node_name="find_vertical_home_moveit")
    arm = robot.get_planning_component("manipulator")

    monitor = robot.get_planning_scene_monitor()
    with monitor.read_only() as scene:
        transform = np.asarray(
            scene.current_state.get_global_link_transform("link_6"),
            dtype=float,
        )

    x_cur, y_cur, z_cur = transform[0, 3], transform[1, 3], transform[2, 3]
    x = args.target_x if args.target_x is not None else x_cur
    y = args.target_y if args.target_y is not None else y_cur
    z = args.target_z if args.target_z is not None else z_cur
    node.get_logger().info(f"Current EE: ({x_cur:.4f}, {y_cur:.4f}, {z_cur:.4f})")
    node.get_logger().info(f"Target pose: ({x:.4f}, {y:.4f}, {z:.4f}) + DOWN_ORI")

    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 1.0  # DOWN_ORI
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0

    params = PlanRequestParameters(robot)
    params.planning_pipeline = "pilz_industrial_motion_planner"
    params.planner_id = "PTP"
    params.max_velocity_scaling_factor = 0.3
    params.max_acceleration_scaling_factor = 0.3
    params.planning_time = 5.0

    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose, pose_link="link_6")
    plan_result = arm.plan(parameters=params)

    if not plan_result:
        params2 = PlanRequestParameters(robot)
        params2.planning_pipeline = "ompl"
        params2.planner_id = "RRTConnect"
        params2.max_velocity_scaling_factor = 0.3
        params2.max_acceleration_scaling_factor = 0.3
        params2.planning_time = 5.0
        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=pose, pose_link="link_6")
        plan_result = arm.plan(parameters=params2)

    if not plan_result:
        node.get_logger().error(
            f"IK 실패: ({x:.4f}, {y:.4f}, {z:.4f}) + DOWN_ORI 도달 불가\n"
            "  → safe_z 값을 줄이거나 다른 XY 위치에서 시도하세요."
        )
    else:
        traj = plan_result.trajectory.joint_trajectory
        joint_names = traj.joint_names
        positions = traj.points[-1].positions

        joint_map = dict(zip(joint_names, positions))
        ordered = [joint_map.get(f"joint_{i}", 0.0) for i in range(1, 7)]
        deg_vals = [math.degrees(r) for r in ordered]

        node.get_logger().info("=" * 50)
        node.get_logger().info(f"HOME joints for DOWN_ORI at z={z:.3f}:")
        node.get_logger().info("=" * 50)
        for i, d in enumerate(deg_vals, 1):
            node.get_logger().info(f"  joint_{i}: {d:.4f}")
        node.get_logger().info(
            f"\n  config.py 에 붙여넣기:\n"
            f"  home_joints_deg: tuple[float, ...] = "
            f"{tuple(round(d, 4) for d in deg_vals)}"
        )

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
