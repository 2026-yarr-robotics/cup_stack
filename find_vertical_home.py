#!/usr/bin/env python3
"""Compute joint angles for perfectly vertical EE at current XY position."""

import math
import threading

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters


def main():
    rclpy.init()
    node = Node("find_vertical_home")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot = MoveItPy(node_name="find_vertical_home_moveit")
    arm = robot.get_planning_component("manipulator")
    robot_model = robot.get_robot_model()

    monitor = robot.get_planning_scene_monitor()
    with monitor.read_only() as scene:
        transform = np.asarray(
            scene.current_state.get_global_link_transform("link_6"),
            dtype=float,
        )

    x, y, z = transform[0, 3], transform[1, 3], transform[2, 3]
    node.get_logger().info(f"Current EE position: ({x:.4f}, {y:.4f}, {z:.4f})")

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
        # fallback to OMPL
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
        node.get_logger().error("IK planning failed!")
    else:
        traj = plan_result.trajectory.joint_trajectory
        joint_names = traj.joint_names
        positions = traj.points[-1].positions

        # Sort to joint_1 ~ joint_6 order
        joint_map = dict(zip(joint_names, positions))
        ordered = [joint_map.get(f"joint_{i}", 0.0) for i in range(1, 7)]

        node.get_logger().info("=" * 50)
        node.get_logger().info("home_joints_deg for perfectly vertical DOWN:")
        node.get_logger().info("=" * 50)
        deg_vals = [math.degrees(r) for r in ordered]
        for i, d in enumerate(deg_vals, 1):
            node.get_logger().info(f"  joint_{i}: {d:.4f}")
        node.get_logger().info(f"\nTuple: {tuple(round(d, 4) for d in deg_vals)}")

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
