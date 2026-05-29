"""ROS 2 entry point for the 4-corner square scan task."""

import sys

import rclpy
from rclpy.node import Node

from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.scan_square import ScanSquareTask


def main(args=None):
    rclpy.init(args=args)
    node = Node("scan_square_node")

    ok = False
    try:
        # Pass the node namespace so MoveItPy binds its controller manager
        # to /<ns>/dsr_moveit_controller; without it the FollowJointTrajectory
        # action client lands at root, never connects, and every move ABORTs
        # while the task still reports success. See CupStackRuntime docstring.
        runtime = CupStackRuntime(
            node, "scan_square_moveit_py", moveit_namespace=node.get_namespace()
        )
        task = ScanSquareTask(runtime)
        ok = task.try_execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Non-zero exit on failure so the launching skill sees it.
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
