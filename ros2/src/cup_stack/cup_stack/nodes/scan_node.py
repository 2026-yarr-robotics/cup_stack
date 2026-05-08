"""ROS 2 entry point for the scan task."""

import rclpy
from rclpy.node import Node

from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.scan import ScanTask


def main(args=None):
    rclpy.init(args=args)
    node = Node("scan_node")

    try:
        runtime = CupStackRuntime(node, "scan_moveit_py")
        task = ScanTask(runtime)
        task.try_execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
