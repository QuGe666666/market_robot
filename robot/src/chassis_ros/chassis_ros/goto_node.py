#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘导航到标记点节点入口
"""
import rclpy
from chassis_ros.nodes import ChassisGotoNode


def main():
    rclpy.init()
    node = None
    try:
        node = ChassisGotoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
