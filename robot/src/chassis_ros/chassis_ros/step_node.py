#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘步进控制节点入口
"""
import rclpy
from chassis_ros.nodes import ChassisStepNode


def main():
    rclpy.init()
    node = None
    try:
        node = ChassisStepNode()
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
