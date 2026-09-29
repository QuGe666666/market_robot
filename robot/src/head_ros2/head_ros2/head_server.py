#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
头部舵机ROS2服务器节点
提供舵机控制服务
"""

import rclpy
from rclpy.node import Node
from head_ros2_interfaces.srv import Connect, Disconnect, IsOnline, Initialize, Rotate, ListPorts, ReadPosition, ReadPositions
import sys
import os

# 添加API路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'api'))
from servo_api import HeadControlSDK


class HeadServer(Node):
    """头部舵机服务器节点"""

    def __init__(self):
        super().__init__('head_server')

        # 初始化SDK
        self.sdk = None

        # 创建服务
        self.connect_srv = self.create_service(Connect, 'head/connect', self.connect_callback)
        self.disconnect_srv = self.create_service(Disconnect, 'head/disconnect', self.disconnect_callback)
        self.is_online_srv = self.create_service(IsOnline, 'head/is_online', self.is_online_callback)
        self.initialize_srv = self.create_service(Initialize, 'head/initialize', self.initialize_callback)
        self.rotate_srv = self.create_service(Rotate, 'head/rotate', self.rotate_callback)
        self.list_ports_srv = self.create_service(ListPorts, 'head/list_ports', self.list_ports_callback)
        self.read_position_srv = self.create_service(ReadPosition, 'head/read_position', self.read_position_callback)
        self.read_positions_srv = self.create_service(ReadPositions, 'head/read_positions', self.read_positions_callback)

        self.get_logger().info('头部舵机服务器已启动')

    def connect_callback(self, request, response):
        """连接服务回调"""
        try:
            port = request.port if request.port else '/dev/ttyUSB0'
            baudrate = request.baudrate if request.baudrate else 9600

            self.get_logger().info(f'连接请求: {port}, {baudrate}')

            # 创建SDK实例
            if self.sdk is None:
                self.sdk = HeadControlSDK(port=port, baudrate=baudrate)
            else:
                self.sdk.disconnect()
                self.sdk.port = port
                self.sdk.baudrate = baudrate

            # 尝试连接
            if self.sdk.connect():
                response.success = True
                response.message = f'成功连接到 {port}'
                self.get_logger().info('连接成功')
            else:
                response.success = False
                response.message = f'连接失败: {port}'
                self.get_logger().error('连接失败')

        except Exception as e:
            response.success = False
            response.message = f'连接异常: {str(e)}'
            self.get_logger().error(f'连接异常: {e}')

        return response

    def disconnect_callback(self, request, response):
        """断开连接服务回调"""
        try:
            if self.sdk:
                self.sdk.disconnect()
                self.sdk = None
                response.success = True
                response.message = '已断开连接'
                self.get_logger().info('断开连接')
            else:
                response.success = False
                response.message = '未连接'
                self.get_logger().warning('断开连接失败: 未连接')

        except Exception as e:
            response.success = False
            response.message = f'断开连接异常: {str(e)}'
            self.get_logger().error(f'断开连接异常: {e}')

        return response

    def is_online_callback(self, request, response):
        """检查在线状态服务回调"""
        try:
            if self.sdk and self.sdk.is_online():
                response.online = True
                response.message = '设备在线'
                self.get_logger().info('设备在线')
            else:
                response.online = False
                response.message = '设备离线'
                self.get_logger().warning('设备离线')

        except Exception as e:
            response.online = False
            response.message = f'检查在线状态异常: {str(e)}'
            self.get_logger().error(f'检查在线状态异常: {e}')

        return response

    def initialize_callback(self, request, response):
        """初始化服务回调"""
        try:
            if not self.sdk or not self.sdk.is_online():
                response.success = False
                response.message = '设备未连接'
                self.get_logger().error('初始化失败: 设备未连接')
                return response

            # 获取舵机ID列表
            servo_ids = list(request.servo_ids) if request.servo_ids else [1, 2]

            self.get_logger().info(f'初始化舵机: {servo_ids}')

            if self.sdk.initialize(servo_ids):
                response.success = True
                response.message = f'初始化成功: {servo_ids}'
                self.get_logger().info('初始化成功')
            else:
                response.success = False
                response.message = '初始化失败'
                self.get_logger().error('初始化失败')

        except Exception as e:
            response.success = False
            response.message = f'初始化异常: {str(e)}'
            self.get_logger().error(f'初始化异常: {e}')

        return response

    def rotate_callback(self, request, response):
        """旋转服务回调"""
        try:
            servo_id = request.servo_id
            angle = int(request.angle)

            self.get_logger().info(f'旋转: ID={servo_id}, 角度={angle}')

            if not self.sdk or not self.sdk.is_online():
                response.success = False
                response.message = '设备未连接'
                self.get_logger().error('旋转失败: 设备未连接')
                return response

            if self.sdk.rotate(servo_id, angle):
                response.success = True
                response.message = f'舵机{servo_id}旋转到{angle}'
                self.get_logger().info('旋转成功')
            else:
                response.success = False
                response.message = '旋转失败'
                self.get_logger().error('旋转失败')

        except Exception as e:
            response.success = False
            response.message = f'旋转异常: {str(e)}'
            self.get_logger().error(f'旋转异常: {e}')

        return response

    def list_ports_callback(self, request, response):
        """列出串口服务回调"""
        try:
            ports = HeadControlSDK.list_ports()
            response.ports = ports
            response.message = f'找到 {len(ports)} 个串口'
            self.get_logger().info(f'找到串口: {ports}')

        except Exception as e:
            response.ports = []
            response.message = f'列出串口异常: {str(e)}'
            self.get_logger().error(f'列出串口异常: {e}')

        return response

    def read_position_callback(self, request, response):
        """读取单个舵机位置服务回调"""
        try:
            servo_id = request.servo_id

            self.get_logger().info(f'读取舵机{servo_id}位置')

            if not self.sdk or not self.sdk.is_online():
                response.success = False
                response.position = -1
                response.message = '设备未连接'
                self.get_logger().error('读取位置失败: 设备未连接')
                return response

            position = self.sdk.read_position(servo_id)

            if position is not None:
                response.success = True
                response.position = position
                response.message = f'舵机{servo_id}位置: {position}'
                self.get_logger().info(f'读取位置成功: {position}')
            else:
                response.success = False
                response.position = -1
                response.message = '读取位置失败'
                self.get_logger().error('读取位置失败')

        except Exception as e:
            response.success = False
            response.position = -1
            response.message = f'读取位置异常: {str(e)}'
            self.get_logger().error(f'读取位置异常: {e}')

        return response

    def read_positions_callback(self, request, response):
        """读取多个舵机位置服务回调"""
        try:
            servo_ids = list(request.servo_ids) if request.servo_ids else [1, 2]

            self.get_logger().info(f'读取舵机{servo_ids}位置')

            if not self.sdk or not self.sdk.is_online():
                response.success = False
                response.positions = []
                response.message = '设备未连接'
                self.get_logger().error('读取位置失败: 设备未连接')
                return response

            positions_dict = self.sdk.read_positions(servo_ids)

            if positions_dict:
                response.success = True
                # 按照servo_ids的顺序返回位置
                response.positions = [positions_dict.get(sid, -1) for sid in servo_ids]
                response.message = f'位置读取成功: {positions_dict}'
                self.get_logger().info(f'批量读取位置成功: {positions_dict}')
            else:
                response.success = False
                response.positions = []
                response.message = '读取位置失败'
                self.get_logger().error('批量读取位置失败')

        except Exception as e:
            response.success = False
            response.positions = []
            response.message = f'读取位置异常: {str(e)}'
            self.get_logger().error(f'读取位置异常: {e}')

        return response


def main(args=None):
    rclpy.init(args=args)

    server = HeadServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        if server.sdk:
            server.sdk.disconnect()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
