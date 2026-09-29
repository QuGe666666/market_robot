#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from head_ros2.srv import Connect, Disconnect, IsOnline, Initialize, Rotate, ListPorts
import time


class HeadTestClient(Node):
    def __init__(self):
        super().__init__('head_test_client')

        # 创建客户端
        self.connect_client = self.create_client(Connect, 'head/connect')
        self.disconnect_client = self.create_client(Disconnect, 'head/disconnect')
        self.is_online_client = self.create_client(IsOnline, 'head/is_online')
        self.initialize_client = self.create_client(Initialize, 'head/initialize')
        self.rotate_client = self.create_client(Rotate, 'head/rotate')
        self.list_ports_client = self.create_client(ListPorts, 'head/list_ports')

    def wait_for_services(self):
        """等待所有服务可用"""
        self.get_logger().info('等待服务可用...')
        clients = [
            self.connect_client,
            self.disconnect_client,
            self.is_online_client,
            self.initialize_client,
            self.rotate_client,
            self.list_ports_client
        ]
        for client in clients:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'服务 {client.srv_name} 不可用')
                return False
        self.get_logger().info('所有服务已就绪')
        return True

    def list_ports(self):
        """列出可用串口"""
        self.get_logger().info('=== 列出可用串口 ===')
        request = ListPorts.Request()
        future = self.list_ports_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'找到端口: {response.ports}')
        self.get_logger().info(f'消息: {response.message}')
        return response.ports

    def connect(self, port='/dev/ttyUSB0', baudrate=9600):
        """连接舵机"""
        self.get_logger().info(f'=== 连接到 {port} (波特率: {baudrate}) ===')
        request = Connect.Request()
        request.port = port
        request.baudrate = baudrate
        future = self.connect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'成功: {response.success}, 消息: {response.message}')
        return response.success

    def is_online(self):
        """检查是否在线"""
        self.get_logger().info('=== 检查在线状态 ===')
        request = IsOnline.Request()
        future = self.is_online_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'在线: {response.online}, 消息: {response.message}')
        return response.online

    def initialize(self, servo_ids=None):
        """初始化舵机"""
        self.get_logger().info(f'=== 初始化舵机 {servo_ids if servo_ids else "全部"} ===')
        request = Initialize.Request()
        if servo_ids:
            request.servo_ids = servo_ids
        future = self.initialize_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'成功: {response.success}, 消息: {response.message}')
        return response.success

    def rotate(self, servo_id, angle):
        """控制舵机旋转"""
        self.get_logger().info(f'=== 舵机 {servo_id} 旋转到位置 {angle} ===')
        request = Rotate.Request()
        request.servo_id = servo_id
        request.angle = angle
        future = self.rotate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'成功: {response.success}, 消息: {response.message}')
        return response.success

    def disconnect(self):
        """断开连接"""
        self.get_logger().info('=== 断开连接 ===')
        request = Disconnect.Request()
        future = self.disconnect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'成功: {response.success}, 消息: {response.message}')
        return response.success

    def run_test(self, port='/dev/ttyUSB0', baudrate=9600):
        """运行完整测试流程"""
        try:
            # 1. 列出端口
            ports = self.list_ports()

            if not ports:
                self.get_logger().warning('未找到可用串口，使用默认端口')

            # 2. 连接
            if not self.connect(port, baudrate):
                self.get_logger().error('连接失败，测试终止')
                return

            time.sleep(0.5)

            # 3. 检查在线状态
            if not self.is_online():
                self.get_logger().error('设备不在线，测试终止')
                self.disconnect()
                return

            # 4. 初始化
            servo_ids = [1, 2]  # 根据实际情况修改
            if not self.initialize(servo_ids):
                self.get_logger().warning('初始化失败，继续测试...')

            time.sleep(0.5)

            # 5. 测试旋转
            self.get_logger().info('开始旋转测试...')

            # 舵机1 俯仰测试 (500-600范围)
            for pos in [500, 550, 600, 550, 500]:
                self.rotate(1, float(pos))
                time.sleep(0.5)

            # 舵机2 偏航测试 (500-600范围)
            for pos in [500, 550, 600, 550, 500]:
                self.rotate(2, float(pos))
                time.sleep(0.5)

            self.get_logger().info('测试完成！')

        except Exception as e:
            self.get_logger().error(f'测试出错: {e}')
        finally:
            # 6. 断开连接
            self.disconnect()


def main(args=None):
    rclpy.init(args=args)

    client = HeadTestClient()

    if not client.wait_for_services():
        client.destroy_node()
        rclpy.shutdown()
        return

    # 运行测试（可修改串口和波特率）
    client.run_test(port='/dev/ttyUSB0', baudrate=9600)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
