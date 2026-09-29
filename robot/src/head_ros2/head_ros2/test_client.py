#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
头部舵机ROS2测试客户端
用于测试ROS2服务器功能
"""

import rclpy
from rclpy.node import Node
from head_ros2_interfaces.srv import Connect, Disconnect, IsOnline, Initialize, Rotate, ListPorts, ReadPosition, ReadPositions
import time


class HeadTestClient(Node):
    """头部舵机测试客户端"""

    def __init__(self):
        super().__init__('head_test_client')

        # 创建服务客户端
        self.connect_client = self.create_client(Connect, 'head/connect')
        self.disconnect_client = self.create_client(Disconnect, 'head/disconnect')
        self.is_online_client = self.create_client(IsOnline, 'head/is_online')
        self.initialize_client = self.create_client(Initialize, 'head/initialize')
        self.rotate_client = self.create_client(Rotate, 'head/rotate')
        self.list_ports_client = self.create_client(ListPorts, 'head/list_ports')
        self.read_position_client = self.create_client(ReadPosition, 'head/read_position')
        self.read_positions_client = self.create_client(ReadPositions, 'head/read_positions')

    def wait_for_services(self, timeout_sec=10.0):
        """等待所有服务可用"""
        self.get_logger().info('等待服务可用...')
        clients = [
            self.connect_client,
            self.disconnect_client,
            self.is_online_client,
            self.initialize_client,
            self.rotate_client,
            self.list_ports_client,
            self.read_position_client,
            self.read_positions_client
        ]

        start_time = time.time()
        while True:
            all_ready = all(client.wait_for_service(timeout_sec=0.1) for client in clients)
            if all_ready:
                self.get_logger().info('所有服务已就绪')
                return True

            if time.time() - start_time > timeout_sec:
                self.get_logger().error('等待服务超时')
                return False

    def list_ports(self):
        """列出可用串口"""
        self.get_logger().info('=== 测试1: 列出可用串口 ===')
        request = ListPorts.Request()
        future = self.list_ports_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'找到端口: {response.ports}')
        self.get_logger().info(f'消息: {response.message}')
        return response.ports

    def connect(self, port='/dev/ttyUSB0', baudrate=9600):
        """连接舵机"""
        self.get_logger().info(f'=== 测试2: 连接到 {port} (波特率: {baudrate}) ===')
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
        self.get_logger().info('=== 测试3: 检查在线状态 ===')
        request = IsOnline.Request()
        future = self.is_online_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'在线: {response.online}, 消息: {response.message}')
        return response.online

    def initialize(self, servo_ids=None):
        """初始化舵机"""
        if servo_ids is None:
            servo_ids = [1, 2]
        self.get_logger().info(f'=== 测试4: 初始化舵机 {servo_ids} ===')
        request = Initialize.Request()
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
        request.angle = float(angle)
        future = self.rotate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'成功: {response.success}, 消息: {response.message}')
        return response.success

    def read_position(self, servo_id):
        """读取单个舵机位置"""
        self.get_logger().info(f'=== 读取舵机{servo_id}位置 ===')
        request = ReadPosition.Request()
        request.servo_id = servo_id
        future = self.read_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info(f'位置: {response.position}')
        self.get_logger().info(f'成功: {response.success}, 消息: {response.message}')
        return response.success, response.position if response.success else None

    def read_positions(self, servo_ids=None):
        """读取多个舵机位置"""
        if servo_ids is None:
            servo_ids = [1, 2]
        self.get_logger().info(f'=== 批量读取舵机{servo_ids}位置 ===')
        request = ReadPositions.Request()
        request.servo_ids = servo_ids
        future = self.read_positions_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            positions_dict = dict(zip(servo_ids, response.positions))
            self.get_logger().info(f'位置: {positions_dict}')
        self.get_logger().info(f'成功: {response.success}, 消息: {response.message}')
        return response.success, dict(zip(servo_ids, response.positions)) if response.success else None

    def disconnect(self):
        """断开连接"""
        self.get_logger().info('=== 测试8: 断开连接 ===')
        request = Disconnect.Request()
        future = self.disconnect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'成功: {response.success}, 消息: {response.message}')
        return response.success

    def run_test(self, port='/dev/ttyUSB0', baudrate=9600):
        """运行完整测试流程"""
        try:
            self.get_logger().info('=' * 60)
            self.get_logger().info('开始头部舵机ROS2功能测试')
            self.get_logger().info('=' * 60)

            # 测试1: 列出端口
            ports = self.list_ports()
            if not ports:
                self.get_logger().warning('未找到可用串口，使用默认端口')

            time.sleep(0.5)

            # 测试2: 连接
            if not self.connect(port, baudrate):
                self.get_logger().error('❌ 连接失败，测试终止')
                self.get_logger().error('请检查：')
                self.get_logger().error('  1. 舵机板是否上电')
                self.get_logger().error('  2. USB线是否连接')
                self.get_logger().error('  3. 串口设备是否正确')
                return

            time.sleep(0.5)

            # 测试3: 检查在线状态
            if not self.is_online():
                self.get_logger().error('❌ 设备不在线，测试终止')
                self.disconnect()
                return

            time.sleep(0.5)

            # 测试4: 初始化
            servo_ids = [1, 2]
            if not self.initialize(servo_ids):
                self.get_logger().warning('⚠ 初始化失败，继续测试...')

            time.sleep(0.5)

            # 测试5: 旋转测试
            self.get_logger().info('=== 测试5: 旋转测试 ===')
            self.get_logger().info('开始舵机1俯仰测试...')

            # 舵机1 俯仰测试
            test_positions = [500, 550, 600, 550, 500]
            for pos in test_positions:
                if not self.rotate(1, pos):
                    self.get_logger().warning(f'舵机1旋转到{pos}失败')
                time.sleep(0.5)

            time.sleep(0.5)

            self.get_logger().info('开始舵机2偏航测试...')
            # 舵机2 偏航测试
            for pos in test_positions:
                if not self.rotate(2, pos):
                    self.get_logger().warning(f'舵机2旋转到{pos}失败')
                time.sleep(0.5)

            # 测试6: 读取位置测试
            time.sleep(0.5)
            self.get_logger().info('=== 测试6: 读取位置测试 ===')

            # 读取单个舵机位置
            self.get_logger().info('读取舵机1位置...')
            self.read_position(1)
            time.sleep(0.3)

            self.get_logger().info('读取舵机2位置...')
            self.read_position(2)
            time.sleep(0.3)

            # 批量读取舵机位置
            self.get_logger().info('批量读取所有舵机位置...')
            self.read_positions([1, 2])

            # 测试7: 复位
            time.sleep(0.5)
            self.get_logger().info('=== 测试7: 复位到初始位置 ===')
            self.initialize(servo_ids)

            time.sleep(0.5)

            self.get_logger().info('=' * 60)
            self.get_logger().info('✅ 所有测试完成!')
            self.get_logger().info('=' * 60)

        except Exception as e:
            self.get_logger().error(f'❌ 测试出错: {e}')
        finally:
            # 断开连接
            self.disconnect()


def main(args=None):
    rclpy.init(args=args)

    client = HeadTestClient()

    # 等待服务
    if not client.wait_for_services():
        client.get_logger().error('服务不可用，请先启动服务器节点')
        client.destroy_node()
        rclpy.shutdown()
        return

    # 运行测试
    # 可以根据实际情况修改串口和波特率
    client.run_test(port='/dev/ttyUSB0', baudrate=9600)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
