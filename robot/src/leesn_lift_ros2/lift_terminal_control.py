#!/usr/bin/env python3
"""Interactive terminal controller for the ROS 2 lift bridge."""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from lh_lift_interfaces.action import MoveLift
from lh_lift_interfaces.srv import SetLiftSpeed


class LiftTerminal(Node):
    def __init__(self):
        super().__init__("lift_terminal_control")
        self.speed = 5.0
        self.speed_client = self.create_client(SetLiftSpeed, "/lift/set_speed")
        self.stop_client = self.create_client(Trigger, "/lift/stop")
        self.move_client = ActionClient(self, MoveLift, "/lift/move_pos")

    def wait_for_interfaces(self):
        print("等待升降机 ROS2 接口...")
        if not self.speed_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("找不到 /lift/set_speed，请先启动 lh_lift_bridge")
        if not self.stop_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("找不到 /lift/stop，请先启动 lh_lift_bridge")
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("找不到 /lift/move_pos，请先启动 lh_lift_bridge")

    def set_speed(self, speed_mm_s):
        request = SetLiftSpeed.Request()
        request.speed_mm_s = float(speed_mm_s)
        future = self.speed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.exception():
            print(f"速度控制失败: {future.exception()}")
            return
        response = future.result()
        print(f"速度 {speed_mm_s:.2f} mm/s: {response.message}")

    def stop(self):
        future = self.stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.exception():
            print(f"停止失败: {future.exception()}")
            return
        print(f"停止: {future.result().message}")

    def move_to(self, target_mm):
        goal = MoveLift.Goal()
        goal.target_mm = float(target_mm)
        goal.max_speed_dps = 1200
        goal.timeout_s = 60.0
        goal.tolerance_mm = 1.0
        send_future = self.move_client.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("目标被拒绝")
            return
        print(f"已发送目标位置: {target_mm:.2f} mm")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        print(f"结果: {result.message}, 最终位置: {result.final_pos_mm:.2f} mm")

    @staticmethod
    def feedback(message):
        feedback = message.feedback
        print(
            f"\r当前位置 {feedback.current_pos_mm:8.2f} mm | "
            f"误差 {feedback.error_mm:8.2f} mm | {feedback.mode}    ",
            end="",
            flush=True,
        )


def main():
    rclpy.init()
    node = LiftTerminal()
    try:
        node.wait_for_interfaces()
        print("\n命令: u 上升, d 下降, s 停止, p 目标位置, v 设置速度, q 退出")
        while rclpy.ok():
            command = input("lift> ").strip().lower()
            if command == "u":
                node.set_speed(abs(node.speed))
            elif command == "d":
                node.set_speed(-abs(node.speed))
            elif command == "s":
                node.stop()
            elif command == "p":
                node.move_to(float(input("目标位置(mm): ")))
            elif command == "v":
                node.speed = abs(float(input("点动速度(mm/s): ")))
                print(f"当前点动速度: {node.speed:.2f} mm/s")
            elif command == "q":
                node.stop()
                break
            elif command:
                print("未知命令: u/d/s/p/v/q")
    except (KeyboardInterrupt, EOFError):
        node.stop()
    except Exception as exc:
        print(f"错误: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
