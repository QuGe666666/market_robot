import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PromptPublisher(Node):
    def __init__(self):
        super().__init__("grounded_sam2_prompt_cli")
        self.declare_parameter("prompt_topic", "/grounded_sam2/prompt")
        self.publisher = self.create_publisher(
            String, str(self.get_parameter("prompt_topic").value), 10
        )

    def publish(self, prompt):
        message = String()
        message.data = prompt
        self.publisher.publish(message)
        rclpy.spin_once(self, timeout_sec=0.2)


def main():
    rclpy.init()
    node = PromptPublisher()
    try:
        print("输入目标描述，回车开始左右相机识别分割；输入 q 退出。")
        while rclpy.ok():
            prompt = input("目标> ").strip()
            if prompt.lower() in {"q", "quit", "exit"}:
                break
            if prompt:
                node.publish(prompt)
                print(f"已提交: {prompt}")
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
