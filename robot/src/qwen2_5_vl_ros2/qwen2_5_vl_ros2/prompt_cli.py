import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PromptClient(Node):
    def __init__(self):
        super().__init__("qwen_vl_prompt_cli")
        self.publisher = self.create_publisher(String, "/qwen_vl/prompt", 10)

    def publish(self, keyword):
        message = String()
        message.data = keyword
        self.publisher.publish(message)
        rclpy.spin_once(self, timeout_sec=0.15)


def main(args=None):
    rclpy.init(args=args)
    node = PromptClient()
    print("输入物品关键词或描述，输入 quit/exit 退出。中英文均可。")
    try:
        while rclpy.ok():
            try:
                keyword = input("qwen-vl> ").strip()
            except EOFError:
                break
            if keyword.lower() in ("quit", "exit"):
                break
            if keyword:
                node.publish(keyword)
                print(f"已发送: {keyword}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
