import rclpy
from rclpy import node
from rdelfin_msgs.msg import CustomMessage


class Listener(node.Node):

    def __init__(self):
        super().__init__("listener")
        self.subscriber_ = self.create_subscription(
            CustomMessage,
            "my_msg",
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg}"')


def main():
    rclpy.init()
    listener = Listener()
    rclpy.spin(listener)


if __name__ == "__main__":
    main()
