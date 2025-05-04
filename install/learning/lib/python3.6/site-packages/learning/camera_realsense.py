import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MyNodeSubscriber(Node):
    def __init__(self):
        super().__init__("MyNodeSubscriber")
        self.subscription = self.create_subscription(
            Image, "/camera/infra1/image_rect_raw", self.listener_callback, 10
        )
        return

    def listener_callback(self, msg):
        # Process the received image message here
        return


def main():
    rclpy.init(args=None)
    try:
        node = MyNodeSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
