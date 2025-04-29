# ----get ros2 msg info of the sensor----#


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class sensor_suber(Node):
    """subscribe the sensor data"""

    def __init__(self, name) -> None:
        super().__init__(name)
        self.declare_parameter("dog_name", "dog2")
        dog_name = self.get_parameter("dog_name").get_parameter_value().string_value
        self.sub = self.create_subscription(
            Range, f"/{dog_name}/ultrasonic_payload", self.sub_callback, 10
        )
        pass

    def sub_callback(self, msg: Range):
        """the callback function of the subscriber"""
        dist = msg.range
        self.get_logger().info(f"the distance is {dist}")


def main(args=None):
    rclpy.init(args=args)
    node = sensor_suber("my_sensor")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
