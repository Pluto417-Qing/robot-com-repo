# ----make the dog walk without stop----#

import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd

my_dog_name = "dog1"

class walk(Node):
    def __init__(self, name):
        """include speed in three directions"""
        super().__init__(name)
        self.speed_x, self.speed_y, self.speed_z = 0.0, 0.0, 0.0
        self.dog_name = my_dog_name
        # self.declare_parameter("speed_x", 0.0)
        # self.declare_parameter("speed_y", 0.0)
        # self.declare_parameter("speed_z", 0.0)
        # self.declare_parameter("gait", "walk")
        # self.declare_parameter("dog_name", my_dog_name)
        # self.dog_name = (
        #     self.get_parameter("dog_name").get_parameter_value().string_value
        # )
        # self.sub = self.create_subscription(
        #     MotionServoCmd, f"/{self.dog_name}/motion_servo_cmd", self.sub_callback, 10
        # )
        self.pub = self.create_publisher(
            MotionServoCmd, f"/{self.dog_name}/motion_servo_cmd", 10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """send the message of how the dog walk without stop"""
        msg = MotionServoCmd()
        msg.motion_id = 303
        # gait=self.get_parameter("gait").get_parameter_value().string_value
        # msg.motion_id=MotionIDDict[gait]
        msg.cmd_type = 1
        msg.value = 2
        msg.vel_des = [self.speed_x, self.speed_y, self.speed_z]
        # speed_x=self.get_parameter("speed_x").get_parameter_value().double_value
        # speed_y=self.get_parameter("speed_y").get_parameter_value().double_value
        # speed_z=self.get_parameter("speed_z").get_parameter_value().double_value
        # msg.vel_des=[self.speed_x, self.speed_y, self.speed_z]
        msg.step_height = [0.05, 0.05]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = walk("walk")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
