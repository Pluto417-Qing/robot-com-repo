# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd

my_dog_name = "dog1"

class DogController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.speed_x, self.speed_y, self.speed_z = 0.0, 0.0, 0.0
        self.dog_name = my_dog_name
        self.pub = self.create_publisher(
            MotionServoCmd, f"/{self.dog_name}/motion_servo_cmd", 10
        )
        self.timer = self.create_timer(0.1, self.move)
        self.msg = MotionServoCmd()
        self.msg.cmd_type = 1
        self.msg.value = 2
        self.msg.step_height = [0.05, 0.05]

    def set_parameter(self,motion_id=303,vel_des=[0.0, 0.0, 0.0]):
        self.msg.motion_id = motion_id
        self.msg.vel_des = vel_des

    def set_speed(self, speed_x=0.0, speed_y=0.0, speed_z=0.0):
        self.speed_x = speed_x
        self.speed_y = speed_y
        self.speed_z = speed_z
        self.msg.vel_des = [self.speed_x, self.speed_y, self.speed_z]

    def walk(self):
        self.msg.motion_id = 303
        self.msg.vel_des = [self.speed_x, self.speed_y, self.speed_z]

    def liedown(self):
        self.msg.motion_id = 101
        self.msg.vel_des = [0.0, 0.0, 0.0]

    def stand(self):
        self.msg.motion_id = 102
        self.msg.vel_des = [0.0, 0.0, 0.0]

    def stop(self):
        self.msg.motion_id = 0   
        self.msg.vel_des = [0.0, 0.0, 0.0]

    def rotate(self, angle):
        # Implement rotation logic here
        pass

    def move(self):
        self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DogController("DogController")
        node.walk()
        rclpy.spin_once(node)
        node.set_speed(0.1, 0.0, 0.0)
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
