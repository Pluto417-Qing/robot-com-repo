import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from PIL import Image as pimg

import assyncio
import io
import socket
import time

class Subscriber(Node):
    def __init__(self, node_name, topic_name, topic_type, qos_profile):
        super().__init__(node_name)
        self.msg = None
        self.subscriber = self.create_subscription(topic_type, topic_name, self.call_back, qos_profile)
        self.is_subscribing = False

    def subscribe_once(self, max_waiting_sec = None, log = False):
        if self.is_subscribing == True:
            self.get_logger().warn(f"Subscriber is subcribing message automaticly. Please close subscriber first.")
            return
        rclpy.spin_once(self, timeout_sec=max_waiting_sec)
        if self.msg is None:
            self.get_logger().warn("Message is None")
        if log == True:
            self.get_logger().info(f"Subscribed message {self.msg}")
        return
    
    def open(self, period=0.0):
        if self.is_subscribing == True:
            self.get_logger().warn(f"Subscriber {self.get_name()} has been opened.")
            return
        async def subscribe_forever(period):
            while self.is_subscribing == True:
                if period > 0:
                    await assyncio.sleep(period)
                rclpy.spin_once(self)

        self.is_subscribing = True
        assyncio.ensure_future(subscribe_forever(period=period))
        self.get_logger().info(f"Opened: Subscriber {self.get_name()}.")
        return 
    
    def close(self):
        self.is_subscribing = False
        self.get_logger().info(f"Closed: Subscriber {self.get_name()}.")
        return 
    
    def call_back(self, message):
        self.msg = message
        return


