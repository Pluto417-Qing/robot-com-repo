import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visual.camera_subscriber import Subscriber

from PIL import Image as pimg

import assyncio
import io
import socket
import time

class Tracker:
    def __init__(self, node_name):
        self.socket = socket.socket()
        self.connected = False
        self.image_subscriber = Subscriber(node_name, '/camera/infra1/image_rect_raw', Image, 10)
        self.x = None
        self.y = None

    def connect(self, ip, port):
        self.socket.connect((ip,port))
        self.connected = True
    
    def close(self):
        self.connected = False
        self.socket.close()

    def track(self, period=0.0):
        self.image_subscriber.open(period=period)

        async def update_forever(period):
            while self.connected == True:
                self.update()
                if period > 0:
                    await assyncio.sleep(period)

        assyncio.ensure_future(update_forever(period=period))
