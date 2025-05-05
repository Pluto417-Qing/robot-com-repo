import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from PIL import Image as pimg

import asyncio
import io
import socket
import time


class Subscriber(Node):
    def __init__(self, node_name, topic_name, topic_type, qos_profile):
        super().__init__(node_name)
        self.msg = None
        self.subscriber = self.create_subscription(
            topic_type, topic_name, self.call_back, qos_profile
        )
        self.is_subscribing = False

    def subscribe_once(self, max_waiting_sec=None, log=False):
        if self.is_subscribing == True:
            self.get_logger().warn(
                f"Subscriber is subcribing message automaticly. Please close subscriber first."
            )
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
                    await asyncio.sleep(period)
                rclpy.spin_once(self)

        self.is_subscribing = True
        asyncio.ensure_future(subscribe_forever(period=period))
        self.get_logger().info(f"Opened: Subscriber {self.get_name()}.")
        return

    def close(self):
        self.is_subscribing = False
        self.get_logger().info(f"Closed: Subscriber {self.get_name()}.")
        return

    def call_back(self, message):
        self.msg = message
        return


class Tracker:
    def __init__(self, node_name):
        self.socket = socket.socket()
        self.connected = False
        self.image_subscriber = Subscriber(
            node_name, "/camera/infra1/image_rect_raw", Image, 10
        )
        self.x = None
        self.y = None

    def connect(self, ip, port):
        self.socket.connect((ip, port))
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
                    await asyncio.sleep(period)

        asyncio.ensure_future(update_forever(period=period))

    def update(self):
        if self.connected == False:
            print("Not connected")
            return
        if self.image_subscriber.msg is None:
            return
        self.data_send(self.image_subscriber.msg)

    def data_send(self, msg: Image):
        pil_image = pimg.frombuffer(
            "L", (msg.width, msg.height), bytes(msg.data), "raw", "L", 0, 1
        )
        with io.BytesIO() as output:
            pil_image = pil_image.convert("RGB")
            pil_image.save(output, format="JPEG")
            img_bytes = output.getvalue()

        print("Sending...")
        try:
            self.socket.sendall(img_bytes)
            self.socket.send("end".encode())
        except socket.timeout:
            print("Timeout")

        print("Receiving...")
        try:
            data = self.socket.recv(1024)
            data = data.decode("utf-8").split(" ")

            self.x = int(data[0])
            self.y = int(data[1])
            if self.x == 1000 or self.y == 1000:
                self.x = None
                self.y = None
        except socket.timeout:
            print("Timeout")
            self.x = None
            self.y = None
        return


if __name__ == "__main__":
    # ip & port of the server
    ip = "192.168.1.1"
    port = 1234
    rclpy.init()
    node_name = "ball_tracker"
    tracker = Tracker(node_name=node_name)
    tracker.connect(ip=ip, port=port)
    tracker.track(period=0.1)
    try:
        while rclpy.ok():
            rclpy.spin_once(tracker.image_subscriber, timeout_sec=0.1)
            if tracker.x is not None and tracker.y is not None:
                print(f"X: {tracker.x}, Y: {tracker.y}")
            else:
                print("No data received")
    except KeyboardInterrupt:
        pass
    finally:
        tracker.close()
        rclpy.shutdown()
