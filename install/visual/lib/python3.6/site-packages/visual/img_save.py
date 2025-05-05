import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import os
import time

debug_level = 0  # 0: no debug, 1: debug, 2: info, 3: error


def log_debug(message):
    if debug_level >= 1:
        print(f"[DEBUG] {message}")


def log_info(message):
    if debug_level >= 2:
        print(f"[INFO] {message}")


def log_error(message):
    if debug_level >= 3:
        print(f"[ERROR] {message}")


class ImageSaver:
    def __init__(self, save_dir="image"):
        self.save_dir = save_dir
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        log_debug(f"ImageSaver initialized. Save directory: {self.save_dir}")

    def save_image(self, msg: Image):
        try:
            log_debug("Converting ROS Image message to PIL Image...")
            # Convert ROS Image message to PIL Image
            pil_image = PILImage.frombuffer(
                "L", (msg.width, msg.height), bytes(msg.data), "raw", "L", 0, 1
            )
            # Generate filename
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            image_path = os.path.join(self.save_dir, f"image_{timestamp}.jpg")
            log_debug(f"Saving image to {image_path}...")
            pil_image.save(image_path)
            log_info(f"Image saved to {image_path}")
        except Exception as e:
            log_error(f"Failed to save image: {e}")


class ImageSubscriber(Node):
    def __init__(self, node_name, topic_name, image_saver: ImageSaver):
        super().__init__(node_name)
        self.image_saver = image_saver
        log_debug(f"Initializing ImageSubscriber with topic: {topic_name}")
        self.subscription = self.create_subscription(
            Image, topic_name, self.image_callback, 10
        )
        self.get_logger().info(f"Subscribed to topic: {topic_name}")

    def image_callback(self, msg: Image):
        log_debug("Received image message. Passing to ImageSaver...")
        self.image_saver.save_image(msg)


def main():
    log_debug("Initializing rclpy...")
    rclpy.init()
    save_dir = "images"
    topic_name = "/camera/infra1/image_rect_raw"
    node_name = "image_saver"

    log_debug(f"Creating ImageSaver with save directory: {save_dir}")
    image_saver = ImageSaver(save_dir=save_dir)

    log_debug(
        f"Creating ImageSubscriber with node name: {node_name} and topic: {topic_name}"
    )
    image_subscriber = ImageSubscriber(
        node_name=node_name, topic_name=topic_name, image_saver=image_saver
    )

    try:
        log_debug("Spinning ImageSubscriber node...")
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        log_info("KeyboardInterrupt received. Shutting down...")
    finally:
        log_debug("Destroying ImageSubscriber node...")
        image_subscriber.destroy_node()
        log_debug("Shutting down rclpy...")
        rclpy.shutdown()


if __name__ == "__main__":
    log_debug("Starting main function...")
    main()
