import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
from PIL import ImageDraw
import os
import time
import cv2
import numpy as np

debug_level = 3  # 0: no debug, 1: debug, 2: info, 3: error


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
        #log_debug(f"ImageSaver initialized. Save directory: {self.save_dir}")

    def save_image_grayscale(self, msg: Image):
        try:
            #log_debug("Converting ROS Image message to grayscale PIL Image...")
            # Convert ROS Image message to grayscale PIL Image
            pil_image = PILImage.frombuffer(
                "L", (msg.width, msg.height), bytes(msg.data), "raw", "L", 0, 1
            )

            # use this logic
            pil_image = self.img_processor(pil_image)

            # Generate filename
            # Generate filename
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            image_path = os.path.join(self.save_dir, f"image_grayscale_{timestamp}.jpg")
            #log_debug(f"Saving grayscale image to {image_path}...")
            pil_image.save(image_path)
            log_info(f"Grayscale image saved to {image_path}")
        except Exception as e:
            log_error(f"Failed to save grayscale image: {e}")

    def save_image_rgb(self, msg: Image):
        try:
            log_debug("Converting ROS Image message to RGB PIL Image...")
            # Convert ROS Image message to RGB PIL Image
            pil_image = PILImage.frombuffer(
                "RGB", (msg.width, msg.height), bytes(msg.data), "raw", "RGB", 0, 1
            )
            # Generate filename
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            image_path = os.path.join(self.save_dir, f"image_rgb_{timestamp}.jpg")
            log_debug(f"Saving RGB image to {image_path}...")
            pil_image.save(image_path)
            log_info(f"RGB image saved to {image_path}")
        except Exception as e:
            log_error(f"Failed to save RGB image: {e}")

    def img_processor(self, pil_image, minRadius=40, maxRadius=60, draw_circles=True):
        """
        find ball in this PIL Image,return changed Image with ball circled
        """
        try:
            # Convert PIL Image to OpenCV image
            cv_image = np.array(pil_image)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            log_info(
                f"[func]img_processor: successfully convert PIL Image to OpenCV image"
            )
            gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

            # Apply GaussianBlur to reduce noise and prevent false circle detection
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Use Hough transform to detect circles
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                1,
                20,
                param1=50,
                param2=30,
                minRadius=minRadius,
                maxRadius=maxRadius,
            )

            # If circles are found, draw them on the image
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                if draw_circles:
                    for x, y, r in circles:
                        cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)  # 绿色圆圈
                        cv2.circle(cv_image, (x, y), 2, (0, 0, 255), 3)  # 红色中心点

                # Convert the modified OpenCV image back to a PIL Image
                log_info(f"found {len(circles)} circles in the image.")
                # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                pil_image_with_circles = PILImage.fromarray(
                    cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                )
                return pil_image_with_circles,len(circles)  # Return the modified image with circles drawn
            else:
                log_info("No circles found in the image.")
                return pil_image  # Return original if no circles found

        except Exception as e:
            log_error(f"Error processing image: {e}")
            return pil_image


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
        #log_debug("Received image message. Passing to ImageSaver...")
        self.image_saver.save_image_grayscale(msg)
        # self.image_saver.save_image_rgb(msg)


def main():
    #log_debug("Initializing rclpy...")
    rclpy.init()
    save_dir = "images"
    topic_name = "/camera/infra1/image_rect_raw"
    # topic_name = "/image_rgb"
    node_name = "image_saver"

    #log_debug(f"Creating ImageSaver with save directory: {save_dir}")
    image_saver = ImageSaver(save_dir=save_dir)

    # log_debug(
    #     f"Creating ImageSubscriber with node name: {node_name} and topic: {topic_name}"
    # )
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
