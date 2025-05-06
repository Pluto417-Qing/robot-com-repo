import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import os
import time
import cv2
import numpy as np
from ultralytics import YOLO


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


class ImgProcessor:
    def __init__(self):
        log_debug("ImgProcessor initialized.")
        # initialize YOLO model
        self.model = YOLO("./models/best.pt")  # Load the YOLOv5 model

    def process_image(self, pil_image: PILImage):
        try:
            log_debug("Processing image with YOLO model...")
            # 检查 PIL 图像模式，如果是灰度图（L），转换为 RGB
            if pil_image.mode == "L":
                log_debug("Converting grayscale image to RGB...")
                pil_image = pil_image.convert("RGB")
            # 将 PIL 图像转换为 OpenCV 格式
            cv_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
            # 使用 YOLO 模型进行推理
            results = self.model.predict(
                source=cv_image,  # 直接传递 OpenCV 图像
                conf=0.25,
                iou=0.7,
                imgsz=640,
            )
            # 在原始图像上绘制检测结果
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # 获取边界框坐标
                    conf = box.conf[0]  # 获取置信度
                    label = box.cls[0]  # 获取类别
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        cv_image,
                        f"{label} {conf:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )
            # 将处理后的 OpenCV 图像转换回 PIL 格式
            processed_image = PILImage.fromarray(
                cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            )
            log_debug("Image processing complete.")
            return processed_image
        except Exception as e:
            log_error(f"Failed to process image with YOLO: {e}")
            return pil_image  # 如果处理失败，返回原始图像


class ImageSaver:
    def __init__(self, save_dir="image"):
        self.save_dir = save_dir

        self.img_processor = ImgProcessor()

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        log_debug(f"ImageSaver initialized. Save directory: {self.save_dir}")

    def save_image_grayscale(self, msg: Image):
        try:
            log_debug("Converting ROS Image message to grayscale PIL Image...")
            # Convert ROS Image message to grayscale PIL Image
            pil_image = PILImage.frombuffer(
                "L", (msg.width, msg.height), bytes(msg.data), "raw", "L", 0, 1
            )

            # use ImgProcessor to process the image if needed
            pil_image = self.img_processor.process_image(pil_image)

            # Generate filename
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            image_path = os.path.join(self.save_dir, f"image_grayscale_{timestamp}.jpg")
            log_debug(f"Saving grayscale image to {image_path}...")
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
        self.image_saver.save_image_grayscale(msg)
        # self.image_saver.save_image_rgb(msg)


def main():
    log_debug("Initializing rclpy...")
    rclpy.init()
    save_dir = "images"
    topic_name = "/camera/infra1/image_rect_raw"
    # topic_name = "/image_rgb"
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
