#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from basic_move.DogController import DogMotionController
import time
import threading
from PIL import Image as PILImage
from visual.img_save import ImageSaver
import os
import cv2
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        self.range_sub = self.create_subscription(
            Range,
            '/dog1/ultrasonic_payload',  
            self.range_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/infra1/image_rect_raw',
            self.image_callback,
            10
        )
        
        self.dog_controller = DogMotionController()
        self.image_saver = ImageSaver()
        
        self.ball_images_dir = "ball_images"
        if not os.path.exists(self.ball_images_dir):
            os.makedirs(self.ball_images_dir)
        
        self.safe_distance = 1
        self.obstacle_detected = False
        self.is_avoiding = False
        self.turn_direction = 1  # 1 for right, -1 for left
        self.ball_detected = False
        self.last_ball_detected = False  # 用于检测状态变化
        
        # 创建避障线程
        self.avoidance_thread = threading.Thread(target=self.avoidance_loop)
        self.avoidance_thread.daemon = True
        self.avoidance_thread.start()
    
    def range_callback(self, msg):
        if msg.range < self.safe_distance:
            self.obstacle_detected = True
            self.get_logger().warn(f'Obstacle detected at {msg.range:.2f}m')
        else:
            self.obstacle_detected = False
            self.get_logger().warn(f'Obstacle not detected at {msg.range:.2f}m')
    
    def save_ball_image(self, image, num_circles):
        """保存检测到球的图像"""
        try:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            image_path = os.path.join(self.ball_images_dir, f"ball_{num_circles}_circles_{timestamp}.jpg")
            
            if isinstance(image, np.ndarray):
                if len(image.shape) == 2: 
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image = PILImage.fromarray(image)
            
            image.save(image_path)
            self.get_logger().info(f'Saved ball image to {image_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save ball image: {str(e)}')
    
    def image_callback(self, msg):
        try:
            pil_image = PILImage.frombuffer(
                "L", (msg.width, msg.height), bytes(msg.data), "raw", "L", 0, 1
            )
            
            processed_image, num_circles = self.image_saver.img_processor(pil_image)
            
            if num_circles > 0:
                self.ball_detected = True
                self.get_logger().info(f'Ball detected! Found {num_circles} circles')
                
                if not self.last_ball_detected:
                    self.save_ball_image(processed_image, num_circles)
            else:
                self.ball_detected = False
            
            self.last_ball_detected = self.ball_detected
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def avoidance_loop(self):
        while rclpy.ok():
            if self.ball_detected:
                if self.dog_controller.background_active:
                    self.get_logger().info('Ball detected, stopping movement')
                    self.dog_controller.stop_background()
            elif self.obstacle_detected and not self.is_avoiding:
                self.is_avoiding = True
                self.get_logger().info('Starting obstacle avoidance')
                
                self.dog_controller.stop_background()
                
                self.avoid_obstacle()
                
                self.is_avoiding = False
            elif not self.obstacle_detected and not self.is_avoiding and not self.ball_detected:
                if not self.dog_controller.background_active:
                    self.get_logger().info('Starting background walking')
                    self.dog_controller.start_background(
                        self.dog_controller.cmd_generator.ACTION_WALK,
                        speed=0.5,
                        degree=0.0
                    )
            
            time.sleep(0.1)
    
    def avoid_obstacle(self):
        self.get_logger().info('Executing avoidance maneuver')
        # 停止前进
        self.dog_controller.stop_background()
        
        # 执行连续转弯
        turn_angle = 90 * self.turn_direction  # 90度或-90度
        self.get_logger().info(f'Turning {turn_angle} degrees')
        
        # 开始转弯
        self.dog_controller.start_background(
            self.dog_controller.cmd_generator.ACTION_WALK,
            speed=0.7,
            degree=turn_angle
        )
        
        # 保持转弯状态2秒
        time.sleep(2)
        
        # 停止转弯
        self.dog_controller.stop_background()
        
        # 继续前进
        self.get_logger().info('Resuming forward movement')
        self.dog_controller.start_background(
            self.dog_controller.cmd_generator.ACTION_WALK,
            speed=0.5,
            degree=0.0
        )

def main(args=None):
    rclpy.init(args=args)
    
    obstacle_detector = ObstacleDetector()
    
    try:
        obstacle_detector.get_logger().info('Standing up the dog...')
        obstacle_detector.dog_controller.stand()
        time.sleep(2)
        obstacle_detector.get_logger().info('Dog should be standing now')

        obstacle_detector.get_logger().info('Starting to walk...')
        obstacle_detector.dog_controller.start_background(
            obstacle_detector.dog_controller.cmd_generator.ACTION_WALK,
            speed=0.5,
            degree=0.0
        )
        obstacle_detector.get_logger().info('Walking command sent')
        
        rclpy.spin(obstacle_detector)
        
    except KeyboardInterrupt:
        print('Shutting down...')
    except Exception as e:
        obstacle_detector.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        obstacle_detector.dog_controller.shutdown()
        obstacle_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
