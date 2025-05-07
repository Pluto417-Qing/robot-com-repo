#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from basic_move.DogController import DogMotionController
import time
import threading

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        self.range_sub = self.create_subscription(
            Range,
            '/ultrasonic_sensor',  
            self.range_callback,
            10
        )
        
        self.dog_controller = DogMotionController()
        
        self.safe_distance = 0.5  # 安全距离
        self.obstacle_detected = False
        self.is_avoiding = False
        
        # 创建避障线程
        self.avoidance_thread = threading.Thread(target=self.avoidance_loop)
        # 将线程设置为守护线程，这样主程序退出时线程会自动结束
        self.avoidance_thread.daemon = True
        self.avoidance_thread.start()
        
        self.get_logger().info('Obstacle detector initialized')
    
    def range_callback(self, msg):
        """处理超声波传感器数据"""
        if msg.range < self.safe_distance:
            self.obstacle_detected = True
            self.get_logger().warn(f'Obstacle detected at {msg.range:.2f}m')
        else:
            self.obstacle_detected = False
    
    def avoidance_loop(self):
        while rclpy.ok():
            if self.obstacle_detected and not self.is_avoiding:
                self.is_avoiding = True
                self.get_logger().info('Starting obstacle avoidance')
                
                self.dog_controller.stop_background()
                
                self.avoid_obstacle()
                
                self.is_avoiding = False
            elif not self.obstacle_detected and not self.is_avoiding:
                if not self.dog_controller.background_active:
                    self.dog_controller.start_background(
                        self.dog_controller.cmd_generator.ACTION_WALK,
                        speed=0.3
                    )
            
            time.sleep(0.1)
    
    def avoid_obstacle(self):
        # 停止前进
        self.dog_controller.stop_background()
        
        # 向右转90度
        self.dog_controller.walk(speed=0.2, degree=90, duration=2.0)
        
        # 前进一段距离
        #self.dog_controller.walk(speed=0.3, degree=0, duration=2.0)
        # 向左转90度
        #self.dog_controller.walk(speed=0.2, degree=-90, duration=2.0)
        
        # 继续前进
        self.dog_controller.start_background(
            self.dog_controller.cmd_generator.ACTION_WALK,
            speed=0.3
        )

def main(args=None):
    rclpy.init(args=args)
    
    obstacle_detector = ObstacleDetector()
    
    try:
        obstacle_detector.dog_controller.stand()
        time.sleep(2)

        obstacle_detector.dog_controller.start_background(
            obstacle_detector.dog_controller.cmd_generator.ACTION_WALK,
            speed=0.3
        )
        
        rclpy.spin(obstacle_detector)
        
    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        obstacle_detector.dog_controller.shutdown()
        obstacle_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 