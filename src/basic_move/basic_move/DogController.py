# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd
import math
import threading
import time
import queue
from typing import Optional

my_dog_name = "dog1"

class DogCommandGenerator:
    """运动命令生成器"""
    def __init__(self):
        self.ACTION_STOP = 0
        self.ACTION_LIE_DOWN = 101
        self.ACTION_STAND = 111
        self.ACTION_WALK = 303
        self.ACTION_TROT = 308  
        self.ACTION_RUN = 305
        
        self.current_cmd = MotionServoCmd()
        self.current_cmd.cmd_type = 1
        self.current_cmd.value = 0
        self.current_cmd.step_height = [0.05, 0.05]
        self.current_cmd.motion_id = self.ACTION_STOP
        self.current_cmd.vel_des = [0.0, 0.0, 0.0]
        
        # 创建一个线程锁，用于在多线程环境下保护共享资源（current_cmd）的访问
        self.lock = threading.Lock()
    
    def generate_command(self, action: Optional[int] = None, 
                        speed: float = 0.0, 
                        degree: float = 0.0) -> MotionServoCmd:
        """生成运动命令"""
        with self.lock:
            if action is not None:
                self.current_cmd.motion_id = action
            
            if degree != 0:
                vel_x = 0.0
                vel_y = 0.0
                vel_z = speed * math.sin(math.radians(degree))
            else:
                vel_x = speed
                vel_y = speed
                vel_z = 0.0
                
            self.current_cmd.vel_des = [vel_x, vel_y, vel_z]
            return self.current_cmd

class DogROSController(Node):
    """ROS 2 通信控制器，负责实际发送命令"""
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if hasattr(self, '_initialized'):
            return
        super().__init__("DogController")
        self.dog_name = my_dog_name
        self.publisher = self.create_publisher(
            MotionServoCmd, f"/{self.dog_name}/motion_servo_cmd", 10
        )
        self.command_queue = queue.Queue()
        self.running = True
        self._initialized = True
        
    def send_command(self, cmd: MotionServoCmd):
        """发送命令到队列"""
        self.command_queue.put(cmd)
        
    def spin(self):
        """处理命令队列"""
        while self.running:
            try:
                cmd = self.command_queue.get(timeout=0.1)
                self.publisher.publish(cmd)
            except queue.Empty:
                continue

class DogMotionController:
    """高级运动控制器"""
    def __init__(self):
        self.cmd_generator = DogCommandGenerator()
        self.ros_controller = DogROSController()
        
        # 启动ROS通信线程
        self.ros_thread = threading.Thread(target=self.ros_controller.spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # 后台运动控制
        self.background_active = False
        self.background_thread = None
        self.background_lock = threading.Lock()
    
    def _send_cmd(self, action=None, speed=0.0, degree=0.0):
        """生成并发送命令"""
        cmd = self.cmd_generator.generate_command(action, speed, degree)
        self.ros_controller.send_command(cmd)
    
    def start_background(self, action, speed=0.0, degree=0.0):
        """启动后台持续运动"""
        self.stop_background()
        
        self.background_active = True
        def bg_task():
            while self.background_active:
                self._send_cmd(action, speed, degree)
                time.sleep(0.1)
        
        self.background_thread = threading.Thread(target=bg_task)
        self.background_thread.start()
    
    def stop_background(self):
        """停止后台运动"""
        self.background_active = False
        if self.background_thread and self.background_thread.is_alive():
            self.background_thread.join()
    
    def execute_action(self, action, speed=0.0, degree=0.0, duration=0.0):
        """执行临时动作"""
        was_running = self.background_active
        
        if was_running:
            self.stop_background()
        
        self._send_cmd(action, speed, degree)
        
        if duration > 0:
            time.sleep(duration)
            if was_running:
                self.start_background(
                    self.cmd_generator.current_cmd.motion_id,
                    speed,
                    degree
                )
            else:
                self._send_cmd(self.cmd_generator.ACTION_STOP)
    
    # 简化接口
    def stand(self):
        self.execute_action(self.cmd_generator.ACTION_STAND)
    
    def walk(self, speed=0.0, degree=0.0, duration=0.0):
        self.execute_action(
            self.cmd_generator.ACTION_WALK,
            speed,
            degree,
            duration
        )
    
    def shutdown(self):
        """清理资源"""
        self.stop_background()
        self.ros_controller.running = False
        self.ros_thread.join()

def main():
    rclpy.init()
    
    controller = DogMotionController()
    
    try:
        # 示例使用
        controller.stand()
        time.sleep(2)
        
        # 持续行走
        controller.start_background(
            controller.cmd_generator.ACTION_WALK,
            speed=0.3
        )
        
        # 3秒后临时转弯
        time.sleep(3)
        controller.walk(speed=0.3, degree=30, duration=3)
        
        # 5秒后停止
        time.sleep(5)
        controller.stop_background()
        controller.stand()
        
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        controller.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()