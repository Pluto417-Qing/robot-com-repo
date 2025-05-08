# 机器人障碍物检测模块

## 功能概述
该模块实现了机器人的障碍物检测和避障功能，同时具备球体检测能力。主要功能包括：
- 超声波传感器障碍物检测
- 基于视觉的球体检测
- 自动避障行为控制
- 检测到球体时自动停止运动

## 主要组件
- `ObstacleDetector` 类：核心检测和控制类
  - 订阅超声波传感器数据 (`/dog1/ultrasonic_payload`)
  - 订阅相机图像数据 (`/camera/infra1/image_rect_raw`)
  - 集成 `DogMotionController` 用于运动控制
  - 集成 `ImageSaver` 用于图像处理

## 类结构详解

### ObstacleDetector 类
核心检测和控制类，继承自 ROS2 的 Node 类。

#### 初始化方法
```python
def __init__(self):
```
- 创建超声波传感器订阅器 (`/dog1/ultrasonic_payload`)
- 创建相机图像订阅器 (`/camera/infra1/image_rect_raw`)
- 初始化运动控制器 (`DogMotionController`)
- 初始化图像保存器 (`ImageSaver`)
- 创建球体图像保存目录
- 设置关键参数：
  - 安全距离：1米
  - 避障状态标志
  - 球体检测状态标志
- 启动避障线程

#### 主要方法

1. **range_callback**
```python
def range_callback(self, msg):
```
- 功能：处理超声波传感器数据
- 实现：
  - 检查距离是否小于安全距离
  - 更新障碍物检测状态
  - 记录日志信息

2. **image_callback**
```python
def image_callback(self, msg):
```
- 功能：处理相机图像数据
- 实现：
  - 将图像数据转换为PIL格式
  - 调用图像处理器检测圆形物体
  - 检测到球体时保存图像
  - 更新球体检测状态

3. **save_ball_image**
```python
def save_ball_image(self, image, num_circles):
```
- 功能：保存检测到的球体图像
- 实现：
  - 生成带时间戳的文件名
  - 处理图像格式转换
  - 保存图像到指定目录

4. **avoidance_loop**
```python
def avoidance_loop(self):
```
- 功能：避障控制循环
- 实现：
  - 持续监控障碍物和球体状态
  - 根据状态执行相应动作：
    - 检测到球体：停止运动
    - 检测到障碍物：执行避障
    - 无障碍物：继续前进

5. **avoid_obstacle**
```python
def avoid_obstacle(self):
```
- 功能：执行避障动作
- 实现：
  - 停止当前运动
  - 执行90度转向
  - 等待2秒
  - 恢复前进

### 辅助类

#### DogMotionController
- 功能：控制机器人的运动
- 主要方法：
  - `stand()`: 控制机器人站立
  - `start_background()`: 开始背景运动
  - `stop_background()`: 停止背景运动
  - `shutdown()`: 关闭控制器

#### ImageSaver
- 功能：处理图像保存和球体检测
- 主要方法：
  - `img_processor()`: 处理图像并检测圆形物体

## 工作流程

1. **初始化阶段**
   - 创建ROS2节点
   - 初始化各个组件
   - 启动避障线程

2. **运行阶段**
   - 持续接收传感器数据
   - 实时处理图像
   - 根据检测结果控制机器人行为

3. **避障流程**
   - 检测到障碍物
   - 停止当前运动
   - 执行转向
   - 继续前进

4. **球体检测流程**
   - 接收图像数据
   - 处理图像检测圆形
   - 检测到球体时停止运动
   - 保存相关图像

## 关键参数配置

- 安全距离：1米
- 转向角度：90度
- 转向时间：2秒
- 运动速度：0.5（正常）/ 0.7（转向）

## 使用说明
1. 确保机器人已正确启动并站立
2. 运行程序后，机器人将自动开始行走
3. 遇到障碍物时会自动避障
4. 检测到球体时会自动停止

## 依赖项
- ROS2
- OpenCV
- PIL (Python Imaging Library)
- NumPy 