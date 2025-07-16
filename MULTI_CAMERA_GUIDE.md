# USB Cam 多相机进程内通信指南

本指南介绍如何在ROS2中使用usb_cam包实现多相机驱动，并启用进程内通信以提高性能。

## 概述

进程内通信（Intra-process Communication）是ROS2的一个重要特性，它允许在同一进程中的节点之间进行零拷贝消息传递，大大提高了性能，特别适合于高频率、大数据量的应用场景，如多相机系统。

## 架构说明

### 传统方式 vs 进程内通信

**传统方式（多进程）：**
```
┌─────────────┐   DDS/网络   ┌─────────────┐
│   Camera1   │──────────────│  处理节点   │
│   Node      │              │             │
└─────────────┘              └─────────────┘
┌─────────────┐   DDS/网络   
│   Camera2   │──────────────
│   Node      │              
└─────────────┘              
```

**进程内通信方式：**
```
┌─────────────────────────────────────────┐
│         Component Container             │
│  ┌─────────────┐    ┌─────────────┐    │
│  │   Camera1   │    │   Camera2   │    │
│  │   Node      │    │   Node      │    │
│  └─────────────┘    └─────────────┘    │
│           │               │            │
│           └───────────────┘            │
│           零拷贝内存共享                 │
└─────────────────────────────────────────┘
```

## 配置文件

系统使用以下配置文件：

### Camera1配置 (`config/params_1.yaml`)
```yaml
/**:
    ros__parameters:
      video_device: "/dev/video0"
      framerate: 30.0
      frame_id: "camera1"
      camera_name: "camera1"
      # ... 其他参数
```

### Camera2配置 (`config/params_2.yaml`)
```yaml
/**:
    ros__parameters:
      video_device: "/dev/video1"  # 不同的设备
      framerate: 30.0
      frame_id: "camera2"
      camera_name: "camera2"
      # ... 其他参数
```

## 启动方法

### 方法1：使用Launch文件（推荐）

```bash
# 构建项目
cd /home/loc/ros2_ws
colcon build --packages-select usb_cam

# source环境
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 启动多相机系统
ros2 launch usb_cam multi_camera_container.launch.py
```

### 方法2：使用启动脚本

```bash
cd /home/loc/ros2_ws/src/usb_cam
chmod +x scripts/start_multi_camera.sh
./scripts/start_multi_camera.sh
```

### 方法3：手动启动组件

```bash
# 1. 启动组件容器
ros2 run rclcpp_components component_container --ros-args -r __node:=usb_cam_container

# 2. 在另一个终端中加载相机组件
# Camera1
ros2 component load /usb_cam_container usb_cam usb_cam::UsbCamNode \
    --node-name camera1 \
    --ros-args --params-file config/params_1.yaml \
    -r image_raw:=camera1/image_raw \
    -r camera_info:=camera1/camera_info

# Camera2  
ros2 component load /usb_cam_container usb_cam usb_cam::UsbCamNode \
    --node-name camera2 \
    --ros-args --params-file config/params_2.yaml \
    -r image_raw:=camera2/image_raw \
    -r camera_info:=camera2/camera_info
```

## 验证和测试

### 查看活动的话题
```bash
ros2 topic list
```

应该看到类似以下的输出：
```
/camera1/camera_info
/camera1/image_raw
/camera2/camera_info
/camera2/image_raw
```

### 查看图像数据
```bash
# 查看camera1的图像
ros2 topic echo /camera1/image_raw

# 查看camera2的图像  
ros2 topic echo /camera2/image_raw
```

### 查看节点信息
```bash
# 查看容器中的组件
ros2 component list /usb_cam_container

# 查看节点图
ros2 node list
ros2 node info /usb_cam_container
```

## 性能优势

使用进程内通信的主要优势：

1. **零拷贝传输**: 消息在节点间直接共享内存，无需序列化/反序列化
2. **降低延迟**: 消除了DDS网络层的开销
3. **减少CPU使用**: 没有消息拷贝和网络处理
4. **提高吞吐量**: 特别适合高频率、大数据量的图像流

## 故障排除

### 常见问题

1. **设备不存在**
   ```bash
   # 检查视频设备
   ls -la /dev/video*
   
   # 创建虚拟设备进行测试
   sudo modprobe v4l2loopback devices=2
   ```

2. **权限问题**
   ```bash
   # 添加用户到video组
   sudo usermod -a -G video $USER
   # 重新登录使生效
   ```

3. **参数文件未找到**
   ```bash
   # 确保参数文件存在
   ls -la config/params_*.yaml
   ```

4. **组件加载失败**
   ```bash
   # 检查组件是否正确注册
   ros2 component types usb_cam
   ```

### 调试命令

```bash
# 查看组件容器日志
ros2 run rclcpp_components component_container --ros-args --log-level debug

# 查看特定节点的参数
ros2 param list /camera1
ros2 param get /camera1 video_device

# 监控系统资源
htop
```

## 扩展多相机

要添加更多相机，只需：

1. 创建新的参数文件（如`params_3.yaml`）
2. 在launch文件中添加相应的配置
3. 确保有足够的USB带宽和系统资源

## 参数详解

`params_1.yaml`和`params_2.yaml`中的主要参数都会被使用：

- **video_device**: 视频设备路径，必须不同
- **framerate**: 帧率设置
- **pixel_format**: 像素格式，影响图像质量和带宽
- **image_width/height**: 图像分辨率
- **brightness/contrast/saturation**: 图像调节参数（通过V4L2控制）
- **auto_white_balance/autoexposure**: 自动控制参数
- **exposure/white_balance**: 手动控制参数

这些参数通过V4L2 API直接控制相机硬件，实现对相机的精确控制。

## 进程内通信技术细节

ROS2的进程内通信使用以下机制：
- **共享指针**: 消息作为`std::shared_ptr`在节点间传递
- **发布订阅优化**: 同进程内的订阅者直接接收发布者的指针
- **生命周期管理**: 自动管理共享消息的内存生命周期
- **类型安全**: 编译时确保消息类型匹配
