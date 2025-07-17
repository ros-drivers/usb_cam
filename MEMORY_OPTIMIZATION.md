# V4L2 Camera - Memory Optimization Guide

## std::move优化实现说明

### 问题分析

在原始的usb_cam实现中，消息发布存在以下问题：

1. `image_transport::CameraPublisher::publish()` 不支持 `std::unique_ptr` 参数
2. 消息通过拷贝而非移动语义传递，造成内存开销
3. 缺乏零拷贝优化

### 解决方案

#### 1. 修改头文件 (usb_cam_node.hpp)

```cpp
// 添加单独的发布器支持std::move
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_raw_publisher;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_camera_info_publisher;

// 保留image_transport用于兼容性
std::shared_ptr<image_transport::CameraPublisher> m_image_publisher;
```

#### 2. 修改构造函数初始化

```cpp
m_image_raw_publisher(this->create_publisher<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::QoS{100})),
m_camera_info_publisher(this->create_publisher<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::QoS{100})),
```

#### 3. 实现零拷贝发布

```cpp
bool UsbCamNode::take_and_send_image()
{
  // ... 填充消息数据 ...
  
  // 使用std::move优化内存，分别发布图像和相机信息
  m_image_raw_publisher->publish(std::move(m_image_msg));
  m_camera_info_publisher->publish(*m_camera_info_msg);
  
  // 重新创建图像消息以供下次使用
  m_image_msg = std::make_unique<sensor_msgs::msg::Image>();
  m_image_msg->header.frame_id = m_parameters.frame_id;
  
  return true;
}
```

### 性能优势

1. **零拷贝传输**: `std::move` 避免大图像数据的内存拷贝
2. **内存效率**: 减少内存峰值使用
3. **低延迟**: 消除不必要的数据复制操作
4. **进程内优化**: 配合intra-process通信实现最佳性能

### 兼容性说明

- 保留 `image_transport::CameraPublisher` 用于压缩图像传输
- 新增单独发布器专门用于零拷贝优化
- 支持现有的topic名称和消息格式

### 使用建议

1. **原始图像**: 使用优化后的单独发布器 + std::move
2. **压缩图像**: 继续使用image_transport进行格式转换
3. **多相机**: 结合intra-process通信获得最佳性能

## 项目重命名说明

### 从 usb_cam 到 v4l2_cam

本项目已从原始的 `usb_cam` 重命名为 `v4l2_cam`，体现以下改进：

1. **更准确的命名**: 突出V4L2设备支持，不局限于USB相机
2. **增强功能**: 多相机支持、进程内通信、内存优化
3. **现代化设计**: C++17特性、改进的错误处理
4. **扩展兼容性**: 支持更多V4L2设备类型

### 主要变更

- 包名: `usb_cam` → `v4l2_cam`
- 命名空间: 保持向后兼容
- Launch文件: 支持多相机配置
- 参数文件: 增强的配置选项

### 向后兼容性

- API接口保持兼容
- 消息格式不变
- 配置参数兼容
- 话题名称兼容
