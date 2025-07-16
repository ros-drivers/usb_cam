#!/bin/bash

# 多相机进程内通信启动脚本
# 使用组件容器在单个进程中运行多个usb_cam节点

echo "=============================================="
echo "USB Cam 多相机进程内通信启动脚本"
echo "=============================================="

# 设置ROS2环境
export ROS_DOMAIN_ID=0
cd /home/loc/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

echo "1. 检查可用的视频设备..."
if ls /dev/video* 1> /dev/null 2>&1; then
    echo "找到的视频设备:"
    ls -la /dev/video*
else
    echo "警告: 没有找到视频设备，启动可能会失败"
    echo "您可以使用虚拟设备进行测试:"
    echo "sudo modprobe v4l2loopback devices=2"
fi

echo ""
echo "2. 启动方法选择:"
echo "   a) 使用launch文件 (推荐)"
echo "   b) 手动启动组件容器"
echo ""

read -p "请选择启动方法 (a/b): " choice

case $choice in
    a|A)
        echo "使用launch文件启动..."
        echo "命令: ros2 launch usb_cam multi_camera_container.launch.py"
        echo ""
        ros2 launch usb_cam multi_camera_container.launch.py
        ;;
    b|B)
        echo "手动启动组件容器..."
        echo ""
        echo "步骤1: 启动组件容器"
        echo "正在后台启动容器..."
        
        # 启动组件容器
        ros2 run rclcpp_components component_container --ros-args -r __node:=usb_cam_container &
        CONTAINER_PID=$!
        
        echo "容器PID: $CONTAINER_PID"
        sleep 2
        
        echo ""
        echo "步骤2: 加载相机组件"
        
        # 加载第一个相机
        echo "加载camera1..."
        ros2 component load /usb_cam_container usb_cam usb_cam::UsbCamNode \
            --node-name camera1 \
            --ros-args --params-file /home/loc/ros2_ws/src/usb_cam/config/params_1.yaml \
            -r image_raw:=camera1/image_raw \
            -r camera_info:=camera1/camera_info
        
        sleep 1
        
        # 加载第二个相机
        echo "加载camera2..."
        ros2 component load /usb_cam_container usb_cam usb_cam::UsbCamNode \
            --node-name camera2 \
            --ros-args --params-file /home/loc/ros2_ws/src/usb_cam/config/params_2.yaml \
            -r image_raw:=camera2/image_raw \
            -r camera_info:=camera2/camera_info
        
        echo ""
        echo "组件已加载。按Ctrl+C退出..."
        
        # 等待用户中断
        trap "echo '正在关闭...'; kill $CONTAINER_PID; exit" INT
        wait $CONTAINER_PID
        ;;
    *)
        echo "无效选择，退出。"
        exit 1
        ;;
esac

echo ""
echo "=============================================="
echo "启动完成！"
echo "在另一个终端中使用以下命令查看话题:"
echo "ros2 topic list"
echo "ros2 topic echo /camera1/image_raw"
echo "ros2 topic echo /camera2/image_raw"
echo "=============================================="
