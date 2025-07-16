#!/usr/bin/env python3
"""
测试多相机组件容器的脚本
"""
import os
import subprocess
import sys
import time

def run_command(cmd, description):
    """运行命令并检查结果"""
    print(f"\n{'='*50}")
    print(f"Running: {description}")
    print(f"Command: {cmd}")
    print(f"{'='*50}")
    
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✓ Success")
        if result.stdout:
            print("Output:", result.stdout)
    else:
        print("✗ Failed")
        if result.stderr:
            print("Error:", result.stderr)
        if result.stdout:
            print("Output:", result.stdout)
    
    return result.returncode == 0

def main():
    """主函数"""
    print("USB Cam 多相机组件容器测试")
    
    # 设置ROS2环境
    print("\n1. 设置ROS2环境...")
    os.environ['ROS_DOMAIN_ID'] = '0'
    
    # 检查设备
    print("\n2. 检查视频设备...")
    run_command("ls -la /dev/video*", "列出视频设备")
    
    # 构建项目
    print("\n3. 构建usb_cam包...")
    build_success = run_command(
        "cd /home/loc/ros2_ws && source /opt/ros/foxy/setup.bash && colcon build --packages-select usb_cam",
        "构建usb_cam包"
    )
    
    if not build_success:
        print("构建失败，请检查错误信息")
        return False
    
    # source工作空间
    print("\n4. Source工作空间...")
    run_command(
        "cd /home/loc/ros2_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash",
        "Source工作空间"
    )
    
    # 检查组件是否可用
    print("\n5. 检查usb_cam组件...")
    component_success = run_command(
        "cd /home/loc/ros2_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 component types usb_cam",
        "检查usb_cam组件类型"
    )
    
    # 启动多相机容器 (这里只是测试命令，不实际运行)
    print("\n6. 多相机容器启动命令:")
    print("ros2 launch usb_cam multi_camera_container.launch.py")
    
    # 显示手动启动组件的命令
    print("\n7. 手动启动组件容器的命令:")
    print("# 启动组件容器:")
    print("ros2 run rclcpp_components component_container --ros-args -r __node:=usb_cam_container")
    print("\n# 在另一个终端中加载第一个相机组件:")
    print("ros2 component load /usb_cam_container usb_cam usb_cam::UsbCamNode --node-name camera1 --ros-args --params-file /home/loc/ros2_ws/src/usb_cam/config/params_1.yaml")
    print("\n# 加载第二个相机组件:")
    print("ros2 component load /usb_cam_container usb_cam usb_cam::UsbCamNode --node-name camera2 --ros-args --params-file /home/loc/ros2_ws/src/usb_cam/config/params_2.yaml")
    
    print("\n" + "="*70)
    print("测试完成！如果构建成功，您可以使用上述命令启动多相机系统。")
    print("进程内通信将在同一容器中的节点之间自动启用。")
    print("="*70)
    
    return True

if __name__ == "__main__":
    main()
