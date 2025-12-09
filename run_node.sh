#!/bin/bash
set -e  # 有命令失败就退出

echo "=== 启动 RealSense 相机节点 ==="
ros2 run realsense2_camera realsense2_camera_node &
sleep 2

echo "=== 进入 piper_ros 目录 ==="
cd ~/piper_ros
source install/setup.bash

echo "=== 激活 Piper CAN 设备 ==="
bash can_activate.sh can_piper 1000000 "1-1.3:1.0"
sleep 1

echo "=== 启动 Piper 单臂节点 ==="
ros2 launch piper start_single_piper.launch.py \
    can_port:=can_piper \
    auto_enable:=true \
    gripper_exist:=false \
    gripper_val_mutiple:=2 &
sleep 2

echo "=== 切换到 MobiMind 工程目录 ==="
cd ~/Main/MobiMind
source install/setup.bash

echo "=== 启动相机脚本 ==="
bash script/run_camera.sh &

echo "✅ 所有节点已启动（都在后台跑），脚本结束。"
exit 0