# piper_robot_arm
用于采集机械臂相关的摄像头以及机械臂的驱动，及采集代码

安装相机ros
Installing the paInstalling the packages:
•	Register the server's public key:
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
•	Make sure apt HTTPS support is installed: sudo apt-get install apt-transport-https
•	Add the server to the list of repositories:
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
•	Install the libraries (see section below if upgrading packages):
sudo apt-get install librealsense2-dkms
•	可能会因为没有关闭安全验证BIOS，而失败
sudo apt-get install librealsense2-utils
The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.
•	Optionally install the developer and debug packages:
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
With dev package installed, you can compile an application with librealsense using g++ -std=c++11 filename.cpp -lrealsense2 or an IDE of your choice.
Reconnect the Intel RealSense depth camera and run: realsense-viewer to verify the installation.
Verify that the kernel is updated :
modinfo uvcvideo | grep "version:" should include realsense string


•	选项 2：从 ROS 服务器安装 librealsense2（不带图形工具和示例）debian 包（此选项不支持 Foxy EOL 发行版）：
o	配置你的 Ubuntu 存储库
o	安装所有 realsense ROS 软件包sudo apt install ros-<ROS_DISTRO>-librealsense2*
	例如，对于 Humble 发行版：sudo apt install ros-humble-librealsense2*
步骤 3：为英特尔® 实感™ 摄像头安装 ROS Wrapper
选项 1：从 ROS 服务器安装 debian 包（此选项不支持 Foxy EOL 发行版）：
•	配置你的 Ubuntu 存储库
•	安装所有 realsense ROS 软件包sudo apt install ros-<ROS_DISTRO>-realsense2-*
•	例如，对于 Humble 发行版：sudo apt install ros-humble-realsense2-*
使用 ros2 运行：
ros2 run realsense2_camera realsense2_camera_node
# or, with parameters, for example - temporal and spatial filters are enabled:
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true

<img width="432" height="582" alt="image" src="https://github.com/user-attachments/assets/bc5d81d3-b43e-4dd0-aa74-df0cc190956c" />
