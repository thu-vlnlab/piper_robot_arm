# piper_robot_arm
用于采集机械臂相关的摄像头以及机械臂的驱动，及采集代码

# 安装相机驱动Installing the packages:
1）Register the server's public key:
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

2）Make sure apt HTTPS support is installed: 
sudo apt-get install apt-transport-https

3）Add the server to the list of repositories:

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

4）Install the libraries (see section below if upgrading packages):
sudo apt-get install librealsense2-dkms #可能会因为bios安全启动未关闭而失败
sudo apt-get install librealsense2-utils
The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.

5）Optionally install the developer and debug packages:
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
With dev package installed, you can compile an application with librealsense using g++ -std=c++11 filename.cpp -lrealsense2 or an IDE of your choice.

Reconnect the Intel RealSense depth camera and run: 
realsense-viewer to verify the installation.

Verify that the kernel is updated :
modinfo uvcvideo | grep "version:" should include realsense string

Upgrading the Packages:
Refresh the local packages cache by invoking:
sudo apt-get update

Upgrade all the installed packages, including librealsense with:
sudo apt-get upgrade

To upgrade selected packages only a more granular approach can be applied:
sudo apt-get --only-upgrade install <package1 package2 ...>
E.g:
sudo apt-get --only-upgrade install  librealsense2-utils librealsense2-dkms

Uninstalling the Packages:
Important Removing Debian package is allowed only when no other installed packages directly refer to it. For example removing librealsense2-udev-rules requires librealsense2 to be removed first.

Remove a single package with:
sudo apt-get purge <package-name>

Remove all RealSense™ SDK-related packages with:
dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge



# 安装相机ros2的节点依赖Installation on Ubuntu
  
<details>
  <summary>
    Step 1: Install the ROS2 distribution 
  </summary>

- #### Ubuntu 24.04:
  - [ROS2 Kilted](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)
  - [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

- #### Ubuntu 22.04:
  - [ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
  - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  #### Ubuntu 20.04
	- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
</details>
  
<details>
  <summary>
    Step 2: Install latest Intel&reg; RealSense&trade; SDK 2.0
  </summary>


  
- #### Option 2: Install librealsense2 (without graphical tools and examples) debian package from ROS servers (Foxy EOL distro is not supported by this option):
  - [Configure](http://wiki.ros.org/Installation/Ubuntu/Sources) your Ubuntu repositories
  - Install all realsense ROS packages by ```sudo apt install ros-<ROS_DISTRO>-librealsense2*```
    - For example, for Humble distro: ```sudo apt install ros-humble-librealsense2*```

</details>
  
<details>
  <summary>
    Step 3: Install ROS Wrapper for Intel&reg; RealSense&trade; cameras
  </summary>
  
#### Option 1: Install debian package from ROS servers (Foxy EOL distro is not supported by this option):
  - [Configure](http://wiki.ros.org/Installation/Ubuntu/Sources) your Ubuntu repositories
  - Install all realsense ROS packages by ```sudo apt install ros-<ROS_DISTRO>-realsense2-*```
  - For example, for Humble distro: ```sudo apt install ros-humble-realsense2-*```
  

  
  </details>

<hr>
# 启动相机节点Start the camera node
with ros2 run:
ros2 run realsense2_camera realsense2_camera_node
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true
with ros2 launch:
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true

