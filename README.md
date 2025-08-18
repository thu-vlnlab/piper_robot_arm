# piper_robot_arm
用于采集机械臂相关的摄像头以及机械臂的驱动，及采集代码

Installing the packages:
Register the server's public key:
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
Make sure apt HTTPS support is installed: sudo apt-get install apt-transport-https

Add the server to the list of repositories:

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
Install the libraries (see section below if upgrading packages):
sudo apt-get install librealsense2-dkms #可能会因为bios安全启动未关闭而失败
sudo apt-get install librealsense2-utils
The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.

Optionally install the developer and debug packages:
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
With dev package installed, you can compile an application with librealsense using g++ -std=c++11 filename.cpp -lrealsense2 or an IDE of your choice.

Reconnect the Intel RealSense depth camera and run: realsense-viewer to verify the installation.

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



Installation on Ubuntu
Step 1: Install the ROS2 distribution
Ubuntu 24.04:
ROS2 Kilted
ROS2 Jazzy
Ubuntu 22.04:
ROS2 Iron
ROS2 Humble
Ubuntu 20.04
ROS2 Foxy
Step 2: Install latest Intel® RealSense™ SDK 2.0
Please choose only one option from the 3 options below (in order to prevent multiple versions installation and workspace conflicts)

Option 1: Install librealsense2 debian package from Intel servers
Jetson users - use the Jetson Installation Guide
Otherwise, install from Linux Debian Installation Guide
In this case treat yourself as a developer: make sure to follow the instructions to also install librealsense2-dev and librealsense2-dkms packages
Option 2: Install librealsense2 (without graphical tools and examples) debian package from ROS servers (Foxy EOL distro is not supported by this option):
Configure your Ubuntu repositories
Install all realsense ROS packages by sudo apt install ros-<ROS_DISTRO>-librealsense2*
For example, for Humble distro: sudo apt install ros-humble-librealsense2*
Option 3: Build from source
Download the latest Intel® RealSense™ SDK 2.0
Follow the instructions under Linux Installation
Step 3: Install ROS Wrapper for Intel® RealSense™ cameras
Option 1: Install debian package from ROS servers (Foxy EOL distro is not supported by this option):
Configure your Ubuntu repositories
Install all realsense ROS packages by sudo apt install ros-<ROS_DISTRO>-realsense2-*
For example, for Humble distro: sudo apt install ros-humble-realsense2-*
Option 2: Install from source
Create a ROS2 workspace

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
Clone the latest ROS Wrapper for Intel® RealSense™ cameras from here into '~/ros2_ws/src/'

git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ~/ros2_ws
Install dependencies

sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
Build
colcon build
Source environment
ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: kilted, jazzy, iron, humble, foxy
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
. install/local_setup.bash
