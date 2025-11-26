# piper_robot_arm
用于采集机械臂相关的摄像头以及机械臂的驱动，及采集代码

## 启动realsense相机节点Start the camera node
  
  #### with ros2 run:
    ros2 run realsense2_camera realsense2_camera_node


# piper机械臂相关ros和驱动


###  寻找can模块

执行

```bash
bash find_all_can_port.sh
```

输入密码后，如果can模块插入了电脑，并被电脑检测到，输出类似如下：

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
```

如果有多个，输出类似如下：

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
Interface can1 is connected to USB port 3-1.1:1.0
```

有多少个can模块就会有多少行类似`Interface can1 is connected to USB port 3-1.1:1.0`的输出

其中`can1`是系统找到的can模块名字，`3-1.1:1.0`是该can模块所链接的usb端口

如果之前已经激活过can模块并其名为其它名字，这里假设名字为`can_piper`则输出如下

```bash
Both ethtool and can-utils are installed.
Interface can_piper is connected to USB port 3-1.4:1.0
Interface can0 is connected to USB port 3-1.1:1.0
```

如果没有检测到can模块，则只会输出如下：

```bash
Both ethtool and can-utils are installed.
```

### 2.2 激活单个can模块, **此处使用`can_activate.sh`脚本**

激活单个can模块的情况分两种，一种是电脑只链接了一个can模块，一种是多个can模块插入电脑但是只激活其中一个。

#### 2.2.1 pc只插入一个usb转can模块

直接执行

```bash
bash can_activate.sh can_piper 1000000 "1-1.3:1.0"
```

此处`can_piper`可以改为任意名字，`1000000`为波特率，使用机械臂的情况下，不可更改


解释：**3-1.4:1.0硬件编码的usb端口插入的can设备，名字被重命名为can_piper，设定波特率为1000000，并激活**

(3) 检查是否激活成功

执行`ifconfig`查看是否有`can_piper`，如果有则can模块设置成功


## 3 运行节点


```shell
cd ~/piper_ros
source install/setup.bash
```

### 3.1 单个机械臂

节点名`piper_single_ctrl`

param


启动控制节点，以下几种方式任选一种运行即可

```shell
# 启动节点
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can_piper -p auto_enable:=true -p gripper_exist:=true -p gripper_val_mutiple:=2
# 也可以用launch节点
ros2 launch piper start_single_piper.launch.py can_port:=can_piper auto_enable:=true gripper_exist:=false gripper_val_mutiple:=2
```

`ros2 topic list`

```shell
/arm_status #机械臂状态，详见下文
/enable_flag #使能标志位，发送给节点，发送true用来使能
/end_pose #机械臂末端位姿状态反馈
/joint_states #订阅关节消息，给这个消息发送关节位置能控制机械臂运动
/joint_states_single #机械臂关节状态反馈
/pos_cmd #末端控制消息
```

发布关节消息

注意，机械臂会抬起，请确保机械臂工作范围内无障碍

机械臂会以默认速度的百分之10运动，夹爪力矩设定0.5N

速度范围限制为1-100,小于1时视为1

位置如下

```shell
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'piper_single'}, name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7'], position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}"

```
# 启动mivii摄像头节点
```shell
cd /Main/MobiMind
source install/setup.bash
bash script/run_camera.sh

```

# 启动数据采集节点
```shell
cd /media/nvidia/nvme0n1/piper
python3 collect_data_ros2_pkl_1126_down.py
python3 collect_data_ros2_pkl_1126_up.py
# --dataset_dir 存放的位置
# --max_timesteps 500 最大步
# --episode_idx 0 文件序号
# --frame_rate 采样频率
# 会默认存放在Elements这个硬盘里
```
# 部署代码
用于部署机械臂点电梯算法

## 云端在autodl，需要用子账号到
![img_v3_02s6_81bed87c-7b39-48e2-84ff-b4438c06be3g](https://github.com/user-attachments/assets/138d9c90-9685-4def-9612-aa03bdf079ea)
ssh到服务器之后，进入/root/autodl-tmp/solarixvla文件夹，python scripts/serve_policy.py --env THU_VLNA --port 6006 policy:default运行代码，需要换模型的话需要改一下模型地址。

在小车端需要一次按照上面打开两个摄像头节点和机械臂节点。

进入小车的/media/nvidia/nvme0n1/piper文件夹，
ssh -p 46740 -L 6006:localhost:6006 root@connect.westd.seetacloud.com -N
并输ssh的密码，然后python main_1124.py	即可
