#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
ROS2版本的机械臂数据采集脚本
只收集机械臂关节数据，不包含图像采集
"""

import os
import time
import numpy as np
import h5py
import pickle
import argparse
import collections
from collections import deque
import sys
import select

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

import dm_env


class ArmDataCollector(Node):
    def __init__(self, args):
        super().__init__('arm_data_collector')
        
        self.args = args
        
        # 初始化数据队列
        self.puppet_arm_deque = deque()
        self.color_image_deque = deque()
        self.depth_image_deque = deque()
        self.bridge = CvBridge()
        self.color_cb_count = 0
        self.depth_cb_count = 0
        # self.master_arm_right_deque = deque()
        # self.puppet_arm_left_deque = deque()
        # self.puppet_arm_right_deque = deque()
        print(f"初始化数据队列，最大长度: {2000}")
        # 初始化ROS2订阅者
        self.init_subscribers()
        
        self.get_logger().info("机械臂数据采集器已初始化")
        self.get_logger().info(f"关节话题: {self.args.puppet_arm_topic}")
        self.get_logger().info(f"彩色话题: {self.args.color_topic}")
        self.get_logger().info(f"深度话题: {self.args.depth_topic} (use_depth_image={self.args.use_depth_image})")
        self.get_logger().info("按 'q' 后回车 可提前结束并保存数据")

    def init_subscribers(self):
        """初始化ROS2订阅者"""
        # 机械臂关节状态订阅
        self.puppet_arm_sub = self.create_subscription(
            JointState, 
            self.args.puppet_arm_topic, 
            self.puppet_arm_callback, 
            1000
        )
        # 彩色图像订阅
        self.color_image_sub = self.create_subscription(
            Image,
            self.args.color_topic,
            self.color_image_callback,
            qos_profile_sensor_data
        )
        # 深度图像订阅
        self.depth_image_sub = self.create_subscription(
            Image,
            self.args.depth_topic,
            self.depth_image_callback,
            qos_profile_sensor_data
        )
        
        # self.master_arm_right_sub = self.create_subscription(
        #     JointState, 
        #     self.args.master_arm_right_topic, 
        #     self.master_arm_right_callback, 
        #     1000
        # )
        
        # self.puppet_arm_left_sub = self.create_subscription(
        #     JointState, 
        #     self.args.puppet_arm_left_topic, 
        #     self.puppet_arm_left_callback, 
        #     1000
        # )
        
        # self.puppet_arm_right_sub = self.create_subscription(
        #     JointState, 
        #     self.args.puppet_arm_right_topic, 
        #     self.puppet_arm_right_callback, 
        #     1000
        # )
        

    def puppet_arm_callback(self, msg):
        if len(self.puppet_arm_deque) >= 2000:
            self.puppet_arm_deque.popleft()
        self.puppet_arm_deque.append(msg)
        print(f"[关节回调] 时间戳: {msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}")

    def color_image_callback(self, msg: Image):
        if len(self.color_image_deque) >= 2000:
            self.color_image_deque.popleft()
        self.color_image_deque.append(msg)
        self.color_cb_count += 1
        if self.color_cb_count % 30 == 0:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(f"[彩色回调] 已收: {self.color_cb_count}, ts={ts:.6f}")

    def depth_image_callback(self, msg: Image):
        if len(self.depth_image_deque) >= 2000:
            self.depth_image_deque.popleft()
        self.depth_image_deque.append(msg)
        self.depth_cb_count += 1
        if self.depth_cb_count % 30 == 0:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(f"[深度回调] 已收: {self.depth_cb_count}, ts={ts:.6f}")
    # def puppet_arm_callback(self, msg):
    #     """主臂左臂回调函数"""
    #     if len(self.puppet_arm_deque) >= 2000:
    #         self.puppet_arm_deque.popleft()
    #     self.puppet_arm_deque.append(msg)
    #     print(f"[回调] 收到新消息，队列长度: {len(self.puppet_arm_deque)}, 时间戳: {msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}")

    # def master_arm_right_callback(self, msg):
    #     """主臂右臂回调函数"""
    #     if len(self.master_arm_right_deque) >= 2000:
    #         self.master_arm_right_deque.popleft()
    #     self.master_arm_right_deque.append(msg)

    # def puppet_arm_left_callback(self, msg):
    #     """从臂左臂回调函数"""
    #     if len(self.puppet_arm_left_deque) >= 2000:
    #         self.puppet_arm_left_deque.popleft()
    #     self.puppet_arm_left_deque.append(msg)

    # def puppet_arm_right_callback(self, msg):
    #     """从臂右臂回调函数"""
    #     if len(self.puppet_arm_right_deque) >= 2000:
    #         self.puppet_arm_right_deque.popleft()
    #     self.puppet_arm_right_deque.append(msg)



    def get_frame(self):
        """获取同步后的单帧（关节 + 彩色图像 + 深度图像），同步逻辑与 collect_data.py 一致"""
        # 多次快速处理回调，尽快填充队列
        rclpy.spin_once(self, timeout_sec=0.1)
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.0)

        def msg_time(m):
            return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        # 检查必要数据是否到达
        if len(self.puppet_arm_deque) == 0 or len(self.color_image_deque) == 0 or \
           (self.args.use_depth_image and len(self.depth_image_deque) == 0):
            self.get_logger().warn(
                f"[get_frame] 待数据: joint={len(self.puppet_arm_deque)}>0, "
                f"color={len(self.color_image_deque)}>0, depth={len(self.depth_image_deque) if self.args.use_depth_image else 'N/A'}>0")
            return False

        # 参考时间：所有使用的流的“最新时间”的最小值
        latest_times = [
            msg_time(self.puppet_arm_deque[-1]),
            msg_time(self.color_image_deque[-1]),
        ]
        if self.args.use_depth_image:
            latest_times.append(msg_time(self.depth_image_deque[-1]))
        frame_time = min(latest_times)

        # 若任何流的最新时间仍早于参考时间，则不同步
        if msg_time(self.puppet_arm_deque[-1]) < frame_time or \
           msg_time(self.color_image_deque[-1]) < frame_time or \
           (self.args.use_depth_image and msg_time(self.depth_image_deque[-1]) < frame_time):
            self.get_logger().warn(
                f"[get_frame] 最新时间不足: joint={msg_time(self.puppet_arm_deque[-1]):.6f}, "
                f"color={msg_time(self.color_image_deque[-1]):.6f}, "
                f"depth={msg_time(self.depth_image_deque[-1]) if self.args.use_depth_image else 'N/A'} vs frame_time={frame_time:.6f}")
            return False

        # 对齐到参考时间：丢弃早于参考时间的消息，并取出一帧
        while len(self.puppet_arm_deque) > 0 and msg_time(self.puppet_arm_deque[0]) < frame_time:
            self.puppet_arm_deque.popleft()
        if len(self.puppet_arm_deque) == 0:
            return False
        puppet_arm_msg = self.puppet_arm_deque.popleft()

        while len(self.color_image_deque) > 0 and msg_time(self.color_image_deque[0]) < frame_time:
            self.color_image_deque.popleft()
        if len(self.color_image_deque) == 0:
            return False
        color_msg = self.color_image_deque.popleft()

        depth_msg = None
        if self.args.use_depth_image:
            while len(self.depth_image_deque) > 0 and msg_time(self.depth_image_deque[0]) < frame_time:
                self.depth_image_deque.popleft()
            if len(self.depth_image_deque) == 0:
                return False
            depth_msg = self.depth_image_deque.popleft()

        return puppet_arm_msg, color_msg, depth_msg


        
        # # 获取时间戳
        # frame_time = self.puppet_arm_deque[-1].header.stamp.sec + self.puppet_arm_deque[-1].header.stamp.nanosec * 1e-9
    
        # print(f"当前帧时间戳: {frame_time}")

        
        # # 同步数据
        # puppet_arm = self.sync_joint_state(self.puppet_arm_deque, frame_time)
        # print(f"同步后的主臂数据: {puppet_arm}")
        # # master_arm_right = self.sync_joint_state(self.master_arm_right_deque, frame_time)
        # # puppet_arm_left = self.sync_joint_state(self.puppet_arm_left_deque, frame_time)
        # # puppet_arm_right = self.sync_joint_state(self.puppet_arm_right_deque, frame_time)
        
        # return (puppet_arm)

    def sync_joint_state(self, deque_data, frame_time):
        """同步关节状态数据"""
        while (len(deque_data) > 1 and 
               deque_data[0].header.stamp.sec + deque_data[0].header.stamp.nanosec * 1e-9 < frame_time):
            deque_data.popleft()
        return deque_data.popleft()

    def image_to_record(self, msg: Image):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return {
            'stamp': ts,
            'width': int(msg.width),
            'height': int(msg.height),
            'encoding': str(msg.encoding),
            'step': int(msg.step),
            'data': bytes(msg.data),
        }



    def collect_data(self):
        """收集数据"""
        timesteps = []
        actions = []
        count = 0
        
        # 设置采集频率
        rate = self.create_rate(self.args.frame_rate)
        
        self.get_logger().info(f"开始收集数据，目标帧数: {self.args.max_timesteps}")

            # 等待各路首帧数据
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            has_joint = len(self.puppet_arm_deque) > 0
            has_color = len(self.color_image_deque) > 0
            has_depth = (len(self.depth_image_deque) > 0) if self.args.use_depth_image else True
            if has_joint and has_color and has_depth:
                break
            self.get_logger().info(f"等待首帧数据... joint={has_joint} color={has_color} depth={has_depth}")
            time.sleep(0.1)
        
        try:
            while count < self.args.max_timesteps and rclpy.ok():
                # 检查是否有键盘输入 'q'（最小改动，非阻塞）
                try:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0)
                    if rlist:
                        user_input = sys.stdin.readline()
                        if user_input and user_input.strip().lower() == 'q':
                            self.get_logger().info("收到 'q'，结束采集并保存数据")
                            break
                except Exception:
                    pass
                # 获取同步数据
                print(f"[主循环] count={count}, 队列长度={len(self.puppet_arm_deque)}")
                print(f"[主循环前] {time.time():.6f}")
                result = self.get_frame()
                print(f"[主循环后] {time.time():.6f}, result={bool(result)}")
                if not result:
                    self.get_logger().warn("数据同步失败，等待...")
                    rate.sleep()
                    continue
                
                count += 1
                puppet_arm, color_msg, depth_msg = result
                
                # 构建观测数据
                obs = collections.OrderedDict()
                
                # 关节状态数据
                obs['qpos'] = np.array(puppet_arm.position)
                obs['qvel'] = np.array(puppet_arm.velocity)
                obs['effort'] = np.array(puppet_arm.effort)
                # 图像数据，按 collect_data.py 模式组织
                images = {}
                color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='passthrough')
                images[self.args.camera_names[0]] = color_img
                obs['images'] = images
                if self.args.use_depth_image:
                    images_depth = {}
                    depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                    images_depth[self.args.camera_names[0]] = depth_img
                    obs['images_depth'] = images_depth
                
                # 底盘速度数据（固定为0）
                obs['base_vel'] = [0.0, 0.0]
                
                # 创建时间步
                if count == 1:
                    ts = dm_env.TimeStep(
                        step_type=dm_env.StepType.FIRST,
                        reward=None,
                        discount=None,
                        observation=obs
                    )
                else:
                    ts = dm_env.TimeStep(
                        step_type=dm_env.StepType.MID,
                        reward=None,
                        discount=None,
                        observation=obs
                    )
                
                action = None
                actions.append(action)
                timesteps.append(ts)
                
                self.get_logger().info(f"已收集帧数据: {count}")
                self.get_logger().info(f"即将 sleep，count={count}, time={time.time():.6f}")
                time.sleep(1.0 / self.args.frame_rate)
                self.get_logger().info(f"sleep 返回，count={count}, time={time.time():.6f}")
        except KeyboardInterrupt:
            self.get_logger().info("收到 Ctrl+C，中止采集并保存已记录数据")
        
        self.get_logger().info(f"数据收集完成，共收集 {len(timesteps)} 个时间步，{len(actions)} 个动作")
        return timesteps, actions


def save_data(args, timesteps, actions, dataset_path):
    """保存数据，图像与关节数据结构对齐 collect_data.py。"""
    data_size = len(actions)
    
    # 数据字典
    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        # '/action': [],
        # '/base_action': [],
    }
    # 图像字典
    for cam_name in args.camera_names:
        data_dict[f'/observations/images/{cam_name}'] = []
        if args.use_depth_image:
            data_dict[f'/observations/images_depth/{cam_name}'] = []
    
    # 处理数据
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])
        # 图像
        for cam_name in args.camera_names:
            data_dict[f'/observations/images/{cam_name}'].append(ts.observation['images'][cam_name])
            if args.use_depth_image and 'images_depth' in ts.observation:
                data_dict[f'/observations/images_depth/{cam_name}'].append(ts.observation['images_depth'][cam_name])
        # data_dict['/action'].append(action)
        # data_dict['/base_action'].append(ts.observation['base_vel'])
    
    # 保存到pkl文件
    t0 = time.time()
    with open(dataset_path + '.pkl', 'wb') as f:
        pickle.dump(data_dict, f)
    print(f'\033[32m\n保存完成: {time.time() - t0:.1f} 秒. {dataset_path}.pkl \033[0m\n')

    # 保存到HDF5文件
    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'] = False
        root.attrs['compress'] = False
        
        # 创建数据集
        obs = root.create_group('observations')
        d_qpos = obs.create_dataset('qpos', (data_size, 7))
        d_qvel = obs.create_dataset('qvel', (data_size, 6))
        d_effort = obs.create_dataset('effort', (data_size, 7))
        # 图像与 collect_data.py 对齐
        image_group = obs.create_group('images')
        # 推断图像尺寸
        first_cam = args.camera_names[0]
        color_sample = np.asarray(data_dict[f'/observations/images/{first_cam}'][0])
        if color_sample.ndim == 2:
            color_h, color_w = color_sample.shape
            color_c = 1
        else:
            color_h, color_w, color_c = color_sample.shape
        for cam_name in args.camera_names:
            _ = image_group.create_dataset(cam_name, (data_size, color_h, color_w, color_c), dtype='uint8',
                                           chunks=(1, color_h, color_w, color_c))
        if args.use_depth_image:
            image_depth_group = obs.create_group('images_depth')
            depth_sample = np.asarray(data_dict[f'/observations/images_depth/{first_cam}'][0])
            depth_h, depth_w = depth_sample.shape[:2]
            depth_dtype = 'uint16' if depth_sample.dtype == np.uint16 else str(depth_sample.dtype)
            for cam_name in args.camera_names:
                _ = image_depth_group.create_dataset(cam_name, (data_size, depth_h, depth_w), dtype=depth_dtype,
                                                     chunks=(1, depth_h, depth_w))

        # 转成规则二维数组再写入，避免列表嵌套导致写入不齐整
        qpos_arr = np.asarray(data_dict['/observations/qpos'])
        qvel_arr = np.asarray(data_dict['/observations/qvel'])
        effort_arr = np.asarray(data_dict['/observations/effort'])

        d_qpos[...] = qpos_arr
        d_qvel[...] = qvel_arr
        d_effort[...] = effort_arr
        # 写入图像
        for cam_name in args.camera_names:
            image_arr = np.asarray(data_dict[f'/observations/images/{cam_name}'])
            root[f'observations/images/{cam_name}'][...] = image_arr
        if args.use_depth_image:
            for cam_name in args.camera_names:
                depth_arr = np.asarray(data_dict[f'/observations/images_depth/{cam_name}'])
                root[f'observations/images_depth/{cam_name}'][...] = depth_arr
    
    print(f'\033[32m\n保存完成: {time.time() - t0:.1f} 秒. %s \033[0m\n' % dataset_path)


def get_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', action='store', type=str, help='数据集目录',
                        default="./data", required=False)
    parser.add_argument('--task_name', action='store', type=str, help='任务名称',
                        default="aloha_arm_only", required=False)
    parser.add_argument('--episode_idx', action='store', type=int, help='回合索引',
                        default=-1, required=False)
    parser.add_argument('--max_timesteps', action='store', type=int, help='最大时间步数',
                        default=2000, required=False)
    
    # 机械臂话题名称
    parser.add_argument('--puppet_arm_topic', action='store', type=str, help='主臂左臂话题',
                        default='/joint_states_single', required=False)
    # 摄像头话题名称（占位，待用户填充）
    parser.add_argument('--color_topic', action='store', type=str, help='彩色图像话题',
                        default='/camera/camera/color/image_raw', required=False)
    parser.add_argument('--depth_topic', action='store', type=str, help='深度图像话题',
                        default='/camera/camera/depth/image_rect_raw', required=False)
    # 相机名称与是否保存深度
    parser.add_argument('--camera_names', action='store', type=str, nargs='+', help='相机名称列表（用于HDF5键）',
                        default=['cam_main'], required=False)
    parser.add_argument('--use_depth_image', action='store', type=bool, help='是否保存深度图像',
                        default=True, required=False)
    # parser.add_argument('--master_arm_right_topic', action='store', type=str, help='主臂右臂话题',
    #                     default='/master/joint_right', required=False)
    # parser.add_argument('--puppet_arm_left_topic', action='store', type=str, help='从臂左臂话题',
    #                     default='/puppet/joint_left', required=False)
    # parser.add_argument('--puppet_arm_right_topic', action='store', type=str, help='从臂右臂话题',
                        # default='/puppet/joint_right', required=False)
    

    
    # 采集参数
    parser.add_argument('--frame_rate', action='store', type=int, help='采集频率',
                        default=30, required=False)
    parser.add_argument('--save_format', action='store', type=str, help='保存格式：pkl 或 hdf5',
                        default='pkl', required=False)
    
    args = parser.parse_args()
    return args


def main():
    """主函数"""
    args = get_arguments()
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建数据采集器
    collector = ArmDataCollector(args)
    
    # 最小改动：保留默认 Ctrl+C 行为（KeyboardInterrupt 捕获于下方 try/except）
    
    try:
        # 收集数据
        timesteps, actions = collector.collect_data()
        
        # 保存
        if len(timesteps) == 0:
            print("\033[31m\n无数据可保存。\033[0m\n")
            return
        dataset_dir = os.path.join(args.dataset_dir, args.task_name)
        if not os.path.exists(dataset_dir):
            os.makedirs(dataset_dir)
        if args.episode_idx < 0:
            all_episodes = [d for d in os.listdir(dataset_dir) if d.startswith('episode')]
            if len(all_episodes) == 0:
                episode_idx = 0
            else:
                all_episodes = sorted(all_episodes, key=lambda x: int(x.split('_')[-1].split('.')[0]))
                episode_idx = int(all_episodes[-1].split('_')[-1].split('.')[0]) + 1
        else:
            episode_idx = args.episode_idx
        dataset_path = os.path.join(dataset_dir, "episode_" + str(episode_idx))
        save_data(args, timesteps, actions, dataset_path)
        
    except KeyboardInterrupt:
        print("\n用户中断，正在退出...")
    finally:
        # 清理资源
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# 运行命令示例:
# python3 collect_data_ros2.py --dataset_dir ~/data --max_timesteps 500 --episode_idx 0 