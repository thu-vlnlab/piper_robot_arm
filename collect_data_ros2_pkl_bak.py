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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import dm_env


class ArmDataCollector(Node):
    def __init__(self, args):
        super().__init__('arm_data_collector')
        
        self.args = args
        
        # 初始化数据队列
        self.puppet_arm_deque = deque()
        # self.master_arm_right_deque = deque()
        # self.puppet_arm_left_deque = deque()
        # self.puppet_arm_right_deque = deque()
        print(f"初始化数据队列，最大长度: {2000}")
        # 初始化ROS2订阅者
        self.init_subscribers()
        
        self.get_logger().info("机械臂数据采集器已初始化")

    def init_subscribers(self):
        """初始化ROS2订阅者"""
        # 机械臂关节状态订阅
        self.puppet_arm_sub = self.create_subscription(
            JointState, 
            self.args.puppet_arm_topic, 
            self.puppet_arm_callback, 
            1000
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
        self.puppet_arm_deque.clear()
        self.puppet_arm_deque.append(msg)
        print(f"[回调] 收到新消息，时间戳: {msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}")
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
        """获取同步的机械臂数据帧"""

        # for _ in range(5):
        #     rclpy.spin_once(self, timeout_sec=0.01)
        rclpy.spin_once(self, timeout_sec=0.1) # 只等待一帧数据
        # 检查是否有足够的数据
        print(f"当前主臂左臂数据队列长度: {len(self.puppet_arm_deque)}")
        if len(self.puppet_arm_deque) == 0:
            print("[get_frame] 队列为空，无法获取数据")
            return False
        
        return self.puppet_arm_deque[-1]  # 返回最新一帧数据
        # return self.puppet_arm_deque.pop()  # 返回最新一帧数据


        
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



    def collect_data(self):
        """收集数据"""
        timesteps = []
        actions = []
        count = 0
        
        # 设置采集频率
        rate = self.create_rate(self.args.frame_rate)
        
        self.get_logger().info(f"开始收集数据，目标帧数: {self.args.max_timesteps}")

            # 等待第一帧数据
        while len(self.puppet_arm_deque) == 0 and rclpy.ok():
            rclpy.spin_once(self)
            self.get_logger().info("等待首帧数据...")
            time.sleep(0.1)
        
        while count < self.args.max_timesteps and rclpy.ok():
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
            puppet_arm= result
            
            # 构建观测数据
            obs = collections.OrderedDict()
            
            # 关节状态数据
            obs['qpos'] = np.array(puppet_arm.position)
            obs['qvel'] = np.array(puppet_arm.velocity)
            obs['effort'] = np.array(puppet_arm.effort)
            # obs['qpos'] = np.concatenate((
            #     np.array(puppet_arm_left.position), 
            #     np.array(puppet_arm_right.position)
            # ), axis=0)
            
            # obs['qvel'] = np.concatenate((
            #     np.array(puppet_arm_left.velocity), 
            #     np.array(puppet_arm_right.velocity)
            # ), axis=0)
            
            # obs['effort'] = np.concatenate((
            #     np.array(puppet_arm_left.effort), 
            #     np.array(puppet_arm_right.effort)
            # ), axis=0)
            
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
            
            # 保存动作数据（主臂关节位置）
            # action = np.concatenate((
            #     np.array(master_arm_left.position), 
            #     np.array(master_arm_right.position)
            # ), axis=0)
            action = None
            actions.append(action)
            timesteps.append(ts)
            
            self.get_logger().info(f"已收集帧数据: {count}")
            self.get_logger().info(f"即将 sleep，count={count}, time={time.time():.6f}")
            # rate.sleep()
            time.sleep(1.0 / self.args.frame_rate)
            self.get_logger().info(f"sleep 返回，count={count}, time={time.time():.6f}")
        
        self.get_logger().info(f"数据收集完成，共收集 {len(timesteps)} 个时间步，{len(actions)} 个动作")
        return timesteps, actions


def save_data(args, timesteps, actions, dataset_path):
    """保存数据（默认PKL；如需完全保留原HDF5逻辑，可通过参数切换）"""
    data_size = len(actions)
    
    # 数据字典 - 移除action相关
    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        # '/action': [],
        # '/base_action': [],
    }
    
    # 处理数据
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])
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
        # joint_dim_qpos = len(timesteps[0].observation['qpos']) 
        # joint_dim_qvel = len(timesteps[0].observation['qpos']) 
        # joint_dim_effort = len(timesteps[0].observation['qpos']) 
        obs = root.create_group('observations')
        d_qpos = obs.create_dataset('qpos', (data_size, 7))
        d_qvel = obs.create_dataset('qvel', (data_size, 6))
        d_effort = obs.create_dataset('effort', (data_size, 7))

        # 转成规则二维数组再写入，避免列表嵌套导致写入不齐整
        qpos_arr = np.asarray(data_dict['/observations/qpos'])
        qvel_arr = np.asarray(data_dict['/observations/qvel'])
        effort_arr = np.asarray(data_dict['/observations/effort'])

        d_qpos[...] = qpos_arr
        d_qvel[...] = qvel_arr
        d_effort[...] = effort_arr
    
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
    
    try:
        # 收集数据
        timesteps, actions = collector.collect_data()
        
        # 检查数据量
        if len(actions) < args.max_timesteps:
            print(f"\033[31m\n保存失败，请记录 {args.max_timesteps} 个时间步的数据。\033[0m\n")
            return
        
        # 保存数据
        dataset_dir = os.path.join(args.dataset_dir, args.task_name)
        if not os.path.exists(dataset_dir):
            os.makedirs(dataset_dir)
        
        # 确定episode索引
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# 运行命令示例:
# python3 collect_data_ros2.py --dataset_dir ~/data --max_timesteps 500 --episode_idx 0 