"""
方案1: 位置补偿 (Offset Compensation)
每次收到新buffer时，用当前实际位置与模型轨迹起始位置的差做补偿，让动作连续
"""
import dataclasses
import enum
import logging
import pathlib
import time
import threading
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非GUI后端
import matplotlib.pyplot as plt
import websocket_client_policy as _websocket_client_policy
import polars as pl
import rich
import tqdm
import tyro

# ROS2导入
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import cv2

logger = logging.getLogger(__name__)

# 全局ROS节点和数据
ros_node = None
latest_joint_state = None
latest_image = None
latest_wrist_image = None
latest_joint_state_time = None
latest_image_time = None
latest_wrist_image_time = None
cv_bridge = CvBridge()
ros_spin_thread = None
ros_spin_active = False


class EnvMode(enum.Enum):
    ALOHA = "aloha"
    ALOHA_SIM = "aloha_sim"
    DROID = "droid"
    LIBERO = "libero"
    THU_VLNA = "thu_vlna"


@dataclasses.dataclass
class Args:
    host: str = "127.0.0.1"
    port: int | None = 6006
    api_key: str | None = None
    num_steps: int = 300
    timing_file: pathlib.Path | None = None
    actions_file: pathlib.Path | None = pathlib.Path("actions_output.pkl")
    env: EnvMode = EnvMode.THU_VLNA
    publish_actions: bool = True
    data_freq: float = 15.0
    actions_per_request: int = 2
    output_dir: pathlib.Path = pathlib.Path("./saved_data_image")


class TimingRecorder:
    def __init__(self) -> None:
        self._timings: dict[str, list[float]] = {}

    def record(self, key: str, time_ms: float) -> None:
        if key not in self._timings:
            self._timings[key] = []
        self._timings[key].append(time_ms)

    def get_stats(self, key: str) -> dict[str, float]:
        times = self._timings[key]
        return {
            "mean": float(np.mean(times)),
            "std": float(np.std(times)),
            "p25": float(np.quantile(times, 0.25)),
            "p50": float(np.quantile(times, 0.50)),
            "p75": float(np.quantile(times, 0.75)),
            "p90": float(np.quantile(times, 0.90)),
            "p95": float(np.quantile(times, 0.95)),
            "p99": float(np.quantile(times, 0.99)),
        }

    def print_all_stats(self) -> None:
        table = rich.table.Table(
            title="[bold blue]Timing Statistics[/bold blue]",
            show_header=True,
            header_style="bold white",
            border_style="blue",
            title_justify="center",
        )
        table.add_column("Metric", style="cyan", justify="left", no_wrap=True, width=20)
        stat_columns = [
            ("Mean", "yellow", "mean"),
            ("Std", "yellow", "std"),
            ("P25", "magenta", "p25"),
            ("P50", "magenta", "p50"),
            ("P75", "magenta", "p75"),
            ("P90", "magenta", "p90"),
            ("P95", "magenta", "p95"),
            ("P99", "magenta", "p99"),
        ]
        for name, style, _ in stat_columns:
            table.add_column(name, justify="right", style=style, no_wrap=True, width=8)
        for key in sorted(self._timings.keys()):
            stats = self.get_stats(key)
            values = [f"{stats[key]:>7.1f}" for _, _, key in stat_columns]
            table.add_row(key, *values)
        console = rich.console.Console(width=None, highlight=True)
        console.print(table)

    def write_parquet(self, path: pathlib.Path) -> None:
        logger.info(f"Writing timings to {path}")
        frame = pl.DataFrame(self._timings)
        path.parent.mkdir(parents=True, exist_ok=True)
        frame.write_parquet(path)


def _get_timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _plot_actions(actions: np.ndarray, save_path: pathlib.Path, timestamp: str) -> None:
    num_steps, num_joints = actions.shape
    fig, axes = plt.subplots(num_joints, 1, figsize=(14, 2.5 * num_joints), sharex=True)
    if num_joints == 1:
        axes = [axes]
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Gripper'] if num_joints == 7 else [f'Joint {i+1}' for i in range(num_joints)]
    time_steps = np.arange(num_steps)
    for i, (ax, name) in enumerate(zip(axes, joint_names)):
        ax.plot(time_steps, actions[:, i], 'b-', linewidth=1.5, label='Executed Action')
        ax.set_ylabel('Rad', fontsize=10)
        ax.set_title(name, fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
        mean_val, std_val = actions[:, i].mean(), actions[:, i].std()
        min_val, max_val = actions[:, i].min(), actions[:, i].max()
        ax.text(0.02, 0.98, f'μ={mean_val:.3f}, σ={std_val:.3f}\nmin={min_val:.3f}, max={max_val:.3f}',
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5), fontsize=8)
    axes[-1].set_xlabel('Step', fontsize=11)
    fig.suptitle(f'[OFFSET_FROM_CMD] Executed Action Trajectories (Total Steps: {num_steps})\n{timestamp}', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plot_path = save_path / f"plot1_executed_actions_offset_from_cmd_{timestamp}.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    logger.info(f"✅ 图1 已保存: {plot_path}")


def _plot_comparison(executed_actions: np.ndarray, actual_positions: np.ndarray, save_path: pathlib.Path, timestamp: str) -> None:
    num_steps, num_joints = executed_actions.shape
    fig, axes = plt.subplots(num_joints, 1, figsize=(14, 2.5 * num_joints), sharex=True)
    if num_joints == 1:
        axes = [axes]
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Gripper'] if num_joints == 7 else [f'Joint {i+1}' for i in range(num_joints)]
    time_steps = np.arange(num_steps)
    for i, (ax, name) in enumerate(zip(axes, joint_names)):
        ax.plot(time_steps, executed_actions[:, i], 'b-', linewidth=1.5, label='Command (Executed Action)', alpha=0.8)
        ax.plot(time_steps, actual_positions[:, i], 'r--', linewidth=1.5, label='Feedback (Actual Position)', alpha=0.8)
        ax.set_ylabel('Rad', fontsize=10)
        ax.set_title(name, fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
        error = np.abs(executed_actions[:, i] - actual_positions[:, i])
        ax.text(0.02, 0.98, f'Mean Error: {error.mean():.4f}\nMax Error: {error.max():.4f}',
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.5), fontsize=8)
    axes[-1].set_xlabel('Step', fontsize=11)
    fig.suptitle(f'[OFFSET_FROM_CMD] Command vs Actual Position Comparison (Total Steps: {num_steps})\n{timestamp}', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plot_path = save_path / f"plot2_comparison_offset_from_cmd_{timestamp}.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    logger.info(f"✅ 图2 已保存: {plot_path}")


def _plot_request_actions(request_records: list, save_path: pathlib.Path, timestamp: str) -> None:
    if len(request_records) == 0:
        logger.warning("没有请求记录，跳过图3绘制")
        return
    num_joints = request_records[0]['actions'].shape[1]
    fig, axes = plt.subplots(num_joints, 1, figsize=(16, 3.5 * num_joints), sharex=True)
    if num_joints == 1:
        axes = [axes]
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Gripper'] if num_joints == 7 else [f'Joint {i+1}' for i in range(num_joints)]
    colors = plt.cm.tab10(np.linspace(0, 1, min(len(request_records), 10)))
    for i, (ax, name) in enumerate(zip(axes, joint_names)):
        for req_idx, record in enumerate(request_records):
            actions = record['actions']
            skipped = record['skipped_count']
            executed_indices = record['executed_indices']
            global_start = record['global_start_step']
            request_id = record['request_id']
            color = colors[req_idx % len(colors)]
            num_actions = len(actions)
            if skipped > 0:
                skipped_x = np.arange(global_start, global_start + skipped)
                skipped_y = actions[:skipped, i]
                if len(skipped_x) == len(skipped_y):
                    ax.plot(skipped_x, skipped_y, '--', color=color, alpha=0.3, linewidth=1.0)
                    ax.scatter(skipped_x, skipped_y, color=color, alpha=0.3, s=15, marker='x')
            if len(executed_indices) > 0:
                exec_x = np.array([global_start + idx for idx in executed_indices])
                exec_y = actions[executed_indices, i]
                ax.plot(exec_x, exec_y, '-', color=color, alpha=0.9, linewidth=1.5,
                       label=f'Req{request_id} (skip:{skipped}, exec:{len(executed_indices)})')
                ax.scatter(exec_x, exec_y, color=color, alpha=0.9, s=20, marker='o')
            discarded_start = skipped + len(executed_indices)
            if discarded_start < num_actions:
                discarded_indices = list(range(discarded_start, num_actions))
                discarded_x = np.array([global_start + idx for idx in discarded_indices])
                discarded_y = actions[discarded_indices, i]
                ax.plot(discarded_x, discarded_y, ':', color=color, alpha=0.15, linewidth=0.8)
        ax.set_ylabel('Rad', fontsize=10)
        ax.set_title(name, fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(loc='upper right', fontsize=7, ncol=2)
    axes[-1].set_xlabel('Global Step (with overlap)', fontsize=11)
    fig.text(0.02, 0.01, 'Legend: ── Executed (solid)  --- Skipped (dashed+X)  ··· Discarded (dotted, faint)', fontsize=9, style='italic')
    fig.suptitle(f'[OFFSET_FROM_CMD] Request Action Sequences (Total Requests: {len(request_records)})\n{timestamp}', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.05)
    plot_path = save_path / f"plot3_request_actions_offset_from_cmd_{timestamp}.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    logger.info(f"✅ 图3 已保存: {plot_path}")


def main(args: Args) -> None:
    timestamp = _get_timestamp()
    logger.info(f"📅 运行时间戳: {timestamp}")
    logger.info(f"🔧 方案: 位置补偿-基于命令 (Offset From Command)")

    # 确保输出目录存在
    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.env == EnvMode.THU_VLNA:
        logger.info("初始化ROS节点获取真实数据...")
        _init_ros_node()

    obs_fn = {
        EnvMode.ALOHA: _random_observation_aloha,
        EnvMode.ALOHA_SIM: _random_observation_aloha,
        EnvMode.DROID: _random_observation_droid,
        EnvMode.LIBERO: _random_observation_libero,
        EnvMode.THU_VLNA: _observation_thu_vlna,
    }[args.env]

    policy = _websocket_client_policy.WebsocketClientPolicy(
        host=args.host, port=args.port, api_key=args.api_key,
    )
    logger.info(f"Server metadata: {policy.get_server_metadata()}")

    for _ in range(2):
        policy.infer(obs_fn())

    timing_recorder = TimingRecorder()
    executed_actions_list = []
    actual_positions_list = []
    request_records = []
    action_published = False

    action_interval = 1.0 / args.data_freq
    logger.info(f"⚙️  执行参数: 频率={args.data_freq}Hz, 间隔={action_interval*1000:.1f}ms, 每{args.actions_per_request}个action后请求新序列")

    action_buffer = None
    action_index = 0
    total_executed = 0
    request_count = 0
    actions_executed_since_request = 0

    current_request_record = None
    current_executed_indices = []

    # 【方案1核心】当前的偏移量
    current_offset = np.zeros(7)
    last_executed_action = None  # 上一个执行的命令（用于计算offset）

    next_action_buffer = None
    request_thread = None
    request_start_time = None
    request_active = False
    request_lock = threading.Lock()

    def async_request_worker(request_id):
        nonlocal next_action_buffer, request_active
        try:
            logger.info(f"🔄 [请求 {request_id}] 后台线程开始请求...")
            start = time.time()
            action_response = policy.infer(obs_fn())
            end = time.time()
            inference_time = end - start
            inference_time_ms = inference_time * 1000
            timing_recorder.record("client_infer_ms", inference_time_ms)
            for key, value in action_response.get("server_timing", {}).items():
                timing_recorder.record(f"server_{key}", value)
            for key, value in action_response.get("policy_timing", {}).items():
                timing_recorder.record(f"policy_{key}", value)
            if 'actions' in action_response:
                with request_lock:
                    next_action_buffer = {
                        'actions': np.array(action_response['actions']),
                        'inference_time': inference_time,
                        'request_id': request_id
                    }
                logger.info(f"✅ [请求 {request_id}] 后台接收完成: shape={next_action_buffer['actions'].shape}, 耗时={inference_time_ms:.1f}ms")
            else:
                logger.error(f"❌ [请求 {request_id}] 返回数据中没有'actions'字段")
        except Exception as e:
            logger.error(f"❌ [请求 {request_id}] 异步请求出错: {e}")
        finally:
            with request_lock:
                request_active = False

    # 首次同步请求
    logger.info("🔄 [初始化] 同步请求首个action序列...")
    first_response = policy.infer(obs_fn())
    if 'actions' in first_response:
        action_buffer = np.array(first_response['actions'])
        action_index = 0

        # 【方案1】首次使用当前位置作为起点，计算初始偏移
        if latest_joint_state is not None:
            current_offset = latest_joint_state - action_buffer[0]
            logger.info(f"✅ [初始化] 初始偏移量: {current_offset}")

        logger.info(f"✅ [初始化] 收到首个序列: shape={action_buffer.shape}")
        current_request_record = {
            'request_id': 0,
            'actions': (action_buffer + current_offset).copy(),  # 记录补偿后的
            'skipped_count': 0,
            'executed_indices': [],
            'global_start_step': 0,
        }
        current_executed_indices = []
    else:
        logger.error("❌ 初始化失败：无法获取首个action序列")
        return

    with tqdm.tqdm(total=args.num_steps, desc="Executing actions [OFFSET]") as pbar:
        while total_executed < args.num_steps:
            action_start = time.time()

            if actions_executed_since_request >= args.actions_per_request and not request_active:
                with request_lock:
                    request_active = True
                    request_count += 1
                    request_start_time = time.time()
                request_thread = threading.Thread(target=async_request_worker, args=(request_count,), daemon=True)
                request_thread.start()

            with request_lock:
                if next_action_buffer is not None:
                    if current_request_record is not None:
                        current_request_record['executed_indices'] = current_executed_indices.copy()
                        request_records.append(current_request_record)

                    new_buffer = next_action_buffer
                    next_action_buffer = None

                    inference_time = new_buffer['inference_time']
                    inference_time_ms = inference_time * 1000
                    actions_to_skip = int(inference_time / action_interval)

                    action_buffer = new_buffer['actions']
                    action_index = min(actions_to_skip, len(action_buffer) - 1) if actions_to_skip > 0 else 0

                    # 【方案1核心】计算新的偏移量：上一个执行的命令 - 新buffer起始位置
                    if last_executed_action is not None:
                        new_start_pos = action_buffer[action_index]
                        current_offset = last_executed_action - new_start_pos
                        offset_norm = np.linalg.norm(current_offset)
                        logger.info(f"🔧 [偏移补偿] 偏移量范数: {offset_norm:.4f}, 偏移向量: [{', '.join([f'{v:.3f}' for v in current_offset])}]")

                    actions_executed_since_request = 0
                    current_request_record = {
                        'request_id': new_buffer['request_id'],
                        'actions': (action_buffer + current_offset).copy(),  # 记录补偿后的
                        'skipped_count': action_index,
                        'executed_indices': [],
                        'global_start_step': total_executed - action_index,
                    }
                    current_executed_indices = []
                    logger.info(f"🔄 [切换Buffer] 请求{new_buffer['request_id']} | 延迟={inference_time_ms:.1f}ms, 跳过{actions_to_skip}个action, 从[{action_index}]开始")
                    timing_recorder.record("actions_skipped", actions_to_skip)

            if action_index >= len(action_buffer):
                logger.warning(f"⚠️  Buffer已用完 (index={action_index}, len={len(action_buffer)})，等待新序列...")
                time.sleep(action_interval)
                continue

            # 【方案1核心】应用偏移量
            raw_action = action_buffer[action_index]
            current_action = raw_action + current_offset

            # 更新上一个执行的命令（用于下次计算offset）
            last_executed_action = current_action.copy()

            current_executed_indices.append(action_index)
            action_index += 1
            total_executed += 1
            actions_executed_since_request += 1

            executed_actions_list.append(current_action.copy())
            if latest_joint_state is not None:
                actual_positions_list.append(latest_joint_state.copy())
            else:
                actual_positions_list.append(np.full(7, np.nan))

            if total_executed % 5 == 1 or total_executed <= 3:
                action_str = ', '.join([f"{val:+.6f}" for val in current_action])
                logger.info(f"  Step {total_executed:3d} | Buffer[{action_index-1}] | Action: [{action_str}]")

            if args.publish_actions and ros_node is not None:
                if not action_published:
                    logger.info("开始发布action到 /joint_states 话题...")
                    action_published = True
                ros_node.publish_action(current_action)

            pbar.update(1)
            action_elapsed = time.time() - action_start
            sleep_time = max(0, action_interval - action_elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    if current_request_record is not None:
        current_request_record['executed_indices'] = current_executed_indices.copy()
        request_records.append(current_request_record)

    logger.info(f"\n✅ 执行完成! 总共执行 {total_executed} 个action, 请求 {request_count} 次")
    timing_recorder.print_all_stats()

    if args.timing_file is not None:
        timing_recorder.write_parquet(args.timing_file)

    if len(executed_actions_list) > 0:
        import pickle
        pkl_path = args.output_dir / f"actions_output_offset_from_cmd_{timestamp}.pkl"
        save_data = {
            'executed_actions': executed_actions_list,
            'actual_positions': actual_positions_list,
            'request_records': request_records,
            'timestamp': timestamp,
            'method': 'offset_from_cmd',
            'args': {
                'num_steps': args.num_steps,
                'data_freq': args.data_freq,
                'actions_per_request': args.actions_per_request,
            }
        }
        with open(pkl_path, 'wb') as f:
            pickle.dump(save_data, f)
        logger.info(f"已保存数据到 {pkl_path}")

        action_array = np.array(executed_actions_list)
        actual_array = np.array(actual_positions_list)
        logger.info(f"Action数组形状: {action_array.shape}")
        logger.info(f"Action范围: min={action_array.min():.3f}, max={action_array.max():.3f}, mean={action_array.mean():.3f}")

        _plot_actions(action_array, args.output_dir, timestamp)
        _plot_comparison(action_array, actual_array, args.output_dir, timestamp)
        _plot_request_actions(request_records, args.output_dir, timestamp)

    global ros_spin_active, ros_spin_thread
    if ros_node is not None:
        ros_spin_active = False
        if ros_spin_thread is not None:
            ros_spin_thread.join(timeout=1.0)
        ros_node.destroy_node()
        rclpy.shutdown()
        logger.info("ROS节点已关闭")


def _random_observation_aloha() -> dict:
    return {
        "state": np.ones((14,)),
        "images": {
            "cam_high": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
            "cam_low": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
            "cam_left_wrist": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
            "cam_right_wrist": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
        },
        "prompt": "do something",
    }


def _random_observation_droid() -> dict:
    return {
        "observation/exterior_image_1_left": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "observation/wrist_image_left": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "observation/joint_position": np.random.rand(7),
        "observation/gripper_position": np.random.rand(1),
        "prompt": "do something",
    }


def _random_observation_libero() -> dict:
    return {
        "observation/state": np.random.rand(8),
        "observation/image": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "observation/wrist_image": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "prompt": "do something",
    }


def _init_ros_node():
    global ros_node

    class ROSDataCollector(Node):
        def __init__(self):
            super().__init__('openpi_data_collector_offset')
            self.joint_sub = self.create_subscription(JointState, '/joint_states_single', self.joint_callback, 10)
            self.image_sub = self.create_subscription(Image, '/miivii_gmsl/image0', self.image_callback, 10)
            self.wrist_image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.wrist_image_callback, 10)
            self.joint_cmd_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.get_logger().info("ROS节点已初始化 [OFFSET]，等待数据...")

        def joint_callback(self, msg):
            global latest_joint_state, latest_joint_state_time
            latest_joint_state = np.array(msg.position[:7])
            latest_joint_state_time = time.time()

        def image_callback(self, msg):
            global latest_image, latest_image_time
            cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            latest_image = cv2.resize(cv_image, (224, 224))
            latest_image_time = time.time()

        def wrist_image_callback(self, msg):
            global latest_wrist_image, latest_wrist_image_time
            cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            latest_wrist_image = cv2.resize(cv_image, (224, 224))
            latest_wrist_image_time = time.time()

        def publish_action(self, action_positions):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'piper_single'
            msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
            msg.position = action_positions.tolist() if isinstance(action_positions, np.ndarray) else action_positions
            msg.velocity = [0.0] * 7
            msg.effort = [0.0] * 7
            self.joint_cmd_pub.publish(msg)

    rclpy.init()
    ros_node = ROSDataCollector()
    global ros_spin_thread, ros_spin_active
    ros_spin_active = True

    def ros_spin_worker():
        while ros_spin_active:
            rclpy.spin_once(ros_node, timeout_sec=0.01)

    ros_spin_thread = threading.Thread(target=ros_spin_worker, daemon=True)
    ros_spin_thread.start()
    logger.info("等待首帧ROS数据...")
    time.sleep(0.5)
    logger.info("ROS节点初始化完成")


def _observation_thu_vlna() -> dict:
    global latest_joint_state, latest_image, latest_wrist_image
    global latest_joint_state_time, latest_image_time, latest_wrist_image_time
    max_wait = 10.0
    wait_start = time.time()
    while latest_joint_state is None or latest_image is None or latest_wrist_image is None:
        missing = []
        if latest_joint_state is None: missing.append("关节状态")
        if latest_image is None: missing.append("主相机(MIIVII)")
        if latest_wrist_image is None: missing.append("腕部相机(RealSense)")
        elapsed = time.time() - wait_start
        if elapsed > max_wait:
            raise RuntimeError(f"等待ROS数据超时({max_wait}秒): {', '.join(missing)}")
        logger.warning(f"⏳ 等待ROS数据 ({elapsed:.1f}s): {', '.join(missing)}")
        time.sleep(0.1)

    current_time = time.time()
    joint_age_ms = (current_time - latest_joint_state_time) * 1000 if latest_joint_state_time else 999
    image_age_ms = (current_time - latest_image_time) * 1000 if latest_image_time else 999
    wrist_age_ms = (current_time - latest_wrist_image_time) * 1000 if latest_wrist_image_time else 999

    if hasattr(_observation_thu_vlna, 'call_count'):
        _observation_thu_vlna.call_count += 1
    else:
        _observation_thu_vlna.call_count = 0
    if _observation_thu_vlna.call_count % 10 == 0:
        logger.info(f"📊 数据年龄 | 关节: {joint_age_ms:.0f}ms, 主相机: {image_age_ms:.0f}ms, 腕相机: {wrist_age_ms:.0f}ms")

    return {
        "observation/state": latest_joint_state,
        "observation/image": latest_image,
        "observation/wrist_image": latest_wrist_image,
        "prompt": "Move the robotic arm, and actuate the end-effector to press the elevator's UP call button.",
    }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main(tyro.cli(Args))
