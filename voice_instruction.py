from std_msgs.msg import String
import subprocess
import rclpy
from rclpy.node import Node
import threading
import signal
import sys


class VoiceTriggerNode(Node):
    def __init__(self):
        super().__init__('voice_trigger_node')

        self.voice_input_sub = self.create_subscription(
            String,
            '/matched_voice_command',
            self.voice_input_callback,
            10
        )

        # 记录当前运行的进程
        self.current_process = None
        self.process_lock = threading.Lock()

        self.get_logger().info("🎤 VoiceTriggerNode 已启动，等待语音指令...")
        self.get_logger().info("💡 按 Ctrl+C 退出监听程序")

    # --------------------------
    # 检查并清理已结束的进程
    # --------------------------
    def check_and_cleanup_process(self):
        with self.process_lock:
            if self.current_process is not None:
                ret = self.current_process.poll()
                if ret is not None:
                    # 进程已结束
                    self.get_logger().info(f"✅ 上一个程序已结束 (返回码: {ret})")
                    self.current_process = None
                    return True
                else:
                    # 进程还在运行
                    return False
            return True  # 没有运行的进程

    # --------------------------
    # 程序 A：电梯上
    # --------------------------
    def run_elevator_up(self):
        if not self.check_and_cleanup_process():
            self.get_logger().warning("⚠️  已有程序在运行中，请等待完成或按 'q' 停止")
            return
        
        self.get_logger().info("⬆️ 启动程序：电梯 上")
        with self.process_lock:
            # 创建新的进程组，子进程不会接收父进程的Ctrl+C
            self.current_process = subprocess.Popen(
                ["python3", "main_dns_1128_offset_from_real_up.py"],
                start_new_session=True
            )
        self.get_logger().info("🚀 程序已启动，按 'q' + Enter 可停止，监听将继续...")

    # --------------------------
    # 程序 B：电梯下
    # --------------------------
    def run_elevator_down(self):
        if not self.check_and_cleanup_process():
            self.get_logger().warning("⚠️  已有程序在运行中，请等待完成或按 'q' 停止")
            return
        
        self.get_logger().info("⬇️ 启动程序：电梯 下")
        with self.process_lock:
            # 创建新的进程组，子进程不会接收父进程的Ctrl+C
            self.current_process = subprocess.Popen(
                ["python3", "main_dns_1128_offset_from_real_down.py"],
                start_new_session=True
            )
        self.get_logger().info("🚀 程序已启动，按 'q' + Enter 可停止，监听将继续...")

    # --------------------------
    # 回调函数：监听语音
    # --------------------------
    def voice_input_callback(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f"🎧 收到语音指令: {cmd}")

        if ("电梯" in cmd and ("上" in cmd or "上键" in cmd)):
            threading.Thread(target=self.run_elevator_up, daemon=True).start()

        elif ("电梯" in cmd and ("下" in cmd or "下键" in cmd)):
            threading.Thread(target=self.run_elevator_down, daemon=True).start()

        # 你还可以扩展更多触发关键词


def main(args=None):
    rclpy.init(args=args)
    node = VoiceTriggerNode()
    
    shutdown_requested = False
    
    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        node.get_logger().info("\n🛑 收到 Ctrl+C，正在退出...")
        shutdown_requested = True
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # 使用循环和 spin_once，这样可以及时响应信号
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理子进程
        node.get_logger().info("🧹 清理资源...")
        with node.process_lock:
            if node.current_process is not None:
                node.get_logger().info("⏹️  正在终止运行中的程序...")
                try:
                    node.current_process.terminate()
                    node.current_process.wait(timeout=3)
                    node.get_logger().info("✅ 程序已终止")
                except Exception as e:
                    node.get_logger().warning(f"终止程序时出错: {e}")
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("👋 已退出")


if __name__ == '__main__':
    main()