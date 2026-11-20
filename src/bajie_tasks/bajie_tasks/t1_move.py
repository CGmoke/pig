#!/usr/bin/env python3
"""T1「八界动起来」——分阶段运动控制，跑完自动留证据。

机器人 12 个可控关节（实测 /joint_states 得到）：
  底盘   wheel_left_joint, wheel_right_joint
  升降   lift_joint（prismatic）
  头部   pitch_joint
  机械臂 joint1..joint6
  夹爪   arm_finger_joint1, arm_finger_joint2

策略：订阅 /joint_states 拿当前位姿，向 /joint_command 发「当前值 + 增量」，
这样不会因为初始位姿未知而甩动。每条消息都带全部 12 个关节，避免控制器丢控制权。

用法：
  ros2 run bajie_tasks t1_move
  ros2 run bajie_tasks t1_move --ros-args -p dry_run:=true          # 只看不发
  ros2 run bajie_tasks t1_move --ros-args -p report:=/workspace/logs/t1_report.json
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEELS = ["wheel_left_joint", "wheel_right_joint"]
GRIPPER = ["arm_finger_joint1", "arm_finger_joint2"]


class T1Move(Node):
    def __init__(self):
        super().__init__("t1_move")
        self.declare_parameter("cmd_topic", "/joint_command")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("report", "/workspace/logs/t1_report.json")
        self.declare_parameter("rate_hz", 30.0)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.dry_run = self.get_parameter("dry_run").value
        self.report_path = self.get_parameter("report").value
        rate = self.get_parameter("rate_hz").value

        self.state = None            # 最新 /joint_states
        self.names = []
        self.phase_log = []

        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.pub = None if self.dry_run else self.create_publisher(
            JointState, self.cmd_topic, 10)

        self.seq = self.build_sequence()
        self.idx = 0
        self.phase_t0 = None
        self.finished = False
        self.t_start = time.time()

        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            f"T1 启动：{len(self.seq)} 个阶段，下发口 {self.cmd_topic}"
            + ("（dry_run）" if self.dry_run else ""))

    # ---------- 阶段序列 ----------
    def build_sequence(self):
        """每个阶段：(名称, 时长 s, 增量函数(u∈[0,1]) -> dict)"""
        return [
            ("底盘前进", 3.0, lambda u: {WHEELS[0]: 6.0 * u, WHEELS[1]: -6.0 * u}),
            ("底盘后退", 2.0, lambda u: {WHEELS[0]: 6.0 * (1 - u), WHEELS[1]: -6.0 * (1 - u)}),
            ("升降上升", 2.5, lambda u: {"lift_joint": 0.08 * u}),
            ("升降回落", 2.0, lambda u: {"lift_joint": 0.08 * (1 - u)}),
            ("头部俯仰", 2.0, lambda u: {"pitch_joint": 0.4 * math.sin(math.pi * u)}),
            ("机械臂展开", 4.0, lambda u: {
                "joint1": 0.6 * u, "joint2": 0.5 * u, "joint3": -0.4 * u,
                "joint4": 0.3 * u, "joint5": 0.3 * u, "joint6": 0.2 * u}),
            ("机械臂收回", 3.0, lambda u: {
                "joint1": 0.6 * (1 - u), "joint2": 0.5 * (1 - u), "joint3": -0.4 * (1 - u),
                "joint4": 0.3 * (1 - u), "joint5": 0.3 * (1 - u), "joint6": 0.2 * (1 - u)}),
            ("夹爪开合", 2.0, lambda u: {
                GRIPPER[0]: 0.03 * math.sin(2 * math.pi * u),
                GRIPPER[1]: -0.03 * math.sin(2 * math.pi * u)}),
        ]

    # ---------- 回调 ----------
    def on_state(self, msg: JointState):
        self.state = msg
        if not self.names:
            self.names = list(msg.name)
            self.get_logger().info(f"锁定 {len(self.names)} 个关节")

    def step(self):
        if self.state is None or self.finished:
            return
        if self.idx >= len(self.seq):
            self.finish()
            return

        name, dur, fn = self.seq[self.idx]
        if self.phase_t0 is None:
            self.phase_t0 = time.time()
            self.get_logger().info(f"[{self.idx+1}/{len(self.seq)}] {name}")

        u = min((time.time() - self.phase_t0) / dur, 1.0)
        delta = fn(u)

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self.names)
        cur = dict(zip(self.state.name, self.state.position))
        cmd.position = [cur.get(n, 0.0) + delta.get(n, 0.0) for n in self.names]

        if self.pub is not None:
            self.pub.publish(cmd)

        if u >= 1.0:
            self.phase_log.append({"phase": name, "duration": dur})
            self.idx += 1
            self.phase_t0 = None

    def finish(self):
        self.finished = True
        elapsed = time.time() - self.t_start
        data = {
            "task": "T1_move",
            "joints": self.names,
            "phases": self.phase_log,
            "elapsed_sec": round(elapsed, 2),
            "dry_run": bool(self.dry_run),
            "cmd_topic": self.cmd_topic,
        }
        try:
            with open(self.report_path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.get_logger().warn(f"报告写入失败：{e}")
        self.get_logger().info(f"T1 完成，用时 {elapsed:.1f}s，报告 → {self.report_path}")


def main():
    rclpy.init()
    node = T1Move()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass          # 被 SIGTERM 打断时信号处理器已 shutdown，忽略重复调用


if __name__ == "__main__":
    main()
