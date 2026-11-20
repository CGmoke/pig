#!/usr/bin/env python3
"""T1「八界动起来」骨架：订阅关节指令接口，按预设序列下发。

当前只实现最小闭环：
  1. 等 /joint_states 出现（确认桥接在发数据）
  2. 按序列下发关节目标（接口名由参数指定——官方桥接常用 /joint_command）
  3. 下发后读回 /joint_states 校验是否跟随

注意：下发接口必须与场景里挂的桥接图一致。abr_scene 默认没有挂 ROS2 图，
需要先执行 scripts/mount_graphs.py 把官方 omni_graphs 挂上，否则这里发出去没人收。

用法：
  ros2 run bajie_tasks t1_move --ros-args -p cmd_topic:=/joint_command -p dry_run:=true
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class T1Move(Node):
    def __init__(self):
        super().__init__("t1_move")
        self.declare_parameter("cmd_topic", "/joint_command")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("amp", 0.25)          # 摆幅（rad）
        self.declare_parameter("period", 4.0)        # 周期（s）

        cmd_topic = self.get_parameter("cmd_topic").value
        self.dry_run = self.get_parameter("dry_run").value
        self.amp = self.get_parameter("amp").value
        self.period = self.get_parameter("period").value

        self.joint_names = []
        self.last_state = None
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)

        if not self.dry_run:
            # 官方桥接的订阅节点通常叫 /joint_command（JointState 类型）
            self.pub = self.create_publisher(JointState, cmd_topic, 10)
            self.get_logger().info(f"下发接口：{cmd_topic}")
        else:
            self.pub = None
            self.get_logger().warn("dry_run=true：只观测，不下发")

        self.t0 = time.time()
        self.create_timer(0.02, self.step)     # 50 Hz

    def on_state(self, msg: JointState):
        self.last_state = msg
        if not self.joint_names:
            self.joint_names = list(msg.name)
            self.get_logger().info(f"发现 {len(self.joint_names)} 个关节：{self.joint_names}")

    def step(self):
        if self.last_state is None:
            return
        t = time.time() - self.t0
        w = 2.0 * math.pi / max(self.period, 0.5)
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self.joint_names)
        # 只让前 3 个关节做正弦摆动，其余保持当前位置
        cmd.position = [
            p + (self.amp * math.sin(w * t) if i < 3 else 0.0)
            for i, p in enumerate(self.last_state.position)
        ]
        if self.pub is not None:
            self.pub.publish(cmd)


def main():
    rclpy.init()
    node = T1Move()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
