#!/usr/bin/env python3
"""关节探针：只订阅、不下发，用来确认桥接是否真的在发数据。

它是 T1 的第一步——先能读到关节状态，才谈得上控制。

用法：
  ros2 run bajie_tasks joint_probe
  ros2 run bajie_tasks joint_probe --ros-args -p topic:=/joint_states
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointProbe(Node):
    def __init__(self):
        super().__init__("joint_probe")
        self.declare_parameter("topic", "/joint_states")
        topic = self.get_parameter("topic").value
        self.count = 0
        self.create_subscription(JointState, topic, self.cb, 10)
        self.get_logger().info(f"订阅 {topic}，等待数据……")

    def cb(self, msg: JointState):
        self.count += 1
        if self.count in (1, 10) or self.count % 100 == 0:
            pairs = ", ".join(
                f"{n}={p:+.4f}" for n, p in list(zip(msg.name, msg.position))[:8]
            )
            self.get_logger().info(
                f"#{self.count} 关节数={len(msg.name)}  {pairs}"
                + ("  ..." if len(msg.name) > 8 else "")
            )


def main():
    rclpy.init()
    node = JointProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
