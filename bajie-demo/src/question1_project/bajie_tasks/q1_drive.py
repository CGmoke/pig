#!/usr/bin/env python3
"""题目一：关节分步驱动与 RViz 展示。

流程（step=all）：lift → head → arm → gripper_open → gripper_close。
- 每个阶段用 smoothstep 在 move_sec 内从当前位姿插值到目标位姿，30 Hz 发布 /joint_states；
- 开始前按官方模型检查每一段轨迹，保留对地面与机身的净空；
- lift / head / arm 三个阶段结束后保持 pause_sec 秒；
- 每个阶段完成时记录全量 10 个关节的 keyframe，最后记录 final_joint_state；
- 输入输出均为 JSONL，输出沿用输入 id。

没有物理仿真：RViz 里看到的是 joint_state_publisher 被本节点替代后的效果，
robot_state_publisher 把 /joint_states 转成 TF，RViz 的 RobotModel 负责渲染。

用法：
  ros2 run bajie_tasks q1_drive \
    --ros-args -p input_file:=inputs/q1_demo_input.jsonl \
               -p output_file:=answers/q1_demo_output.jsonl
"""
import json
import os
from pathlib import Path
import random
import time

import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from bajie_tasks.q1_safety import ClearanceModel, JOINTS, UnsafeMotion, ZERO

# 向前上方抬展，避免原演示中大幅负向 joint2 导致腕部下降到地面。
# 范围只是候选，是否播放仍由基于官方几何的整段预检决定。
SEED_RANGES = {
    "lift_joint": (0.08, 0.40),
    "pitch_joint": (-0.35, 0.35),
    "joint1": (-0.35, 0.35),
    "joint2": (0.25, 0.60),
    "joint3": (-0.20, 0.25),
    "joint4": (-0.30, 0.30),
    "joint5": (-0.42, -0.10),
    "joint6": (-0.40, 0.40),
}
GRIPPER_OPEN_RANGE = (0.25, 0.48)

STAGES_ALL = ["lift", "head", "arm", "gripper_open", "gripper_close"]
PAUSE_STAGES = {"lift", "head", "arm"}


class Q1Drive(Node):
    def __init__(self):
        super().__init__("q1_drive")
        self.declare_parameter("input_file", "inputs/q1_demo_input.jsonl")
        self.declare_parameter("output_file", "answers/q1_demo_output.jsonl")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("robot_description", "")

        model_share = Path(get_package_share_directory("bajie_robot"))
        description = self.get_parameter("robot_description").value
        if not description:
            description = xacro.process_file(str(model_share / "urdf/bajie_robot.urdf.xacro")).toxml()
        self.clearance = ClearanceModel(description, model_share)

        self.input_file = self.get_parameter("input_file").value
        self.output_file = self.get_parameter("output_file").value
        hz = float(self.get_parameter("publish_hz").value)
        self.rate_hz = hz
        # 不用 create_rate：Rate.sleep 依赖 executor 定时器，本节点不 spin 会永久阻塞
        self.period = 1.0 / hz
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.current = ZERO.copy()

    # ---------------- 输入输出 ----------------
    def read_cases(self):
        cases = []
        with open(self.input_file, "r", encoding="utf-8") as f:
            for lineno, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    cases.append(json.loads(line))
                except json.JSONDecodeError as e:
                    self.get_logger().error(f"{self.input_file}:{lineno} JSON 解析失败：{e}")
        return cases

    def write_results(self, results):
        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.output_file, "w", encoding="utf-8") as f:
            for item in results:
                f.write(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n")

    # ---------------- 动作 ----------------
    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(self.current[j]) for j in JOINTS]
        self.pub.publish(msg)

    def snapshot(self):
        return {j: round(float(self.current[j]), 6) for j in JOINTS}

    @staticmethod
    def sample_targets(seed):
        """用 seed 生成可复现的目标位姿。"""
        rng = random.Random(seed)
        targets = {}
        targets["lift_joint"] = rng.uniform(*SEED_RANGES["lift_joint"])
        targets["pitch_joint"] = rng.uniform(*SEED_RANGES["pitch_joint"])
        for j in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"):
            targets[j] = rng.uniform(*SEED_RANGES[j])
        open_angle = rng.uniform(*GRIPPER_OPEN_RANGE)
        targets["arm_finger_joint1"] = open_angle
        targets["arm_finger_joint2"] = open_angle
        return targets

    @staticmethod
    def goal_for(stage, targets):
        if stage == "lift":
            return {"lift_joint": targets["lift_joint"]}
        if stage == "head":
            return {"pitch_joint": targets["pitch_joint"]}
        if stage == "arm":
            return {j: targets[j] for j in
                    ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")}
        if stage == "gripper_open":
            a = targets["arm_finger_joint1"]
            return {"arm_finger_joint1": a, "arm_finger_joint2": a}
        if stage == "gripper_close":
            return {"arm_finger_joint1": 0.0, "arm_finger_joint2": 0.0}
        return {}

    def plan(self, stages, targets):
        """预检全部阶段；任一段失败则不发布本测例的运动。"""
        state = self.current.copy()
        plan = []
        # 多行 JSONL 中，回零同样必须检查，避免上一例结束后突然跳回零位。
        for name in (["reset"] if state != ZERO else []) + stages:
            goal = ZERO if name == "reset" else self.goal_for(name, targets)
            end = {**state, **goal}
            try:
                ground, body = self.clearance.certify(state, end)
            except UnsafeMotion as exc:
                raise UnsafeMotion(f"{name}：{exc}") from exc
            plan.append((name, goal, ground, body))
            state = end
        return plan

    def move_to(self, goal, move_sec):
        """smoothstep 插值，同时保持未列出的关节不动。"""
        start = dict(self.current)
        steps = max(1, int(round(move_sec * self.rate_hz)))
        for i in range(1, steps + 1):
            u = i / steps
            s = u * u * (3.0 - 2.0 * u)
            for j in JOINTS:
                if j in goal:
                    self.current[j] = start[j] + (goal[j] - start[j]) * s
            self.publish_state()
            time.sleep(self.period)

    def hold(self, seconds):
        end = self.get_clock().now().nanoseconds / 1e9 + seconds
        while self.get_clock().now().nanoseconds / 1e9 < end and rclpy.ok():
            self.publish_state()
            time.sleep(self.period)

    def run_case(self, case):
        cid = case.get("id", "unknown")
        step = case.get("step", "all")
        seed = int(case.get("seed", 0))
        move_sec = float(case.get("move_sec", 3.0))
        pause_sec = float(case.get("pause_sec", 5.0))

        stages = STAGES_ALL if step == "all" else [step]
        unknown = [s for s in stages if s not in STAGES_ALL]
        if unknown:
            self.get_logger().warn(f"[{cid}] 未知 step={unknown}，忽略")
            stages = []

        targets = self.sample_targets(seed)
        try:
            plan = self.plan(stages, targets)
        except UnsafeMotion as exc:
            raise UnsafeMotion(f"[{cid}] 预检失败，停止演示：{exc}") from exc
        keyframes = []

        # 第一帧明确发布已预检的起点，RViz 不必等到关节开始活动才显示模型。
        if plan:
            self.publish_state()
        for name, goal, ground, body in plan:
            self.get_logger().info(
                f"[{cid}] {name}：{move_sec:.2f}s，净空下界：地面 {ground:.3f} m，机身 {body:.3f} m"
            )
            self.move_to(goal, move_sec)
            if name == "reset":
                continue
            keyframes.append({"name": name, "joint_state": self.snapshot()})
            if name in PAUSE_STAGES:
                self.get_logger().info(f"[{cid}] {name} 完成，保持 {pause_sec:.1f}s")
                self.hold(pause_sec)

        result = {
            "id": cid,
            "step": step,
            "keyframes": keyframes,
            "final_joint_state": self.snapshot(),
        }
        self.get_logger().info(f"[{cid}] 完成，{len(keyframes)} 个关键帧")
        return result


def main():
    rclpy.init()
    node = Q1Drive()
    try:
        cases = node.read_cases()
        node.get_logger().info(f"读取 {len(cases)} 个测例：{node.input_file}")
        results = []
        for case in cases:
            if not rclpy.ok():
                break
            results.append(node.run_case(case))
            node.write_results(results)
        node.get_logger().info(f"输出 {len(results)} 行：{node.output_file}")
    except KeyboardInterrupt:
        pass
    except UnsafeMotion as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1) from None
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 信号处理已 shutdown 时忽略重复调用


if __name__ == "__main__":
    main()
