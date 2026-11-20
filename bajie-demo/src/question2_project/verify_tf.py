"""以 ROS 2 robot_state_publisher + TF2 独立复核题二的两组输出。"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from fk import BajieChain
from solve_q2 import HAND_KEY, INPUT_POSE_KEY, JOINT_NAMES, OBJECT_KEY, read_cases, solve_case

# 给复核节点、关节话题和 TF 帧各自使用独立名字，避免读到工作区里其他演示的位姿。
JOINT_TOPIC = '/q2_verify/joint_states'
BASE = 'q2_verify/base_footprint'
HAND = 'q2_verify/objects_link'
OBJECT = 'q2_verify/scene_object'
POSITION_TOLERANCE_M = 1e-8
ROTATION_TOLERANCE_RAD = 1e-7


def verification_cases(example):
    yield example
    lifted = deepcopy(example)
    lifted['id'] += '_lift_0p1m'
    lifted['joint_state']['lift_joint'] += 0.1
    yield lifted
    bent = deepcopy(example)
    bent['id'] += '_bent_arm_rotated_object'
    bent['joint_state'].update(lift_joint=0.12, pitch_joint=0.3, joint1=0.4, joint2=0.5,
                               joint3=-0.2, joint4=0.3, joint5=-0.4, joint6=0.15,
                               arm_finger_joint1=0.2, arm_finger_joint2=0.2)
    bent[INPUT_POSE_KEY]['rpy'] = [0.2, -0.3, 0.5]
    yield bent


def quat_product(a, b):
    x, y, z, w = a
    u, v, t, s = b
    return (w*u + x*s + y*t - z*v, w*v - x*t + y*s + z*u,
            w*t + x*v - y*u + z*s, w*s - x*u - y*v - z*t)


def broadcast_object(broadcaster, stamp, pose):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = BASE
    transform.child_frame_id = OBJECT
    roll, pitch, yaw = (angle / 2 for angle in pose['rpy'])
    # 直接按 Z、Y、X 轴四元数复合，不调用求解器的矩阵/位姿转换。
    qx = (math.sin(roll), 0, 0, math.cos(roll))
    qy = (0, math.sin(pitch), 0, math.cos(pitch))
    qz = (0, 0, math.sin(yaw), math.cos(yaw))
    translation, rotation = transform.transform.translation, transform.transform.rotation
    translation.x, translation.y, translation.z = pose['xyz']
    rotation.x, rotation.y, rotation.z, rotation.w = quat_product(quat_product(qz, qy), qx)
    broadcaster.sendTransform(transform)


def read_reference(node, publisher, broadcaster, buffer, rsp, case):
    deadline = time.monotonic() + 8.0
    last_error = '没有收到 robot_state_publisher 的关节订阅'
    states = JointState()
    states.name = list(JOINT_NAMES)
    states.position = [float(case['joint_state'][name]) for name in JOINT_NAMES]
    while time.monotonic() < deadline:
        if rsp.poll() is not None:
            raise RuntimeError(f'robot_state_publisher 提前退出（状态 {rsp.returncode}）')
        if publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
            continue
        stamp = node.get_clock().now()
        states.header.stamp = stamp.to_msg()
        publisher.publish(states)
        broadcast_object(broadcaster, states.header.stamp, case[INPUT_POSE_KEY])
        # 要求同一时间戳，不能误拿上一用例遗留的最新 TF。
        sample_deadline = min(time.monotonic() + 0.3, deadline)
        while time.monotonic() < sample_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            try:
                return {HAND_KEY: buffer.lookup_transform(BASE, HAND, stamp).transform,
                        OBJECT_KEY: buffer.lookup_transform(HAND, OBJECT, stamp).transform}
            except TransformException as exc:
                last_error = str(exc)
    raise RuntimeError(f'{case["id"]} 未收到配套 TF：{last_error}')


def errors(pose, reference):
    p, q = reference.translation, reference.rotation
    position = math.dist(pose['xyz'], (p.x, p.y, p.z))
    delta = quat_product((-q.x, -q.y, -q.z, q.w), pose['quat_xyzw'])
    # 绝对值忽略 q 和 -q 的符号差，atan2 对接近零的小角度也稳定。
    angle = 2 * math.atan2(math.hypot(*delta[:3]), abs(delta[3]))
    return position, angle


def verify(model, cases, rsp):
    rclpy.init(args=[])
    node = rclpy.create_node('q2_tf_check')
    try:
        publisher = node.create_publisher(JointState, JOINT_TOPIC, 10)
        broadcaster = TransformBroadcaster(node)
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        count = 0
        for case in cases:
            calculated = solve_case(model, case)
            references = read_reference(node, publisher, broadcaster, buffer, rsp, case)
            for key, reference in references.items():
                position, angle = errors(calculated[key], reference)
                print(f'{case["id"]} {key}: 位置误差 {position:.3e} m，旋转误差 {angle:.3e} rad',
                      flush=True)
                if position > POSITION_TOLERANCE_M or angle > ROTATION_TOLERANCE_RAD:
                    raise RuntimeError(f'{case["id"]} 的 {key} 超过误差阈值')
            count += 1
        print(f'通过：{count} 个测例，每例两组位姿；阈值 '
              f'{POSITION_TOLERANCE_M:g} m / {ROTATION_TOLERANCE_RAD:g} rad', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', required=True, type=Path)
    parser.add_argument('--input', required=True, type=Path, help='题面零位示例 JSONL')
    args = parser.parse_args()
    try:
        model = BajieChain.from_urdf(args.urdf)
        example = next((case for _, case in read_cases(args.input) if case.get('id') == 'demo_zero'), None)
        if example is None:
            raise ValueError('输入中找不到正式零位示例 demo_zero')
        binary = Path(get_package_prefix('robot_state_publisher')) / 'lib/robot_state_publisher/robot_state_publisher'
        with tempfile.TemporaryDirectory(prefix='bajie-q2-tf-') as directory:
            params = Path(directory) / 'rsp.yaml'
            urdf = json.dumps(args.urdf.read_text(encoding='utf-8'), ensure_ascii=False)
            params.write_text(f'/**:\n  ros__parameters:\n    robot_description: {urdf}\n'
                              '    frame_prefix: "q2_verify/"\n    publish_frequency: 100.0\n', encoding='utf-8')
            with subprocess.Popen([str(binary), '--ros-args', '--params-file', str(params),
                                   '-r', '__node:=q2_rsp', '-r', f'joint_states:={JOINT_TOPIC}']) as rsp:
                try:
                    verify(model, verification_cases(example), rsp)
                finally:
                    if rsp.poll() is None:
                        rsp.terminate()
                        try:
                            rsp.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            rsp.kill()
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'TF 复核失败：{exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
