"""题二 JSONL 命令行入口；不需要启动 ROS。"""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from fk import BajieChain, finite_number, matrix_to_pose, pose_to_matrix, rigid_inverse

JOINT_NAMES = ('lift_joint', 'pitch_joint', *(f'joint{i}' for i in range(1, 7)),
               'arm_finger_joint1', 'arm_finger_joint2')
HAND_KEY = 'objects_link_in_base_footprint'
OBJECT_KEY = 'scene_object_in_objects_link'
INPUT_POSE_KEY = 'scene_object_in_base_footprint'


def solve_case(model, case):
    if not isinstance(case, dict) or not isinstance(case.get('id'), str) or not case['id'].strip():
        raise ValueError('测例必须是含非空字符串 id 的对象')
    joints = case.get('joint_state')
    if not isinstance(joints, dict):
        raise ValueError('joint_state 必须是关节名到数值的对象')
    for name in JOINT_NAMES:
        if name not in joints:
            raise ValueError(f'joint_state 缺少 {name}')
        finite_number(joints[name], f'joint_state.{name}')
    base_object = pose_to_matrix(case.get(INPUT_POSE_KEY))
    with np.errstate(over='raise', invalid='raise'):
        base_hand = model.forward(joints)
        hand_object = rigid_inverse(base_hand) @ base_object
    return {'id': case['id'], HAND_KEY: matrix_to_pose(base_hand),
            OBJECT_KEY: matrix_to_pose(hand_object)}


def read_cases(source):
    with source.open(encoding='utf-8') as stream:
        for number, line in enumerate(stream, 1):
            if line.strip():
                try:
                    yield number, json.loads(line)
                except ValueError as exc:
                    raise ValueError(f'{source}:{number}: {exc}') from exc


def solve_file(model, source, output):
    if source.resolve() == output.resolve():
        raise ValueError('输入和输出不能是同一个文件')
    # 先完成所有行的计算；出错时不改写已有答案，也不留下半份答案。
    rows = []
    for number, case in read_cases(source):
        try:
            result = solve_case(model, case)
            rows.append(json.dumps(result, ensure_ascii=False, allow_nan=False, separators=(',', ':')))
        except (ValueError, FloatingPointError) as exc:
            raise ValueError(f'{source}:{number}: {exc}') from exc
    if not rows:
        raise ValueError(f'{source}: 未找到任何测例')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent) as temporary:
        draft = Path(temporary) / output.name
        draft.write_text('\n'.join(rows) + '\n', encoding='utf-8')
        draft.replace(output)
    for row in rows:
        print(row)
    print(f'已计算 {len(rows)} 条测例；作答已写入 {output}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ('--urdf', '--input', '--output'):
        parser.add_argument(flag, required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        model = BajieChain.from_urdf(args.urdf)
        print('URDF:', args.urdf)
        print('关节链:', ' → '.join(joint.name for joint in model.joints))
        print('输入:', args.input)
        solve_file(model, args.input, args.output)
    except (ValueError, OSError, ET.ParseError, KeyError, AttributeError) as exc:
        print(f'题二运行失败：{exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
