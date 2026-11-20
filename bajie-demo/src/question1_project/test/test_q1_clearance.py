"""在 ROS 2 下使用安装的官方八界模型验证题一净空，无须启动 RViz。"""
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
import xacro

from bajie_tasks.q1_drive import Q1Drive, STAGES_ALL
from bajie_tasks.q1_safety import BODY_CLEARANCE, GROUND_CLEARANCE, ClearanceModel, UnsafeMotion, ZERO


class Q1ClearanceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        share = Path(get_package_share_directory('bajie_robot'))
        model = xacro.process_file(str(share / 'urdf/bajie_robot.urdf.xacro')).toxml()
        cls.clearance = ClearanceModel(model, share)

    def test_legacy_seed_7_pose_hits_ground(self):
        pose = dict(ZERO, lift_joint=0.183626, pitch_joint=-0.244406, joint1=0.241495,
                    joint2=-1.142051, joint3=0.291147, joint4=-0.270880,
                    joint5=-1.159401, joint6=0.005949)
        with self.assertRaisesRegex(UnsafeMotion, '地面'):
            self.clearance.certify(pose, pose)

    def test_body_clearance_is_independent_of_ground(self):
        pose = dict(ZERO, joint1=-1.2, joint2=0.6)
        with self.assertRaisesRegex(UnsafeMotion, 'lift_link'):
            self.clearance.certify(pose, pose)

    def test_safe_endpoints_can_have_an_unsafe_middle(self):
        start = dict(ZERO, joint1=1.2, joint3=0.7)
        end = dict(ZERO, joint1=-1.1, joint2=-0.6, joint3=1.0)
        self.clearance.certify(start, start)
        self.clearance.certify(end, end)
        with self.assertRaisesRegex(UnsafeMotion, '地面'):
            self.clearance.certify(start, end)

    def test_seeded_stages_and_return_to_zero(self):
        for seed in range(128):
            with self.subTest(seed=seed):
                targets = Q1Drive.sample_targets(seed)
                self.assertEqual(targets, Q1Drive.sample_targets(seed))
                state = ZERO.copy()
                for stage in STAGES_ALL:
                    next_state = {**state, **Q1Drive.goal_for(stage, targets)}
                    ground, body = self.clearance.certify(state, next_state)
                    self.assertGreaterEqual(ground, GROUND_CLEARANCE)
                    self.assertGreaterEqual(body, BODY_CLEARANCE)
                    state = next_state
                self.clearance.certify(state, ZERO)


if __name__ == '__main__':
    unittest.main()
