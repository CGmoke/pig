#!/usr/bin/env python3
"""Run independent toy touches in Isaac Sim 4.5 with ROS 2 feedback."""
import argparse
import json
import os
from pathlib import Path
import time
import traceback

import numpy as np

from touch import ARM, JOINTS, TOYS, Planner, answer, contact_pair, load_cases, rigid_inverse, smooth_segment, write_answers
from bridge import install_bridge
from preview import Feed

ROBOT = '/World/abr_machine/robot_root'
BASE = ROBOT + '/base/base_footprint'
HAND = ROBOT + '/gripper/objects_link'
FINGERS = tuple(f'{ROBOT}/gripper/gripper_{side}_link' for side in ('left', 'right'))
TOY_ROOT = '/World/company/toys/'
DT = 1 / 60


def log(message):
    print(f'[q3] {message}', flush=True)


def run_case(app, case, scene, planner, preview=None):
    import carb
    import omni.graph.core as og
    import omni.timeline
    import omni.usd
    from omni.physx import get_physx_simulation_interface
    from omni.physx.bindings._physx import ContactEventType
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    import rclpy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState

    def world(path):
        m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
        m = m.RemoveScaleShear()
        return np.asarray(m).T.copy()

    def sample_preview():
        cache = UsdGeom.XformCache()

        def position(prim):
            return list(cache.GetLocalToWorldTransform(prim).ExtractTranslation())

        return dict(base=position(stage.GetPrimAtPath(BASE)),
                    arm=[position(p) for p in preview_arm],
                    hand=position(stage.GetPrimAtPath(HAND)),
                    fingers=[position(stage.GetPrimAtPath(p)) for p in FINGERS],
                    target=position(stage.GetPrimAtPath(toy)))

    def place(prim, desired):
        x = UsdGeom.Xformable(prim)
        old = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
        parent = np.eye(4) if x.GetResetXformStack() else np.asarray(
            UsdGeom.XformCache().GetParentToWorldTransform(prim)).T
        desired = desired.copy()
        desired[:3, :3] *= np.linalg.norm(old[:3, :3], axis=0)
        x.MakeMatrixXform().Set(Gf.Matrix4d((np.linalg.inv(parent) @ desired).T.tolist()))

    ctx = omni.usd.get_context()
    tl = omni.timeline.get_timeline_interface()
    tl.stop()
    if ctx.get_stage() is not None:
        ctx.close_stage()
        app.update()
    log(f'{case.id}: opening {scene}')
    if not ctx.open_stage(str(scene)):
        raise RuntimeError('Official scene failed to open')
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    # The scene can include optional camera/lidar and mobility graphs. The task uses only its own bridge.
    for path in [p.GetPath() for p in stage.Traverse() if p.GetTypeName() == "OmniGraph"]:
        stage.GetPrimAtPath(path).SetActive(False)
    toy = TOY_ROOT + TOYS[case.name]
    if preview:
        # Match link names below the robot to avoid depending on USD group layout.
        arm_root = stage.GetPrimAtPath(ROBOT)
        link_names = ('arm_base_link', *(f'arm{i}_link' for i in range(1, 7)))
        links = {p.GetName(): p for p in Usd.PrimRange(arm_root) if p.GetName() in link_names}
        if missing := set(link_names) - links.keys():
            raise RuntimeError(f'Missing preview links: {sorted(missing)}')
        preview_arm = [links[name] for name in link_names]
    for path in (toy, HAND, BASE, *FINGERS):
        if not stage.GetPrimAtPath(path):
            raise RuntimeError(f'Missing official scene prim: {path}')
    if not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(stage.GetPrimAtPath(toy))):
        raise RuntimeError(f'Missing toy colliders; install the complete official models/: {toy}')
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Scene):
            api = PhysxSchema.PhysxSceneAPI.Apply(p)
            api.CreateEnableGPUDynamicsAttr(False).Set(False)
            api.CreateBroadphaseTypeAttr('MBP').Set('MBP')
            api.CreateTimeStepsPerSecondAttr(60).Set(60)
    carb.settings.get_settings().set_bool('/app/player/useFixedTimeStepping', True)
    tl.set_target_framerate(60)
    tl.set_play_every_frame(True)
    assembly = '/World/abr_machine'
    place(stage.GetPrimAtPath(assembly), case.base @ rigid_inverse(world(BASE)) @ world(assembly))
    place(stage.GetPrimAtPath(toy), case.base @ case.target)
    # The task holds the mobile base at the input pose; this is an in-memory world anchor.
    anchor = UsdPhysics.FixedJoint.Define(stage, '/World/BajieQ3BaseAnchor')
    anchor.CreateBody1Rel().SetTargets([BASE])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(*case.base[:3, 3]))
    quat = Gf.Matrix4d(case.base.T.tolist()).ExtractRotationQuat()
    anchor.CreateLocalRot0Attr().Set(Gf.Quatf(quat.GetReal(), Gf.Vec3f(*quat.GetImaginary())))
    for path in (toy, *FINGERS):
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr().Set(0.)
    # Match a specific toy and a specific fingertip at the same physics step.
    contacts = []
    def received(headers, data):
        for header in headers:
            if header.type not in (ContactEventType.CONTACT_FOUND, ContactEventType.CONTACT_PERSIST):
                continue
            a, b = (str(PhysicsSchemaTools.intToSdfPath(v)) for v in (header.actor0, header.actor1))
            if contact_pair(a, b, toy, FINGERS):
                for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                    point = data[i]
                    if np.linalg.norm(point.impulse) > 1e-6 and point.separation <= .001:
                        contacts.append((a, b, list(point.position), float(point.separation)))
    subscription = get_physx_simulation_interface().subscribe_contact_report_events(received)
    install_bridge(og, BASE)
    node = rclpy.create_node('bajie_q3_touch')
    states = []
    clocks = []
    node.create_subscription(JointState, '/q3/joint_states', lambda msg: states.append(msg), 10)
    node.create_subscription(Clock, '/q3/clock', lambda msg: clocks.append(msg), 10)
    pub = node.create_publisher(JointState, '/q3/joint_command', 10)
    a = SingleArticulation(prim_path=BASE)
    tl.play()
    try:
        log(f'{case.id}: initializing PhysX and ROS bridge')
        for _ in range(4):
            app.update()
        log(f"{case.id}: physics warm start complete")
        a.initialize()
        ids = np.array([a.get_dof_index(j) for j in JOINTS])
        init = np.array(list(case.joints.values()), dtype=np.float32)
        a.set_joint_positions(init, joint_indices=ids)
        a.set_joint_velocities(np.zeros_like(init), joint_indices=ids)
        a.apply_action(ArticulationAction(joint_positions=init, joint_indices=ids))

        def actual():
            return a.get_joint_positions(joint_indices=ids)

        # Finger 2 is USD's mimic of finger 1; position commands only go to the driven joint.
        msg = JointState()
        msg.name = list(JOINTS[:-1])
        def command(positions):
            msg.position = list(map(float, positions[:-1]))
            pub.publish(msg)
            contacts.clear()
            app.update()
            rclpy.spin_once(node, timeout_sec=0)
            rclpy.spin_once(node, timeout_sec=0)
            if len(states) > 16:
                del states[:-8]
            if len(clocks) > 16:
                del clocks[:-8]

        deadline = time.monotonic() + 12
        while not (states and clocks and pub.get_subscription_count()):
            if time.monotonic() >= deadline:
                raise RuntimeError(f'ROS handshake failed: states={len(states)}, clocks={len(clocks)}, cmd_subscribers={pub.get_subscription_count()}')
            command(init)
            time.sleep(.005)
        log(f'{case.id}: ROS feedback ready, start target={np.round(init, 4).tolist()} actual={np.round(actual(), 4).tolist()} base={world(BASE)[:3, 3].tolist()} toy={world(toy)[:3, 3].tolist()}')
        if np.max(np.abs(actual() - init)) > .04:
            raise RuntimeError('Could not establish requested initial joint state')
        start_fk = planner.forward(actual()[2:8])
        actual_hand = rigid_inverse(world(BASE)) @ world(HAND)
        if np.linalg.norm(start_fk[:3, 3] - actual_hand[:3, 3]) > .035:
            raise RuntimeError(f'Official FK does not match sim: predicted={start_fk[:3,3]} actual={actual_hand[:3,3]}')

        aim = case.target[:3, 3].copy()
        # Aim one fingertip rather than the empty space between the open fingers.
        radial = aim[:2] / np.linalg.norm(aim[:2])
        aim[:2] += .023 * np.array([-radial[1], radial[0]])
        waypoints = list(planner.descent(aim, bottom=.036))
        if preview:
            preview.publish(case.id, case.name, sample_preview, force=True)
        previous = init.copy()
        for n, arm in enumerate(waypoints):
            target = init.copy()
            target[2:8] = arm
            for positions in smooth_segment(previous, target, DT, speed=.60 if n == 0 else .25):
                step_start = time.monotonic() if preview else 0
                command(positions)
                if contacts:
                    q = actual()
                    log(f'{case.id}: contact={contacts[0]} actual={np.round(q, 5).tolist()}')
                    row = answer(case, dict(zip(JOINTS, q)), world(BASE), world(HAND), world(toy))
                    if preview:
                        actor_a, actor_b, point, _ = contacts[0]
                        side = 'left' if FINGERS[0] in (actor_a, actor_b) else 'right'
                        preview.publish(case.id, case.name, sample_preview,
                                        contact={'point': point, 'finger': side})
                        time.sleep(.75)
                    return row
                if preview:
                    preview.publish(case.id, case.name, sample_preview)
                    time.sleep(max(0., DT - (time.monotonic() - step_start)))
            if n == 0 or n % 4 == 0 or n == len(waypoints)-1:
                log(f'{case.id}: waypoint {n}/{len(waypoints)-1} command={np.round(arm, 3).tolist()} actual={np.round(actual()[2:8], 3).tolist()} hand={world(HAND)[:3, 3].round(4).tolist()} toy={world(toy)[:3,3].round(4).tolist()}')
            previous = target
        raise RuntimeError(f'{case.id}: no verified fingertip contact')
    finally:
        tl.stop()
        subscription = None
        node.destroy_node()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('scene', 'urdf', 'input'):
        p.add_argument(f'--{name}', type=Path, required=True)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--output', type=Path, help='Write all three verified rows atomically')
    group.add_argument('--case', choices=list(TOYS), help='Verify one toy; never write the final answer')
    p.add_argument('--preview', type=Path, help='Sample live scene transforms into a local JSON feed')
    args = p.parse_args()
    preview = Feed(args.preview) if args.preview else None
    cases = load_cases(args.input)
    selected = [c for c in cases if args.case is None or args.case == c.name]
    planners = [Planner(args.urdf, c) for c in selected]
    if not args.scene.is_file():
        p.error('scene file not found')
    import scipy.optimize  # Load before starting Kit, to surface missing dependencies early.
    from isaacsim import AppFramework
    kit = str(Path(__file__).with_name('physics.kit').resolve())
    isaac = Path(os.environ.get('ISAAC_PATH', '/isaac-sim'))
    argv = [kit, '--no-window', '--portable']
    for folder in ('exts', 'extscache', 'extsPhysics', 'extsDeprecated'):
        argv.extend(('--ext-folder', str(isaac / folder)))
    log('starting minimal Isaac Sim 4.5 (CPU PhysX, ROS 2, no viewport)')
    app = AppFramework('bajie-q3', argv)
    try:
        import rclpy
        rclpy.init(args=[])
        try:
            rows = [run_case(app, c, args.scene.resolve(), planner, preview) for c, planner in zip(selected, planners)]
        finally:
            rclpy.shutdown()
        if args.output:
            write_answers(args.output, cases, rows)
            log(f'wrote three verified contacts: {args.output}')
        else:
            log('single-case verified (no final answer written): ' + json.dumps(rows[0], allow_nan=False))
    except BaseException:
        traceback.print_exc()
        # Kit shutdown may exit the process itself; keep failures nonzero.
        os._exit(1)
    else:
        app.close()


if __name__ == '__main__':
    main()
