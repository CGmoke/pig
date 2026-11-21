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

ROBOT = '/World/abr_machine/robot_root'
BASE = ROBOT + '/base/base_footprint'
HAND = ROBOT + '/gripper/objects_link'
FINGERS = tuple(f'{ROBOT}/gripper/gripper_{side}_link' for side in ('left', 'right'))
TOY_ROOT = '/World/company/toys/'
DT = 1 / 60


def log(message):
    print(f'[q3] {message}', flush=True)


def focus_viewport(stage, base):
    """Point the session camera at the robot when running with a window.

    Writes the camera transform directly instead of using the viewport command
    stack, which fails on stages whose up-axis differs from the Kit default.
    """
    try:
        from pxr import Gf, Usd, UsdGeom
        cam = stage.GetPrimAtPath('/OmniverseKit_Persp')
        if not cam:
            log('viewport camera setup skipped: /OmniverseKit_Persp not found')
            return
        center = base[:3, 3]
        eye = center + np.array([-2.2, -2.2, 1.6])
        target = center + np.array([0., 0., .5])
        view = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*eye.tolist()), Gf.Vec3d(*target.tolist()), Gf.Vec3d(0, 0, 1))
        xform = UsdGeom.Xformable(cam)
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            xform.ClearXformOpOrder()
            xform.MakeMatrixXform().Set(view.GetInverse())
        log(f'viewport focused at {np.round(center, 2).tolist()}')
    except Exception as exc:
        log(f'viewport camera setup skipped: {exc}')


def run_case(app, case, scene, planner, gui=False):
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
    toy_range = (UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
                 .ComputeWorldBound(stage.GetPrimAtPath(toy)).ComputeAlignedRange())
    toy_top = float(toy_range.GetMax()[2])
    # The task holds the mobile base at the input pose; this is an in-memory world anchor.
    anchor = UsdPhysics.FixedJoint.Define(stage, '/World/BajieQ3BaseAnchor')
    anchor.CreateBody1Rel().SetTargets([BASE])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(*case.base[:3, 3]))
    quat = Gf.Matrix4d(case.base.T.tolist()).ExtractRotationQuat()
    anchor.CreateLocalRot0Attr().Set(Gf.Quatf(quat.GetReal(), Gf.Vec3f(*quat.GetImaginary())))
    if gui:
        app.update()
        focus_viewport(stage, world(BASE))
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
        # GUI 的窗口与渲染是异步初始化的，多走几帧再取 physics view。
        for _ in range(30 if gui else 4):
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
        # 取玩具顶部与放置原点中较低者作为下探基准：不规则玩具（如恐龙）的最高点
        # 可能不在原点上方，只看包围盒顶部会停在半空。
        bottom = max(.004, min(toy_top, aim[2]) + planner.pad_down - .003)
        log(f'{case.id}: toy_top={toy_top:.4f} m aim={np.round(aim, 4).tolist()} '
            f'pad_down={planner.pad_down:.4f} m descent_bottom={bottom:.4f} m')
        waypoints = list(planner.descent(aim, bottom=bottom, step=.004))
        previous = init.copy()
        for n, arm in enumerate(waypoints):
            target = init.copy()
            target[2:8] = arm
            for positions in smooth_segment(previous, target, DT, speed=.60 if n == 0 else .25):
                command(positions)
                if contacts:
                    q = actual()
                    row = answer(case, dict(zip(JOINTS, q)), world(BASE), world(HAND), world(toy))
                    log(f'{case.id}: contact={contacts[0]} actual={np.round(q, 5).tolist()}')
                    pressed = 0
                    # 首次接触后再缓慢下压最多 3 mm，确认接触持续存在，避免只是擦边。
                    for press_arm in planner.press(q[2:8], depth=.003, step=.001):
                        target[2:8] = press_arm
                        command(target)
                        if not contacts:
                            break
                        q = actual()
                        row = answer(case, dict(zip(JOINTS, q)), world(BASE), world(HAND), world(toy))
                        pressed += 1
                    log(f'{case.id}: press_steps={pressed} '
                        f'hand_z={world(HAND)[2, 3]:.4f} toy_top={toy_top:.4f}')
                    return row
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
    p.add_argument('--gui', action='store_true', help='Open an Isaac Sim window and render the scene')
    args = p.parse_args()
    cases = load_cases(args.input)
    selected = [c for c in cases if args.case is None or args.case == c.name]
    planners = [Planner(args.urdf, c) for c in selected]
    if not args.scene.is_file():
        p.error('scene file not found')
    import scipy.optimize  # Load before starting Kit, to surface missing dependencies early.
    isaac = Path(os.environ.get('ISAAC_PATH', '/isaac-sim'))
    log(f"starting minimal Isaac Sim 4.5 (CPU PhysX, ROS 2, {'GUI' if args.gui else 'no viewport'})")
    from isaacsim import AppFramework
    kit = ((isaac / 'apps' / 'isaacsim.exp.base.kit') if args.gui
           else Path(__file__).with_name('physics.kit').resolve())
    argv = [str(kit), '--portable']
    if args.gui:
        # GPU 显存与容器内存都有限，降低渲染分辨率以容纳三例连续运行。
        argv += ['--/app/renderer/resolution/width=960',
                 '--/app/renderer/resolution/height=540']
    else:
        argv.insert(1, '--no-window')
    for folder in ('exts', 'extscache', 'extsPhysics', 'extsDeprecated'):
        argv.extend(('--ext-folder', str(isaac / folder)))
    app = AppFramework('bajie-q3', argv)
    try:
        if args.gui:
            import omni.kit.app
            omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
                'isaacsim.ros2.bridge', True)
            from omni.kit.viewport.window import ViewportWindow, get_viewport_window_instances
            windows = list(get_viewport_window_instances())
            if not windows:
                windows = [ViewportWindow('Viewport', width=1280, height=720)]
            for win in windows:
                win.visible = True
            for _ in range(5):
                app.update()
            log(f'viewport windows ready: {len(windows)}')
        import rclpy
        rclpy.init(args=[])
        try:
            rows = [run_case(app, c, args.scene.resolve(), planner, args.gui)
                    for c, planner in zip(selected, planners)]
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
