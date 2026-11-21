#!/usr/bin/env python3
"""Run independent toy touches in Isaac Sim 4.5 with ROS 2 feedback."""
import argparse
import json
import os
from pathlib import Path
import time
import traceback

import numpy as np

from touch import (JOINTS, TOYS, Planner, ReturnFailure, TouchMiss, answer, check_touch,
                   classify_contact, execute_adaptive, execute_return, is_surface_contact,
                   load_cases, rigid_inverse, run_strategy, smooth_segment, write_answers)
from bridge import install_bridge

ROBOT = '/World/abr_machine/robot_root'
BASE = ROBOT + '/base/base_footprint'
HAND = ROBOT + '/gripper/objects_link'
FINGERS = tuple(f'{ROBOT}/gripper/gripper_{side}_link' for side in ('left', 'right'))
TOY_ROOT = '/World/company/toys/'
DT = 1 / 60


def log(message):
    print(f'[q3] {message}', flush=True)


def focus_viewport(stage, center, close_up=False):
    """Point the session camera at the robot or the measured contact point.

    Writes the camera transform directly instead of using the viewport command
    stack, which fails on stages whose up-axis differs from the Kit default.
    """
    try:
        from pxr import Gf, Usd, UsdGeom
        cam = stage.GetPrimAtPath('/OmniverseKit_Persp')
        if not cam:
            log('viewport camera setup skipped: /OmniverseKit_Persp not found')
            return
        eye = center + (np.array([-.28, -.22, .14]) if close_up
                        else np.array([-2.2, -2.2, 1.6]))
        target = center if close_up else center + np.array([0., 0., .5])
        view = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*eye.tolist()), Gf.Vec3d(*target.tolist()), Gf.Vec3d(0, 0, 1))
        xform = UsdGeom.Xformable(cam)
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            xform.ClearXformOpOrder()
            xform.MakeMatrixXform().Set(view.GetInverse())
        label = 'contact close-up' if close_up else 'viewport'
        log(f'{label} focused at {np.round(center, 4).tolist()}')
    except Exception as exc:
        log(f'viewport camera setup skipped: {exc}')


def collider_points(colliders, base, Usd, UsdGeom):
    """Sample authored collider geometry in robot base coordinates, not render bounds."""
    time_code = Usd.TimeCode.Default()
    transforms = UsdGeom.XformCache(time_code)
    to_base = rigid_inverse(base)
    arrays = []
    for prim in colliders:
        geometry = UsdGeom.Mesh(prim)
        if geometry:
            points = np.array(geometry.GetPointsAttr().Get(time_code), dtype=float)
            if points.size == 0:
                continue
        elif prim.IsA(UsdGeom.Cube):
            half = float(UsdGeom.Cube(prim).GetSizeAttr().Get(time_code)) / 2
            linear = np.linspace(-half, half, 9)
            points = np.stack(np.meshgrid(linear, linear, linear, indexing='ij'), axis=-1).reshape(-1, 3)
            points = points[np.any(np.abs(points) == half, axis=1)]
        else:
            continue
        pose = to_base @ np.asarray(transforms.GetLocalToWorldTransform(prim)).T
        arrays.append(points @ pose[:3, :3].T + pose[:3, 3])
    return np.vstack(arrays) if arrays else np.empty((0, 3))


def run_case(app, case, scene, planner, gui=False, strategy='baseline'):
    import carb
    import omni.graph.core as og
    import omni.kit.app
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
    log(f'{case.id}: strategy={strategy} opening {scene}')
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
    colliders = [p for p in Usd.PrimRange(stage.GetPrimAtPath(toy))
                 if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
    if not colliders:
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
        focus_viewport(stage, world(BASE)[:3, 3])
    robot_prims = Usd.PrimRange(stage.GetPrimAtPath(ROBOT))
    moving = set(FINGERS) | {str(p.GetPath()) for p in robot_prims if p.HasAPI(UsdPhysics.RigidBodyAPI)
                             and p.GetName() in ('gripper_link', *(f'arm{i}_link' for i in range(1, 7)))}
    for path in sorted({toy} | moving):
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr().Set(0.)
    contacts = []
    blocked = []
    target_near = False
    closest_positive_gap = float('inf')
    def received(headers, data):
        nonlocal closest_positive_gap, target_near
        for header in headers:
            if header.type not in (ContactEventType.CONTACT_FOUND, ContactEventType.CONTACT_PERSIST):
                continue
            a, b = (str(PhysicsSchemaTools.intToSdfPath(v)) for v in (header.actor0, header.actor1))
            kind = classify_contact(a, b, toy, FINGERS, moving)
            if kind is None:
                continue
            for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                point = data[i]
                impulse = float(np.linalg.norm(point.impulse))
                gap = float(point.separation)
                if kind == 'target':
                    target_near |= gap <= .001
                    if impulse > 1e-6 and 0. < gap:
                        closest_positive_gap = min(closest_positive_gap, gap)
                # Keep the existing conservative obstacle guard; only target
                # success requires the actual collision surfaces to meet.
                valid = (is_surface_contact(gap, impulse) if kind == 'target'
                         else impulse > 1e-6 and gap <= .001)
                if valid:
                    entries = contacts if kind == 'target' else blocked
                    entries.append((a, b, list(point.position), gap, impulse))
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
            nonlocal target_near
            msg.position = list(map(float, positions[:-1]))
            pub.publish(msg)
            contacts.clear()
            blocked.clear()
            target_near = False
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

        start_toy = world(toy)
        max_shift = 0.
        steps = 0
        blocked_steps = 0
        visited = [actual().copy()]

        def advance(positions):
            nonlocal max_shift, steps, blocked_steps
            command(positions)
            visited.append(actual().copy())
            steps += 1
            blocked_steps += bool(blocked)
            obj = world(toy)
            max_shift = max(max_shift, float(np.linalg.norm(obj[:3, 3] - start_toy[:3, 3])))
            return obj

        def report(outcome, phase, detail=''):
            positive_gap = (f'{1e3 * closest_positive_gap:.6f}'
                            if np.isfinite(closest_positive_gap) else 'none')
            log(f'{case.id}: strategy={strategy} outcome={outcome} phase={phase} '
                f'command_steps={steps} max_toy_shift_mm={1000 * max_shift:.3f} '
                f'non_target_steps={blocked_steps} closest_positive_gap_mm={positive_gap} {detail}')

        def return_home(row, hit):
            # Freeze the score snapshot before releasing. Timeline pause is
            # only for presentation; resume the same physics session to move.
            if gui:
                tl.pause()
                focus_viewport(stage, np.asarray(hit[2]), close_up=True)
                hold_until = time.monotonic() + 2.5
                while time.monotonic() < hold_until and omni.kit.app.get_app().is_running():
                    app.update()
                focus_viewport(stage, world(BASE)[:3, 3])
                tl.play()

            def step(positions):
                command(positions)
                return actual(), a.get_joint_velocities(joint_indices=ids), target_near, blocked

            log(f'{case.id}: return_home=start replay_samples={len(visited)}')
            try:
                count, error, speed = execute_return(init, visited, DT, step)
            except ReturnFailure as exc:
                log(f'{case.id}: return_home=failed reason={exc}')
                raise
            log(f'{case.id}: return_home=settled command_steps={count} '
                f'max_joint_error={error:.6f} max_joint_speed={speed:.6f}')
            return row

        if strategy == 'adaptive':
            phase = 'plan'
            try:
                base = world(BASE)
                points = collider_points(colliders, base, Usd, UsdGeom)
                label, route = planner.adaptive_waypoints(points, (rigid_inverse(base) @ start_toy)[:3, :3])
                log(f'{case.id}: profile={label} geometry_points={len(points)} '
                    f'open_rad={route[1].positions[-2]:.3f} limit_rad={route[-1].positions[-2]:.3f}')

                def observe(positions, current_phase):
                    nonlocal phase
                    if phase != current_phase:
                        phase = current_phase
                        log(f'{case.id}: adaptive phase={phase}')
                    obj = advance(positions)
                    if not check_touch(phase, contacts, blocked):
                        return None
                    q = actual()
                    hit = min(contacts, key=lambda p: p[3])
                    row = answer(case, dict(zip(JOINTS, q)), world(BASE), world(HAND), obj)
                    report('contact', phase, f'contact={hit} actual={np.round(q, 5).tolist()}')
                    return row, hit

                row, hit = execute_adaptive(init, route, DT, observe)
            except TouchMiss as exc:
                report('miss', phase, f'reason={exc} hand={world(HAND)[:3, 3].round(5).tolist()} '
                       f'toy={world(toy)[:3, 3].round(5).tolist()} actual={actual().round(5).tolist()}')
                raise
            return return_home(row, hit)

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
                obj = advance(positions)
                if contacts:
                    q = actual()
                    hit = min(contacts, key=lambda p: p[3])
                    row = answer(case, dict(zip(JOINTS, q)), world(BASE), world(HAND), obj)
                    log(f'{case.id}: contact={hit} actual={np.round(q, 5).tolist()}')
                    pressed = 0
                    # 首次接触后再缓慢下压最多 3 mm，确认接触持续存在，避免只是擦边。
                    for press_arm in planner.press(q[2:8], depth=.003, step=.001):
                        target[2:8] = press_arm
                        obj = advance(target)
                        if not contacts:
                            break
                        hit = min(contacts, key=lambda p: p[3])
                        q = actual()
                        row = answer(case, dict(zip(JOINTS, q)), world(BASE), world(HAND), obj)
                        pressed += 1
                    log(f'{case.id}: press_steps={pressed} '
                        f'hand_z={world(HAND)[2, 3]:.4f} toy_top={toy_top:.4f}')
                    report('contact', 'descent', f'contact={hit}')
                    return return_home(row, hit)
            if n == 0 or n % 4 == 0 or n == len(waypoints)-1:
                log(f'{case.id}: waypoint {n}/{len(waypoints)-1} command={np.round(arm, 3).tolist()} actual={np.round(actual()[2:8], 3).tolist()} hand={world(HAND)[:3, 3].round(4).tolist()} toy={world(toy)[:3,3].round(4).tolist()}')
            previous = target
        report('miss', 'descent', 'reason=no verified fingertip contact')
        raise TouchMiss(f'{case.id}: no verified fingertip contact')
    finally:
        # Leave the arm at the last physical pose until the next independent
        # test starts; calling stop here would reset it on screen.
        tl.pause()
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
    p.add_argument('--strategy', default='baseline', choices=('baseline', 'adaptive', 'adaptive-fallback'),
                   help='Compare adaptive light-close against the original descent')
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
            rows = []
            for case, planner in zip(selected, planners):
                def attempt(strategy):
                    # Each call closes the stage and restores input poses and joints.
                    return run_case(app, case, args.scene.resolve(), planner, args.gui, strategy)
                rows.append(run_strategy(args.strategy, attempt, lambda text: log(f'{case.id}: {text}')))
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
