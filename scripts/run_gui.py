#!/usr/bin/env python3
"""GUI 启动器：开窗口 → 显式打开官方场景 → 自动播放时间轴。

为什么不能直接把 usd 路径丢给 isaac-sim.sh：
  实测（Isaac Sim 4.5）命令行位置参数不会触发 open_stage，Kit 只加载默认的
  World0.usd 空场景，窗口是开了但里面空无一物，也不播放——官方 ROS2 图全挂在
  OnPlaybackTick 上，不播放就一个包都不发。

为什么播放要等：
  open_stage() 返回时 stage 还在异步加载，紧接着调 play() 会被丢掉
  （实测 is_playing() 直接回 False）。所以先用 get_stage_loading_status()
  等 Load 阶段走完，再 play，并重试确认。

用法（容器内，已 source humble）：
  /isaac-sim/python.sh /workspace/scripts/run_gui.py \
      --scene /workspace/src/bajie_sim/scene/abr_scene.usd
"""
import argparse
import sys
import traceback

parser = argparse.ArgumentParser()
parser.add_argument("--scene", default="/workspace/src/bajie_sim/scene/abr_scene.usd")
parser.add_argument("--width", type=int, default=1600)
parser.add_argument("--height", type=int, default=900)
parser.add_argument("--no-play", action="store_true", help="只打开不播放，便于先看模型")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({
    "headless": False,
    "width": args.width,
    "height": args.height,
})

import omni.usd        # noqa: E402
import omni.timeline   # noqa: E402
from pxr import UsdPhysics  # noqa: E402


def log(msg):
    print(f"[run_gui] {msg}", flush=True)


def wait_stage_loaded(timeout_frames=900):
    """等 stage 异步加载完成；API 不可用时安静跳过，不阻断主流程。"""
    ctx = omni.usd.get_context()
    for _ in range(timeout_frames):
        try:
            _status, loaded, total = ctx.get_stage_loading_status()
        except Exception:
            return  # 拿不到状态就不等，交给后面的 play 重试兜底
        if total > 0 and loaded >= total:
            return
        sim_app.update()


def main():
    log(f"打开场景: {args.scene}")
    ctx = omni.usd.get_context()
    if not ctx.open_stage(args.scene):
        log(f"!! 打开失败: {args.scene}")
        return 1

    stage = ctx.get_stage()
    dp = stage.GetDefaultPrim()
    log(f"默认 Prim = {dp.GetPath() if dp else '(无)'}")

    joints = sum(1 for p in stage.Traverse()
                 if p.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint",
                                        "PhysicsFixedJoint", "PhysicsJoint"))
    arts = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI))
    log(f"Articulation Root = {arts}, 关节 = {joints}")

    if args.no_play:
        log("跳过播放（--no-play）")
        return 0

    wait_stage_loaded()

    timeline = omni.timeline.get_timeline_interface()
    playing = False
    for _ in range(20):
        timeline.play()
        sim_app.update()
        playing = timeline.is_playing()
        if playing:
            break
    log(f"时间轴播放中 = {playing}")

    # 用仿真时钟推进做交叉验证：标志位为真但时钟不动 = 物理没跑
    try:
        t0 = timeline.get_current_time()
        for _ in range(30):
            sim_app.update()
        t1 = timeline.get_current_time()
        log(f"仿真时间 {t0:.3f} → {t1:.3f}")
        if playing and t1 <= t0:
            log("!! 标志位为真但时钟没推进")
    except Exception as exc:
        log(f"时钟采样跳过（{exc}）")

    return 0


try:
    rc = main()
except Exception:
    log("!! 初始化异常：")
    traceback.print_exc()
    rc = 1

if rc == 0:
    log("进入窗口循环（关闭窗口即退出）")
    while sim_app.is_running():
        sim_app.update()
else:
    log("初始化失败，退出")

sim_app.close()
sys.exit(rc)
