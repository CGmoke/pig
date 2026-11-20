# 题三 · Isaac Sim 玩具触碰

**当前状态：桥接诊断可复用，三例触碰流程待实现。** 题三代码、场景连接配置和测试放在本目录，不复制官方大体积场景资源。

从作品根目录执行。先准备 Isaac Sim 4.5、ROS 2 Humble 及可用的 ROS 2 Bridge，将路径改成自己的安装路径：

```bash
export BAJIE_SIM_DIR=/你的场景目录/ecovacs-bajie-sim
export ISAAC_SIM_DIR=/你的安装目录/isaac-sim
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
set -o pipefail
"$ISAAC_SIM_DIR/python.sh" src/question3_project/bridge_probe.py \
  --scene "$BAJIE_SIM_DIR/scene/abr_scene.usd" 2>&1 | tee assets/q3_bridge_probe.log
```

探针仅检查是否同时收到 `/clock` 和 `/joint_states`，不会发送触碰动作。题一与诊断建议分别运行，避免在同一 ROS 2 域混用两个 `/joint_states` 来源。

正式任务将逐行处理 [`inputs/q3_demo_input.jsonl`](../../inputs/q3_demo_input.jsonl)，在实际触碰时读取反馈，生成 `answers/q3_demo_output.jsonl`。三例完成前不创建该作答文件。
