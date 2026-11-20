# 题二 · 正向运动学与坐标变换

**当前状态：待实现。** 在本目录放置运行入口、运动学模块和测试，不把赛题中的示例输出作为正式作答。

- 输入：[`inputs/q2_demo_input.jsonl`](../../inputs/q2_demo_input.jsonl)；正式输出：`answers/q2_demo_output.jsonl`。
- 基于官方 URDF，沿 `base_footprint` 到 `objects_link` 的关节链计算夹爪位姿，再用逆变换求物体在夹爪坐标系中的位姿。
- 验收：先用零位示例，再用单关节与非零姿态和 ROS 2 TF 交叉核对。两个输出位姿都包含 `xyz`、`rpy` 和 `quat_xyzw`。
