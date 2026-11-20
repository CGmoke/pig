# Question 2 Forward Kinematics Implementation Plan

> **For agentic workers:** Verify each step with a failing-then-passing test and finish with an independent ROS 2 TF comparison.

**Goal:** Generate the official question 2 answer from the official URDF and the input JSONL.

**Architecture:** `fk.py` computes reusable rigid transforms and the model's base-to-gripper chain without a running ROS graph. `solve_q2.py` validates the contest protocol and writes answers. `verify_tf.py` independently verifies results using `robot_state_publisher` and TF2 under ROS 2 Humble.

**Tech Stack:** Python 3.10+, NumPy, ROS 2 Humble for model expansion and TF verification, unittest.

**Spec:** `docs/赛题.md` §2.3 and the confirmed independent computation + TF verification design.

## Global constraints

- Use official URDF geometry and joint origins unchanged; no simulator or GUI needed.
- Preserve the JSONL schema and case IDs; poses have `xyz`, `rpy`, and `[x,y,z,w]`.
- Keep the official input and question 1 unchanged; store synthetic verification inputs under tests or construct them in the verifier.

## Execution

1. Test the official zero pose, 0.1 m lift, off-chain joints, rotated-object reconstruction, and CLI JSONL/error behavior. Confirm tests fail before adding solver.
2. Implement `BajieChain.from_urdf()` / `forward()`, pose converters, `solve_case()` and CLI. Run unit tests and generate the official answer.
3. Use `robot_state_publisher`, `JointState` and TF2 for a separate reference for three poses. Publish the object as a TF frame to independently check both output transforms. Record errors in the terminal log.
4. Document how to rerun and learn from the lift check; confirm the answer matches the official example to displayed precision. Save answer and run log in the submission directory.
