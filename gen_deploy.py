#!/usr/bin/env python3
"""自动为 Go1 训练日志目录生成 deploy.yaml"""
import sys, os, yaml

if len(sys.argv) < 2:
    print(f"用法: {sys.argv[0]} /path/to/log/2026-07-20_10-20-13")
    sys.exit(1)

log_dir = sys.argv[1]
exported = os.path.join(log_dir, "exported", "policy.onnx")
params_dir = os.path.join(log_dir, "params")

if not os.path.exists(exported):
    print(f"错误: 找不到 {exported}")
    sys.exit(1)

# 读取 ONNX 模型维度
import onnxruntime as ort
sess = ort.InferenceSession(exported)
obs_dim = sess.get_inputs()[0].shape[1]
act_dim = sess.get_outputs()[0].shape[1]

print(f"模型: obs={obs_dim}, act={act_dim}")

# 观测配置 (顺序必须与训练一致!)
obs_terms = [
    "base_ang_vel",
    "projected_gravity",
    "velocity_commands_safe",
    "joint_pos_rel_safe",
    "joint_vel_rel_safe",
    "last_action",
]

# 检查是否需要 height_scan
obs_dims = 3 + 3 + 3 + 12 + 12 + 12  # = 45 (平地)
if obs_dim == 232:
    obs_terms.append("height_scan")
    obs_dims = obs_dims + 187

assert obs_dims == obs_dim, f"观测维度 {obs_dims} != 模型期望 {obs_dim}"

deploy = {
    "step_dt": 0.02,
    "joint_ids_map": [0.0, 3.0, 6.0, 9.0, 1.0, 4.0, 7.0, 10.0, 2.0, 5.0, 8.0, 11.0],
    "default_joint_pos": [-0.1, 0.1, -0.1, 0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5],
    "stiffness": [30.0] * 12,
    "damping":    [1.0] * 12,
    "actions": {
        "JointPositionAction": {
            "joint_ids": None,
            "scale": [0.25] * 12,
            "offset": [-0.1, 0.1, -0.1, 0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5],
            "clip": None,
        }
    },
    "observations": {
        "obs": {t: {"scale": None, "clip": None, "params": None} for t in obs_terms}
    },
}

os.makedirs(params_dir, exist_ok=True)
out = os.path.join(params_dir, "deploy.yaml")
with open(out, "w") as f:
    yaml.dump(deploy, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

print(f"已生成: {out}")
print(f"观测: {obs_terms} ({obs_dim}维)")
