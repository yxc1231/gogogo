// Copyright (c) 2026, Go1 RL Deployment.
// All rights reserved.

#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace isaaclab {

// Apply a rate limit to joystick commands so abrupt stick reversals do not
// become physically impossible velocity steps at the policy input.
REGISTER_OBSERVATION(velocity_commands_safe) {
    using clock = std::chrono::steady_clock;
    static std::array<float, 3> smoothed{0.0f, 0.0f, 0.0f};
    static clock::time_point last_update{};

    auto & joystick = env->robot->data.joystick;
    const auto ranges = env->cfg["commands"]["base_velocity"]["ranges"];
    const std::array<float, 3> target{
        std::clamp(joystick->ly(), ranges["lin_vel_x"][0].as<float>(), ranges["lin_vel_x"][1].as<float>()),
        std::clamp(-joystick->lx(), ranges["lin_vel_y"][0].as<float>(), ranges["lin_vel_y"][1].as<float>()),
        std::clamp(-joystick->rx(), ranges["ang_vel_z"][0].as<float>(), ranges["ang_vel_z"][1].as<float>())
    };

    const auto smoothing = param::config["FSM"]["Velocity"]["command_smoothing"];
    const float lin_accel_limit = smoothing["lin_accel_limit"].as<float>(1.5f);
    const float ang_accel_limit = smoothing["ang_accel_limit"].as<float>(3.0f);
    const float reset_timeout = smoothing["reset_timeout"].as<float>(0.1f);

    const auto now = clock::now();
    if (last_update.time_since_epoch().count() == 0 ||
        std::chrono::duration<float>(now - last_update).count() > reset_timeout) {
        smoothed = {0.0f, 0.0f, 0.0f};
    }
    last_update = now;

    const std::array<float, 3> max_delta{
        lin_accel_limit * env->step_dt,
        lin_accel_limit * env->step_dt,
        ang_accel_limit * env->step_dt
    };
    for (size_t i = 0; i < smoothed.size(); ++i) {
        const float delta = std::clamp(target[i] - smoothed[i], -max_delta[i], max_delta[i]);
        smoothed[i] += delta;
    }

    return std::vector<float>(smoothed.begin(), smoothed.end());
}

// 替代 joint_pos_rel: 安全版, 不访问 params (避免 yaml-cpp try-catch 缺陷)
REGISTER_OBSERVATION(joint_pos_rel_safe) {
    auto & asset = env->robot;
    std::vector<float> data(asset->data.joint_pos.size());
    for(size_t i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }
    return data;
}

// 替代 joint_vel_rel: 安全版, 不访问 params
REGISTER_OBSERVATION(joint_vel_rel_safe) {
    auto & asset = env->robot;
    auto & data = asset->data.joint_vel;
    return std::vector<float>(data.data(), data.data() + data.size());
}

// height_scan 观测: 仿真无地形扫描, 返回全零 (平面近似)
REGISTER_OBSERVATION(height_scan) {
    // 网格: resolution=0.1, size=[1.6, 1.0] → 17 × 11 = 187 个采样点
    static const int W = 17, H = 11;
    static const int N = W * H;  // 187
    // 平面地面: 所有高度扫描值为 0 (相对于基准高度)
    return std::vector<float>(N, 0.0f);
}

// 如需 gait_clock 观测，取消下方注释并调整参数
// REGISTER_OBSERVATION(gait_clock) {
//     static float phase_accum = 0.0f;
//     const float freq = 2.0f;
//     const float theta0 = 0.5f * 2.0f * M_PI;
//     const float theta1 = 0.0f;
//     const float theta2 = 0.0f;
//     
//     float dt = env->step_dt;
//     phase_accum += dt * freq;
//     phase_accum = fmodf(phase_accum, 1.0f);
//     if (phase_accum < 0.0f) phase_accum += 1.0f;
//     
//     float t_rad = 2.0f * M_PI * phase_accum;
//     float p_fr = fmodf(t_rad + theta1 + theta2, 2.0f * M_PI);
//     float p_fl = fmodf(t_rad + theta0 + theta2, 2.0f * M_PI);
//     float p_rr = fmodf(t_rad + theta0,        2.0f * M_PI);
//     float p_rl = fmodf(t_rad + theta1,        2.0f * M_PI);
//     if (p_fr < 0) p_fr += 2.0f * M_PI;
//     if (p_fl < 0) p_fl += 2.0f * M_PI;
//     if (p_rr < 0) p_rr += 2.0f * M_PI;
//     if (p_rl < 0) p_rl += 2.0f * M_PI;
//     
//     return {
//         sinf(p_fr), sinf(p_fl), sinf(p_rr), sinf(p_rl),
//         cosf(p_fr), cosf(p_fl), cosf(p_rr), cosf(p_rl)
//     };
// }

} // namespace isaaclab

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    const auto deploy_cfg = YAML::LoadFile(policy_dir / "params" / "deploy.yaml");
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_cfg,
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    if (deploy_cfg["use_encoder"].as<bool>(false)) {
        env->alg = std::make_unique<isaaclab::HimOrtRunner>(
            policy_dir / "exported" / "policy.onnx",
            policy_dir / "exported" / "encoder.onnx",
            deploy_cfg["history_length"].as<size_t>());
    } else {
        env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");
    }

    // Safety limits are deliberately expressed in policy action order.  Validate
    // their length once at startup so a future policy/config mismatch cannot
    // silently clamp the wrong joints.
    const auto safety_limits = cfg["safety_limits"];
    if (safety_limits) {
        joint_pos_min_ = safety_limits["joint_pos_min"].as<std::vector<float>>();
        joint_pos_max_ = safety_limits["joint_pos_max"].as<std::vector<float>>();
        const bool torque_protection = safety_limits["torque_protection"].as<bool>(true);
        if (torque_protection) {
            torque_thresholds_ = safety_limits["torque_thresholds"].as<std::vector<float>>();
        }

        const auto action_dim = env->robot->data.joint_ids_map.size();
        if (joint_pos_min_.size() != action_dim || joint_pos_max_.size() != action_dim ||
            (torque_protection && torque_thresholds_.size() != action_dim)) {
            throw std::runtime_error("safety_limits arrays must match the policy action dimension");
        }
        for (size_t i = 0; i < action_dim; ++i) {
            if (joint_pos_min_[i] >= joint_pos_max_[i]) {
                throw std::runtime_error("safety_limits contains a joint_pos_min >= joint_pos_max");
            }
            if (torque_protection && torque_thresholds_[i] <= 0.0f) {
                throw std::runtime_error("safety_limits torque_thresholds must be positive");
            }
        }
        if (torque_protection) {
            spdlog::info("RL safety enabled: joint targets clamped, hip/thigh torque trip = {:.1f} Nm, calf = {:.2f} Nm",
                         torque_thresholds_[0], torque_thresholds_[8]);
        } else {
            spdlog::warn("RL safety: torque protection disabled; joint target and orientation protections remain enabled");
        }
    }

    // 注册安全检查: 姿态角过大 → 切回 Passive
    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
    // 任一电机的实际估计扭矩超限 → 切回 Passive。
    this->registered_checks.emplace_back(
        std::make_pair(
            [this]()->bool{ return torque_limit_exceeded(); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    static std::array<bool, 12> clamp_reported{};
    for (int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        const float unclamped = action[i];
        action[i] = std::clamp(action[i], joint_pos_min_[i], joint_pos_max_[i]);
        if (action[i] != unclamped && !clamp_reported[i]) {
            spdlog::warn("RL safety: action[{}] clamped from {:.3f} to {:.3f} rad", i, unclamped, action[i]);
            clamp_reported[i] = true;
        } else if (action[i] == unclamped) {
            clamp_reported[i] = false;
        }
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }

    // 每秒打印一次 RL 状态 (50Hz policy × 50 = 1Hz)
    static int cnt = 0;
    if (++cnt % 50 == 0) {
        auto& d = env->robot->data;
        spdlog::info("RL | gravity=[{:.2f} {:.2f} {:.2f}]",
                     d.projected_gravity_b.x(), d.projected_gravity_b.y(), d.projected_gravity_b.z());
        spdlog::info("    q_actual  FR=[{:.3f} {:.3f} {:.3f}] FL=[{:.3f} {:.3f} {:.3f}] RR=[{:.3f} {:.3f} {:.3f}] RL=[{:.3f} {:.3f} {:.3f}]",
                     d.joint_pos[1], d.joint_pos[5], d.joint_pos[9],
                     d.joint_pos[0], d.joint_pos[4], d.joint_pos[8],
                     d.joint_pos[3], d.joint_pos[7], d.joint_pos[11],
                     d.joint_pos[2], d.joint_pos[6], d.joint_pos[10]);
        spdlog::info("    q_desired FR=[{:.3f} {:.3f} {:.3f}] FL=[{:.3f} {:.3f} {:.3f}] RR=[{:.3f} {:.3f} {:.3f}] RL=[{:.3f} {:.3f} {:.3f}]",
                     lowcmd->msg_.motor_cmd()[0].q(), lowcmd->msg_.motor_cmd()[1].q(), lowcmd->msg_.motor_cmd()[2].q(),
                     lowcmd->msg_.motor_cmd()[3].q(), lowcmd->msg_.motor_cmd()[4].q(), lowcmd->msg_.motor_cmd()[5].q(),
                     lowcmd->msg_.motor_cmd()[6].q(), lowcmd->msg_.motor_cmd()[7].q(), lowcmd->msg_.motor_cmd()[8].q(),
                     lowcmd->msg_.motor_cmd()[9].q(), lowcmd->msg_.motor_cmd()[10].q(), lowcmd->msg_.motor_cmd()[11].q());
    }
}

bool State_RLBase::torque_limit_exceeded()
{
    if (torque_thresholds_.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    const auto& joint_ids = env->robot->data.joint_ids_map;
    for (size_t i = 0; i < joint_ids.size(); ++i) {
        const auto motor_id = static_cast<size_t>(joint_ids[i]);
        const float torque = lowstate->msg_.motor_state()[motor_id].tau_est();
        const float torque_threshold = torque_thresholds_[i];
        if (!std::isfinite(torque) || std::fabs(torque) > torque_threshold) {
            spdlog::error(
                "RL safety: torque limit exceeded, action[{}] motor[{}] tau={:.2f} Nm (limit {:.2f} Nm)",
                i, motor_id, torque, torque_threshold);
            return true;
        }
    }
    return false;
}

State_RLBase::~State_RLBase() = default;
