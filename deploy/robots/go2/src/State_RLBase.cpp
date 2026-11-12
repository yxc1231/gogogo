#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <cmath>

// 只添加这个： gait_clock 注册到 isaaclab 命名空间
namespace isaaclab {
// REGISTER_OBSERVATION(gait_clock) {
//     // 防御性：每个环境应有独立相位，但单机器人可用 static
//     static float phase_accum = 0.0f;
//     const float freq = 2.0f;  // Hz, 与 Python 一致
    
//     // Theta 必须转为弧度：0.5 周期 = π 弧度
//     const float theta0 = 0.5f * 2.0f * M_PI;  // π
//     const float theta1 = 0.0f;
//     const float theta2 = 0.0f;
    
//     float dt = env->step_dt;
//     phase_accum += dt * freq;
//     // 使用 fmod 处理可能的大 dt 跳变，确保在 [0, 1) 区间
//     phase_accum = fmodf(phase_accum, 1.0f);
//     if (phase_accum < 0.0f) phase_accum += 1.0f;  // 防御负数
    
//     float t_rad = 2.0f * M_PI * phase_accum;  // 当前相位对应的弧度 [0, 2π)
    
//     // 四条腿相位计算（与 Python 逻辑一一对应）
//     // FR: t + theta1 + theta2
//     // FL: t + theta0 + theta2
//     // RR: t + theta0
//     // RL: t + theta1
//     float p_fr = fmodf(t_rad + theta1 + theta2, 2.0f * M_PI);
//     float p_fl = fmodf(t_rad + theta0 + theta2, 2.0f * M_PI);
//     float p_rr = fmodf(t_rad + theta0,        2.0f * M_PI);
//     float p_rl = fmodf(t_rad + theta1,        2.0f * M_PI);
    
//     // 确保正数（fmod 可能返回负值）
//     if (p_fr < 0) p_fr += 2.0f * M_PI;
//     if (p_fl < 0) p_fl += 2.0f * M_PI;
//     if (p_rr < 0) p_rr += 2.0f * M_PI;
//     if (p_rl < 0) p_rl += 2.0f * M_PI;
    
//     return {
//         sinf(p_fr), sinf(p_fl), sinf(p_rr), sinf(p_rl),
//         cosf(p_fr), cosf(p_fl), cosf(p_rr), cosf(p_rl)
//     };
// }
// }

} // namespace isaaclab

// 下面是官方原始代码，完全不变
State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}

State_RLBase::~State_RLBase() = default;