// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "isaaclab/manager/observation_manager.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/algorithms/algorithms.h"
#include <iostream>
#include "isaaclab/utils/utils.h"
#include <spdlog/spdlog.h>

namespace isaaclab
{

class ObservationManager;
class ActionManager;

class ManagerBasedRLEnv
{
public:
    // Constructor
    ManagerBasedRLEnv(YAML::Node cfg, std::shared_ptr<Articulation> robot_)
    :cfg(cfg), robot(std::move(robot_))
    {
        // Parse configuration
        this->step_dt = cfg["step_dt"].as<float>();
        robot->data.joint_ids_map = cfg["joint_ids_map"].as<std::vector<float>>();
        robot->data.joint_pos.resize(robot->data.joint_ids_map.size());
        robot->data.joint_vel.resize(robot->data.joint_ids_map.size());

        { // default joint positions
            auto default_joint_pos = cfg["default_joint_pos"].as<std::vector<float>>();
            robot->data.default_joint_pos = Eigen::VectorXf::Map(default_joint_pos.data(), default_joint_pos.size());
        }
        { // joint stiffness and damping
            robot->data.joint_stiffness = cfg["stiffness"].as<std::vector<float>>();
            robot->data.joint_damping = cfg["damping"].as<std::vector<float>>();
        }

        robot->update();

        // load managers
        action_manager = std::make_unique<ActionManager>(cfg["actions"], this);
        observation_manager = std::make_unique<ObservationManager>(cfg["observations"], this);
    }

    void reset()
    {
        global_phase = 0;
        episode_length = 0;
        robot->update();
        action_manager->reset();
        observation_manager->reset();
        if (alg) alg->reset();
    }

    void step()
    {
        episode_length += 1;
        robot->update();
        auto obs = observation_manager->compute();
        
        // 调试: 打印前3帧的完整观测值
        static int debug_cnt = 0;
        if (++debug_cnt <= 3) {
            auto& policy_obs = obs.begin()->second;
            spdlog::info("OBS[{}] dim={} total:", debug_cnt, policy_obs.size());
            spdlog::info("  cmd:      [{:.3f} {:.3f} {:.3f}]", policy_obs[0], policy_obs[1], policy_obs[2]);
            spdlog::info("  gyro:     [{:.3f} {:.3f} {:.3f}]", policy_obs[3], policy_obs[4], policy_obs[5]);
            spdlog::info("  gravity:  [{:.3f} {:.3f} {:.3f}]", policy_obs[6], policy_obs[7], policy_obs[8]);
            spdlog::info("  jpos_rel: [{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}]",
                policy_obs[9], policy_obs[10], policy_obs[11], policy_obs[12],
                policy_obs[13], policy_obs[14], policy_obs[15], policy_obs[16],
                policy_obs[17], policy_obs[18], policy_obs[19], policy_obs[20]);
            spdlog::info("  jvel_rel: [{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}]",
                policy_obs[21], policy_obs[22], policy_obs[23], policy_obs[24],
                policy_obs[25], policy_obs[26], policy_obs[27], policy_obs[28],
                policy_obs[29], policy_obs[30], policy_obs[31], policy_obs[32]);
            spdlog::info("  last_act: [{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}]",
                policy_obs[33], policy_obs[34], policy_obs[35], policy_obs[36],
                policy_obs[37], policy_obs[38], policy_obs[39], policy_obs[40],
                policy_obs[41], policy_obs[42], policy_obs[43], policy_obs[44]);
        }
        
        auto action = alg->act(obs);
        
        // 调试: 打印 raw action (ONNX 直接输出)
        if (debug_cnt <= 3) {
            spdlog::info("  raw_act:  [{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}]",
                action[0], action[1], action[2], action[3],
                action[4], action[5], action[6], action[7],
                action[8], action[9], action[10], action[11]);
        }
        
        action_manager->process_action(action);
    }

    float step_dt;
    
    YAML::Node cfg;

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    long episode_length = 0;
    float global_phase = 0.0f;
};

};
