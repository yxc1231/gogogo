// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state_mode, std::string state_string);
    
    // 声明析构函数（不在这里定义）
    virtual ~State_RLBase();
    
    void enter()
    {
        // set gain
        for (int i = 0; i < env->robot->data.joint_stiffness.size(); ++i)
        {
#ifndef GO1_SIM_MODE
            // RL sends position targets through the Go1 legacy MIT interface.
            lowcmd->msg_.motor_cmd()[i].mode() = 0x0A;
#endif
            lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
            lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
            lowcmd->msg_.motor_cmd()[i].dq() = 0;
            lowcmd->msg_.motor_cmd()[i].tau() = 0;
        }

        env->robot->update();
        // Start policy thread
        policy_thread_running = true;
        policy_thread = std::thread([this]{
            using clock = std::chrono::high_resolution_clock;
            const std::chrono::duration<double> desiredDuration(env->step_dt);
            const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

            auto sleepTill = clock::now() + dt;
            env->reset();

            while (policy_thread_running)
            {
                env->step();
                std::this_thread::sleep_until(sleepTill);
                sleepTill += dt;
            }
        });
    }

    void run();
    
    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }
    }

private:
    bool torque_limit_exceeded();

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::thread policy_thread;
    bool policy_thread_running = false;

    // These arrays use policy action order, not the Unitree SDK motor order.
    std::vector<float> joint_pos_min_;
    std::vector<float> joint_pos_max_;
    std::vector<float> torque_thresholds_;
};

// 将 REGISTER_FSM 移到文件末尾，类定义之后
REGISTER_FSM(State_RLBase)
