// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include <algorithm>

class State_Passive : public FSMState
{
public:
    State_Passive(int state, std::string state_string = "Passive") 
    : FSMState(state, state_string) 
    {
        auto motor_mode = param::config["FSM"]["Passive"]["mode"];
        if(motor_mode.IsDefined())
        {
            auto values = motor_mode.as<std::vector<int>>();
            for(int i(0); i<values.size(); ++i)
            {
                lowcmd->msg_.motor_cmd()[i].mode() = values[i];
            }
        }

        release_duration_ = param::config["FSM"]["Passive"]["release_duration"].as<float>(0.0f);
        release_kp_ = param::config["FSM"]["Passive"]["release_kp"].as<std::vector<float>>();
    } 

    void enter()
    {
        // set gain
        static auto kd = param::config["FSM"]["Passive"]["kd"].as<std::vector<float>>();
        for(int i(0); i < kd.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
#ifndef GO1_SIM_MODE
            // Go1 legacy SDK: 0x00 is the hardware damping mode.
            motor.mode() = 0x00;
#endif
            // Capture the measured pose before changing gains.  This avoids a
            // position-command step when leaving FixStand or RL.
            hold_q_[i] = lowstate->msg_.motor_state()[i].q();
            motor.q() = hold_q_[i];
            motor.kp() = release_kp_[i];
            motor.kd() = kd[i];
            motor.dq() = 0;
            motor.tau() = 0;
        }
        release_t0_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
    }

    void run()
    {
        const float elapsed = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - release_t0_;
        const float remain = release_duration_ > 0.0f
            ? std::clamp(1.0f - elapsed / release_duration_, 0.0f, 1.0f)
            : 0.0f;
        for(int i(0); i < lowcmd->msg_.motor_cmd().size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            if (i < release_kp_.size()) {
                motor.kp() = release_kp_[i] * remain;
                motor.q() = remain > 0.0f ? hold_q_[i] : lowstate->msg_.motor_state()[i].q();
            } else {
                motor.q() = lowstate->msg_.motor_state()[i].q();
            }
        }
    }

private:
    std::vector<float> release_kp_;
    std::vector<float> hold_q_ = std::vector<float>(12, 0.0f);
    float release_duration_ = 0.0f;
    double release_t0_ = 0.0;
};

REGISTER_FSM(State_Passive)
