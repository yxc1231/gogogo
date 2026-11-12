/**********************************************************************
 Copyright (c) 2026, Go1 FSM State Machine System. All rights reserved.
***********************************************************************/

#include "go1_fsm/states/stand_state.h"
#include "go1_fsm/utils/joint_mapper.h"
#include <cstdio>
#include <cmath>

namespace GO1_FSM {

void StandState::onEnter(FSState& state) {
    resetTiming();
    _torque_violation_count = 0;
    state.emergency_triggered = false;
    
    // 记录进入时的关节角度 (插值起点)
    for (int i = 0; i < NUM_JOINTS; ++i) {
        _q_init[i] = state.q[i];
    }
    
    // 位置控制模式: 固定增益, 只插值位置
    state.motor_mode = MOTOR_MODE_SERVO;
    state.Kp = _cfg.Kp;
    state.Kd = _cfg.Kd;
    state.dq_des.fill(0.0f);
    state.tau_ff = _cfg.tau_ff;
    
    printf("[FSM] Enter Stand | transition=%.1fs torque_limit=%.0f N.m\n",
           _cfg.transition_time, _cfg.torque_limit);
}

void StandState::onUpdate(FSState& state) {
    advanceTiming(state.control_dt);
    
    float rate = _elapsed_time / _cfg.transition_time;
    
    if (rate < 1.0f) {
        jointLinearInterpolation(_q_init, _cfg.target_q, state.q_des, rate);
    } else {
        state.q_des = _cfg.target_q;
    }
    
    // 安全层
    checkTorqueSafety(state);
}

void StandState::checkTorqueSafety(FSState& state) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (std::fabs(state.tau_est[i]) > _cfg.torque_limit) {
            _torque_violation_count++;
            if (_torque_violation_count >= _cfg.torque_violation_threshold) {
                printf("[FSM] TORQUE VIOLATION! joint=%s tau=%.2f > %.0f -> STOP\n",
                       jointIndexToName(i), state.tau_est[i], _cfg.torque_limit);
                state.emergency_triggered = true;
                return;
            }
        }
    }
    if (_torque_violation_count > 0) _torque_violation_count--;
}

void StandState::onExit(FSState& state) {
    printf("[FSM] Exit Stand state | elapsed=%.2fs steps=%lu\n",
           _elapsed_time, _step_count);
}

}  // namespace GO1_FSM
