/**********************************************************************
 Copyright (c) 2026, Go1 FSM State Machine System. All rights reserved.
***********************************************************************/

#include "go1_fsm/states/damping_state.h"
#include <cstdio>

namespace GO1_FSM {

void DampingState::onEnter(FSState& state) {
    resetTiming();
    
    // 切换到阻尼模式 (所有电机自由回零)
    state.motor_mode = MOTOR_MODE_DAMPING;
    
    // 急停时清零所有控制量
    state.q_des.fill(0.0f);
    state.dq_des.fill(0.0f);
    state.tau_ff.fill(0.0f);
    state.Kp.fill(0.0f);
    state.Kd.fill(0.0f);
    
    printf("[FSM] Enter Damping state (EMERGENCY STOP)\n");
}

void DampingState::onUpdate(FSState& state) {
    advanceTiming(state.control_dt);
    
    // 阻尼模式下: 保持 mode=0x00, 所有控制量归零
    // 电机处于自由阻尼状态, 不施加任何主动力矩
    
    state.motor_mode = MOTOR_MODE_DAMPING;
    state.q_des.fill(0.0f);
    state.dq_des.fill(0.0f);
    state.tau_ff.fill(0.0f);
    state.Kp.fill(0.0f);
    state.Kd.fill(0.0f);
}

void DampingState::onExit(FSState& state) {
    printf("[FSM] Exit Damping state | elapsed=%.2fs steps=%lu\n",
           _elapsed_time, _step_count);
}

}  // namespace GO1_FSM
