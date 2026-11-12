/**********************************************************************
 Stand / Down 状态切换 & 插值演示测试
 不依赖硬件, 验证 PD 输出和插值曲线
***********************************************************************/

#include "go1_fsm/state_machine.h"
#include <cstdio>
#include <cmath>

using namespace GO1_FSM;

static void printJointRow(const char* label, const std::array<float, NUM_JOINTS>& arr) {
    printf("  %-8s", label);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        printf("%+7.3f ", arr[i]);
    }
    printf("\n");
}

static void printState(const FSState& s, const BaseState* st) {
    printf("--- step=%lu time=%.3fs state=%-8s mode=0x%02X ---\n",
           s.motion_time, s.motion_time * s.control_dt,
           st ? st->getName().c_str() : "?", s.motor_mode);
    printJointRow("q_des", s.q_des);
    printJointRow("Kp   ", s.Kp);
    printJointRow("Kd   ", s.Kd);
    if (s.emergency_triggered) printf("  *** EMERGENCY TRIGGERED ***\n");
    printf("\n");
}

int main() {
    printf("\n========================================\n");
    printf("  Stand / Down 状态切换 & 插值演示\n");
    printf("========================================\n\n");

    StateMachine fsm;
    FSState state;
    state.control_dt = 0.002f;

    // 初始: Damping
    state.motion_time++;
    fsm.update(state);
    printf("[初始] Damping, 准备切换到 Stand...\n\n");

    // ============================================================
    // 阶段 1: Damping -> Stand (插值过渡)
    // ============================================================
    printf("========== 阶段 1: Damping -> Stand (平滑插值) ==========\n");
    fsm.requestTransition(FSMStateID::STAND, state);

    // 采样点: rate=0, 0.25, 0.5, 0.75, 1.0, 1.5
    int sample_points[] = {0, 125, 250, 375, 500, 750};
    int next_sample = 0;

    for (int i = 0; i < 800; ++i) {
        state.motion_time++;
        fsm.update(state);

        if (next_sample < 6 && i == sample_points[next_sample]) {
            float rate = (i < 500) ? (float)i / 500.0f : 1.0f;
            printf("  >>> rate=%.2f (step %d)\n", rate, i);
            printState(state, fsm.getCurrentState());
            next_sample++;
        }
    }
    printf("Stand 保持中, q_des[FR_1]=%.3f Kp[FR_1]=%.1f\n\n",
           state.q_des[FR_1], state.Kp[FR_1]);

    // ============================================================
    // 阶段 2: Stand -> Down (插值过渡)
    // ============================================================
    printf("========== 阶段 2: Stand -> Down (平滑插值) ==========\n");
    fsm.requestTransition(FSMStateID::DOWN, state);

    sample_points[0] = 0; sample_points[1] = 125;
    sample_points[2] = 250; sample_points[3] = 375;
    sample_points[4] = 500; sample_points[5] = 750;
    next_sample = 0;

    for (int i = 0; i < 800; ++i) {
        state.motion_time++;
        fsm.update(state);

        if (next_sample < 6 && i == sample_points[next_sample]) {
            float rate = (i < 500) ? (float)i / 500.0f : 1.0f;
            printf("  >>> rate=%.2f (step %d)\n", rate, i);
            printState(state, fsm.getCurrentState());
            next_sample++;
        }
    }
    printf("Down 保持中, q_des[FR_1]=%.3f Kp[FR_1]=%.1f\n\n",
           state.q_des[FR_1], state.Kp[FR_1]);

    // ============================================================
    // 阶段 3: Down -> Stand (来回切换)
    // ============================================================
    printf("========== 阶段 3: Down -> Stand (回到站立) ==========\n");
    fsm.requestTransition(FSMStateID::STAND, state);

    for (int i = 0; i < 600; ++i) {
        state.motion_time++;
        fsm.update(state);
    }
    printf("Stand 恢复, q_des[FR_1]=%.3f\n\n", state.q_des[FR_1]);

    // ============================================================
    // 阶段 4: 力矩安全层测试 (Stand 状态下模拟超限)
    // ============================================================
    printf("========== 阶段 4: 力矩安全层测试 ==========\n");
    printf("模拟关节力矩超限 (50 N.m > 限制 33 N.m)...\n");

    for (int i = 0; i < 15; ++i) {
        // 模拟力矩超限
        for (int j = 0; j < NUM_JOINTS; ++j) {
            state.tau_est[j] = 50.0f;
        }
        state.motion_time++;
        fsm.update(state);

        if (state.emergency_triggered) {
            printf("  *** 第 %d 步: emergency_triggered! 自动急停 ***\n", i);
            // 模拟 FSMController 的急停逻辑
            fsm.forceTransition(FSMStateID::DAMPING, state);
            state.emergency_triggered = false;
            break;
        }
    }
    printState(state, fsm.getCurrentState());

    // ============================================================
    printf("========================================\n");
    printf("  Stand/Down 测试完成!\n");
    printf("========================================\n\n");
    return 0;
}
