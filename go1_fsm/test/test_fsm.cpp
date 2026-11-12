/**********************************************************************
 Copyright (c) 2026, Go1 FSM State Machine System. All rights reserved.
***********************************************************************/

#include "go1_fsm/state_machine.h"
#include "go1_fsm/fsm_state.h"
#include <cstdio>
#include <cstring>
#include <cassert>

using namespace GO1_FSM;

// ============================================================
// 测试夹具: 初始化 StateMachine + 模拟 FSState
// ============================================================
static int g_passed = 0;
static int g_failed = 0;

#define TEST(name) \
    printf("\n  === %s ===\n", name);

#define CHECK(cond, msg) do { \
    if (cond) { \
        printf("    [PASS] %s\n", msg); \
        g_passed++; \
    } else { \
        printf("    [FAIL] %s  (line %d)\n", msg, __LINE__); \
        g_failed++; \
    } \
} while(0)

// 模拟一个控制周期 (推进状态机)
static void tick(StateMachine& fsm, FSState& state) {
    state.motion_time++;
    state.control_dt = 0.002f;
    // 模拟一些关节角度 (非零, 使插值可见)
    for (int i = 0; i < NUM_JOINTS; ++i) {
        state.q[i]  = 0.1f * (i % 3);     // 髋≈0, 大腿≈0.1, 小腿≈0.2
        state.dq[i] = 0.0f;
    }
    fsm.update(state);
}

// ============================================================
int main() {
    printf("\n");
    printf("========================================\n");
    printf("  Go1 FSM 状态机单元测试\n");
    printf("========================================\n");

    // ---- 测试 1: 初始状态应为 Damping ----
    TEST("初始状态: 应自动进入 Damping")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        CHECK(fsm.isInState(FSMStateID::DAMPING), "初始状态 = DAMPING");
    }

    // ---- 测试 2: Damping -> Stand (合法) ----
    TEST("Damping -> Stand (合法)")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        bool ok = fsm.requestTransition(FSMStateID::STAND, state);
        CHECK(ok, "requestTransition 返回 true");
        CHECK(fsm.isInState(FSMStateID::STAND), "当前状态 = STAND");
    }

    // ---- 测试 3: Stand -> Down (合法) ----
    TEST("Stand -> Down (合法)")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        fsm.requestTransition(FSMStateID::STAND, state);
        bool ok = fsm.requestTransition(FSMStateID::DOWN, state);
        CHECK(ok, "requestTransition 返回 true");
        CHECK(fsm.isInState(FSMStateID::DOWN), "当前状态 = DOWN");
    }

    // ---- 测试 4: Down -> Stand (合法) ----
    TEST("Down -> Stand (合法, 双向)")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        fsm.requestTransition(FSMStateID::STAND, state);
        fsm.requestTransition(FSMStateID::DOWN, state);
        bool ok = fsm.requestTransition(FSMStateID::STAND, state);
        CHECK(ok, "Down -> Stand 返回 true");
        CHECK(fsm.isInState(FSMStateID::STAND), "当前状态 = STAND");
    }

    // ---- 测试 5: Stand -> Model (合法, 唯一通路) ----
    TEST("Stand -> Model (唯一合法通路)")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        fsm.requestTransition(FSMStateID::STAND, state);
        bool ok = fsm.requestTransition(FSMStateID::MODEL, state);
        CHECK(ok, "Stand -> Model 返回 true");
        CHECK(fsm.isInState(FSMStateID::MODEL), "当前状态 = MODEL");
    }

    // ---- 测试 6: Model -> Stand (合法) ----
    TEST("Model -> Stand (合法)")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        fsm.requestTransition(FSMStateID::STAND, state);
        fsm.requestTransition(FSMStateID::MODEL, state);
        bool ok = fsm.requestTransition(FSMStateID::STAND, state);
        CHECK(ok, "Model -> Stand 返回 true");
        CHECK(fsm.isInState(FSMStateID::STAND), "当前状态 = STAND");
    }

    // ---- 测试 7: Down -> Model (非法! 必须被拒绝) ----
    TEST("Down -> Model (非法, 必须拒绝)")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        fsm.requestTransition(FSMStateID::STAND, state);
        fsm.requestTransition(FSMStateID::DOWN, state);
        // 此时在 Down, 尝试非法跳转到 Model
        bool ok = fsm.requestTransition(FSMStateID::MODEL, state);
        CHECK(!ok, "Down -> Model 返回 false (拒绝)");
        CHECK(fsm.isInState(FSMStateID::DOWN), "当前状态仍为 DOWN (未变)");
    }

    // ---- 测试 8: Damping -> Model (非法! 必须被拒绝) ----
    TEST("Damping -> Model (非法, 必须拒绝)")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        // 在 Damping, 尝试直接跳 Model
        bool ok = fsm.requestTransition(FSMStateID::MODEL, state);
        CHECK(!ok, "Damping -> Model 返回 false (拒绝)");
        CHECK(fsm.isInState(FSMStateID::DAMPING), "当前状态仍为 DAMPING (未变)");
    }

    // ---- 测试 9: 任意状态 -> Damping (急停, 始终合法) ----
    TEST("任意 -> Damping (急停, 始终合法)")
    {
        // 9a: Stand -> Damping
        {
            StateMachine fsm;
            FSState state;
            tick(fsm, state);
            fsm.requestTransition(FSMStateID::STAND, state);
            bool ok = fsm.requestTransition(FSMStateID::DAMPING, state);
            CHECK(ok, "Stand -> Damping 返回 true");
            CHECK(fsm.isInState(FSMStateID::DAMPING), "当前状态 = DAMPING (来自 Stand)");
        }
        // 9b: Down -> Damping
        {
            StateMachine fsm;
            FSState state;
            tick(fsm, state);
            fsm.requestTransition(FSMStateID::STAND, state);
            fsm.requestTransition(FSMStateID::DOWN, state);
            bool ok = fsm.requestTransition(FSMStateID::DAMPING, state);
            CHECK(ok, "Down -> Damping 返回 true");
            CHECK(fsm.isInState(FSMStateID::DAMPING), "当前状态 = DAMPING (来自 Down)");
        }
        // 9c: Model -> Damping
        {
            StateMachine fsm;
            FSState state;
            tick(fsm, state);
            fsm.requestTransition(FSMStateID::STAND, state);
            fsm.requestTransition(FSMStateID::MODEL, state);
            bool ok = fsm.requestTransition(FSMStateID::DAMPING, state);
            CHECK(ok, "Model -> Damping 返回 true");
            CHECK(fsm.isInState(FSMStateID::DAMPING), "当前状态 = DAMPING (来自 Model)");
        }
    }

    // ---- 测试 10: 同状态切换 (无操作, 应返回 true) ----
    TEST("同状态请求 (无操作)")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        fsm.requestTransition(FSMStateID::STAND, state);
        // 再次请求 Stand (已经在 Stand)
        bool ok = fsm.requestTransition(FSMStateID::STAND, state);
        CHECK(ok, "Stand -> Stand 返回 true (no-op)");
        CHECK(fsm.isInState(FSMStateID::STAND), "当前状态仍为 STAND");
    }

    // ---- 测试 11: 插值行为验证 (Stand 状态) ----
    TEST("Stand 插值行为: onEnter 记录初始 q, 逐步过渡到 target_q")
    {
        StateMachine fsm;
        FSState state;
        
        state.control_dt = 0.002f;
        
        // 初始 Damping
        fsm.update(state);  // 不进 tick, 直接 update (q=0)
        CHECK(fsm.isInState(FSMStateID::DAMPING), "初始 = DAMPING");
        CHECK(state.q_des[0] == 0.0f, "Damping 下 q_des 归零");
        
        // 手动设置关节角度 (模拟悬挂姿态), 再切 Stand
        for (int i = 0; i < NUM_JOINTS; ++i) {
            state.q[i] = 0.5f;
        }
        
        // 切换时 onEnter 记录 _q_init = 0.5
        fsm.requestTransition(FSMStateID::STAND, state);
        CHECK(fsm.isInState(FSMStateID::STAND), "切换到 STAND");
        
        // 刚进入 Stand 后第一帧 (不修改 q)
        state.motion_time++;
        fsm.update(state);
        // _q_init 应该仍是 0.5, target_q[FR_0]=0.0, rate 极小
        // q_des[FR_0] 应接近 0.5
        CHECK(state.q_des[FR_0] > 0.4f && state.q_des[FR_0] < 0.6f,
              "第一帧 q_des 接近初始值 (插值起点)");
        CHECK(state.motor_mode == MOTOR_MODE_SERVO,
              "MIT 模式已激活 (0x0A)");
        
        // 模拟跑完过渡时间 (1.0s / 0.002s = 500 步)
        for (int i = 0; i < 600; ++i) {
            state.motion_time++;
            fsm.update(state);
        }
        // 过渡完成后, q_des 应接近 target_q[FR_0] (= 0.0)
        CHECK(state.q_des[FR_0] < 0.01f && state.q_des[FR_0] > -0.01f,
              "过渡完成后 q_des[FR_0] 接近 target (0.0)");
    }

    // ---- 测试 12: Damping 断电行为 ----
    TEST("Damping: 电机模式为 0x00, 所有控制量归零")
    {
        StateMachine fsm;
        FSState state;
        tick(fsm, state);
        // 已经在 Damping
        CHECK(state.motor_mode == MOTOR_MODE_DAMPING,
              "motor_mode = 0x00 (Damping)");
        CHECK(state.q_des[0] == 0.0f && state.Kp[0] == 0.0f && state.Kd[0] == 0.0f,
              "q_des/Kp/Kd 全部归零");
    }

    // ---- 测试 13: Stand 力矩安全层 (超限急停) ----
    TEST("Stand 力矩安全层: tau_est 超限触发 emergency_triggered")
    {
        StateMachine fsm;
        FSState state;
        
        // 初始 Damping
        state.motion_time++;
        fsm.update(state);
        fsm.requestTransition(FSMStateID::STAND, state);
        
        // 正常情况: 不应触发急停
        for (int i = 0; i < 5; ++i) {
            state.motion_time++;
            fsm.update(state);
        }
        CHECK(!state.emergency_triggered, "正常力矩: emergency_triggered = false");
        CHECK(fsm.isInState(FSMStateID::STAND), "仍在 Stand");
        
        // 模拟力矩超限 (所有关节)
        for (int i = 0; i < NUM_JOINTS; ++i) {
            state.tau_est[i] = 50.0f;  // > torque_limit (33.0)
        }
        
        // 连续超限到阈值 (默认10次)
        for (int i = 0; i < 10; ++i) {
            state.motion_time++;
            fsm.update(state);
        }
        CHECK(state.emergency_triggered, "连续超限: emergency_triggered = true");
        
        // 控制器层应自动急停 (模拟 FSMController::update 逻辑)
        if (state.emergency_triggered) {
            fsm.forceTransition(FSMStateID::DAMPING, state);
            state.emergency_triggered = false;
        }
        CHECK(fsm.isInState(FSMStateID::DAMPING), "力矩超限 -> 自动进入 DAMPING");
    }

    // ---- 测试 14: Model 推理降采样 (50 Hz) ----
    TEST("Model 推理降采样: 每 10 个控制周期推理一次")
    {
        StateMachine fsm;
        FSState state;
        state.control_dt = 0.002f;
        
        state.motion_time++;
        fsm.update(state);
        fsm.requestTransition(FSMStateID::STAND, state);
        fsm.requestTransition(FSMStateID::MODEL, state);
        
        // 跑 25 个控制周期, 只有第 10, 20 步会触发 do_inference
        for (int i = 0; i < 25; ++i) {
            state.motion_time++;
            fsm.update(state);
        }
        // Model 状态下电机模式应保持 MIT
        CHECK(state.motor_mode == MOTOR_MODE_SERVO, "Model 下 motor_mode = MIT");
        CHECK(fsm.isInState(FSMStateID::MODEL), "仍在 Model");
        
        // 验证退出日志中包含正确的推理周期数
        fsm.requestTransition(FSMStateID::STAND, state);
        // 25 steps / 10 decimation = 2 inference cycles
    }

    // ============================================================
    printf("\n========================================\n");
    printf("  测试结果: %d 通过, %d 失败\n", g_passed, g_failed);
    printf("========================================\n\n");

    return g_failed > 0 ? 1 : 0;
}
