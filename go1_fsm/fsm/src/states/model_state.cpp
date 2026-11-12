/**********************************************************************
 Copyright (c) 2026, Go1 FSM State Machine System. All rights reserved.
***********************************************************************/

#include "go1_fsm/states/model_state.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <chrono>

namespace GO1_FSM {

void ModelState::onEnter(FSState& state) {
    resetTiming();
    _state_ptr = &state;
    _latest_action.fill(0.0f);
    
    state.motor_mode = MOTOR_MODE_SERVO;
    
    // 加载模型
    _loader = ModelLoader::create(_cfg.model_format, _cfg.device);
    _model_loaded = false;
    if (_loader) {
        _model_loaded = _loader->load(_cfg.model_path);
    }
    
    int inference_hz = static_cast<int>(1.0f / state.control_dt) / _cfg.inference_decimation;
    printf("[FSM] Enter Model | format=%s path=%s device=%s inference=%dHz (async)\n",
           _cfg.model_format.c_str(), _cfg.model_path.c_str(), _cfg.device.c_str(), inference_hz);
    
    if (_model_loaded) {
        // 启动异步推理线程
        _infer_running = true;
        _infer_thread = std::thread(&ModelState::inferenceLoop, this);
        printf("[FSM] Inference thread started.\n");
    } else {
        printf("[FSM] WARNING: Model not loaded! PD fallback only.\n");
    }
}

void ModelState::inferenceLoop() {
    // 固定 50 Hz: period = 20ms = 20000us
    constexpr long long PERIOD_US = 20000;
    std::array<float, NUM_JOINTS> prev_action{};
    int loop_cnt = 0;
    
    printf("[FSM] Inference loop: fixed 50 Hz (period=%lld us)\n", PERIOD_US);
    
    // 绝对定时: 预计算下一次唤醒时间
    auto next_wake = std::chrono::steady_clock::now();
    
    while (_infer_running) {
        next_wake += std::chrono::microseconds(PERIOD_US);
        
        // 构建观测 → 推理
        auto obs = buildObservation(*_state_ptr, prev_action,
                                     _cfg.default_pose, _cfg.joint_sign,
                                     _cfg.joint_order, _cfg.obs_dim);
        std::vector<float> act;
        
        if (_loader->inference(obs, act) && act.size() >= static_cast<size_t>(_cfg.act_dim)) {
            std::array<float, NUM_JOINTS> action;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                float a = act[i] * _cfg.jointScale(i);
                a = std::max(-_cfg.jointClip(i), std::min(_cfg.jointClip(i), a));
                action[i] = a;
                prev_action[i] = a;
            }
            {
                std::lock_guard<std::mutex> lock(_action_mutex);
                _latest_action = action;
            }
        }
        
        // 每隔 2.5s 打印一次推理频率
        if (loop_cnt % 125 == 0 && loop_cnt > 0) {
            static auto last_print = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_print).count();
            printf("[RL] %.1f Hz (actual), exec=%.2f ms\n", 125.0/dt,
                   dt * 1000.0 / 125.0);
            last_print = now;
        }
        loop_cnt++;
        
        // 绝对等待: sleep_until 精确到微秒, 不漂移
        std::this_thread::sleep_until(next_wake);
    }
    printf("[FSM] Inference loop stopped. Total cycles: %d\n", loop_cnt);
}

void ModelState::onUpdate(FSState& state) {
    advanceTiming(state.control_dt);
    _state_ptr = &state;  // 更新指针给推理线程用
    
    if (_model_loaded) {
        std::array<float, NUM_JOINTS> action;
        {
            std::lock_guard<std::mutex> lock(_action_mutex);
            action = _latest_action;
        }
        
        // 按模型顺序的动作 → 按 SDK 顺序写入电机
        for (int j = 0; j < NUM_JOINTS; ++j) {
            int sdk_i = _cfg.joint_order[j];  // 模型位 j → SDK 索引
            state.q_des[sdk_i] = _cfg.default_pose[j] + action[j];
        }
        state.dq_des.fill(0.0f);
        state.Kp = _cfg.Kp;
        state.Kd = _cfg.Kd;
        state.tau_ff.fill(0.0f);
        return;
    }
    
    // PD 回退
    for (int i = 0; i < NUM_JOINTS; ++i) {
        state.q_des[i] = state.q[i];
        state.dq_des[i] = 0.0f;
        state.Kp[i] = _cfg.Kp[i];
        state.Kd[i] = _cfg.Kd[i];
        state.tau_ff[i] = 0.0f;
    }
}

void ModelState::onExit(FSState& state) {
    // 停止推理线程
    _infer_running = false;
    if (_infer_thread.joinable()) {
        _infer_thread.join();
    }
    _loader.reset();
    _state_ptr = nullptr;
    
    printf("[FSM] Exit Model | elapsed=%.2fs steps=%lu\n", _elapsed_time, _step_count);
}

}  // namespace GO1_FSM
