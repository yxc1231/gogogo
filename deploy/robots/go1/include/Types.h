// Copyright (c) 2026, Go1 RL Deployment.
// All rights reserved.
//
// Go1 适配层: 支持两种模式
//   - 实机模式 (默认): 将旧版 unitree_legged_sdk (UDP) 封装为 DDS 兼容接口
//   - 仿真模式 (-DGO1_SIM_MODE): 直接使用 DDS SDK2 对接 MuJoCo 仿真

#pragma once

#ifdef GO1_SIM_MODE
// ===================================================================
// 仿真模式: 使用 go2 DDS 协议对接 MuJoCo
// ===================================================================
#include "unitree/dds_wrapper/robots/go2/go2.h"

using LowCmd_t = unitree::robot::go2::publisher::LowCmd;
using LowState_t = unitree::robot::go2::subscription::LowState;

#else
// ===================================================================
// 实机模式: UDP 封装 (旧版 unitree_legged_sdk)
// ===================================================================

#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstring>
#include <spdlog/spdlog.h>

// ===================================================================
// 1. MotorCmd 包装器 — 把旧 struct 字段映射为 .q()/.kp() 等方法
// ===================================================================
class Go1MotorCmdRef
{
public:
    UNITREE_LEGGED_SDK::MotorCmd* ptr;

    Go1MotorCmdRef() : ptr(nullptr) {}
    Go1MotorCmdRef(UNITREE_LEGGED_SDK::MotorCmd* p) : ptr(p) {}

    float& q()   { return ptr->q; }
    float& kp()  { return ptr->Kp; }
    float& kd()  { return ptr->Kd; }
    float& dq()  { return ptr->dq; }
    float& tau() { return ptr->tau; }
    uint8_t& mode() { return ptr->mode; }
};

// ===================================================================
// 2. MotorState 包装器 — .q() / .dq() / .tau_est()
// ===================================================================
class Go1MotorStateRef
{
public:
    UNITREE_LEGGED_SDK::MotorState* ptr;

    Go1MotorStateRef() : ptr(nullptr) {}
    Go1MotorStateRef(UNITREE_LEGGED_SDK::MotorState* p) : ptr(p) {}

    float& q()  { return ptr->q; }
    float& dq() { return ptr->dq; }
    float& tau_est() { return ptr->tauEst; }
};

// ===================================================================
// 3. IMU 包装器 — .gyroscope() / .quaternion()
// ===================================================================
class Go1IMUStateRef
{
public:
    UNITREE_LEGGED_SDK::IMU* ptr;

    Go1IMUStateRef() : ptr(nullptr) {}
    Go1IMUStateRef(UNITREE_LEGGED_SDK::IMU* p) : ptr(p) {}

    std::array<float, 3>& gyroscope()  { return ptr->gyroscope; }
    std::array<float, 4>& quaternion() { return ptr->quaternion; }
};

// ===================================================================
// 4. LowCmd 消息包装
// ===================================================================
class Go1LowCmdMsg
{
public:
    UNITREE_LEGGED_SDK::LowCmd raw;

    Go1LowCmdMsg()
    {
        memset(&raw, 0, sizeof(raw));
        raw.levelFlag = 0xff;
        for (int i = 0; i < 20; i++) {
            motor_cmd_refs_[i] = Go1MotorCmdRef(&raw.motorCmd[i]);
        }
    }

    std::array<Go1MotorCmdRef, 20>& motor_cmd() { return motor_cmd_refs_; }

private:
    std::array<Go1MotorCmdRef, 20> motor_cmd_refs_;
};

// ===================================================================
// 5. LowState 消息包装
// ===================================================================
class Go1LowStateMsg
{
public:
    UNITREE_LEGGED_SDK::LowState raw;

    Go1LowStateMsg()
    {
        memset(&raw, 0, sizeof(raw));
        imu_ref_ = Go1IMUStateRef(&raw.imu);
        for (int i = 0; i < 20; i++) {
            motor_state_refs_[i] = Go1MotorStateRef(&raw.motorState[i]);
        }
    }

    Go1IMUStateRef& imu_state() { return imu_ref_; }
    std::array<Go1MotorStateRef, 20>& motor_state() { return motor_state_refs_; }

private:
    Go1IMUStateRef imu_ref_;
    std::array<Go1MotorStateRef, 20> motor_state_refs_;
};

// ===================================================================
// 6. 全局 UDP 连接 (LowCmd 和 LowState 共享)
//    注意: 旧 SDK 的 UDP 流程:
//      Recv() → GetRecv(state)  // 接收
//      SetSend(cmd) → Send()   // 发送
// ===================================================================
class Go1UDP
{
public:
    static Go1UDP& instance()
    {
        static Go1UDP inst;
        return inst;
    }

    void init(UNITREE_LEGGED_SDK::LowCmd& init_cmd)
    {
        if (initialized_) return;
        udp_ = std::make_unique<UNITREE_LEGGED_SDK::UDP>(
            UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007);
        udp_->InitCmdData(init_cmd);
        initialized_ = true;
    }

    UNITREE_LEGGED_SDK::UDP* udp() { return udp_.get(); }

    // 接收: Recv + GetRecv
    void recvState(UNITREE_LEGGED_SDK::LowState& state)
    {
        udp_->Recv();
        udp_->GetRecv(state);
    }

    // 发送: SetSend + Send
    void sendCmd(UNITREE_LEGGED_SDK::LowCmd& cmd)
    {
        std::lock_guard<std::mutex> lock(send_mutex_);
        udp_->SetSend(cmd);
        udp_->Send();
    }

    std::mutex send_mutex_;

private:
    Go1UDP() = default;
    bool initialized_ = false;
    std::unique_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
};

// ===================================================================
// 7. LowState_t — 订阅者包装 (兼容 DDS LowState 接口)
// ===================================================================
class LowState_t
{
public:
    using SharedPtr = std::shared_ptr<LowState_t>;

    LowState_t()
    {
        // UDP 在首次 LowCmd_t 构造时初始化 (需要 InitCmdData)
        last_update_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(timeout_ms_);
    }

    // ---- DDS SubscriptionBase 兼容接口 ----
    Go1LowStateMsg msg_;
    std::mutex mutex_;
    unitree::common::UnitreeJoystick joystick;

    void update()
    {
        // UDP Recv + GetRecv → 更新 msg_.raw
        Go1UDP::instance().recvState(msg_.raw);

        // 仅当接收到新数据时更新时间戳 (通过 tick 判断)
        if (msg_.raw.tick != last_tick_) {
            last_update_time_ = std::chrono::steady_clock::now();
            last_tick_ = msg_.raw.tick;
        }

        // --- Joystick 解析 (从 wirelessRemote 字节数组) ---
        const auto& wr = msg_.raw.wirelessRemote;
        bool all_zero = true;
        for (int i = 0; i < 40; i++) {
            if (wr[i] != 0) { all_zero = false; break; }
        }

        if (!all_zero) {
            last_joystick_time_ = std::chrono::steady_clock::now();
            unitree::common::REMOTE_DATA_RX key;
            memcpy(&key, &wr[0], 40);
            joystick.extract(key);
        }
    }

    bool isTimeout()
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_update_time_;
        return elapsed > std::chrono::milliseconds(timeout_ms_);
    }

    void wait_for_connection()
    {
        auto t0 = std::chrono::steady_clock::now();
        bool warned = false;
        while (isTimeout()) {
            update();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!warned && std::chrono::steady_clock::now() - t0 > std::chrono::seconds(2)) {
                warned = true;
                spdlog::warn("Waiting for Go1 connection...");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (warned) {
            spdlog::info("Go1 connected.");
        }
    }

private:
    uint32_t timeout_ms_ = 1000;
    uint32_t joystick_timeout_ms_ = 3000;
    uint32_t last_tick_ = 0;
    std::chrono::steady_clock::time_point last_update_time_;
    std::chrono::steady_clock::time_point last_joystick_time_;
};

// ===================================================================
// 8. LowCmd_t — 发布者包装 (兼容 DDS LowCmd 接口)
// ===================================================================
class LowCmd_t
{
public:
    using SharedPtr = std::shared_ptr<LowCmd_t>;

    LowCmd_t()
    {
        // 首次构造时初始化 UDP (InitCmdData 需要绑定一个 LowCmd)
        Go1UDP::instance().init(msg_.raw);
    }

    Go1LowCmdMsg msg_;

    void unlockAndPublish()
    {
        // SetSend + Send: 将 msg_.raw 发送到机器人
        Go1UDP::instance().sendCmd(msg_.raw);
    }
};

#endif // GO1_SIM_MODE

// ===================================================================
// 9. 类型别名 — 供共享 FSM 框架使用
// ===================================================================
// LowCmd_t 和 LowState_t 已直接定义，无需别名
// FSMState 中的 static 成员声明:
//   static std::unique_ptr<LowCmd_t> lowcmd;
//   static std::shared_ptr<LowState_t> lowstate;
