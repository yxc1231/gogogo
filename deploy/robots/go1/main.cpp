// Copyright (c) 2026, Go1 RL Deployment.
// All rights reserved.

#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();

    // A Go1 starts returning LOWLEVEL state packets only after it has received
    // a command packet.  Bootstrap that exchange with an explicitly safe
    // damping command, rather than waiting indefinitely for a packet that has
    // not yet been requested.
    for (int i = 0; i < 12; ++i) {
        auto& motor = FSMState::lowcmd->msg_.motor_cmd()[i];
        motor.mode() = 0x00;  // damping mode
        motor.q() = 0.0f;
        motor.dq() = 0.0f;
        motor.kp() = 0.0f;
        motor.kd() = 0.0f;
        motor.tau() = 0.0f;
    }
    FSMState::lowcmd->unlockAndPublish();

    spdlog::info("Waiting for connection to Go1 robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to Go1 robot.");
}

int main(int argc, char** argv)
{
    // 加载参数 (param::helper 会读取 config.yaml)
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go1 Controller (UDP) \n";

    // Go1 使用 UDP 通信, 不需要 DDS ChannelFactory 初始化
    // UDP 在 LowState_t / LowCmd_t 构造时自动初始化

    init_fsm_state();

    // 初始化 FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + A] to enter FixStand mode.\n";
    std::cout << "And then press [L2 + X] to start RL velocity control.\n";

    while (true)
    {
        sleep(1);
    }

    return 0;
}
