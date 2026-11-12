// Copyright (c) 2026, Go1 RL Deployment.
// All rights reserved.
//
// 仿真模式入口: 通过 DDS 对接 MuJoCo 仿真环境

#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if (!lowcmd_sub->isTimeout())
    {
        spdlog::critical("Another process is using the lowcmd channel, please close it first.");
        exit(1);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for MuJoCo simulation...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to MuJoCo simulation.");
}

int main(int argc, char** argv)
{
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go1 Controller (MuJoCo Simulation) \n";

    // DDS 初始化 (仿真模式)
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

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
