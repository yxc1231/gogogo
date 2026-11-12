/**********************************************************************
 Copyright (c) 2026, Go1 FSM State Machine System. All rights reserved.
***********************************************************************/

#include "go1_fsm/fsm_controller.h"
#include "go1_fsm/remote.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <csignal>
#include <atomic>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

// 全局控制器指针 (用于信号处理)
static GO1_FSM::RemoteCmd g_last_remote = GO1_FSM::RemoteCmd::NONE;
static GO1_FSM::FSMController* g_controller = nullptr;
static std::atomic<bool> g_running(true);

// 键盘输入线程
void keyboardThread() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Go1 FSM Control Panel" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Keys:" << std::endl;
    std::cout << "    s - Stand (站立)" << std::endl;
    std::cout << "    d - Down  (蹲下)" << std::endl;
    std::cout << "  space - Damping (急停)" << std::endl;
    std::cout << "    m - Model (RL推理, 仅Stand下有效)" << std::endl;
    std::cout << "    q - Quit" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    while (g_running) {
        char key = std::cin.get();
        
        if (!g_controller) continue;
        
        switch (key) {
            case 's':
            case 'S':
                std::cout << ">>> Request STAND" << std::endl;
                g_controller->requestStand();
                break;
            case 'd':
            case 'D':
                std::cout << ">>> Request DOWN" << std::endl;
                g_controller->requestDown();
                break;
            case ' ':
                std::cout << ">>> Request DAMPING (EMERGENCY STOP)" << std::endl;
                g_controller->requestDamping();
                break;
            case 'm':
            case 'M':
                std::cout << ">>> Request MODEL (RL Inference)" << std::endl;
                if (!g_controller->requestModel()) {
                    std::cout << "    FAILED: Model can only be entered from Stand!" << std::endl;
                }
                break;
            case 'q':
            case 'Q':
                std::cout << ">>> Quit" << std::endl;
                g_controller->requestDamping();  // 先急停
                g_running = false;
                break;
            default:
                break;
        }
    }
}

// 信号处理 (Ctrl+C)
void signalHandler(int signum) {
    std::cout << "\n[FSM] Signal " << signum << " received. Stopping..." << std::endl;
    if (g_controller) {
        g_controller->requestDamping();
    }
    g_running = false;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Go1 FSM - State Machine Controller" << std::endl;
    std::cout << "  Communication: LOW-LEVEL" << std::endl;
    std::cout << "  WARNING: Ensure robot is hung up!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // ============================================================
    // 初始化 Unitree SDK
    // ============================================================
    Safety safe(LeggedType::Go1);
    UDP udp(LOWLEVEL, 8090, "192.168.123.10", 8007);
    LowCmd cmd = {0};
    LowState state = {0};
    
    udp.InitCmdData(cmd);
    
    // ============================================================
    // 初始化 FSM Controller
    // ============================================================
    GO1_FSM::FSMController controller;
    controller.init(&udp, &safe, &cmd, &state);
    
    // 从 JSON 加载运行时配置 (修改 JSON 无需重编译)
    controller.loadConfig("../config");
    
    g_controller = &controller;
    
    // 设置控制周期
    float dt = 0.002f;  // 500 Hz
    
    // ============================================================
    // 创建实时控制线程 (参考 SDK LoopFunc 模式)
    // ============================================================
    LoopFunc loop_control("fsm_control", dt, [&]() {
        controller.update();
    });
    LoopFunc loop_udpSend("udp_send", dt, 3, [&]() {
        controller.send();
    });
    LoopFunc loop_udpRecv("udp_recv", dt, 3, [&]() {
        controller.recv();
    });

    // ============================================================
    // 启动线程
    // ============================================================
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    
    // ============================================================
    // 键盘输入线程
    // ============================================================
    std::thread kb_thread(keyboardThread);
    
    // ============================================================
    // 主循环: 打印状态
    // ============================================================
    while (g_running) {
        usleep(50000);  // 50ms 轮询, 灵敏响应
        
        // 遥控器控制 (R2+ 避免与机器人内置 L2+B 冲突)
        auto cmd = GO1_FSM::parseRemote(state.wirelessRemote.data());
        if (cmd != g_last_remote && cmd != GO1_FSM::RemoteCmd::NONE) {
            g_last_remote = cmd;
            switch (cmd) {
                case GO1_FSM::RemoteCmd::R2_Y:
                    printf(">>> Remote: R2+Y → STAND\n");
                    controller.requestStand();
                    break;
                case GO1_FSM::RemoteCmd::R2_A:
                    printf(">>> Remote: R2+A → DOWN\n");
                    controller.requestDown();
                    break;
                case GO1_FSM::RemoteCmd::R2_B:
                    printf(">>> Remote: R2+B → DAMPING\n");
                    controller.requestDamping();
                    break;
                case GO1_FSM::RemoteCmd::R2_X:
                    printf(">>> Remote: R2+X → MODEL\n");
                    if (!controller.requestModel())
                        printf("    FAILED: Model only from Stand!\n");
                    break;
                default: break;
            }
        }
        if (cmd == GO1_FSM::RemoteCmd::NONE) g_last_remote = GO1_FSM::RemoteCmd::NONE;
        
        // 每秒打印一次状态
        static int print_cnt = 0;
        if (++print_cnt % 20 == 0) {
            auto& fs = controller.getFSState();
            printf("[FSM] State: %-8s | tick: %u | battery: %.0f%% | "
                   "q[FR_1]: %+.3f rad | dq[FR_1]: %+.3f rad/s\n",
                   fsmStateName(controller.getCurrentState()),
                   fs.tick, fs.battery_soc,
                   fs.q[FR_1], fs.dq[FR_1]);
        }
    }
    
    // 清理
    kb_thread.join();
    printf("[FSM] Shutdown complete.\n");
    
    return 0;
}
