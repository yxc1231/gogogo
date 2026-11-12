/**********************************************************************
 读取 Go1 机器人状态数据 — 持续收发测试
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <cstdio>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

int main() {
    std::cout << "=== Go1 State Reader (持续收发) ===" << std::endl;
    std::cout << "Target: 192.168.123.10:8007 | Local: :8090" << std::endl;
    std::cout << "请确保机器人已进入 Low-Level 模式 (遥控器 L2+B)!" << std::endl;
    std::cout << "Reading... (Ctrl+C stop)\n" << std::endl;

    InitEnvironment();  // 内存锁定, 实时优先级

    UDP udp(LOWLEVEL, 8090, "192.168.123.10", 8007);
    LowCmd  cmd   = {0};
    LowState state = {0};
    udp.InitCmdData(cmd);

    int last_tick = -1;
    for (int i = 0; i < 500; ++i) {
        udp.Recv();
        udp.GetRecv(state);
        
        // 设置控制指令 (阻尼模式, 安全)
        for (int j = 0; j < 12; ++j) {
            cmd.motorCmd[j].mode = 0x00;
            cmd.motorCmd[j].q = 0; cmd.motorCmd[j].dq = 0;
            cmd.motorCmd[j].tau = 0; cmd.motorCmd[j].Kp = 0; cmd.motorCmd[j].Kd = 0;
        }
        
        udp.SetSend(cmd);
        udp.Send();  // 必须发送! 机器人才会回复状态

        if (state.tick != last_tick) {
            last_tick = state.tick;
            printf("[%04d] tick=%-6u batt=%-3d%% rpy=[%+6.2f %+6.2f %+6.2f] "
                   "FR_0: q=%+7.3f dq=%+7.3f tau=%+6.2f "
                   "FR_1: q=%+7.3f FR_2: q=%+7.3f foot=[%d %d %d %d]\n",
                   i, state.tick, state.bms.SOC,
                   state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2],
                   state.motorState[FR_0].q, state.motorState[FR_0].dq, state.motorState[FR_0].tauEst,
                   state.motorState[FR_1].q, state.motorState[FR_2].q,
                   state.footForce[0], state.footForce[1], state.footForce[2], state.footForce[3]);
        }

        usleep(2000);  // 2ms ≈ 500Hz
    }

    std::cout << "\nDone." << std::endl;
    return 0;
}
