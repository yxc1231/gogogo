/**********************************************************************
 关节方向测试: 阻尼模式, 只打印位置和速度
 用手掰关节, 观察 q 的正负变化, 确定编码器方向
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <cstdio>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

int main() {
    printf("=== Go1 Joint Direction Test ===\n");
    printf("Motors in DAMPING (no torque). Move joints by hand.\n");
    printf("q > 0 = positive direction, q < 0 = negative\n\n");

    UDP udp(LOWLEVEL, 8090, "192.168.123.10", 8007);
    LowCmd  cmd   = {0};
    LowState state = {0};
    udp.InitCmdData(cmd);

    // 全部阻尼
    for (int i = 0; i < 12; ++i) {
        cmd.motorCmd[i].mode = 0x00;
        cmd.motorCmd[i].q = 0; cmd.motorCmd[i].dq = 0;
        cmd.motorCmd[i].tau = 0; cmd.motorCmd[i].Kp = 0; cmd.motorCmd[i].Kd = 0;
    }

    const char* names[12] = {
        "FR_0","FR_1","FR_2", "FL_0","FL_1","FL_2",
        "RR_0","RR_1","RR_2", "RL_0","RL_1","RL_2"
    };

    printf("%-8s ", "tick");
    for (int i = 0; i < 12; ++i) printf("%-10s ", names[i]);
    printf("\n");
    for (int i = 0; i < 12; ++i) printf("---------- ");
    printf("\n");

    int last_tick = -1;
    while (true) {
        udp.Recv();
        udp.GetRecv(state);
        udp.SetSend(cmd);
        udp.Send();

        if (state.tick != last_tick) {
            last_tick = state.tick;
            printf("%-8u ", state.tick);
            for (int i = 0; i < 12; ++i)
                printf("%+8.3f   ", state.motorState[i].q);
            printf("\n");
        }
        usleep(2000);
    }
    return 0;
}
