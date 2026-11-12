// Copyright (c) 2026, Go1 RL Deployment.
// All rights reserved.
//
// 仿真模式: 使用 DDS (Unitree SDK2) 对接 MuJoCo 仿真环境
// MuJoCo 仿真以 go2 DDS 协议发布 LowState / 订阅 LowCmd

#pragma once

#include "unitree/dds_wrapper/robots/go2/go2.h"

using LowCmd_t = unitree::robot::go2::publisher::LowCmd;
using LowState_t = unitree::robot::go2::subscription::LowState;
