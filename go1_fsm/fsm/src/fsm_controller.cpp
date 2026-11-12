/**********************************************************************
 Copyright (c) 2026, Go1 FSM State Machine System. All rights reserved.
***********************************************************************/

#include "go1_fsm/fsm_controller.h"
#include "go1_fsm/config_loader.h"

// Unitree SDK headers
#include "unitree_legged_sdk/unitree_legged_sdk.h"

namespace GO1_FSM {

FSMController::FSMController() {
    // 使用自定义配置初始化各状态
    auto stand_state   = std::make_unique<StandState>(_stand_cfg);
    auto down_state    = std::make_unique<DownState>(_down_cfg);
    auto damping_state = std::make_unique<DampingState>(_damping_cfg);
    auto model_state   = std::make_unique<ModelState>(_model_cfg);
    
    _fsm.registerState(std::move(stand_state));
    _fsm.registerState(std::move(down_state));
    _fsm.registerState(std::move(damping_state));
    _fsm.registerState(std::move(model_state));
}

FSMController::~FSMController() = default;

void FSMController::init(void* udp, void* safety, void* low_cmd, void* low_state) {
    _udp       = udp;
    _safety    = safety;
    _low_cmd   = low_cmd;
    _low_state = low_state;
    
    printf("[FSM] Controller initialized.\n");
}

bool FSMController::loadConfig(const std::string& config_dir) {
    printf("[FSM] Loading configs from: %s\n", config_dir.c_str());
    
    bool ok = true;
    ok &= ConfigLoader::loadStand(config_dir + "/stand.yaml", _stand_cfg);
    ok &= ConfigLoader::loadDown(config_dir + "/down.yaml", _down_cfg);
    ok &= ConfigLoader::loadDamping(config_dir + "/damping.yaml", _damping_cfg);
    ok &= ConfigLoader::loadModel(config_dir + "/model.yaml", _model_cfg);
    
    // 将加载的配置同步到已注册的状态实例
    // (通过重新注册来更新配置)
    auto stand_state   = std::make_unique<StandState>(_stand_cfg);
    auto down_state    = std::make_unique<DownState>(_down_cfg);
    auto damping_state = std::make_unique<DampingState>(_damping_cfg);
    auto model_state   = std::make_unique<ModelState>(_model_cfg);
    
    _fsm.registerState(std::move(stand_state));
    _fsm.registerState(std::move(down_state));
    _fsm.registerState(std::move(damping_state));
    _fsm.registerState(std::move(model_state));
    
    if (ok) {
        printf("[FSM] All configs loaded successfully.\n");
    } else {
        printf("[FSM] WARNING: Some configs failed to load, using defaults.\n");
    }
    return ok;
}

// ============================================================
// 控制循环
// ============================================================

void FSMController::recv() {
    if (_udp == nullptr) return;
    auto* udp = static_cast<UNITREE_LEGGED_SDK::UDP*>(_udp);
    udp->Recv();  // 仅接收数据到内部缓冲区
}

void FSMController::update() {
    _fs_state.motion_time++;
    
    // 从 SDK 同步机器人状态 (GetRecv 必须在 Recv 之后的另一线程调用)
    syncFromSDK();
    
    // 运行状态机
    _fsm.update(_fs_state);
    
    // 安全层: Stand/Down 力矩超限自动急停
    if (_fs_state.emergency_triggered) {
        printf("[FSM] Safety layer triggered! Forcing Damping.\n");
        _fsm.forceTransition(FSMStateID::DAMPING, _fs_state);
        _fs_state.emergency_triggered = false;  // 清除标志
    }
    
    // SDK 安全检查 (PositionLimit + PowerProtect)
    applySafety();
    
    // 同步控制指令到 SDK LowCmd
    syncToSDK();
}

void FSMController::send() {
    if (_udp == nullptr || _low_cmd == nullptr) return;
    if (_fs_state.passive_release) return;  // Passive: 停止发送, 机器人超时切 High-Level
    
    auto* udp = static_cast<UNITREE_LEGGED_SDK::UDP*>(_udp);
    auto* cmd = static_cast<UNITREE_LEGGED_SDK::LowCmd*>(_low_cmd);
    
    udp->SetSend(*cmd);
    udp->Send();  // 必须发送! SDK 中 SetSend 只拷贝到缓冲区, Send 才实际传输
}

// ============================================================
// 数据同步
// ============================================================

void FSMController::syncFromSDK() {
    if (_udp == nullptr || _low_state == nullptr) return;
    
    auto* udp   = static_cast<UNITREE_LEGGED_SDK::UDP*>(_udp);
    auto* state = static_cast<UNITREE_LEGGED_SDK::LowState*>(_low_state);
    
    udp->GetRecv(*state);
    
    // 同步 12 个关节状态
    for (int i = 0; i < NUM_JOINTS; ++i) {
        _fs_state.q[i]       = state->motorState[i].q;
        _fs_state.dq[i]      = state->motorState[i].dq;
        _fs_state.tau_est[i] = state->motorState[i].tauEst;
    }
    
    // 同步 IMU
    _fs_state.rpy[0]  = state->imu.rpy[0];
    _fs_state.rpy[1]  = state->imu.rpy[1];
    _fs_state.rpy[2]  = state->imu.rpy[2];
    _fs_state.gyro[0] = state->imu.gyroscope[0];
    _fs_state.gyro[1] = state->imu.gyroscope[1];
    _fs_state.gyro[2] = state->imu.gyroscope[2];
    _fs_state.acc[0]  = state->imu.accelerometer[0];
    _fs_state.acc[1]  = state->imu.accelerometer[1];
    _fs_state.acc[2]  = state->imu.accelerometer[2];
    
    // 足端力
    for (int i = 0; i < 4; ++i) {
        _fs_state.foot_force[i] = static_cast<float>(state->footForce[i]);
    }
    
    // 电池
    _fs_state.battery_soc = static_cast<float>(state->bms.SOC);
    
    // 时间戳
    _fs_state.tick = state->tick;
}

void FSMController::syncToSDK() {
    if (_low_cmd == nullptr) return;
    
    auto* cmd = static_cast<UNITREE_LEGGED_SDK::LowCmd*>(_low_cmd);
    
    // 同步 12 个关节控制指令
    for (int i = 0; i < NUM_JOINTS; ++i) {
        cmd->motorCmd[i].mode = _fs_state.motor_mode;
        cmd->motorCmd[i].q    = _fs_state.q_des[i];
        cmd->motorCmd[i].dq   = _fs_state.dq_des[i];
        cmd->motorCmd[i].tau  = _fs_state.tau_ff[i];
        cmd->motorCmd[i].Kp   = _fs_state.Kp[i];
        cmd->motorCmd[i].Kd   = _fs_state.Kd[i];
    }
}

// ============================================================
// 安全检查 (封装 SDK Safety 模块)
// ============================================================

void FSMController::applySafety() {
    if (_safety == nullptr || _low_cmd == nullptr || _low_state == nullptr) return;
    
    auto* safe  = static_cast<UNITREE_LEGGED_SDK::Safety*>(_safety);
    auto* cmd   = static_cast<UNITREE_LEGGED_SDK::LowCmd*>(_low_cmd);
    auto* state = static_cast<UNITREE_LEGGED_SDK::LowState*>(_low_state);
    
    // 位置限制
    safe->PositionLimit(*cmd);
    
    // 功率保护 (factor=1 → 10% power limit, 适合新手)
    int res = safe->PowerProtect(*cmd, *state, 1);
    if (res < 0) {
        printf("[FSM] PowerProtect triggered! Emergency stop.\n");
        requestDamping();
    }
}

// ============================================================
// 状态切换接口
// ============================================================

bool FSMController::requestStand() {
    return _fsm.requestTransition(FSMStateID::STAND, _fs_state);
}

bool FSMController::requestDown() {
    return _fsm.requestTransition(FSMStateID::DOWN, _fs_state);
}

bool FSMController::requestDamping() {
    return _fsm.requestTransition(FSMStateID::DAMPING, _fs_state);
}

bool FSMController::requestModel() {
    return _fsm.requestTransition(FSMStateID::MODEL, _fs_state);
}

bool FSMController::requestPassive() {
    return _fsm.requestTransition(FSMStateID::PASSIVE, _fs_state);
}

// ============================================================
// 配置
// ============================================================

void FSMController::setStandConfig(const config::StandConfig& cfg) {
    _stand_cfg = cfg;
    // 如果当前正在 Stand 状态, 动态更新 (下次 onEnter 生效)
}

void FSMController::setDownConfig(const config::DownConfig& cfg) {
    _down_cfg = cfg;
}

void FSMController::setDampingConfig(const config::DampingConfig& cfg) {
    _damping_cfg = cfg;
}

void FSMController::setModelConfig(const config::ModelConfig& cfg) {
    _model_cfg = cfg;
}

// ============================================================
// 查询
// ============================================================

FSMStateID FSMController::getCurrentState() const {
    return _fsm.getCurrentStateID();
}

bool FSMController::isModelLoaded() const {
    auto* state = dynamic_cast<const ModelState*>(_fsm.getCurrentState());
    if (state != nullptr) {
        return state->isModelLoaded();
    }
    return false;
}

}  // namespace GO1_FSM
