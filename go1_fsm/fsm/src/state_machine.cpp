/**********************************************************************
 Copyright (c) 2026, Go1 FSM State Machine System. All rights reserved.
***********************************************************************/

#include "go1_fsm/state_machine.h"

namespace GO1_FSM {

StateMachine::StateMachine() {
    registerState(std::make_unique<StandState>());
    registerState(std::make_unique<DownState>());
    registerState(std::make_unique<DampingState>());
    registerState(std::make_unique<ModelState>());
    registerState(std::make_unique<PassiveState>());
}

void StateMachine::registerState(std::unique_ptr<BaseState> state) {
    FSMStateID id = state->getStateID();
    _states[id] = std::move(state);
}

bool StateMachine::isTransitionValid(FSMStateID from, FSMStateID to) const {
    // 任何状态都可以急停 → Damping
    if (to == FSMStateID::DAMPING) return true;
    
    // Damping 可以恢复到 Stand
    if (from == FSMStateID::DAMPING && to == FSMStateID::STAND) return true;
    
    // Stand ↔ Down
    if (from == FSMStateID::STAND && to == FSMStateID::DOWN) return true;
    if (from == FSMStateID::DOWN && to == FSMStateID::STAND) return true;
    
    // Stand → Model (唯一通路!)
    if (from == FSMStateID::STAND && to == FSMStateID::MODEL) return true;
    
    // Model → Stand
    if (from == FSMStateID::MODEL && to == FSMStateID::STAND) return true;

    // 任意状态 → Passive (释放控制)
    if (to == FSMStateID::PASSIVE) return true;
    // Passive → Damping (重新接管)
    if (from == FSMStateID::PASSIVE && to == FSMStateID::DAMPING) return true;

    return false;
}

bool StateMachine::requestTransition(FSMStateID target_id, FSState& state) {
    if (_current_state == nullptr) {
        // 首次进入: 必须从 Damping 开始
        if (target_id != FSMStateID::DAMPING) {
            printf("[FSM] ERROR: Initial state must be DAMPING!\n");
            return false;
        }
    } else {
        FSMStateID current_id = _current_state->getStateID();
        
        if (current_id == target_id) {
            // 相同状态, 无需切换
            return true;
        }
        
        if (!isTransitionValid(current_id, target_id)) {
            printf("[FSM] ERROR: Illegal transition %s -> %s!\n",
                   fsmStateName(current_id), fsmStateName(target_id));
            return false;
        }
    }
    
    // 执行切换
    forceTransition(target_id, state);
    return true;
}

void StateMachine::forceTransition(FSMStateID target_id, FSState& state) {
    // 退出当前状态
    if (_current_state != nullptr) {
        _current_state->onExit(state);
    }
    
    // 查找目标状态
    auto it = _states.find(target_id);
    if (it == _states.end()) {
        printf("[FSM] ERROR: State %s not registered!\n", fsmStateName(target_id));
        return;
    }
    
    _current_state = it->second.get();
    
    // 进入新状态
    _current_state->onEnter(state);
    
    printf("[FSM] Transition -> %s\n", fsmStateName(target_id));
}

void StateMachine::update(FSState& state) {
    if (_current_state == nullptr) {
        // 未初始化, 默认进入 Damping
        forceTransition(FSMStateID::DAMPING, state);
    }
    _current_state->onUpdate(state);
}

FSMStateID StateMachine::getCurrentStateID() const {
    if (_current_state == nullptr) return FSMStateID::DAMPING;
    return _current_state->getStateID();
}

const BaseState* StateMachine::getCurrentState() const {
    return _current_state;
}

bool StateMachine::isInState(FSMStateID id) const {
    return getCurrentStateID() == id;
}

}  // namespace GO1_FSM
