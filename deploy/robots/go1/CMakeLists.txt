cmake_minimum_required(VERSION 3.12)
project(go1_controller)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

# ============================================================
# unitree_legged_sdk (Go1 实机 UDP 通信, 仅 go1_ctrl 需要)
# ============================================================
set(UNITREE_LEGGED_SDK_ROOT "/home/yxc/WS/go1_fsm/third_party/unitree_legged_sdk")

# 自动检测架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(UNITREE_LIB_DIR ${UNITREE_LEGGED_SDK_ROOT}/lib/cpp/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(UNITREE_LIB_DIR ${UNITREE_LEGGED_SDK_ROOT}/lib/cpp/arm64)
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_library(UNITREE_LEGGED_LIB unitree_legged_sdk
    PATHS ${UNITREE_LIB_DIR} NO_DEFAULT_PATH)
if(NOT UNITREE_LEGGED_LIB)
    message(FATAL_ERROR "unitree_legged_sdk not found in ${UNITREE_LIB_DIR}")
endif()
message(STATUS "Unitree Legged SDK: ${UNITREE_LEGGED_LIB}")

# ============================================================
# 公共 Include 路径 (两个 target 共用)
# ============================================================
include_directories(
    /usr/include/eigen3
    /usr/local/include/ddscxx
    /usr/local/include/iceoryx/v2.0.2
    /opt/unitree_robotics/include
    /opt/unitree_robotics/include/ddscxx
    include
    ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/include
    ${PROJECT_SOURCE_DIR}/../../include/
)

# ============================================================
# 源文件
# ============================================================
file(GLOB_RECURSE SRC_LIST src/*.cpp)

# ============================================================
# 库路径
# ============================================================
link_directories(/opt/unitree_robotics/lib)

# ============================================================
# Target 1: go1_ctrl (实机, UDP)
# ============================================================
add_executable(go1_ctrl main.cpp ${SRC_LIST})
target_include_directories(go1_ctrl PRIVATE
    ${UNITREE_LEGGED_SDK_ROOT}/include   # 旧版 SDK 头文件
)
target_link_libraries(go1_ctrl
    ${UNITREE_LEGGED_LIB}                # Go1 UDP
    unitree_sdk2                         # 共享 FSM 框架
    ddsc ddscxx                          # SDK2 的 DDS 依赖
    pthread rt
    libboost_program_options.a libyaml-cpp.a fmt
    ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so
)

# ============================================================
# Target 2: go1_sim (仿真, DDS)
# ============================================================
add_executable(go1_sim main_sim.cpp ${SRC_LIST})
target_compile_definitions(go1_sim PRIVATE GO1_SIM_MODE)
target_link_libraries(go1_sim
    unitree_sdk2                         # DDS 通信
    ddsc ddscxx                          # DDS 库
    pthread rt
    libboost_program_options.a libyaml-cpp.a fmt
    ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so
)
