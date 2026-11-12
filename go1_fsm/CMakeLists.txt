cmake_minimum_required(VERSION 3.10)
project(go1_fsm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie")
set(CMAKE_INSTALL_RPATH "$ORIGIN/../third_party/onnxruntime/lib/amd64")

# ============================================================
# 路径配置
# ============================================================
if(NOT DEFINED UNITREE_SDK_ROOT)
    if(EXISTS "${CMAKE_SOURCE_DIR}/third_party/unitree_legged_sdk")
        set(UNITREE_SDK_ROOT "${CMAKE_SOURCE_DIR}/third_party/unitree_legged_sdk")
    else()
        set(UNITREE_SDK_ROOT "${CMAKE_SOURCE_DIR}/../unitree_legged_sdk")
    endif()
endif()
message(STATUS "Unitree SDK: ${UNITREE_SDK_ROOT}")

set(BOOST_ROOT        "${CMAKE_SOURCE_DIR}/third_party/boost")
set(BOOST_INCLUDEDIR  "${BOOST_ROOT}")
set(BOOST_LIBRARYDIR  "${BOOST_ROOT}/lib/amd64")

# 所有目标的公共 include 路径
set(COMMON_INCLUDES
    ${CMAKE_SOURCE_DIR}/fsm/include
    ${CMAKE_SOURCE_DIR}
)
set(SDK_INCLUDES ${COMMON_INCLUDES} ${UNITREE_SDK_ROOT}/include ${BOOST_INCLUDEDIR})
set(SDK_LIBS pthread ${UNITREE_ROBOT_LIB} ${BOOST_LIBRARYDIR}/libboost_system.a)

# 自动检测架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(UNITREE_LIB_DIR ${UNITREE_SDK_ROOT}/lib/cpp/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(UNITREE_LIB_DIR ${UNITREE_SDK_ROOT}/lib/cpp/arm64)
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_library(UNITREE_ROBOT_LIB unitree_legged_sdk
    PATHS ${UNITREE_LIB_DIR} NO_DEFAULT_PATH)
if(NOT UNITREE_ROBOT_LIB)
    message(FATAL_ERROR "SDK not found in ${UNITREE_LIB_DIR}")
endif()

# ============================================================
# 可选推理后端
# ============================================================
option(USE_ONNX  "ONNX Runtime backend" OFF)
option(USE_TORCH "LibTorch backend"     OFF)
if(USE_ONNX)
    find_path(ONNX_INCLUDE_DIR onnxruntime_cxx_api.h PATHS ${CMAKE_SOURCE_DIR}/third_party/onnxruntime/include)
    find_library(ONNX_LIB onnxruntime PATHS ${CMAKE_SOURCE_DIR}/third_party/onnxruntime/lib/amd64)
    if(ONNX_INCLUDE_DIR AND ONNX_LIB)
        add_definitions(-DHAS_ONNXRUNTIME)
        include_directories(${ONNX_INCLUDE_DIR})
        message(STATUS "ONNX: ${ONNX_LIB}")
    endif()
endif()
if(USE_TORCH)
    find_package(Torch QUIET)
    if(Torch_FOUND)
        add_definitions(-DHAS_LIBTORCH)
        message(STATUS "LibTorch: found")
    endif()
endif()

# ============================================================
# FSM 核心源文件 (所有目标共用)
# ============================================================
set(FSM_CORE
    fsm/src/state_machine.cpp
    fsm/src/model_loader.cpp
    fsm/src/states/stand_state.cpp
    fsm/src/states/down_state.cpp
    fsm/src/states/damping_state.cpp
    fsm/src/states/model_state.cpp
)

# ============================================================
# 目标: go1_fsm (主程序)
# ============================================================
add_executable(go1_fsm ${FSM_CORE} fsm/src/fsm_controller.cpp app/main.cpp)
target_include_directories(go1_fsm PRIVATE ${SDK_INCLUDES} ${ONNX_INCLUDE_DIR})
target_link_directories(go1_fsm PRIVATE ${CMAKE_SOURCE_DIR}/third_party/onnxruntime/lib/amd64)
target_link_libraries(go1_fsm ${SDK_LIBS} ${ONNX_LIB} ${TORCH_LIBRARIES})

# ============================================================
# 目标: test_fsm (单元测试, 无硬件依赖)
# ============================================================
add_executable(test_fsm ${FSM_CORE} test/test_fsm.cpp)
target_include_directories(test_fsm PRIVATE ${COMMON_INCLUDES})
target_link_libraries(test_fsm pthread ${ONNX_LIB})

# ============================================================
# 目标: test_stand_down (状态演示)
# ============================================================
add_executable(test_stand_down ${FSM_CORE} test/test_stand_down.cpp)
target_include_directories(test_stand_down PRIVATE ${COMMON_INCLUDES})
target_link_libraries(test_stand_down pthread ${ONNX_LIB})

# ============================================================
# 目标: test_read_state (SDK 通信测试)
# ============================================================
add_executable(test_read_state test/test_read_state.cpp)
target_include_directories(test_read_state PRIVATE ${SDK_INCLUDES})
target_link_libraries(test_read_state ${SDK_LIBS})

# ============================================================
# 目标: test_joint_dir (关节方向测试, 阻尼+只读)
# ============================================================
add_executable(test_joint_dir test/test_joint_dir.cpp)
target_include_directories(test_joint_dir PRIVATE ${SDK_INCLUDES})
target_link_libraries(test_joint_dir ${SDK_LIBS})
