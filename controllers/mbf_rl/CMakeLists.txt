cmake_minimum_required(VERSION 3.10)
project(mbf_rl VERSION 4.0.0 LANGUAGES CXX)

# Set C++ standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# System environment detection
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

set(USE_CMAKE OFF CACHE BOOL "Use Cmake build system")
if(USE_CMAKE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

# Get project root directory
get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/controllers/mbf_rl/policy")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        add_compile_definitions(USE_ROS2)

        find_package(ament_cmake REQUIRED)
        find_package(joint_state_broadcaster REQUIRED)
        find_package(robot_state_publisher REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(robot_msgs REQUIRED)
        find_package(robot_joint_controller REQUIRED)
        find_package(rclpy REQUIRED)
        find_package(std_srvs REQUIRED)
        find_package(geometry_msgs REQUIRED)

        ament_package()
    else()
        message(FATAL_ERROR "Only ROS2 supported")
    endif()
endif()

# Build type ------------------------------------------------------
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()

# Inference Runtime ------------------------------------------------
# ONNX as default backend, Torch optional fallback.
option(USE_ONNX "Enable ONNX Runtime backend" ON)
option(USE_TORCH "Enable LibTorch backend" OFF)
set(ONNX_RUNTIME_DIR "$ENV{HOME}/onnxruntime" CACHE PATH "Path to ONNX Runtime prebuilt directory")
set(LIBTORCH_DIR "$ENV{HOME}/libtorch" CACHE PATH "Path to LibTorch prebuilt directory")

set(ONNX_RUNTIME_LIB "")
if(USE_ONNX)
    if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
        include_directories("${ONNX_RUNTIME_DIR}/include")

        if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
            set(ONNX_LIB_CANDIDATES onnxruntime libonnxruntime.so onnxruntime.so)
        elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
            set(ONNX_LIB_CANDIDATES onnxruntime libonnxruntime.dylib onnxruntime.dylib)
        elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
            set(ONNX_LIB_CANDIDATES onnxruntime onnxruntime.lib)
        endif()

        find_library(ONNX_RUNTIME_LIB_PATH
            NAMES ${ONNX_LIB_CANDIDATES}
            PATHS "${ONNX_RUNTIME_DIR}/lib"
            NO_DEFAULT_PATH
        )

        if(ONNX_RUNTIME_LIB_PATH)
            if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
                add_library(onnxruntime_prebuilt STATIC IMPORTED GLOBAL)
            else()
                add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
            endif()
            set_target_properties(onnxruntime_prebuilt PROPERTIES
                IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
                INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
            )
            set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
            add_compile_definitions(USE_ONNX)
            message(STATUS "Found ONNX Runtime at: ${ONNX_RUNTIME_DIR}")
            message(STATUS "ONNX Runtime library: ${ONNX_RUNTIME_LIB_PATH}")
        else()
            message(FATAL_ERROR "ONNX Runtime enabled but library not found in ${ONNX_RUNTIME_DIR}/lib")
        endif()
    else()
        message(FATAL_ERROR "ONNX Runtime enabled but not found at ${ONNX_RUNTIME_DIR}")
    endif()
endif()

if(USE_TORCH)
    if(EXISTS "${LIBTORCH_DIR}/include" AND EXISTS "${LIBTORCH_DIR}/lib")
        message(STATUS "Found LibTorch at: ${LIBTORCH_DIR}")
        set(CMAKE_PREFIX_PATH "${LIBTORCH_DIR};${CMAKE_PREFIX_PATH}")
        find_package(Torch REQUIRED)
        add_compile_definitions(USE_TORCH)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
    else()
        message(FATAL_ERROR "LibTorch enabled but not found at ${LIBTORCH_DIR}")
    endif()
endif()

if(NOT USE_ONNX AND NOT USE_TORCH)
    message(FATAL_ERROR "At least one inference backend must be enabled: USE_ONNX or USE_TORCH")
endif()

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Add Python library path to RPATH
if(Python3_LIBRARIES)
    get_filename_component(PYTHON_LIB_DIR "${Python3_LIBRARIES}" DIRECTORY)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PYTHON_LIB_DIR}")
    message(STATUS "Added Python library path to RPATH: ${PYTHON_LIB_DIR}")
endif()
if(USE_ONNX)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${ONNX_RUNTIME_DIR}/lib")
    message(STATUS "Added ONNX Runtime path to RPATH: ${ONNX_RUNTIME_DIR}/lib")
endif()
if(USE_TORCH)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${LIBTORCH_DIR}/lib")
    message(STATUS "Added LibTorch path to RPATH: ${LIBTORCH_DIR}/lib")
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
    set(YAML_CPP_LIBRARIES yaml-cpp)
link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})
elseif(SYSTEM_TYPE STREQUAL "Darwin")
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
    set(YAML_CPP_LIBRARIES ${YAML_CPP_LIB})
    link_directories(/opt/homebrew/lib)
    if(YAML_CPP_INCLUDE_DIR)
        include_directories(${YAML_CPP_INCLUDE_DIR})
    endif()

    # Find OpenMP (required by LibTorch on macOS)
    find_library(OMP_LIB NAMES omp PATHS /opt/homebrew/lib /opt/homebrew/opt/libomp/lib)
    if(OMP_LIB)
        message(STATUS "Found OpenMP library: ${OMP_LIB}")
        set(OPENMP_LIBRARIES ${OMP_LIB})
        # Add OpenMP path to RPATH (for LibTorch dependency)
        get_filename_component(OMP_LIB_DIR "${OPENMP_LIBRARIES}" DIRECTORY)
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${OMP_LIB_DIR}")
        message(STATUS "Added OpenMP to RPATH: ${OMP_LIB_DIR}")
    else()
        message(WARNING "OpenMP not found. Please install: brew install libomp")
    endif()
endif()

include_directories(
    include
    library/observation_buffer
    library/rl_sdk
    library/loop
    library/fsm
    library/vector_math
    library/inference_runtime
    library/logger
    fsm_robot
)

# inference_runtime -----------------------------------------------
add_library(inference_runtime library/inference_runtime/inference_runtime.cpp)
target_link_libraries(inference_runtime PUBLIC TBB::tbb Threads::Threads)
if(USE_TORCH)
    target_link_libraries(inference_runtime PUBLIC ${TORCH_LIBRARIES})
    target_compile_definitions(inference_runtime PRIVATE USE_TORCH)
endif()
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(inference_runtime PRIVATE USE_ONNX)
endif()

# rl_sdk core library --------------------------------------------
add_library(rl_sdk library/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk PUBLIC
    inference_runtime
    Python3::Python
    Python3::Module
    TBB::tbb
)

find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS rl_sdk DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for observation buffer
add_library(observation_buffer library/observation_buffer/observation_buffer.cpp)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# Executable ------------------------------------------------------
if(NOT USE_CMAKE)
    add_executable(mbf_rl src/mbf_rl.cpp)

    target_link_libraries(mbf_rl
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )

    ament_target_dependencies(mbf_rl
        joint_state_broadcaster
        robot_state_publisher
        rclcpp
        std_msgs
        robot_msgs
        robot_joint_controller
        rclpy
        std_srvs
        geometry_msgs
    )

    install(TARGETS mbf_rl DESTINATION lib/${PROJECT_NAME})

    # Walk These Ways (WTW) standalone controller
    add_executable(mbf_rl_wtw src/mbf_rl_wtw.cpp)

    target_link_libraries(mbf_rl_wtw
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )

    ament_target_dependencies(mbf_rl_wtw
        joint_state_broadcaster
        robot_state_publisher
        rclcpp
        std_msgs
        robot_msgs
        robot_joint_controller
        rclpy
        std_srvs
        geometry_msgs
    )

    install(TARGETS mbf_rl_wtw DESTINATION lib/${PROJECT_NAME})
endif()

# Policy install --------------------------------------------------
if(NOT USE_CMAKE AND $ENV{ROS_DISTRO} MATCHES "foxy|humble")
    install(
        DIRECTORY ${PROJECT_ROOT_DIR}/controllers/mbf_rl/policy
        DESTINATION share/${PROJECT_NAME}
    )
endif()

message(STATUS "========================================")
message(STATUS "       MBF_RL Build Configuration       ")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "System: ${SYSTEM_TYPE}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "USE_CMAKE: ${USE_CMAKE}")
message(STATUS "----------------------------------------")
message(STATUS "Backend Configuration:")
message(STATUS "USE_TORCH: ${USE_TORCH}")
message(STATUS "USE_ONNX: ${USE_ONNX}")
if(USE_TORCH)
    message(STATUS "LibTorch version: ${Torch_VERSION}")
endif()
if(USE_ONNX)
    message(STATUS "ONNX Runtime dir: ${ONNX_RUNTIME_DIR}")
endif()