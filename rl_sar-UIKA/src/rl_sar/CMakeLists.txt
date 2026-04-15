cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

# Set C++ standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# System environment detection
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_compile_definitions(USE_ROS2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
elseif(UPPER_BUILD_TYPE STREQUAL "RELEASE")
    add_compile_options(-O3)
endif()

# Get project root directory
get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/policy")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)                        
find_package(rclcpp REQUIRED)     
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  
find_package(interfaces REQUIRED)                                                                                              
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)     

# Inference Runtime
set(INFERENCE_RUNTIME_DIR "${PROJECT_ROOT_DIR}/library/inference_runtime")

# ONNX Runtime Setup
set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")
set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")

# Detect Jetson platform
set(IS_JETSON FALSE)
if(EXISTS "/etc/nv_tegra_release")
    set(IS_JETSON TRUE)
    message(STATUS "Jetson platform detected - ONNX Runtime enabled")
endif()

if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
    set(USE_ONNX "ON")
    add_compile_definitions(USE_ONNX)

    message(STATUS "Found ONNX Runtime at: ${ONNX_RUNTIME_DIR}")

    # Read ONNX Runtime version
    if(EXISTS "${ONNX_RUNTIME_DIR}/VERSION_NUMBER")
        file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
        string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
    endif()

    include_directories("${ONNX_RUNTIME_DIR}/include")

    # Detect library extension
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(LIB_EXT "so")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(LIB_EXT "dylib")
    endif()

    # Find library
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_NAMES onnxruntime.lib)
    else()
        set(LIB_NAMES libonnxruntime.${LIB_EXT} onnxruntime.${LIB_EXT})
    endif()

    find_library(ONNX_RUNTIME_LIB_PATH
        NAMES ${LIB_NAMES}
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
        message(STATUS "ONNX Runtime library: ${ONNX_RUNTIME_LIB_PATH}")
    else()
        message(WARNING "ONNX Runtime library not found")
        set(USE_ONNX "OFF")
    endif()
else()
    message(WARNING "ONNX Runtime not found. Please run: bash download_inference_runtime.sh onnx")
endif()

# LibTorch Setup
set(USE_TORCH "OFF")
set(TORCH_LIBRARIES "")
set(LIBTORCH_DIR "${INFERENCE_RUNTIME_DIR}/libtorch")

if(EXISTS "${LIBTORCH_DIR}/include" AND EXISTS "${LIBTORCH_DIR}/lib")
    set(USE_TORCH "ON")
    add_compile_definitions(USE_TORCH)

    message(STATUS "Found LibTorch at: ${LIBTORCH_DIR}")

    set(CMAKE_PREFIX_PATH "${LIBTORCH_DIR};${CMAKE_PREFIX_PATH}")
    find_package(Torch QUIET)

    if(Torch_FOUND)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
        message(STATUS "Torch libraries: ${TORCH_LIBRARIES}")
    else()
        set(USE_TORCH "OFF")
        message(WARNING "LibTorch found but find_package(Torch) failed")
    endif()
else()
    message(WARNING "LibTorch not found. Please run: bash download_inference_runtime.sh libtorch")
endif()

# RPATH Configuration for inference libraries
if(USE_TORCH OR USE_ONNX)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

    set(PREBUILT_LIB_PATHS "")
    if(USE_TORCH)
        list(APPEND PREBUILT_LIB_PATHS "${LIBTORCH_DIR}/lib")
    endif()
    if(USE_ONNX)
        list(APPEND PREBUILT_LIB_PATHS "${ONNX_RUNTIME_DIR}/lib")
    endif()
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PREBUILT_LIB_PATHS}")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
    endif()

    message(STATUS "Configured RPATH:")
    foreach(path ${PREBUILT_LIB_PATHS})
        message(STATUS "  - ${path}")
    endforeach()
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
endif()

include_directories(
    include
    /home/nvidia/UIKA/Quadruped_Uika/install/interfaces/include/interfaces 
    library/core/matplotlibcpp
    library/core/observation_buffer
    library/core/rl_sdk
    library/core/loop
    library/core/fsm
    library/core/vector_math
    library/core/motion_loader
    library/core/inference_runtime
    library/core/logger
    fsm_robot
)

# library for inference runtime
add_library(inference_runtime library/core/inference_runtime/inference_runtime.cpp)
if(USE_TORCH)
    target_link_libraries(inference_runtime PUBLIC ${TORCH_LIBRARIES})
    target_compile_definitions(inference_runtime PRIVATE USE_TORCH)
endif()
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(inference_runtime PRIVATE USE_ONNX)
endif()

# library for motion loader
add_library(motion_loader library/core/motion_loader/motion_loader.cpp)

# library for rl sdk
add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk PUBLIC
    motion_loader
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

# library for observation buffer
add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)

# executable for rl real uika (ROS2 topic interface)
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(uika src/rl_real_uika.cpp)
    target_link_libraries(uika
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )
    ament_target_dependencies(uika
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        interfaces
    )
    install(TARGETS uika DESTINATION lib/${PROJECT_NAME})

    # mock publisher for testing
    add_executable(uika_mock_publisher src/uika_mock_publisher.cpp)
    ament_target_dependencies(uika_mock_publisher
        rclcpp
        std_msgs
    )
    install(TARGETS uika_mock_publisher DESTINATION lib/${PROJECT_NAME})
endif()

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

message(STATUS "========================================")
message(STATUS "       RL_SAR Build Configuration       ")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "System: ${SYSTEM_TYPE}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "----------------------------------------")
message(STATUS "Backend Configuration:")
message(STATUS "USE_TORCH: ${USE_TORCH}")
message(STATUS "USE_ONNX: ${USE_ONNX}")
if(USE_TORCH)
    if(Torch_VERSION)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
    else()
        message(STATUS "LibTorch: Available")
    endif()
endif()
if(USE_ONNX)
    if(ONNX_RUNTIME_VERSION)
        message(STATUS "ONNX Runtime version: ${ONNX_RUNTIME_VERSION}")
    elseif(onnxruntime_VERSION)
        message(STATUS "ONNX Runtime version: ${onnxruntime_VERSION}")
    else()
        message(STATUS "ONNX Runtime: Available")
    endif()
endif()
message(STATUS "========================================")
