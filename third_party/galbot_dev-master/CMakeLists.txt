cmake_minimum_required(VERSION 3.16)
project(galbot_app CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 关闭 RPATH
set(CMAKE_SKIP_RPATH TRUE)

# 定义 SDK 根目录
set(SDK_DIR "/opt/galbot/galbot_sdk/linux-x86_64-gcc940")

# 【关键修复】手动把所有的头文件路径加进来！
include_directories(
        ${SDK_DIR}/include
        ${SDK_DIR}/include/galbot_sdk
        ${SDK_DIR}/include/eigen3
        ${SDK_DIR}/include/pcl-1.13
        ${SDK_DIR}/include/opencv4
)

# 手动把库文件路径加进来！
link_directories(${SDK_DIR}/lib)

# 你的程序
add_executable(galbot_app src/main.cpp)

# 链接库（手动列出，绕过 common.cmake 的拼接 Bug）
target_link_libraries(galbot_app
        galbot_sdk fastcdr fastrtps boost_thread spdlog tinyxml2
        foonathan_memory-0.7.3 ssl crypto embosa protobuf rt z dl pthread
        embosa_basic_interface tf2_base tf2_embosa opencv_core opencv_imgproc
        opencv_imgcodecs png jpeg pcl_common gomp
)