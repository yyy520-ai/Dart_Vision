cmake_minimum_required(VERSION 3.16)

# 1. 必须定义项目名称（解决 ${PROJECT_NAME} 未定义问题）
project(io_camera)

# 2. 显式指定C++标准（解决1ms字面量等潜在语法问题）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(yaml-cpp REQUIRED)

# 添加serial子目录
add_subdirectory(serial)

# 创建io静态库目标
add_library(io STATIC 
    hikrobot/hikrobot.cpp 
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    interface/hseven.cpp
)

# ===================== hikrobot 配置 =====================
target_include_directories(io PUBLIC 
    hikrobot/include
)

# 兼容x86_64/aarch64/arm64三种架构写法
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")  # 兼容arm64/aarch64
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
endif()

# ===================== mindvision 配置 =====================
target_include_directories(io PUBLIC 
    mindvision/include
)

# 兼容x86_64/aarch64/arm64三种架构写法
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/x64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# ===================== 库链接（核心修复） =====================
# 1. 先链接依赖库MVSDK，再链接MvCameraControl（链接顺序：被依赖库在前）
# 2. 移除未定义的${PROJECT_NAME}和${MVSDK_LIB_DIR}，直接链接到io目标
# 3. 补充pthread（迈威SDK通常依赖线程库）
target_link_libraries(io 
    PUBLIC
    MVSDK           # 迈威底层SDK库
    MvCameraControl # 迈威相机控制库
    usb-1.0         # USB库
    yaml-cpp        # yaml解析库
    serial          # 串口库
    pthread         # 线程库（迈威SDK必需）
)

# 可选：导出库路径（避免运行时找不到.so文件）
set_target_properties(io PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_WITH_INSTALL_RPATH ON
)