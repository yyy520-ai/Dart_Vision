cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

set(VA_LIBRARY /lib/x86_64-linux-gnu/libva.so.2)
set(VA_X11_LIBRARY /lib/x86_64-linux-gnu/libva-x11.so.2)
set(VA_DRM_LIBRARY /lib/x86_64-linux-gnu/libva-drm.so.2)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs calib3d videoio)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
# Ensure serial headers are visible to targets that include io headers directly
include_directories(${PROJECT_SOURCE_DIR}/io/serial/include)

add_subdirectory(tools)
add_subdirectory(io)
# ommited tasks/omniperception: removed to simplify project since auto_aim was deleted
add_subdirectory(tasks/auto_dart)

####################src###################
add_executable(dart src/dart.cpp)
add_executable(dart_debug src/dart_debug.cpp)
target_link_libraries(dart ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_dart)
target_link_libraries(dart_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_dart)
target_link_libraries(dart ${VA_LIBRARY} ${VA_X11_LIBRARY} ${VA_DRM_LIBRARY})
target_link_libraries(dart_debug ${VA_LIBRARY} ${VA_X11_LIBRARY} ${VA_DRM_LIBRARY})

##################calibration################
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(capture calibration/capture.cpp)

target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(calibrate_camera ${VA_LIBRARY} ${VA_X11_LIBRARY} ${VA_DRM_LIBRARY})
target_link_libraries(calibrate_handeye ${VA_LIBRARY} ${VA_X11_LIBRARY} ${VA_DRM_LIBRARY})
target_link_libraries(calibrate_robotworld_handeye ${VA_LIBRARY} ${VA_X11_LIBRARY} ${VA_DRM_LIBRARY})
target_link_libraries(capture ${VA_LIBRARY} ${VA_X11_LIBRARY} ${VA_DRM_LIBRARY})

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
