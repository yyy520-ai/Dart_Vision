cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_genai_ubuntu22_2025.0.0.0_x86_64/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    yolov8.cpp
    voter.cpp
    ekfpnp.cpp
    yolo11.cpp
    shooter.cpp
)

target_link_libraries(auto_aim openvino::runtime)