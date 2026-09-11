cmake_minimum_required(VERSION 3.5)
project(piper_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz
    DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/joint_state_bridge.py
                  scripts/lidar_pointcloud_correction.py
    DESTINATION lib/${PROJECT_NAME})

ament_package()
