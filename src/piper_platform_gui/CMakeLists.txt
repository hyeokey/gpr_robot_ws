cmake_minimum_required(VERSION 3.16)
project(piper_platform_gui)

if(POLICY CMP0100)
  # Default (OLD) policy excludes .hh from AUTOMOC scanning; our Q_OBJECT
  # class lives in a .hh header, so this must be NEW for moc to run at all.
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Qml Quick)

qt5_add_resources(PIPER_PLATFORM_GUI_RCC src/piper_platform_gui.qrc)

add_library(PiperPlatformGui SHARED
  src/PiperPlatformGui.cc
  include/piper_platform_gui/PiperPlatformGui.hh
  ${PIPER_PLATFORM_GUI_RCC}
)

target_include_directories(PiperPlatformGui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(PiperPlatformGui
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(PiperPlatformGui
  gz-gui8::gz-gui8
  Qt5::Core
  Qt5::Qml
  Qt5::Quick
)

# gz-sim looks for GUI plugins by bare filename on GZ_GUI_PLUGIN_PATH; it
# does not use the lib<name>.so.<version> SONAME convention.
set_target_properties(PiperPlatformGui PROPERTIES
  PREFIX "lib"
  NO_SONAME ON
)

install(TARGETS PiperPlatformGui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
