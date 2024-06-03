cmake_minimum_required(VERSION 3.5)
project(netft_utils)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(THIS_PACKAGE_INCLUDE_DEPENDS
ament_cmake
rclcpp
rclcpp_lifecycle
std_msgs
common_interfaces
hardware_interface
geometry_msgs
pluginlib
tf2
tf2_ros
netft_interfaces
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()
find_package(Boost REQUIRED COMPONENTS system thread program_options)

include_directories(src include /usr/include/ ${Boost_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Library for actually talking to the sensor
add_library(netft_rdt_driver SHARED
  src/netft_rdt_driver.cpp
)
target_compile_features(netft_rdt_driver PUBLIC cxx_std_17)
target_include_directories(netft_rdt_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(netft_rdt_driver ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(netft_rdt_driver ${Boost_LIBRARIES})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(netft_rdt_driver PRIVATE "NETFT_RDT_DRIVER_BUILDING_LIBRARY")

# Executable to enable a ROS node bringup
add_executable(netft_node src/netft_node.cpp)
target_include_directories(netft_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(netft_node netft_rdt_driver)

# Library for bringup with ros2_controls hardware_interface
add_library(
  netft_hardware_interface
  SHARED
  src/netft_hardware_interface.cpp
)
target_compile_features(netft_hardware_interface PUBLIC cxx_std_17)
target_include_directories(netft_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/netft_hardware_interface>
)
target_link_libraries(netft_hardware_interface PUBLIC netft_rdt_driver)
ament_target_dependencies(
  netft_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(netft_hardware_interface PRIVATE "NETFT_HARDWARE_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface netft_hardware_interface_plugin.xml)

install(TARGETS netft_hardware_interface
  EXPORT export_netft_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
    TARGETS netft_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS netft_rdt_driver
  EXPORT export_netft_rdt_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_netft_rdt_driver HAS_LIBRARY_TARGET)
ament_export_targets(export_netft_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
