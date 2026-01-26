cmake_minimum_required(VERSION 3.0.2)
project(dnb_gantry_simulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    geometry_msgs
    std_msgs
    std_srvs
    dnb_rmi_library
    dnb_frame_manager
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
)

add_service_files(
  FILES
  MoveGantry.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy message_runtime geometry_msgs std_msgs std_srvs dnb_rmi_library dnb_frame_manager visualization_msgs tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dnb_gantry_simulator_node
    src/gantry_controller.cpp
    src/gantry_driver.cpp
    src/gantry_node.cpp
)

add_dependencies(dnb_gantry_simulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(dnb_gantry_simulator_node
    ${catkin_LIBRARIES}
)

install(
    TARGETS dnb_gantry_simulator_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
