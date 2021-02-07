cmake_minimum_required(VERSION 2.8.3)
project(UR3e_RG2_ER5_pick_place)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
             gazebo_msgs
             message_generation
             cmake_modules
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_message_files(
   FILES
   object_sphere.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    message_runtime
)
find_package(Eigen REQUIRED)
find_package(TinyXML2 REQUIRED)




###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(pick_place src/full_robot_pick_place.cpp)
add_dependencies(pick_place
    UR3e_RG2_ER5_pick_place_generate_messages_cpp
)

target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
