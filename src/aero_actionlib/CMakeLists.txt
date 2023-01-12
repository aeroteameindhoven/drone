cmake_minimum_required(VERSION 3.0.2)
project(aero_actionlib)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    actionlib
    mavros
    mavros_extras
    mavros_msgs
    mavlink
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES aero_actionlib
    CATKIN_DEPENDS roscpp rospy std_msgs actionlib
    mavros mavros_extras mavros_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ~/workspace/drone/devel/include
)

add_executable(aero_actionlibs_takeoff src/takeoff_server.cpp)
add_executable(aero_actionlibs_landing src/landing_server.cpp)

add_dependencies(aero_actionlibs_takeoff ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aero_actionlibs_landing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(aero_actionlibs_takeoff ${catkin_LIBRARIES})
target_link_libraries(aero_actionlibs_landing ${catkin_LIBRARIES})