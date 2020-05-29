cmake_minimum_required(VERSION 3.0.2)
project(mtc_pickplace)

find_package(catkin REQUIRED COMPONENTS
    roscpp
	  moveit_core
	  moveit_task_constructor_core
	  moveit_ros_planning_interface
	  rosparam_shortcuts
    # mtc_pour
    std_msgs
    rviz_marker_tools
)

catkin_package(
   INCLUDE_DIRS include 
#  LIBRARIES mtc_pickplace
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




add_library(${PROJECT_NAME}_lib src/pickplace.cpp)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME mtc_pickplace_pick_place)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}_lib main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(tfBroadcaster src/tfBroadcaster.cpp)
target_link_libraries(tfBroadcaster ${catkin_LIBRARIES})
add_dependencies(tfBroadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)