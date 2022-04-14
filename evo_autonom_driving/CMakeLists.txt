cmake_minimum_required(VERSION 3.0.2)
project(evo_autonom_driving)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(POINT_CLOUD_FILTER point_cloud_filter)
set(TARGET_LOCALIZER target_localizer)
set(DRIVE_BENEATH_TABLE_CONTROLLER drive_beneath_table_controller)
set(EVO_DRIVE_INSIDE_SERVER evo_drive_inside_server)
set(EVO_DRIVE_INSIDE_ClIENT evo_drive_inside_client)


add_definitions(-Wno-deprecated-declarations -Wno-comment -Wno-deprecated -Wall -Werror )


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  message_generation
  std_msgs
  actionlib_msgs
  actionlib
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  tf2
  tf2_ros
)

add_action_files(
  FILES
  DriveInside.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
   roscpp
   std_msgs
   message_runtime
   visualization_msgs
   geometry_msgs
   actionlib_msgs
   actionlib
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  src/common/ClusterExtractor.cpp
  src/common/Util.cpp
  src/common/RobotUtil.cpp
  src/common/GICP.cpp
  src/TargetLocalizer.cpp
  src/PointCloudFilter.cpp
  src/DriveBeneathTableController.cpp
  src/InitialEstimateCalculator.cpp
)

target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_14)


target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  pcl_visualization
)

#EVO_DRIVE_INSIDE_ClIENT
add_executable(${EVO_DRIVE_INSIDE_ClIENT}
  src/evo_drive_inside_client.cpp
)

target_link_libraries(${EVO_DRIVE_INSIDE_ClIENT}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_dependencies(${EVO_DRIVE_INSIDE_ClIENT}
  ${catkin_EXPORTED_TARGETS}
)

#EVO_DRIVE_INSIDE_SERVER
add_executable(${EVO_DRIVE_INSIDE_SERVER}
  src/evo_drive_inside_server.cpp
)

target_link_libraries(${EVO_DRIVE_INSIDE_SERVER}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_dependencies(${EVO_DRIVE_INSIDE_SERVER}
  ${catkin_EXPORTED_TARGETS}
)

#DRIVE_BENEATH_TABLE_CONTROLLER
add_executable(${DRIVE_BENEATH_TABLE_CONTROLLER}
  src/drive_beneath_table_controller_node.cpp
)

target_link_libraries(${DRIVE_BENEATH_TABLE_CONTROLLER}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_dependencies(${DRIVE_BENEATH_TABLE_CONTROLLER}
  ${catkin_EXPORTED_TARGETS}
)

# TARGET_LOCALIZER
add_executable(${TARGET_LOCALIZER}
  src/target_localizer_node.cpp
)

target_link_libraries(${TARGET_LOCALIZER}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_dependencies(${TARGET_LOCALIZER}
  ${catkin_EXPORTED_TARGETS}
)

# POINT_CLOUD_FILTER
add_executable(${POINT_CLOUD_FILTER}
  src/point_cloud_filter_node.cpp
)

target_link_libraries(${POINT_CLOUD_FILTER}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_dependencies(${POINT_CLOUD_FILTER}
  ${catkin_EXPORTED_TARGETS}
)

# Mark executables and/or libraries for installation
install(
  TARGETS 
   ${EVO_DRIVE_INSIDE_ClIENT} 
   ${EVO_DRIVE_INSIDE_SERVER} 
   ${DRIVE_BENEATH_TABLE_CONTROLLER}
   ${TARGET_LOCALIZER} 
   ${POINT_CLOUD_FILTER}
   ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
install(
  DIRECTORY include/${PROJECT_NAME}/common/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(${CATKIN_ENABLE_TESTING})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/evo_autonom_driving_test.cpp
  test/evo_Tests.cpp
  test/initial_estimator_tests.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)

endif()