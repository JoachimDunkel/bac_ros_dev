cmake_minimum_required(VERSION 3.0.2)
project(ira_laser_tools)
add_compile_options(-std=c++14)

find_package(catkin 
  REQUIRED COMPONENTS 
    laser_geometry
    roscpp 
    sensor_msgs 
    std_msgs 
    tf 
    pcl_conversions
    pcl_ros
    )

# bugfix for g++-Warning
#   "<command-line>:0:0: warning: missing whitespace after the macro name"
remove_definitions(-DDISABLE_LIBUSB-1.0)


catkin_package(
   INCLUDE_DIRS
#  LIBRARIES laser_merger
#  CATKIN_DEPENDS laser_geometry roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)


add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)
target_link_libraries(laserscan_multi_merger ${catkin_LIBRARIES} ${PCL_LIBRARIES})


install(TARGETS laserscan_multi_merger 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
