cmake_minimum_required(VERSION 2.6)
project(test5)
add_definitions(-std=c++11)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS}  -Wall -g -ggdb")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  roscpp
  roslib
  tf
  tf2_eigen
  tf_conversions
  sensor_msgs
  std_msgs
  libpointmatcher_ros
)
catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  )
add_executable(fix_images src/fix_images.cpp)  
target_link_libraries(fix_images  PUBLIC ceres ${OpenCV_LIBS}  )
install(TARGETS fix_images
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# add_executable(test5 src/main.cpp)  
# target_link_libraries(test5  PUBLIC ceres ${OpenCV_LIBS}  )
# install(TARGETS test5
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )