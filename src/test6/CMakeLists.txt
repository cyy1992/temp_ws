cmake_minimum_required(VERSION 2.8.3)
project(test6)
# add_compile_options(-std=c++11 -O3 -DNDEBUG -pthread)
add_compile_options(-std=c++14)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(cartographer REQUIRED)
# include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
# find_package(cartographer_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  roscpp
  roslib
  tf
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  tf2_eigen
  tf_conversions
  sensor_msgs
  geometry_msgs
  std_msgs
  

#   libpointmatcher_ros
#   gps_common
)

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
#   ${libpointmatcher_ros_INCLUDE_DIRS}
#   ${cartographer_INCLUDE_DIRS}
#   ${cartographer_ros_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
#   "${CARTOGRAPHER_CMAKE_DIR}/../../../include"
)
# include_directories(
# BEFORE ${GLOG_INCLUDE_DIRS})
# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# #   
# )
# link_directories(${CARTOGRAPHER_CMAKE_DIR}/../../../lib/)

# add_executable(test6 src/test6_main.cpp)
# target_link_libraries(test6 ${catkin_LIBRARIES} cartographer cartographer_ros)
link_directories(
lib
${catkin_LIBRARIES}
# ${libpointmatcher_LIBRARIES}
# ${CARTOGRAPHER_CMAKE_DIR}/../../../lib/
)

# add_executable(detect_tags src/detect_tag.cpp)
# target_link_libraries(detect_tags ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} apriltag_mit)

# add_executable(point_matcher_mapping src/point_matcher_mapping.cpp)
# target_link_libraries(point_matcher_mapping  ${catkin_LIBRARIES} cartographer cartographer_ros ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(pbstream_convertor src/pbstream_convertor.cpp)
# target_link_libraries(pbstream_convertor cartographer_ros cartographer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
# 
# add_executable(merge_submap src/merge_submap_from_nodes.cpp)
# target_link_libraries(merge_submap cartographer_ros cartographer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(pub_submap_points src/pub_submap_points.cpp)
# target_link_libraries(pub_submap_points  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(calib_gps src/calib_gps.cpp)
# target_link_libraries(calib_gps  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test6 src/test6_main.cpp)
# target_link_libraries(test6  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

# add_executable(tf_to_path src/tf_to_path.cpp)
# target_link_libraries(tf_to_path  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

# add_executable(read_json src/read_json.cpp)
# target_link_libraries(read_json -ljsoncpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add_executable(read_yaml src/read_yaml.cpp)
# target_link_libraries(read_yaml ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})
# add_executable(string_operator src/string_operator.cpp)
# target_link_libraries(string_operator -ljsoncpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})


# add_executable(detect_block_tags src/detect_block_tags.cpp)
# target_link_libraries(detect_block_tags -ljsoncpp 
# image_projector apriltag
# ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# 
add_executable(kdtree src/kd_tree.cpp)

# add_executable(laserscan_to_pointcloud2 src/laserscan_to_pointcloud2.cpp)
# target_link_libraries(laserscan_to_pointcloud2  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# add_subdirectory(launch)

install(TARGETS kdtree
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
    lib/libapriltag.so
    lib/libapriltag.so.3
    lib/libapriltag.so.3.1.0
    lib/libimage_projector.so
    lib/libpointmatcher_ros.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
# install(FILES lib/libapriltag_mit.so
#   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )
file(GLOB_RECURSE HDRS "*.h")
foreach(HDR ${HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      ${INSTALL_DIR}
  )
endforeach()
