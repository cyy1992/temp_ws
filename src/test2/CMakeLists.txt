cmake_minimum_required(VERSION 2.8.3)
project(test2)
add_compile_options(-std=c++11 -O3 -DNDEBUG -pthread)
# add_compile_options(-std=c++14)
# add_definitions(-std=c++11)
SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O3")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -g -ggdb")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include
  ${EIGEN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS})

add_executable(kalman_test src/kalman_test.cpp)
target_link_libraries(kalman_test ${OpenCV_LIBRARIES})

