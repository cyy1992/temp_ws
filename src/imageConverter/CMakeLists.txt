cmake_minimum_required(VERSION 2.6)
project(imageconverter)
add_definitions(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug") 

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include
  ${EIGEN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(imageconverter main.cpp)
target_link_libraries(imageconverter  PUBLIC ${OpenCV_LIBS})
install(TARGETS imageconverter RUNTIME DESTINATION bin)
