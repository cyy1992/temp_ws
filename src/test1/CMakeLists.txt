cmake_minimum_required(VERSION 2.8.3)
project(test1)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Release") 
add_compile_options(-std=c++11 -O3 -g -DNDEBUG -pthread)
file(GLOB TAG_FILES include/AprilTags/tag*.c)
set(APRILTAG_SRCS include/AprilTags/apriltag.c include/AprilTags/apriltag_pose.c include/AprilTags/apriltag_quad_thresh.c 
include/AprilTags/apriltag_family.c)
aux_source_directory(include/AprilTags/common COMMON_SRC)
# Library
set_source_files_properties(SOURCE ${TAG_FILES} PROPERTIES COMPILE_FLAGS -O0)
set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include/
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/eigen3
    ${Boost_INCLUDE_DIRS}
)
link_libraries( ${Boost_LIBRARIES})

# add_library(apriltag SHARED ${APRILTAG_SRCS} ${COMMON_SRC} ${TAG_FILES})
add_library(apriltag_static STATIC ${APRILTAG_SRCS} ${COMMON_SRC} ${TAG_FILES})

# target_link_libraries(apriltag Threads::Threads m)


add_library(imreg_fmt
    #src/image_dft.cpp
    src/image_transforms.cpp
    src/image_registration.cpp
)

target_link_libraries(imreg_fmt
    ${OpenCV_LIBS}
#     ${FFTW_LIBRARIES}
)
add_executable(image_main
    src/image_main.cpp
    src/ImageProjector.cpp 
)

target_link_libraries(image_main
    imreg_fmt
    
)

add_executable(memory_test
    src/memory_test.cpp
    src/ImageProjector.cpp 
)

target_link_libraries(memory_test
    imreg_fmt
    
)

add_executable(${PROJECT_NAME}_node src/detect_tag.cpp src/ImageProjector.cpp )
# include/AprilTags/Edge.cc include/AprilTags/FloatImage.cc include/AprilTags/GLine2D.cc 
# include/AprilTags/GLineSegment2D.cc include/AprilTags/Gaussian.cc include/AprilTags/GrayModel.cc 
# include/AprilTags/Homography33.cc include/AprilTags/MathUtil.cc include/AprilTags/Quad.cc include/AprilTags/Segment.cc
# include/AprilTags/TagDetection.cc include/AprilTags/TagDetector.cc include/AprilTags/TagFamily.cc include/AprilTags/UnionFindSimple.cc)
target_link_libraries(${PROJECT_NAME}_node apriltag_static pthread ${OpenCV_LIBRARIES}  
)
