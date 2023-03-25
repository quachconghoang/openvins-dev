cmake_minimum_required(VERSION 3.5)

project(ov_superpoint)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "release")
add_definitions(-w)

#add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer)


find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)
#find_package(vpi 2 REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp rosbag sensor_msgs cv_bridge
        INCLUDE_DIRS src/
        LIBRARIES ${PROJECT_NAME}_lib
)

include_directories(
        src
        ${PROJECT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
)

add_library(tensorrtbuffer SHARED
        src/tensorrtbuffer/logger.cpp)


add_library(${PROJECT_NAME}_lib SHARED
  src/super_point.cpp
  src/super_glue.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
      nvinfer
      nvonnxparser
      ${OpenCV_LIBRARIES}
      ${CUDA_LIBRARIES}
      yaml-cpp
      tensorrtbuffer
#        vpi
)

add_executable(${PROJECT_NAME}_image inference_image.cpp)
target_link_libraries(${PROJECT_NAME}_image ${PROJECT_NAME}_lib)
