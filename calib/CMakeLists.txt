cmake_minimum_required(VERSION 3.0.0)
project(calib)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

add_definitions(-w)

find_package(catkin REQUIRED COMPONENTS roscpp tf sensor_msgs image_transport cv_bridge)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
   ${catkin_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIR}
   ${OpenCV_INCLUDE_DIRS}
   ${PROJECT_SOURCE_DIR}/../orbslam3
   ${PROJECT_SOURCE_DIR}/../orbslam3/include
   ${PROJECT_SOURCE_DIR}/../orbslam3/include/CameraModels   
   ${PROJECT_SOURCE_DIR}/../orbslam3/Thirdparty/Sophus
)

# Node for monocular camera
add_executable(calib
   main.cpp
   calib.cpp
   edge.cpp
)

target_link_libraries(calib
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ORB_SLAM3
)