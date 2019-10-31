cmake_minimum_required(VERSION 3.5.0)
add_compile_options(-std=c++11)
project(sineva_stereo)

find_package(catkin REQUIRED
    roscpp
    geometry_msgs
    sensor_msgs
    std_msgs
    cv_bridge
    tf
)

find_package(OpenCV 3.3.1 REQUIRED)

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
    nodes/sineva_stereo_node.cpp
    src/descriptor.cpp
    src/elas.cpp
    src/filter.cpp
    src/matrix.cpp
    src/stereomatch.cpp
    src/triangle.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)


