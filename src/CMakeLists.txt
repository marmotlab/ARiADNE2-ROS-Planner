cmake_minimum_required(VERSION 3.0.2)
project(ariadne2)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        pcl_ros
        pcl_conversions
        nav_msgs
        octomap_ros
        octomap_msgs
        sensor_msgs
)

find_package(pybind11 REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros pcl_conversions nav_msgs octomap_ros octomap_msgs
        DEPENDS PCL octomap
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${pybind11_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(map_handler src/map_handler.cpp)
add_executable(terrain_segmentation src/terrain_segmentation.cpp)

target_link_libraries(map_handler
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
)

target_link_libraries(terrain_segmentation
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

pybind11_add_module(collision_checker src/collision_checker.cpp)

target_link_libraries(collision_checker PRIVATE
        ${catkin_LIBRARIES}
        pybind11::module
        ${Python3_LIBRARIES}
)

set_target_properties(collision_checker PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/${PROJECT_NAME}
)

install(TARGETS collision_checker
        LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)