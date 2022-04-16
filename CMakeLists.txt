cmake_minimum_required(VERSION 3.0.2)
project(rm_track)

find_package(casadi REQUIRED)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint

        rm_msgs
        rm_common

        tf2_geometry_msgs
        apriltag_ros
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        roslint

        rm_msgs
        rm_common

        tf2_geometry_msgs
        apriltag_ros
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}
        src/main.cpp
        src/time_cache.cpp
        src/logic_filter.cpp
        src/logic_selector.cpp
        src/ekf/ekf_base.cpp
        src/rm_track.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        casadi
        )

roslint_cpp()
