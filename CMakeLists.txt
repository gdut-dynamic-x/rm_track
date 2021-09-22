cmake_minimum_required(VERSION 3.0.2)
project(rm_track)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint

        rm_msgs
        rm_common

        tf2_geometry_msgs
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
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_library(${PROJECT_NAME}
        src/cache.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

roslint_cpp()
