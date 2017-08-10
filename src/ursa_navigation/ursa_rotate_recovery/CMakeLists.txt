cmake_minimum_required(VERSION 2.8.3)
project(ursa_rotate_recovery)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            costmap_2d
            nav_core
            pluginlib
            base_local_planner
        )

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ursa_rotate_recovery
    CATKIN_DEPENDS
        roscpp
        pluginlib
)

add_library(ursa_rotate_recovery src/ursa_rotate_recovery.cpp)
add_dependencies(ursa_rotate_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ursa_rotate_recovery ${catkin_LIBRARIES})

install(TARGETS ursa_rotate_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES ursa_rotate_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

