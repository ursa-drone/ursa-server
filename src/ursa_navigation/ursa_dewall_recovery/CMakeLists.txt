cmake_minimum_required(VERSION 2.8.3)
project(ursa_dewall_recovery)

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
    LIBRARIES ursa_dewall_recovery
    CATKIN_DEPENDS
        roscpp
        pluginlib
)

add_library(ursa_dewall_recovery src/ursa_dewall_recovery.cpp)
add_dependencies(ursa_dewall_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ursa_dewall_recovery ${catkin_LIBRARIES})

install(TARGETS ursa_dewall_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES ursa_dewall_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

