cmake_minimum_required(VERSION 2.8.3)
project(alex_path_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        costmap_2d
        geometry_msgs
        geographic_msgs
        roscpp
        rosbag
        pluginlib
        project11
        project11_msgs
        project11_navigation
        genmsg
        actionlib_msgs
        actionlib
        dubins_curves
        dynamic_reconfigure
        geographic_visualization_msgs
        alex_path_planner_common
        )

find_package(GDAL REQUIRED)

#add_message_files(
#        FILES
#        StateMsg.msg
#        Trajectory.msg
#        DubinsPath.msg
#        Plan.msg
#)

add_action_files(DIRECTORY action FILES alex_path_planner.action)
generate_messages(DEPENDENCIES actionlib_msgs geographic_msgs)

generate_dynamic_reconfigure_options(
        cfg/alex_path_planner.cfg
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/src/executive
        ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/src/planner/*
        ${dubinscurves_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIRS}
        include
)

catkin_package(
#        INCLUDE_DIRS include
        #  LIBRARIES asv_helm
        CATKIN_DEPENDS costmap_2d geometry_msgs roscpp project11 project11_navigation pluginlib
        #  DEPENDS system_lib
)

add_library(alex_path_planner_common # confusing; should change or move to the new package
        src/common/map/Map.cpp
        src/common/dynamic_obstacles/Distribution.cpp
        src/common/dynamic_obstacles/DynamicObstacle.cpp
        src/common/dynamic_obstacles/DynamicObstaclesManager1.cpp
        src/common/map/Costmap2DMap.cpp
        src/common/map/GeoTiffMap.cpp
        src/common/map/GridWorldMap.cpp
        src/common/dynamic_obstacles/BinaryDynamicObstaclesManager.cpp
        src/common/dynamic_obstacles/DynamicObstaclesManagerBase.h
        src/common/dynamic_obstacles/GaussianDynamicObstaclesManager.cpp
        )

target_link_libraries(alex_path_planner_common ${GDAL_LIBRARIES})

add_library(alex_planner
        src/planner/Planner.cpp
        src/planner/search/Vertex.cpp
        src/planner/search/Edge.cpp
        src/planner/utilities/StateGenerator.cpp
        src/planner/SamplingBasedPlanner.cpp
        src/planner/AStarPlanner.cpp
        src/planner/utilities/Ribbon.cpp
        src/planner/utilities/RibbonManager.cpp
        src/planner/PotentialFieldPlanner.cpp src/planner/PotentialFieldPlanner.h
        src/planner/BitStarPlanner.cpp)

add_dependencies(alex_planner alex_path_planner_common)

target_link_libraries(alex_planner alex_path_planner_common ${catkin_LIBRARIES})

add_library(alex_executive
        src/executive/executive.cpp
        )

target_link_libraries(alex_executive alex_planner alex_path_planner_common)

add_executable(${PROJECT_NAME}_node src/path_planner_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node project11_msgs_gencpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)


add_library(${PROJECT_NAME}_plugins src/path_planner_plugin.cpp)
add_dependencies(${PROJECT_NAME}_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES} alex_executive project11_navigation)

add_executable(controller_test_node src/controller_test_node.cpp)
add_dependencies(controller_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(controller_test_node project11_msgs_gencpp path_planner_utilities)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        alex_executive
        )

target_link_libraries(controller_test_node
        ${catkin_LIBRARIES}
        alex_executive
        )

catkin_add_gtest(test_planner test/planner/test_planner.cpp)
target_link_libraries(test_planner planner ${catkin_LIBRARIES})

catkin_add_gtest(test_system test/system/test_executive.cpp test/system/NodeStub.cpp)
target_link_libraries(test_system executive)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

#install(TARGETS ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

install(FILES plugins.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
