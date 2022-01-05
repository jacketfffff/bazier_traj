cmake_minimum_required(VERSION 3.0.2)
project(bezier_traj)
#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/sdf_tools)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        visualization_msgs
        cv_bridge
        nav_msgs
        tf
        geometry_msgs
        quadrotor_msgs
        rospy
        message_generation
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS
        system
        thread)
if(NOT Boost_FOUND)
        message("NOT found Boost")
endif()

add_message_files(
        FILES
        PositionCommand.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime
)

link_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/lib/mosek8_1)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/sdf_tools/include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(
        include
        SYSTEM
        third_party
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

add_library(planner_parameter src/planner_parameter.cpp)
target_link_libraries(planner_parameter ${catkin_LIBRARIES})

add_library(collision_map src/collision_map.cpp)
target_link_libraries(collision_map ${catkin_LIBRARIES} planner_parameter sdf_tools ${PCL_LIBRARIES})

add_library(collision_model src/collision_model.cpp)
target_link_libraries(collision_model ${catkin_LIBRARIES} planner_parameter)

#add_library(collision_checking src/collision_checking.cpp)
#target_link_libraries(collision_checking ${catkin_LIBRARIES} ${PCL_LIBRARIES} planner_parameter sdf_tools collision_model)

#add_library(path_finder src/path_finder.cpp)
#target_link_libraries(path_finder ${catkin_LIBRARIES})

add_library(a_star src/astar.cpp)
target_link_libraries(a_star ${catkin_LIBRARIES} planner_parameter collision_model)

#add_library(kino_aStar src/kinodynamic_astar.cpp)
#target_link_libraries(kino_aStar a_star ${catkin_LIBRARIES})

add_library(random_obs_generation src/random_obs_generation.cpp)
target_link_libraries(random_obs_generation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(robot_collision_model src/collision_model.cpp)
target_link_libraries(robot_collision_model ${catkin_LIBRARIES} planner_parameter)

add_library(region_segmentation src/region_segmentation.cpp)
target_link_libraries(region_segmentation ${catkin_LIBRARIES} planner_parameter)

add_library(bezier_trajectory src/bezier_trajectory.cpp)
target_link_libraries(bezier_trajectory ${catkin_LIBRARIES} planner_parameter mosek64)


add_executable(random_obs_generation_node src/random_obs_generation_node.cpp)
target_link_libraries(random_obs_generation_node ${catkin_LIBRARIES} random_obs_generation ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(odom_generation src/odom_generation.cpp)
target_link_libraries(odom_generation ${catkin_LIBRARIES})

add_executable(bezier_traj_node src/bazier_traj_node.cpp)
target_link_libraries(bezier_traj_node collision_map planner_parameter ${catkin_LIBRARIES} ${PCL_LIBRARIES} sdf_tools a_star region_segmentation mosek64)

# add_executable(grid_search src/astar.cpp)
# target_link_libraries(grid_search ${catkin_LIBRARIES} sdf_tools)

#add_library(b_traj src/traj_node.cpp)
#target_link_libraries(b_traj ${catkin_LIBRARIES} ${PCL_LIBRARIES} sdf_tools)
