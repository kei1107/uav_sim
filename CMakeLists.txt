cmake_minimum_required(VERSION 2.8.3)
project(uav_sim)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        mavros_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)


include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(test_node src/node/offb_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})


add_executable(multi_test_node src/node/multi_offb_node.cpp)
target_link_libraries(multi_test_node ${catkin_LIBRARIES})
