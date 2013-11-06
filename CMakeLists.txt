cmake_minimum_required(VERSION 2.8.3)
project(nao_detector)

find_package(catkin REQUIRED COMPONENTS pcl_ros roscpp rospy std_msgs message_generation genmsg sensor_msgs)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(nao_detector_algo src/nao_detector_algo.cpp)
target_link_libraries(nao_detector_algo ${catkin_LIBRARIES})

add_executable(nao_detector_kp src/nao_detector_keypoints.cpp)
target_link_libraries(nao_detector_kp ${catkin_LIBRARIES})

add_executable(nao_simulator src/nao_simulator.cpp)
target_link_libraries(nao_simulator ${catkin_LIBRARIES})

