cmake_minimum_required(VERSION 2.8.3)
project(auv_task)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge image_transport sensor_msgs)

find_package(OpenCV REQUIRED)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()
include_directories(
	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(talker_image src/talker.cpp)
target_link_libraries(talker_image
	${OpenCV_LIBRARIES}
	${catkin_LIBRARIES}
)
add_dependencies(talker_image beginner_tutorials_generate_messages_cpp)

add_executable(listener_image src/listener.cpp)
target_link_libraries(listener_image
	${OpenCV_LIBRARIES}
	${catkin_LIBRARIES}
)
add_dependencies(listener_image beginner_tutorials_generate_messages_cpp)

add_executable(kalman src/kalman.cpp)
target_link_libraries(kalman
	${OpenCV_LIBRARIES}
	${catkin_LIBRARIES}
)
add_dependencies(kalman beginner_tutorials_generate_messages_cpp)

add_executable(trackerdisp src/trackerdisp.cpp)
target_link_libraries(trackerdisp
	${OpenCV_LIBRARIES}
	${catkin_LIBRARIES}
)
add_dependencies(trackerdisp beginner_tutorials_generate_messages_cpp)
