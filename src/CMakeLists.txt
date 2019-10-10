cmake_minimum_required(VERSION 2.8.3) 
project(generate_cali_samples)

find_package(catkin REQUIRED COMPONENTS
 cv_bridge 
 roscpp
 sensor_msgs 
 image_transport 
)

find_package(OpenCV 3.0 REQUIRED)

catkin_package(
)

include_directories(
	include 
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

#set(CPP_SOURCES src/img_sub.cpp) 

add_executable(generate_cali_samples
	src/main.cpp
	)
 
target_link_libraries(generate_cali_samples 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	)

