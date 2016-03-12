cmake_minimum_required(VERSION 2.8.3)
project(ros_uw_osl_nessie_vs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  	cv_bridge
  	image_transport
  	roscpp
  	sensor_msgs
  	std_msgs
  	VISP REQUIRED
  	message_generation
)
add_message_files(
  	FILES
  	PilotRequest.msg
)
generate_messages(
  	DEPENDENCIES
  	std_msgs
)
catkin_package(
	DEPENDS
	VISP
	CATKIN_DEPENDS message_runtime
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
  	${VISP_INCLUDE_DIRS}
)

add_executable( servoNessie4Point2DCamVelocity src/servoNessie4Point2DCamVelocity.cpp )
target_link_libraries ( servoNessie4Point2DCamVelocity 
	${catkin_LIBRARIES} 
	${VISP_LIBRARIES}
)
add_executable( findOrientedBoundingBoxe src/findOrientedBoundingBoxe.cpp )
target_link_libraries ( findOrientedBoundingBoxe 
	${catkin_LIBRARIES} 
	${VISP_LIBRARIES}
)

