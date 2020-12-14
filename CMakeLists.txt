cmake_minimum_required(VERSION 3.0.2)
project(vicon_franka_integration)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp 
	vrpn_client_ros
	interactive_markers
	moveit_core
	moveit_visual_tools
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_perception
	pluginlib
	geometric_shapes
	pcl_ros
	pcl_conversions
	rosbag
	tf2_ros
	tf2_eigen
	tf2_geometry_msgs
	)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include
	#  LIBRARIES vicon_franka_integration
	CATKIN_DEPENDS 
		vrpn_client_ros 
		geometry_msgs 
		tf2_ros
		tf2_geometry_msgs
		moveit_core
		moveit_visual_tools
		moveit_ros_planning_interface
		interactive_markers
	DEPENDS
		EIGEN3
	#  DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	)

add_executable(${PROJECT_NAME}_node src/vicon_franka_integration_node.cpp)
add_executable(vicon_pick_place_node src/vicon_pick_place_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(vicon_pick_place_node ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS vicon_pick_place_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
