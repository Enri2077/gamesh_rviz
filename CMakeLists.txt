cmake_minimum_required(VERSION 2.8.3)
project(gamesh_rviz)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(
	catkin REQUIRED COMPONENTS
	roscpp
	rospy
	roslib
	std_msgs
	shape_msgs
	geometry_msgs
	rviz
	pluginlib
	class_loader
)

## Qt widgets:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(OpenGL REQUIRED)

## Qt signals and slots to avoid defining "emit", "slots", etc
## because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
	LIBRARIES
		gamesh_mesh_rviz
		gamesh_mesh_rviz_core
		gamesh_colored_mesh_rviz
		gamesh_colored_mesh_rviz_core
		${OPENGL_LIBRARIES}
	INCLUDE_DIRS
#TODO ?		gamesh_bridge
		gamesh_mesh_rviz/src
		gamesh_colored_mesh_rviz/src
		${OPENGL_INCLUDE_DIR}
	CATKIN_DEPENDS
		roscpp 
		rospy
		roslib
		std_msgs
		shape_msgs
		geometry_msgs 
		rviz 
		pluginlib 
		class_loader
)

include_directories(
	gamesh_mesh_rviz/src
	gamesh_colored_mesh_rviz/src
	${catkin_INCLUDE_DIRS}
)

include_directories(
	SYSTEM
		${OPENGL_INCLUDE_DIR}
		${QT_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(gamesh_mesh_rviz)
add_subdirectory(gamesh_colored_mesh_rviz)

install(
	FILES
		gamesh_mesh_rviz_plugin_description.xml
		gamesh_colored_mesh_rviz_plugin_description.xml
	DESTINATION
		${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(
#	DIRECTORY
#		icons
#	DESTINATION
#		${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
