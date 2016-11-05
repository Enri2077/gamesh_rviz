# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/mesh_display_custom.h)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(GAMESH_MESH_LIB_NAME gamesh_mesh_rviz)

add_library(${GAMESH_MESH_LIB_NAME}_core src/mesh_display_custom.cpp ${MOC_SOURCES})
target_link_libraries(${GAMESH_MESH_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_dependencies(${GAMESH_MESH_LIB_NAME}_core ${catkin_EXPORTED_TARGETS})

add_library(${GAMESH_MESH_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${GAMESH_MESH_LIB_NAME} ${GAMESH_MESH_LIB_NAME}_core ${catkin_LIBRARIES})

add_dependencies(${GAMESH_MESH_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(
	TARGETS
		${GAMESH_MESH_LIB_NAME}
		${GAMESH_MESH_LIB_NAME}_core
	ARCHIVE DESTINATION
		${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION
		${CATKIN_PACKAGE_LIB_DESTINATION}
)

#TODO ?
#install(
#	DIRECTORY
#		textures
#	DESTINATION
#		${CATKIN_PACKAGE_SHARE_DESTINATION}/gamesh_mesh_rviz
#)


