cmake_minimum_required(VERSION 2.8.3)
project(gb_dialog)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dialogflow_ros_msgs
  roslint
  std_srvs
  std_msgs
  sound_play
  actionlib
  actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES gb_dialog
	CATKIN_DEPENDS roscpp dialogflow_ros_msgs actionlib_msgs
	DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references,-whitespace/braces,-build/c++11")
roslint_cpp()

add_library(gb_dialog
	src/DialogInterface.cpp
)
add_dependencies(gb_dialog ${catkin_EXPORTED_TARGETS})
target_link_libraries(gb_dialog ${catkin_LIBRARIES})


add_executable(example_df_node src/example/exampleDF.cpp)
add_dependencies(example_df_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(example_df_node ${catkin_LIBRARIES} gb_dialog)

install(TARGETS gb_dialog
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)
