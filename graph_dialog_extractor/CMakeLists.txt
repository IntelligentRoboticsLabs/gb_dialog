cmake_minimum_required(VERSION 2.8.3)
project(graph_dialog_extractor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  bica
  bica_graph
  roscpp
  rospy
  bica_msgs
  gb_dialog
  roslint
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES graph_dialog_extractor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references")
roslint_cpp()

add_library(floor_df_lib src/libs/floorDF_lib.cpp)
add_dependencies(floor_df_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(floor_df_lib ${catkin_LIBRARIES} gb_dialog)

add_library(order_ready_df_lib src/libs/orderReadyDF_lib.cpp)
add_dependencies(order_ready_df_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(order_ready_df_lib ${catkin_LIBRARIES} gb_dialog)

add_library(confirm_order_df_lib src/libs/confirmOrderDF_lib.cpp)
add_dependencies(confirm_order_df_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(confirm_order_df_lib ${catkin_LIBRARIES} gb_dialog)

add_executable(graph_dialog_extractor_node src/graph_dialog_extractor_node.cpp)
add_dependencies(graph_dialog_extractor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_dialog_extractor_node
  floor_df_lib
  order_ready_df_lib
  confirm_order_df_lib
  ${catkin_LIBRARIES})

add_executable(test_graph_dialog_extractor src/test_graph_dialog_extractor.cpp)
add_dependencies(test_graph_dialog_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_graph_dialog_extractor ${catkin_LIBRARIES})
