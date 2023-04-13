cmake_minimum_required(VERSION 3.5)
project(gb_dialog)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dialogflow_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sound_play REQUIRED)

set(dependencies
    rclcpp
    dialogflow_ros2_interfaces
    std_msgs
    std_srvs
    sound_play
)

include_directories(include)

add_library(gb_dialog SHARED
  src/DialogInterface.cpp
  include/gb_dialog/DialogInterface.hpp
)

ament_target_dependencies(gb_dialog ${dependencies})

add_executable(example_df_node src/example/exampleDF.cpp)
ament_target_dependencies(example_df_node ${dependencies})
target_link_libraries(example_df_node ${PROJECT_NAME})

install(TARGETS
  gb_dialog
  example_df_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(gb_dialog)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()