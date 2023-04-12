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

add_library(gb_dialog SHARED
  src/DialogInterface.cpp
)

ament_target_dependencies(gb_dialog ${dependencies})
ament_export_libraries(gb_dialog)

add_executable(example_df_node src/example/exampleDF.cpp)
ament_target_dependencies(example_df_node ${dependencies})

include_directories(include)

install(TARGETS
  gb_dialog
  example_df_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()