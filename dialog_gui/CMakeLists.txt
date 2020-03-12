cmake_minimum_required(VERSION 2.8.3)
project(dialog_gui)

add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_dialogflow
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
foreach (dir launch)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
