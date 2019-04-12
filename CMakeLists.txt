cmake_minimum_required(VERSION 2.8.3)
project(vrx_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  vrx_control
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vrx_bringup
  CATKIN_DEPENDS joint_state_publisher robot_state_publisher vrx_control
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Install launch files
foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
