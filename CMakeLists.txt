cmake_minimum_required(VERSION 2.8.3) #3.0.2
project(tiago_arm_effort_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(CATKIN_COMPONENTS
  controller_interface
  ddynamic_reconfigure
  hardware_interface
  rbdl
  roscpp
  pluginlib
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS} 
  genmsg 
  message_generation
)

find_package(pinocchio REQUIRED)

add_message_files(
  FILES
  TorqueCmd.msg
  EofPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS} message_runtime
#  DEPENDS system_lib
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  /opt/openrobots/include/
)

include_directories(
  include
)

# add_executable(example src/example.cpp)
# target_link_libraries(example pinocchio::pinocchio)

add_library(${PROJECT_NAME} src/controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} pinocchio::pinocchio)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES tiago_arm_effort_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

if (CATKIN_ENABLE_TESTING)

    set(TEST_REQUIRED_CATKIN_COMPONENTS
        controller_manager_msgs
        gazebo_msgs
        ${REQUIRED_CATKIN_COMPONENTS})

    find_package(rostest REQUIRED)
    find_package(catkin REQUIRED COMPONENTS ${TEST_REQUIRED_CATKIN_COMPONENTS})

    include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

endif(CATKIN_ENABLE_TESTING)