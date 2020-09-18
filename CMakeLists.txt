cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  serial
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Attitude.msg
  State.msg
  Setpoint.msg
  DrawingBoard.msg
  DrawingBoard10.msg
  ActuatorControl.msg

  # custom messages
  FIXED_TARGET_POSITION_P2M.msg
  FIXED_TARGET_RETURN_M2P.msg
  OBSTACLE_POSITION_M2P.msg
  TASK_STATUS_CHANGE_P2M.msg
  TASK_STATUS_MONITOR_M2P.msg
  VISION_NUM_SCAN_M2P.msg
  VISION_ONE_NUM_GET_M2P.msg
  YAW_SP_CALCULATED_M2P.msg
  FailureRecord.msg
  GRAB_STATUS_M2P.msg
  VISION_POSITION_GET_M2P.msg
  Distance.msg
  Vision_Position_Raw.msg
  FIXED_BOX_POSITION_P2M.msg
  FIXED_BOX_RETURN_M2P.msg
  Distance_Measure_Enable.msg
  Task_Status_to_Vision.msg
  Vision_Position_Multiple.msg

  #add for 2020 uavcomp
  requestCommand_L2F.msg
  attributeStatus_F2L.msg
  positionXY.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
#   Service1.srv
	CommandTOL.srv
	CommandBool.srv	# for simulation -libn 2016.08.15
	SetMode.srv		# for simulation -libn 2016.08.15
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs  # Or other packages containing msgs
)


catkin_package(
CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime serial
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(state_machine_demo     src/state_machine_demo.cpp)
add_executable(for_test     src/for_test.cpp)
add_executable(vision_test              src/vision_test.cpp)
add_executable(vision_process_test              src/vision_process_test.cpp)
add_executable(state_machine_g1             src/state_machine_g1.cpp)
add_executable(state_machine_g2             src/state_machine_g2.cpp)
add_executable(state_machine_final             src/state_machine_final.cpp)
add_executable(state_machine_leader             src/state_machine_leader.cpp)
add_executable(state_machine_follower_1             src/state_machine_follower_1.cpp)
add_executable(state_machine_follower_2             src/state_machine_follower_2.cpp)
add_executable(state_machine_follower_3             src/state_machine_follower_3.cpp)
add_executable(state_machine_follower_4             src/state_machine_follower_4.cpp)
add_executable(state_machine_commander             src/state_machine_commander.cpp)
add_executable(state_machine_fullauto_commander             src/state_machine_fullauto_commander.cpp)
add_executable(state_machine_fullauto_demo            src/state_machine_fullauto_demo.cpp)
add_executable(emergency            src/emergency.cpp)
add_executable(collaborate_uav1            src/collaborate_uav1.cpp)
add_executable(collaborate_uav2            src/collaborate_uav2.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(state_machine_demo      state_machine_generate_messages_cpp)
add_dependencies(for_test      state_machine_generate_messages_cpp)
add_dependencies(vision_test            state_machine_generate_messages_cpp)
add_dependencies(vision_process_test            state_machine_generate_messages_cpp)
add_dependencies(state_machine_g1            state_machine_generate_messages_cpp)
add_dependencies(state_machine_g2            state_machine_generate_messages_cpp)
add_dependencies(state_machine_final            state_machine_generate_messages_cpp)
add_dependencies(state_machine_leader            state_machine_generate_messages_cpp)
add_dependencies(state_machine_follower_1            state_machine_generate_messages_cpp)
add_dependencies(state_machine_follower_2            state_machine_generate_messages_cpp)
add_dependencies(state_machine_follower_3            state_machine_generate_messages_cpp)
add_dependencies(state_machine_follower_4            state_machine_generate_messages_cpp)
add_dependencies(state_machine_commander            state_machine_generate_messages_cpp)
add_dependencies(state_machine_fullauto_commander            state_machine_generate_messages_cpp)
add_dependencies(state_machine_fullauto_demo            state_machine_generate_messages_cpp)
add_dependencies(emergency            state_machine_generate_messages_cpp)
add_dependencies(collaborate_uav1            state_machine_generate_messages_cpp)
add_dependencies(collaborate_uav2            state_machine_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(state_machine_demo       ${catkin_LIBRARIES})
target_link_libraries(for_test       ${catkin_LIBRARIES})
target_link_libraries(vision_test             ${catkin_LIBRARIES})
target_link_libraries(vision_process_test             ${catkin_LIBRARIES})
target_link_libraries(state_machine_g1              ${catkin_LIBRARIES})
target_link_libraries(state_machine_g2              ${catkin_LIBRARIES})
target_link_libraries(state_machine_final              ${catkin_LIBRARIES})
target_link_libraries(state_machine_leader              ${catkin_LIBRARIES})
target_link_libraries(state_machine_follower_1              ${catkin_LIBRARIES})
target_link_libraries(state_machine_follower_2              ${catkin_LIBRARIES})
target_link_libraries(state_machine_follower_3              ${catkin_LIBRARIES})
target_link_libraries(state_machine_follower_4              ${catkin_LIBRARIES})
target_link_libraries(state_machine_commander              ${catkin_LIBRARIES})
target_link_libraries(state_machine_fullauto_commander              ${catkin_LIBRARIES})
target_link_libraries(state_machine_fullauto_demo              ${catkin_LIBRARIES})
target_link_libraries(emergency              ${catkin_LIBRARIES})
target_link_libraries(collaborate_uav1              ${catkin_LIBRARIES})
target_link_libraries(collaborate_uav2              ${catkin_LIBRARIES})