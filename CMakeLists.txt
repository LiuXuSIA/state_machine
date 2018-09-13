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
  VISITION_POSITION_GET_M2P.msg

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
CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
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
#add_executable(state_machine 			src/state_machine.cpp)
add_executable(send4setpoint 			src/send4setpoint.cpp)
add_executable(send3point      src/send3point.cpp)
add_executable(state_machine_demo     src/state_machine_demo.cpp)
add_executable(state_machine_V1      src/state_machine_V1.cpp)
add_executable(for_test     src/for_test.cpp)
add_executable(state_machine_demo_TL     src/state_machine_demo_TL.cpp)
#add_executable(send_expected_pos 		src/send_expected_pos.cpp)
#add_executable(send10picture_position 	src/send10picture_position.cpp)
add_executable(offb_simulation_test 	src/offb_simulation_test.cpp)
add_executable(get_board_position 	src/get_board_position.cpp)
#add_executable(pub_board_position 	src/pub_board_position.cpp)
#add_executable(send_board_position 	src/send_board_position.cpp)
#add_executable(get_board_position_receive 	src/get_board_position_receive.cpp)
#add_executable(mavlink_sub_test src/mavlink_sub_msg.cpp)
#add_executable(mavlink_pub_test src/mavlink_pub_msg.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(state_machine 			state_machine_generate_messages_cpp)
add_dependencies(send4setpoint 			state_machine_generate_messages_cpp)
add_dependencies(send3point      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo      state_machine_generate_messages_cpp)
add_dependencies(state_machine_V1      state_machine_generate_messages_cpp)
add_dependencies(for_test      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo_TL      state_machine_generate_messages_cpp)
#add_dependencies(send_expected_pos 		state_machine_generate_messages_cpp)
#add_dependencies(send10picture_position state_machine_generate_messages_cpp)
add_dependencies(offb_simulation_test 	state_machine_generate_messages_cpp)
add_dependencies(get_board_position 	state_machine_generate_messages_cpp)
#add_dependencies(pub_board_position 	state_machine_generate_messages_cpp)
#add_dependencies(send_board_position 	state_machine_generate_messages_cpp)
#add_dependencies(get_board_position_receive 	state_machine_generate_messages_cpp)
#add_dependencies(mavlink_sub_test state_machine_generate_messages_cpp)
#add_dependencies(mavlink_pub_test state_machine_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(state_machine  			${catkin_LIBRARIES})
target_link_libraries(send4setpoint  			${catkin_LIBRARIES})
target_link_libraries(send3point       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo       ${catkin_LIBRARIES})
target_link_libraries(state_machine_V1       ${catkin_LIBRARIES})
target_link_libraries(for_test       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo_TL       ${catkin_LIBRARIES})
#target_link_libraries(send_expected_pos  		${catkin_LIBRARIES})
#target_link_libraries(send10picture_position  	${catkin_LIBRARIES})
target_link_libraries(offb_simulation_test  	${catkin_LIBRARIES})
target_link_libraries(get_board_position  	${catkin_LIBRARIES})
#target_link_libraries(pub_board_position  	${catkin_LIBRARIES})
#target_link_libraries(send_board_position  	${catkin_LIBRARIES})
#target_link_libraries(get_board_position_receive  	${catkin_LIBRARIES})
#target_link_libraries(mavlink_sub_test ${catkin_LIBRARIES})
#target_link_libraries(mavlink_pub_test ${catkin_LIBRARIES})

