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
#add_executable(state_machine 			src/state_machine.cpp)
add_executable(send3point      src/send3point.cpp)
add_executable(state_machine_demo     src/state_machine_demo.cpp)
add_executable(state_machine_v1.1      src/state_machine_v1.1.cpp)
add_executable(state_machine_v1.0      src/state_machine_v1.0.cpp)
#add_executable(state_machine_v1.2      src/state_machine_v1.2.cpp)
add_executable(for_test     src/for_test.cpp)
add_executable(state_machine_demo_TL     src/state_machine_demo_TL.cpp)
add_executable(state_machine_VISION     src/state_machine_VISION.cpp)
add_executable(state_machine_demo_TLO     src/state_machine_demo_TLO.cpp)
add_executable(state_machine_demo_NV     src/state_machine_demo_NV.cpp)
add_executable(state_machine_demo_NVNO     src/state_machine_demo_NVNO.cpp)
add_executable(state_machine_demo_TLOS     src/state_machine_demo_TLOS.cpp)
add_executable(state_machine_demo_NVOS     src/state_machine_demo_NVOS.cpp)
add_executable(state_machine_demo_NO     src/state_machine_demo_NO.cpp)
add_executable(state_machine_demo_COM     src/state_machine_demo_COM.cpp)
add_executable(serial_test              src/serial_test.cpp)
add_executable(distance_sensor              src/distance_sensor.cpp)
add_executable(vision_position              src/vision_position.cpp)
add_executable(state_machine_VELO              src/state_machine_VELO.cpp)
add_executable(state_machine_SENSOR              src/state_machine_SENSOR.cpp)
add_executable(state_machine_FixedP_Grab              src/state_machine_FixedP_Grab.cpp)
add_executable(grab_test              src/grab_test.cpp)
add_executable(vision_test              src/vision_test.cpp)
add_executable(state_machine_TL_fix              src/state_machine_TL_fix.cpp)
add_executable(state_machine_TL_fix_hover1              src/state_machine_TL_fix_hover1.cpp)
add_executable(state_machine_GRAB             src/state_machine_GRAB.cpp)
add_executable(state_machine_sensor2              src/state_machine_sensor2.cpp)
add_executable(vision_process_test              src/vision_process_test.cpp)
add_executable(coordinate_exchange              src/coordinate_exchange.cpp)
add_executable(state_machine_G_HR              src/state_machine_G_HR.cpp)
add_executable(state_machine_Continue_Grab              src/state_machine_Continue_Grab.cpp)
add_executable(state_machine_Continue_Grab_ChangeP              src/state_machine_Continue_Grab_ChangeP.cpp)
add_executable(state_machine_Continue_Grab_FixT              src/state_machine_Continue_Grab_FixT.cpp)
add_executable(state_machine_fix_yaw              src/state_machine_fix_yaw.cpp)
add_executable(state_machine_Continue_Grab_ChangeP_Yaw              src/state_machine_Continue_Grab_ChangeP_Yaw.cpp)
add_executable(state_machine_Continue_Grab_FixT_Yaw              src/state_machine_Continue_Grab_FixT_Yaw.cpp)
add_executable(state_machine_FixedP_Grab_Yaw              src/state_machine_FixedP_Grab_Yaw.cpp)
add_executable(state_machine_FixedP_Full_v1.0              src/state_machine_FixedP_Full_v1.0.cpp)
add_executable(state_machine_FixedP_Full_v2.0              src/state_machine_FixedP_Full_v2.0.cpp)
add_executable(state_machine_vision_test              src/state_machine_vision_test.cpp)
add_executable(vision_position_dis_z             src/vision_position_dis_z.cpp)
#add_executable(send_expected_pos 		src/send_expected_pos.cpp)
#add_executable(send10picture_position 	src/send10picture_position.cpp)
#add_executable(pub_board_position 	src/pub_board_position.cpp)
#add_executable(send_board_position 	src/send_board_position.cpp)
#add_executable(get_board_position_receive 	src/get_board_position_receive.cpp)
#add_executable(mavlink_sub_test src/mavlink_sub_msg.cpp)
#add_executable(mavlink_pub_test src/mavlink_pub_msg.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(state_machine 			state_machine_generate_messages_cpp)
add_dependencies(send3point      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo      state_machine_generate_messages_cpp)
add_dependencies(state_machine_v1.1      state_machine_generate_messages_cpp)
add_dependencies(state_machine_v1.0      state_machine_generate_messages_cpp)
#add_dependencies(state_machine_v1.2      state_machine_generate_messages_cpp)
add_dependencies(for_test      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo_TL      state_machine_generate_messages_cpp)
add_dependencies(state_machine_VISION      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo_TLO      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo_NV      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo_NVNO      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo_TLOS      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo_NVOS      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo_NO      state_machine_generate_messages_cpp)
add_dependencies(state_machine_demo_COM      state_machine_generate_messages_cpp)
add_dependencies(serial_test            state_machine_generate_messages_cpp)
add_dependencies(distance_sensor            state_machine_generate_messages_cpp)
add_dependencies(vision_position            state_machine_generate_messages_cpp)
add_dependencies(state_machine_VELO            state_machine_generate_messages_cpp)
add_dependencies(state_machine_SENSOR            state_machine_generate_messages_cpp)
add_dependencies(state_machine_FixedP_Grab            state_machine_generate_messages_cpp)
add_dependencies(grab_test            state_machine_generate_messages_cpp)
add_dependencies(vision_test            state_machine_generate_messages_cpp)
add_dependencies(state_machine_TL_fix            state_machine_generate_messages_cpp)
add_dependencies(state_machine_TL_fix_hover1            state_machine_generate_messages_cpp)
add_dependencies(state_machine_GRAB            state_machine_generate_messages_cpp)
add_dependencies(state_machine_sensor2            state_machine_generate_messages_cpp)
add_dependencies(vision_process_test            state_machine_generate_messages_cpp)
add_dependencies(coordinate_exchange            state_machine_generate_messages_cpp)
add_dependencies(state_machine_G_HR            state_machine_generate_messages_cpp)
add_dependencies(state_machine_Continue_Grab            state_machine_generate_messages_cpp)
add_dependencies(state_machine_Continue_Grab_ChangeP            state_machine_generate_messages_cpp)
add_dependencies(state_machine_Continue_Grab_FixT            state_machine_generate_messages_cpp)
add_dependencies(state_machine_fix_yaw            state_machine_generate_messages_cpp)
add_dependencies(state_machine_Continue_Grab_ChangeP_Yaw            state_machine_generate_messages_cpp)
add_dependencies(state_machine_Continue_Grab_FixT_Yaw            state_machine_generate_messages_cpp)
add_dependencies(state_machine_FixedP_Grab_Yaw            state_machine_generate_messages_cpp)
add_dependencies(state_machine_FixedP_Full_v1.0           state_machine_generate_messages_cpp)
add_dependencies(state_machine_FixedP_Full_v2.0           state_machine_generate_messages_cpp)
add_dependencies(state_machine_vision_test           state_machine_generate_messages_cpp)
add_dependencies(vision_position_dis_z           state_machine_generate_messages_cpp)
#add_dependencies(send_expected_pos 		state_machine_generate_messages_cpp)
#add_dependencies(send10picture_position state_machine_generate_messages_cpp)
#add_dependencies(pub_board_position 	state_machine_generate_messages_cpp)
#add_dependencies(send_board_position 	state_machine_generate_messages_cpp)
#add_dependencies(get_board_position_receive 	state_machine_generate_messages_cpp)
#add_dependencies(mavlink_sub_test state_machine_generate_messages_cpp)
#add_dependencies(mavlink_pub_test state_machine_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(state_machine  			${catkin_LIBRARIES})
target_link_libraries(send3point       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo       ${catkin_LIBRARIES})
target_link_libraries(state_machine_v1.1       ${catkin_LIBRARIES})
target_link_libraries(state_machine_v1.0       ${catkin_LIBRARIES})
#target_link_libraries(state_machine_v1.2       ${catkin_LIBRARIES})
target_link_libraries(for_test       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo_TL       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo_TLO       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo_NV       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo_NVNO       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo_TLOS       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo_NVOS       ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo_NO        ${catkin_LIBRARIES})
target_link_libraries(state_machine_demo_COM        ${catkin_LIBRARIES})
target_link_libraries(serial_test             ${catkin_LIBRARIES})
target_link_libraries(distance_sensor             ${catkin_LIBRARIES})
target_link_libraries(vision_position             ${catkin_LIBRARIES})
target_link_libraries(state_machine_VISION             ${catkin_LIBRARIES})
target_link_libraries(state_machine_VELO             ${catkin_LIBRARIES})
target_link_libraries(state_machine_SENSOR             ${catkin_LIBRARIES})
target_link_libraries(state_machine_FixedP_Grab             ${catkin_LIBRARIES})
target_link_libraries(grab_test             ${catkin_LIBRARIES})
target_link_libraries(vision_test             ${catkin_LIBRARIES})
target_link_libraries(state_machine_TL_fix             ${catkin_LIBRARIES})
target_link_libraries(state_machine_TL_fix_hover1             ${catkin_LIBRARIES})
target_link_libraries(state_machine_GRAB             ${catkin_LIBRARIES})
target_link_libraries(state_machine_sensor2             ${catkin_LIBRARIES})
target_link_libraries(vision_process_test             ${catkin_LIBRARIES})
target_link_libraries(coordinate_exchange             ${catkin_LIBRARIES})
target_link_libraries(state_machine_G_HR             ${catkin_LIBRARIES})
target_link_libraries(state_machine_Continue_Grab            ${catkin_LIBRARIES})
target_link_libraries(state_machine_Continue_Grab_ChangeP             ${catkin_LIBRARIES})
target_link_libraries(state_machine_Continue_Grab_FixT             ${catkin_LIBRARIES})
target_link_libraries(state_machine_fix_yaw             ${catkin_LIBRARIES})
target_link_libraries(state_machine_Continue_Grab_ChangeP_Yaw             ${catkin_LIBRARIES})
target_link_libraries(state_machine_Continue_Grab_FixT_Yaw             ${catkin_LIBRARIES})
target_link_libraries(state_machine_FixedP_Grab_Yaw             ${catkin_LIBRARIES})
target_link_libraries(state_machine_FixedP_Full_v1.0              ${catkin_LIBRARIES})
target_link_libraries(state_machine_FixedP_Full_v2.0              ${catkin_LIBRARIES})
target_link_libraries(state_machine_vision_test              ${catkin_LIBRARIES})
target_link_libraries(vision_position_dis_z              ${catkin_LIBRARIES})
#target_link_libraries(send_expected_pos  		${catkin_LIBRARIES})
#target_link_libraries(send10picture_position  	${catkin_LIBRARIES})
#target_link_libraries(pub_board_position  	${catkin_LIBRARIES})
#target_link_libraries(send_board_position  	${catkin_LIBRARIES})
#target_link_libraries(get_board_position_receive  	${catkin_LIBRARIES})
#target_link_libraries(mavlink_sub_test ${catkin_LIBRARIES})
#target_link_libraries(mavlink_pub_test ${catkin_LIBRARIES})

