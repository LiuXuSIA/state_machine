cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Attitude.msg
  State.msg
  Setpoint.msg
  DrawingBoard.msg
  ActuatorControl.msg
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
  std_msgs  # Or other packages containing msgs
)


catkin_package(
CATKIN_DEPENDS message_runtime
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
add_executable(state_machine src/state_machine.cpp)
add_executable(send4setpoint src/send4setpoint.cpp)
add_executable(send_expected_pos src/send_expected_pos.cpp)
add_executable(send10picture_position src/send10picture_position.cpp)
add_executable(offb_simulation_test src/offb_simulation_test.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(state_machine state_machine_generate_message_cpp)
add_dependencies(send4setpoint send4setpoint_generate_message_cpp)
add_dependencies(send_expected_pos send_expected_pos_generate_message_cpp)
add_dependencies(send10picture_position send10picture_position_generate_message_cpp)
add_dependencies(offb_simulation_test offb_simulation_test_generate_message_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(state_machine  ${catkin_LIBRARIES})
target_link_libraries(send4setpoint  ${catkin_LIBRARIES})
target_link_libraries(send_expected_pos  ${catkin_LIBRARIES})
target_link_libraries(send10picture_position  ${catkin_LIBRARIES})
target_link_libraries(offb_simulation_test  ${catkin_LIBRARIES})


