# state_machine

# 1 How to make
```
cd ${HOME}
mkdir -p state_machine_ws/src
cd ${HOME}/state_machine_ws/src
catkin_init_workspace
git clone https://github.com/bing0037/state_machine.git
cd .. # to workspace directory
catkin_make
```

# 2 How to run
## 2-1 simulation
run pixhawk connection:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
run pixhawk&gazebo:
```
cd ~/code/pixhawk/px4_vtol/	#your pixhawk directory
make posix_sitl_default gazebo
```
run state_machine:
```
source ${HOME}/state_machine_ws/devel/setup.bash
rosrun state_machine send4setpoint
```
```
source ${HOME}/state_machine_ws/devel/setup.bash
rosrun state_machine send10picture_position
```
```
source ${HOME}/state_machine_ws/devel/setup.bash
rosrun state_machine offb_simulation_test
```
switch pixhawk to proper mode:
```
rosrun mavros mavsys mode -c OFFBOARD
```
```
rosrun mavros mavsafety arm
```
## 2-2 real flight(connected to pixhawk)
run pixhawk connection:
```
roslaunch mavros px4.launch
```
run state_machine:
```
source ${HOME}/state_machine_ws/devel/setup.bash
rosrun state_machine send4setpoint
```
```
source ${HOME}/state_machine_ws/devel/setup.bash
rosrun state_machine send10picture_position
```
```
source ${HOME}/state_machine_ws/devel/setup.bash
rosrun state_machine offb_simulation_test
```

# Tips:
## How to connect to pix
https://github.com/SIA-UAVGP/mavros

# Get the topic's messages
rqt_graph


