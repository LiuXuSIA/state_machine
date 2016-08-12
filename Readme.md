# state_machine

# How to make
```
cd ${HOME}
mkdir -p state_machine_ws/src
cd ${HOME}/state_machine_ws/src
catkin_init_workspace
git clone https://github.com/bing0037/state_machine.git
cd .. # to workspace directory
catkin_make
```

# How to use
## run state_machine nodes
```
roscore
```
```
source ${HOME}/state_machine_ws/devel/setup.bash
rosrun state_machine send4setpoint
```
```
source ${HOME}/state_machine_ws/devel/setup.bash
rosrun state_machine state_machine
```

## run mavros node
### How to connect to pix
https://github.com/SIA-UAVGP/mavros

# Get the topic's messages
rqt_graph


