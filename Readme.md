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

# How to connect to pix
https://github.com/SIA-UAVGP/mavros


