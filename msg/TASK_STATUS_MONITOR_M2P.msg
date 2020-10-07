# 
# @brief    : custom mavlink msg, message id="159" name="TASK_STATUS_MONITOR_M2P"
# @time     : 2016/09/13
# @auhtor   : liuxu <liuxu.ccc@gmail.com> 
# 

# Return current task statue and taget point (Mavros->Pix->GCS)

std_msgs/Header header

uint64  timestamp     #timestamp
uint8   task_status   #task status NO. about Task State Machine, 0~16~30
uint8   loop_value    #loop number of current task, 0~5
float32 target_x      #current target X position (NED), in m
float32 target_y      #current target Y position (NED), in m
float32 target_z      #current target Z position (NED), in m
float32 sensor_distance