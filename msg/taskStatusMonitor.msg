# 
# @brief    : custom mavlink msg, message id="159" name="TASK_STATUS_MONITOR_M2P"
# @time     : 2016/09/13
# @auhtor   : liuxu <liuxu.ccc@gmail.com> 
# 

# Return current task statue and taget point (Mavros->Pix->GCS)

std_msgs/Header header

string   task_status   #task status NO. about Task State Machine, 0~16~30
float32  target_x      #current target X position (NED), in m
float32  target_y      #current target Y position (NED), in m
float32  target_z      #current target Z position (NED), in m
