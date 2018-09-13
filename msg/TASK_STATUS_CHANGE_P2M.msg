# 
# @brief    : custom mavlink msg, message id="158" name="TASK_STATUS_CHANGE_P2M"
# @time     : 2016/09/13
# @auhtor   : liuxu <liuxu.ccc@gmail.com> 
# 

# GCS would change the task status of RUAV by this message (GCS->Pix->Mavros)

std_msgs/Header header

uint8 task_status       # task status NO. about Task State Machine, 0~16~30
uint8 loop_value        # loop number of current task, 0~5
