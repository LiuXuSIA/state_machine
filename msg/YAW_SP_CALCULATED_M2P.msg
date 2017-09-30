# 
# @brief    : custom mavlink msg, message id="155" name="YAW_SP_CALCULATED_M2P"
# @time     : 2016/09/29
# @auhtor   : libingbing <bike_travel@163.com> 
# 

std_msgs/Header header

# A yaw setpoint calculated by SBC from FIXED_TARGET_POSITION message (Mavros->Pix->GCS)
uint64  timestamp        # timestamp
float32 yaw_sp           # the calculated yaw set point, in rad (-pi ... pi)
