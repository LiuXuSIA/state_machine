# 
# @brief    : custom mavlink msg, message id="155" name="YAW_SP_CALCULATED_M2P"
# @time     : 2018/09/13
# @auhtor   : liuxu <liuxu.ccc@gmail.com> 
# 

# A yaw setpoint calculated by SBC from FIXED_TARGET_POSITION message (Mavros->Pix->GCS)

std_msgs/Header header

uint64  timestamp        # timestamp
float32 yaw_sp           # the calculated yaw set point, in rad (-pi ... pi)
