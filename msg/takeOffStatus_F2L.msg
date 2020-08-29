# 
# @brief    : custom message to judge if the folloewer uavs start successfukly, name="takeOffStatus_F2L.msg"
# @time     : 2020/08/28
# @auhtor   : liuxu <liuxu.ccc@gmail.com> 
# 

#Information about component grabbing

std_msgs/Header header

uint64 timestamp   #timestamp
uint8  UAV_index #whether grab the box
uint8  value #whether grab the box