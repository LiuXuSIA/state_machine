# 
# @brief    : custom mavlink msg, message id="161" name="VISION_POSITION_GET_M2P"
# @time     : 2016/09/3
# @auhtor   : liuxu <liuxu.ccc@gmail.com> 
# 

std_msgs/Header header

# Scan all numbers and get their positions in the beginning (Mavros->Pix->GCS)
uint64  timestamp                    # timestamp
uint8   loop_value                   # current loop_value number
float32 component_position_x         # current component_position X position (NED)
float32 component_position_y         # current component_position Y position (NED)
float32 component_position_z         # current component_position Z position (NED)
