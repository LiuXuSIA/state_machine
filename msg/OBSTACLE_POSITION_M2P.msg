# 
# @brief    : custom mavlink msg, message id="165" name="OBSTACLE_POSITION_M2P"
# @time     : 2016/09/29
# @auhtor   : libingbing <bike_travel@163.com>
# 

std_msgs/Header header

# Get the obstacle position (Mavros->Pix->GCS)
uint64  timestamp       # timestamp 
float32 obstacle_x      # obstacle X position (NED) 
float32 obstacle_y      # obstacle Y position (NED) 
float32 obstacle_z      # obstacle Z position (NED) 
uint8   obstacle_valid  # whether the obstacle is valid 
