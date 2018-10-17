# 
# @brief    : custom mavlink msg, message id="153" name="FIXED_TARGET_RETURN_M2P"
# @time     : 2018/09/13
# @auhtor   : liuxu <liuxu.ccc@gmail.com> 

# Pix gets FIXED_TARGET_POSITION message would return this message (Mavros->Pix->GCS)

std_msgs/Header header

uint64  timestamp       #timestamp
float32 home_x          #home point X position (NED), in m
float32 home_y          #home point Y position (NED), in m
float32 home_z          #home point Z position (NED), in m
float32 component_x     #component_x  point X position (NED), in m
float32 component_y     #component_y  point Y position (NED), in m
float32 component_z     #component_z  point Z position (NED), in m
float32 construction_x  #construction_x point X position (NED), in m
float32 construction_y  #construction_y point Y position (NED), in m
float32 construction_z  #construction_z point Z position (NED), in m
float32 home_yaw_sp
float32 component_yaw_sp
float32 construction_yaw_sp
