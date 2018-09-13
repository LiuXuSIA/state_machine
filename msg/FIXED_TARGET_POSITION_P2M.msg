# 
# @brief    : custom mavlink msg, message id="152" name="FIXED_TARGET_POSITION_P2M"
# @time     : 2018/09/13
# @auhtor   : liuxu <liuxu.ccc@gmail.com> 
#

# Some fixed positions that would be goten before takeoff by GPS (GCS->Pix->Mavros)

float32 home_x          #home point X position (NED), in m
float32 home_y          #home point Y position (NED), in m
float32 home_z          #home point Z position (NED), in m
float32 component_x     #component_x  point X position (NED), in m
float32 component_y     #component_y  point Y position (NED), in m
float32 component_z     #component_z  point Z position (NED), in m
float32 construction_x  #construction_x point X position (NED) (left), in m
float32 construction_y  #construction_y point Y position (NED) (left), in m
float32 construction_z  #construction_z point Z position (NED) (left), in m
