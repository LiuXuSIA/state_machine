# 
# @brief    : custom mavlink msg, message id="161" name="VISION_NUM_SCAN_M2P"
# @time     : 2016/09/29
# @auhtor   : libingbing <bike_travel@163.com> 
# 

std_msgs/Header header

# Scan all numbers and get their positions in the beginning (Mavros->Pix->GCS)
uint64  timestamp       # timestamp
uint8   board_num       # current board number
float32 board_x         # current board number X position (NED)
float32 board_y         # current board number Y position (NED)
float32 board_z         # current board number Z position (NED)
uint8   board_valid     # whether this number is gotten
