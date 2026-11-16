# Shelf stock for every product. Published by item_handler on /warehouse/stock.

std_msgs/Header header
ProductStock[] products
string carried_item     # Gazebo model the robot is carrying, empty if none
uint32 delivered_count  # units sitting on the user counter
