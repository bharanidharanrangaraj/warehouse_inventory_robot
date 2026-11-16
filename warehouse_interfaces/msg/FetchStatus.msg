# State of the retrieval robot. Published by fetch_manager on /warehouse/fetch_status.

string ROBOT_STARTING="STARTING"     # waiting for Nav2 / localisation
string ROBOT_IDLE="IDLE"             # ready for orders
string ROBOT_BUSY="BUSY"             # working on an order
string ROBOT_RETURNING="RETURNING"   # going back to the charging dock

std_msgs/Header header
string robot_state       # one of the ROBOT_* constants
string message           # what the robot is doing, in plain words
bool has_current         # true while an order is being executed
OrderRecord current      # valid when has_current is true
OrderRecord[] queue      # waiting orders, oldest first
OrderRecord[] history    # finished orders, newest first
