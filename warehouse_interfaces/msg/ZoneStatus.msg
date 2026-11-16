# Which inventory zone the robot is in. Published by zone_detector on
# /warehouse/current_zone (continuously) and /warehouse/zone_events (ENTER/EXIT).

string EVENT_ENTER="ENTER"
string EVENT_EXIT="EXIT"
string EVENT_INSIDE="INSIDE"
string EVENT_NONE="NONE"

std_msgs/Header header

string zone_id        # e.g. "A", "DROP", "DOCK"; empty when outside every zone
string zone_name      # human readable name, e.g. "Capacitors"
string zone_type      # RACK | DROP | DOCK
string event          # one of the EVENT_* constants

# Robot pose in the map frame when this status was computed
float64 robot_x
float64 robot_y
float64 robot_yaw
