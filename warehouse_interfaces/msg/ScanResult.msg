# Raw result of one zone scan. Published by marker_scanner on /inventory/scan_results.

std_msgs/Header header
string zone_id
string scan_mode         # "aruco" (camera) or "rfid_sim" (simulated RFID reader)
int32[] detected_ids     # tags confirmed in at least min_detections frames/reads
uint32 frames_processed
float32 duration         # seconds
