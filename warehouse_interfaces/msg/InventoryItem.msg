# One catalogued stock item, identified by the ArUco tag (or RFID tag) on its bin.

string STATUS_OK="OK"                 # found in its expected zone
string STATUS_MISSING="MISSING"       # expected zone scanned, item not found anywhere
string STATUS_MISPLACED="MISPLACED"   # found, but in the wrong zone
string STATUS_UNKNOWN="UNKNOWN"       # tag seen that is not in the catalogue
string STATUS_PENDING="PENDING"       # expected zone not scanned yet
string STATUS_RELOCATED="RELOCATED"   # robot moved it to the pick/drop station

int32 marker_id
string sku
string name
uint32 quantity
string expected_zone   # empty for UNKNOWN tags
string found_zone      # empty if not seen
string status          # one of the STATUS_* constants
