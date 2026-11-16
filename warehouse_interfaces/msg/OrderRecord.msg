# One customer order handled by fetch_manager (retrieval mode).

string STATE_QUEUED="QUEUED"              # waiting for the robot
string STATE_TO_RACK="TO_RACK"            # driving to the product's rack
string STATE_CONFIRMING="CONFIRMING"      # reading the rack's bin labels with the camera
string STATE_PICKING="PICKING"            # taking one unit off the shelf
string STATE_TO_USER="TO_USER"            # carrying it to the user counter
string STATE_HANDING_OVER="HANDING_OVER"  # putting it down on the counter
string STATE_DELIVERED="DELIVERED"        # done
string STATE_FAILED="FAILED"              # could not be completed (see message)

string order_id
string product          # product id, e.g. "diode"
string product_name     # human readable, e.g. "Diode"
string rack_id          # rack that stocks the product, e.g. "D"
string state            # one of the STATE_* constants
string message          # latest progress or error text
float64 requested_at    # wall-clock time the order was placed (seconds since epoch)
float64 duration        # seconds from start to finish; time so far while running
