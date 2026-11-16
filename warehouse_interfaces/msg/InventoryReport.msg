# Inventory reconciliation for one rack zone, or for the whole warehouse
# when zone_id == "ALL". Published by inventory_manager on /inventory/report.

std_msgs/Header header
string zone_id
string zone_state        # PENDING | OK | ATTENTION | MISSING (empty for "ALL")
InventoryItem[] items

uint32 expected_count    # catalogue items that belong in this zone
uint32 ok_count
uint32 missing_count
uint32 misplaced_count
uint32 unknown_count
uint32 relocated_count
float32 accuracy         # inventory record accuracy = ok_count / expected_count
