cmake_minimum_required(VERSION 3.8)
project(warehouse_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Custom messages and services used by the Warehouse Inventory Robot
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZoneStatus.msg"
  "msg/InventoryItem.msg"
  "msg/InventoryReport.msg"
  "msg/ScanResult.msg"
  "msg/OrderRecord.msg"
  "msg/FetchStatus.msg"
  "msg/ProductStock.msg"
  "msg/StockReport.msg"
  "srv/ScanZone.srv"
  "srv/RequestItem.srv"
  "srv/PickItem.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
