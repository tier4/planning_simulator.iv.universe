std_msgs/Header header
uuid_msgs/UniqueID id
dummy_perception_publisher/InitialState initial_state
sensor_msgs/Imu imu
autoware_perception_msgs/Semantic semantic
autoware_perception_msgs/Shape shape
npc_simulator/LaneFollowMode lane_follow_mode
npc_simulator/LaneChangeDir lane_change_dir
int32 lane_change_id

uint8 ADD=0
uint8 MODIFY=1
uint8 MODIFYTURNDIRECTION=2
uint8 MODIFYTWIST=3
uint8 MODIFYLANECHANGE=4
uint8 MODIFYTARGETLANE=5
uint8 MODIFYACCEL=6
uint8 MODIFYROUTE=7
uint8 MODIFYCONSIDERVEHICLE=8
uint8 DELETE=9
uint8 DELETEALL=10
int32 action

float32 accel
float32 target_vel
float32 offset_rate_from_center
bool stop_by_vehicle
std_msgs/Int32MultiArray target_route
