std_msgs/Header operator_header

uint8 CONTROL_MODE_DIRECT = 0
uint8 CONTROL_MODE_SHARED = 1
uint8 CONTROL_MODE_WAYPOINT = 2
uint8 CONTROL_MODE_PATH_GUIDANCE = 3
uint8 CONTROL_MODE_PERCEPTION_MODIFICATION = 4
uint8 CONTROL_MODE_SAFECORRIDOR = 5
uint8 CONTROL_MODE_NONE = 99

uint8 VIDEO_RATE_CONTROL_MODE_SINGLE = 0
uint8 VIDEO_RATE_CONTROL_MODE_COLLECTIVE = 1
uint8 VIDEO_RATE_CONTROL_MODE_AUTOMATIC = 2

uint8 TOD_STATUS_IDLE = 0
uint8 TOD_STATUS_UPLINK_ONLY = 1
uint8 TOD_STATUS_TELEOPERATION = 2

string operator_operator_ip_address
string operator_broker_ip_address
uint8 operator_control_mode
uint8 operator_video_rate_mode
int8 operator_input_device                 # Details at include/tod_msgs/inputDevice.h

uint8 tod_status

std_msgs/Header vehicle_header
string vehicle_vehicle_ip_address
bool vehicle_emergency_stop_released
bool vehicle_long_approved
bool vehicle_lat_approved
uint8 vehicle_control_mode
string vehicle_nav_status
string vehicle_gps_pos_type
