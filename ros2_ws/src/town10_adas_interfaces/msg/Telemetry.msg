# Publication stamp uses ROS time; source_time_s retains original CSV time.
std_msgs/Header header
string run_id
uint64 sequence
float64 source_time_s
string phase
float64 ego_speed_mps
float64 target_speed_mps
float64 distance_m
float64 closing_speed_mps
float64 observed_ttc_s
bool observed_fcw
bool observed_aeb_active
float64 observed_brake_percent
float64 ego_steer
float64 ego_accel_mps2
