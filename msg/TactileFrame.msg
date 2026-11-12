# One retained SDK callback for ONE fingertip. Version 1.
uint16 schema_version
# ROS clock at callback receipt; frame_id is empty (physical frame unverified).
std_msgs/Header header
builtin_interfaces/Time publish_stamp
string source
string side
string finger
uint8 channel
# New UUID for each cache/driver session; receipt_sequence is per finger.
string session_id
uint64 receipt_sequence
bool sdk_frame_id_valid
int64 sdk_frame_id
string sdk_frame_id_raw
bool sdk_timestamp_valid
float64 sdk_timestamp_raw
string sdk_timestamp_repr
string sdk_timestamp_unit
string sdk_timestamp_epoch
# Local host monotonic clock: do not compare across machines or boots.
uint64 receive_monotonic_ns
float64 receive_to_publish_sec
# Cumulative per-finger counters within this session.
uint64 overwritten_frames
uint64 duplicate_callbacks
# Observations, not inferred restart/wrap classification.
string[] identity_warnings
TactileBlock f6
TactileBlock contact_points
TactileBlock dist_force
string[] validation_errors
