std_msgs/Header header

# returns true if radio is connected.
bool is_connected

# returns [0, 100] for receiver RSSI.
uint8 receiver_rssi

# channels values.
int16[<=32] channels

# sets true if a channel is overridden.
bool[<=32] active_overrides
