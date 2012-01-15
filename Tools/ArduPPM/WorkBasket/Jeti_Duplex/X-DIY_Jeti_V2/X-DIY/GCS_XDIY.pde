#if GCS_PROTOCOL == GCS_PROTOCOL_XDIY
void acknowledge(byte id, byte check1, byte check2) {}
void send_message(byte id) {}
void send_message(byte id, long param) {}
void send_message(byte severity, const char *str) {}
void print_current_waypoints(){}
void print_waypoint(struct Location *cmd, byte index){}
void print_waypoints(){}
#endif