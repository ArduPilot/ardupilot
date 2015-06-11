/*
  declarations for sbus.h
 */
bool sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values, 
                 bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values);

