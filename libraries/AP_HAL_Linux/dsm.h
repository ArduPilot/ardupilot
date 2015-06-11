/*
  declaration of dsm_decode from dsm.cpp
 */
bool dsm_decode(uint64_t frame_time, const uint8_t dsm_frame[16], 
                uint16_t *values, 
                uint16_t *num_values, 
                uint16_t max_values);

