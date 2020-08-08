#include <AP_HAL/AP_HAL.h>

struct GyroFrame {
    float x[1024];
    float y[1024];
    float z[1024];
};

extern const GyroFrame gyro_frames[];
extern const uint32_t NUM_FRAMES;
extern const uint16_t SAMPLE_RATE;