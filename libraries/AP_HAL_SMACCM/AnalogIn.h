
#ifndef __AP_HAL_SMACCM_ANALOGIN_H__
#define __AP_HAL_SMACCM_ANALOGIN_H__

#include <AP_HAL_SMACCM.h>

class SMACCM::SMACCMAnalogSource : public AP_HAL::AnalogSource {
public:
    SMACCMAnalogSource(float v);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
private:
    float _v;
};

class SMACCM::SMACCMAnalogIn : public AP_HAL::AnalogIn {
public:
    SMACCMAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);
    AP_HAL::AnalogSource* channel(int16_t n, float scale);
};
#endif // __AP_HAL_SMACCM_ANALOGIN_H__
