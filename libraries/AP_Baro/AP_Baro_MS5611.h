
#ifndef __AP_BARO_MS5611_H__
#define __AP_BARO_MS5611_H__

#include <stdint.h>
#include "AP_Baro.h"

class AP_Baro_MS5611 : public AP_Baro
{
    public:
    AP_Baro_MS5611();
    void    init();
    uint8_t update();
    int32_t get_pressure();
    float   get_temp();

    
    void _send_reset();
    void _start_conversion_D1();
    void _start_conversion_D2();
    bool _adc_read(int32_t * value);

    private:

    int32_t _raw_pres;
    int32_t _raw_temp;

};

#endif // __AP_BARO_MS5611_H__

