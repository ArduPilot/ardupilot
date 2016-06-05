/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>


#ifdef USERHOOK_VARIABLES


#define LOG_ENVIRO_MSG      0x21
#define MASK_LOG_ENVIRO     (1UL<<20)


class PMS3003
{
public:
    PMS3003(int16_t pm10 =0, // constructor
            int16_t pm25=0,
            int16_t pm100=0,
            int16_t cp030=0,
            int16_t cp050=0,
            int16_t cp10=0,
            int16_t cp25=0,
            int16_t cp50=0,
            int16_t cp100=0,
            int16_t hum=0,
            int16_t temp_c=0)
        : _PM10(pm10),
          _PM25(pm25),
          _PM100(pm100),
          _CP030(cp030),
          _CP050(cp050),
          _CP10(cp10),
          _CP25(cp25),
          _CP50(cp50),
          _CP100(cp100),
          _HUM(hum),
          _TEMP_C(temp_c)
          {
          };
    // probe and initialise the sensor
    bool init(void);
    void _measure(void);
    void _collect(void);
    void _timer(void);

    int16_t _PM10,
    _PM25,
    _PM100,
    _CP030,
    _CP050,
    _CP10,
    _CP25,
    _CP50,
    _CP100,
    _HUM,
    _TEMP_C;

    uint32_t _last_sample_time_ms;
    uint32_t _measurement_started_ms;

};


#endif  // USERHOOK_VARIABLES


