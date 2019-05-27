#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include "Parameters.h"
#include "ch.h"

class AP_Periph_FW {
public:
    void init();
    void update();

    Parameters g;

    void can_start();
    void can_update();
    void can_mag_update();
    void can_gps_update();
    void can_baro_update();

    void load_parameters();

    AP_SerialManager serial_manager;

#ifdef HAL_PERIPH_ENABLE_GPS
    AP_GPS gps;
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    Compass compass;
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    AP_Baro baro;
#endif
    
    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;
    uint32_t last_baro_update_ms;
};

extern AP_Periph_FW periph;

extern "C" {
void can_printf(const char *fmt, ...);
}

