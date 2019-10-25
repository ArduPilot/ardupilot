#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Common/AP_FWVersion.h>

#if defined(HAL_PERIPH_NEOPIXEL_COUNT) || defined(HAL_PERIPH_ENABLE_NCP5623_LED)
#define AP_PERIPH_HAVE_LED
#endif

#include "Parameters.h"
#include "ch.h"

/*
  app descriptor compatible with MissionPlanner
 */
struct app_descriptor {
    uint8_t sig[8] = { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 };
    uint32_t image_crc1 = 0;
    uint32_t image_crc2 = 0;
    uint32_t image_size = 0;
    uint32_t git_hash = 0;
    uint8_t  version_major = AP::fwversion().major;
    uint8_t version_minor = AP::fwversion().minor;
    uint8_t reserved[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
};
extern const struct app_descriptor app_descriptor;

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
    void can_airspeed_update();
    void can_rangefinder_update();

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

#ifdef HAL_PERIPH_ENABLE_ADSB
    void adsb_init();
    void adsb_update();
    void can_send_ADSB(struct __mavlink_adsb_vehicle_t &msg);
    struct {
        mavlink_message_t msg;
        mavlink_status_t status;
    } adsb;
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    AP_Airspeed airspeed;
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    RangeFinder rangefinder;
#endif
    
    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;
    uint32_t last_baro_update_ms;
    uint32_t last_airspeed_update_ms;
};

extern AP_Periph_FW periph;

extern "C" {
void can_printf(const char *fmt, ...);
}

