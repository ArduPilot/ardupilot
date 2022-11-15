#include "AP_Frsky_Backend.h"

#if AP_FRSKY_TELEM_ENABLED

#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RPM/AP_RPM.h>

extern const AP_HAL::HAL& hal;

bool AP_Frsky_Backend::init()
{
    // if SPort Passthrough is using external data then it will
    // override this to do nothing:
    return init_serial_port();
}

bool AP_Frsky_Backend::init_serial_port()
{
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_Frsky_Backend::loop, void),
            "FrSky",
            1024,
            AP_HAL::Scheduler::PRIORITY_RCIN,
            1)) {
        return false;
    }
    // we don't want flow control for either protocol
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    return true;
}

/*
  thread to loop handling bytes
 */
void AP_Frsky_Backend::loop(void)
{
    // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
    _port->begin(initial_baud(), 0, 0);

    while (true) {
        hal.scheduler->delay(1);
        send();
    }
}

/*
 * get vertical speed from ahrs, if not available fall back to baro climbrate, units is m/s
 * for FrSky D and SPort protocols
 */
float AP_Frsky_Backend::get_vspeed_ms(void)
{

    {
        // release semaphore as soon as possible
        AP_AHRS &_ahrs = AP::ahrs();
        Vector3f v;
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        if (_ahrs.get_velocity_NED(v)) {
            return -v.z;
        }
    }

    auto &_baro = AP::baro();
    WITH_SEMAPHORE(_baro.get_semaphore());
    return _baro.get_climb_rate();
}

/*
 * prepare altitude between vehicle and home location data
 * for FrSky D and SPort protocols
 */
void AP_Frsky_Backend::calc_nav_alt(void)
{
    _SPort_data.vario_vspd = (int32_t)(get_vspeed_ms()*100); //convert to cm/s

    Location loc;
    float current_height = 0; // in centimeters above home

    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());
    if (_ahrs.get_location(loc)) {
        current_height = loc.alt*0.01f;
        if (!loc.relative_alt) {
            // loc.alt has home altitude added, remove it
            current_height -= _ahrs.get_home().alt*0.01f;
        }
    }

    _SPort_data.alt_nav_meters = float_to_uint16(current_height);
    _SPort_data.alt_nav_cm = float_to_uint16((current_height - _SPort_data.alt_nav_meters) * 100);
}

/*
 * format the decimal latitude/longitude to the required degrees/minutes
 * for FrSky D and SPort protocols
 */
float AP_Frsky_Backend::format_gps(float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/*
 * prepare gps data
 * for FrSky D and SPort protocols
 */
void AP_Frsky_Backend::calc_gps_position(void)
{
    AP_AHRS &_ahrs = AP::ahrs();

    Location loc;

    if (_ahrs.get_location(loc)) {
        float lat = format_gps(fabsf(loc.lat/10000000.0f));
        _SPort_data.latdddmm = lat;
        _SPort_data.latmmmm = (lat - _SPort_data.latdddmm) * 10000;
        _SPort_data.lat_ns = (loc.lat < 0) ? 'S' : 'N';

        float lon = format_gps(fabsf(loc.lng/10000000.0f));
        _SPort_data.londddmm = lon;
        _SPort_data.lonmmmm = (lon - _SPort_data.londddmm) * 10000;
        _SPort_data.lon_ew = (loc.lng < 0) ? 'W' : 'E';

        float alt = loc.alt * 0.01f;
        _SPort_data.alt_gps_meters = float_to_uint16(alt);
        _SPort_data.alt_gps_cm = float_to_uint16((alt - _SPort_data.alt_gps_meters) * 100);

        const float speed = AP::ahrs().groundspeed();
        _SPort_data.speed_in_meter = float_to_int16(speed);
        _SPort_data.speed_in_centimeter = float_to_uint16((speed - _SPort_data.speed_in_meter) * 100);
    } else {
        _SPort_data.latdddmm = 0;
        _SPort_data.latmmmm = 0;
        _SPort_data.lat_ns = 0;
        _SPort_data.londddmm = 0;
        _SPort_data.lonmmmm = 0;
        _SPort_data.alt_gps_meters = 0;
        _SPort_data.alt_gps_cm = 0;
        _SPort_data.speed_in_meter = 0;
        _SPort_data.speed_in_centimeter = 0;
    }

    _SPort_data.yaw = (uint16_t)((_ahrs.yaw_sensor / 100) % 360); // heading in degree based on AHRS and not GPS
}

/*
 * prepare rpm data
 * for FrSky D and SPort protocols
 */
bool AP_Frsky_Backend::calc_rpm(const uint8_t instance, int32_t &value) const
{
#if AP_RPM_ENABLED
    const AP_RPM* rpm = AP::rpm();
    if (rpm == nullptr) {
        return false;
    }

    float rpm_value;
    if (!rpm->get_rpm(instance, rpm_value)) {
        return false;
    }
    value = static_cast<int32_t>(roundf(rpm_value));
    return true;
#else
    return false;
#endif
}

#endif  // AP_FRSKY_TELEM_ENABLED
