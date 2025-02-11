#include "AP_Frsky_Backend.h"

#if AP_FRSKY_TELEM_ENABLED

#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_RCTelemetry/AP_RCTelemetry.h>
#include <GCS_MAVLink/GCS.h>



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
 * prepare altitude between vehicle and home location data
 * for FrSky D and SPort protocols
 */
void AP_Frsky_Backend::calc_nav_alt(void)
{
    _SPort_data.vario_vspd = (int32_t)(AP_RCTelemetry::get_vspeed_ms()*100); //convert to cm/s

    float current_height = AP_RCTelemetry::get_nav_alt_m();
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

void AP_Frsky_Backend::calc_power(uint8_t cells_count)
{
    float cell_mvolts = 0;
    const AP_BattMonitor &_battery = AP::battery();
    const auto &cells_mvolts = _battery.get_cell_voltages();
      
    _SPort_data.batt_volt = _battery.voltage()*1000;

    
    for (uint8_t i = 0; i < cells_count; i++) {
        
        //from battery object
        cell_mvolts = cells_mvolts.cells[i];
        
        //or calc it from total voltage - default !!!
        cell_mvolts = _battery.voltage()*1000/cells_count;

        _SPort_data.cells_mvolts[i] = (uint16_t)cell_mvolts;

    }
    
    float current;
    if (_battery.current_amps(current)) {
        _SPort_data.power_ampers = current;
        _SPort_data.power_volts = _battery.voltage(); 
    }
    
}

void AP_Frsky_Backend::calc_time(void)
{
        
    uint16_t time_passed_s = AP_HAL::millis()/1000;   //AP running time in seconds
    _SPort_data.hours= (uint8_t)(time_passed_s/3600);                 // time since start in hours
    _SPort_data.minutes = (uint8_t)((time_passed_s % 3600)/60);       // minutes
    _SPort_data.seconds = (uint8_t)(time_passed_s % 60);              // seconds       

}

void AP_Frsky_Backend::calc_temp(void)
{
    // float temp1 = 0;
    // float temp2 = 0;
    // AP::baro().get_temperature(temp1, temp2);
    // _SPort_data.temp1 = (temp1);
    // _SPort_data.temp2 = (temp2);    

    _SPort_data.temp1 = gcs().custom_mode();                            // flight mode
    _SPort_data.temp2 = AP::gps().num_sats() * 10 + AP::gps().status(); // number of satellites * 10 + status

}


void AP_Frsky_Backend::calc_acc(void)
{   
    // 1 >> 0.001  

    // Vector3f acc = AP::ahrs().get_accel_ef();
    Vector3f acc = AP::ahrs().get_gyro_drift();
    // Vector3f acc = AP::ahrs().get_gyro_latest();
    _SPort_data.acc_x = acc.x;
    _SPort_data.acc_y = acc.y;
    _SPort_data.acc_z = acc.z;

    _SPort_data.acc_x = 1;
    _SPort_data.acc_y = 2;
    _SPort_data.acc_z = 3;

}



#endif  // AP_FRSKY_TELEM_ENABLED
