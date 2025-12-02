// #include "AOA_AOA.h"
// #include <AP_Math/AP_Math.h>
// #include <AP_HAL/AP_HAL.h>
// #include <AP_HAL/I2CDevice.h>
// #include <AP_Common/AP_Common.h>
// #include <GCS_MAVLink/GCS.h>



// #if AP_AIRSPEED_ENABLED

// extern const AP_HAL::HAL &hal;

// #define SENSERION_I2C_ADDR 0x3C
// #define SENSERION_SAMPLES_HZ 20
// #define SENSERION_CALLBACK_US (1000000UL / HONEYWELL_SAMPLES_HZ)
// #define SENSERION_I2C_REG 0x00
// #define maxDp 125.0f // maximum differential pressure in Pa for AoA sensor


// AP_AOA::AP_AOA(AP_Airspeed &_frontend, uint8_t _instance, int8_t bus, uint8_t address)
//     : AP_Airspeed_Backend(_frontend, _instance),
//       _address(address)
// {
//     _dev = hal.i2c_mgr->get_device(bus, address);
//     _last_pressureAOA = 0.0f;
//     _last_AoA = 0.0f;
//     _healthy = false;
// }

// bool AP_AOA::init()
// {
//     if (!_dev) {
//         _healthy = false;
//         return false;
//     }

//     // Register periodic callback at ~20 Hz
//     _dev->register_periodic_callback(
//         SENSERION_CALLBACK_US,
//         FUNCTOR_BIND_MEMBER(&AP_AOA::timer, void)
//     );

//     _healthy = true;
//     return true;
// }

// bool AP_AOA::get_differential_pressure(float &pressure)
// {
//     pressure = _last_pressureAOA;
//     return _healthy;
// }

// void AP_AOA::timer() 
// {
//     if (!_dev){
//         _healthy = false;
//         return;
//     }

//     //--------------------------
//     // READ AoA SENSOR (2 bytes)
//     //--------------------------
//     uint8_t buf[2] = {0, 0};
//     if (!_dev->read_registers(SENSERION_I2C_REG, buf, 2)) {
//         _healthy = false;
//         return;
//     }

//     // Convert signed 16-bit value → differential pressure (Pa)
//     int16_t readValAoA = (int16_t)((buf[0] << 8) | buf[1]);
//     pressureAoA = float(readValAoA) / 240.0f;

//     //--------------------------
//     // READ PITOT q
//     //--------------------------
//     AP_Airspeed *airspeed = AP::airspeed();

//     if (!airspeed) {
//         _healthy = false;
//         return;
//     }

//     float q_local = 0.0f;

//     if (!airspeed->get_differential_pressure(q_local)) {
//         _healthy = false;
//         return;
//     }

//     q = q_local;   // store q internally

//     //--------------------------
//     // COMPUTE AOA
//     //--------------------------
//     AoA = constrain((pressureAoA / q) * (10.0f / 0.42f), -90.0f, 90.0f);

//     if (fabsf(pressureAoA) >= maxDp) {
//         AoA = 999;
//         gcs().send_text(MAV_SEVERITY_CRITICAL, "AOA Max DP warning");
//     }

//     //--------------------------
//     // STORE INTO BACKEND STATE
//     //--------------------------
//     WITH_SEMAPHORE(sem);
//     _last_AoA = AoA;
//     _last_pressureAOA = pressureAoA;
//     _healthy = true;
// }


#include "AP_AOA.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Param/AP_Param.h>


#if AP_AIRSPEED_ENABLED

extern const AP_HAL::HAL &hal;

#define SENSERION_I2C_ADDR        0x3C
#define SENSERION_SAMPLES_HZ      20
#define SENSERION_CALLBACK_US     (1000000UL / SENSERION_SAMPLES_HZ)
#define SENSERION_I2C_REG         0x00
#define maxDp                     125.0f   // Max differential pressure (Pa)


//-------------------------------------------------------------
// Constructor
//-------------------------------------------------------------
AP_AOA::AP_AOA(AP_Airspeed &fs, uint8_t inst, int8_t bus, uint8_t address)
    : AP_Airspeed_Backend(fs, inst),
      _address(address)
{
    _dev = hal.i2c_mgr->get_device(bus, address);
    _last_pressureAOA = 0.0f;
    _last_AoA = 0.0f;
    _healthy = false;
}


//-------------------------------------------------------------
// Init
//-------------------------------------------------------------
bool AP_AOA::init()
{
    if (!_dev) {
        _healthy = false;
        return false;
    }

    // register periodic 20 Hz callback
    _dev->register_periodic_callback(
        SENSERION_CALLBACK_US,
        FUNCTOR_BIND_MEMBER(&AP_AOA::timer, void)
    );

    _healthy = true;
    return true;
}

// AP_Param::GroupInfo AP_AOA::parameters[] = {
//     AP_GROUPINFO("AOA_VAL", 0, _last_AoA) // expose _last_AoA
// };   this will be a static variable and not a live updating one so dont use bc MP wont call it

//-------------------------------------------------------------
// Required backend API
//-------------------------------------------------------------
bool AP_AOA::get_differential_pressure(float &pressure)
{
    pressure = _last_pressureAOA;
    return _healthy;
}




//-------------------------------------------------------------
// Timer callback (20 Hz)
//-------------------------------------------------------------
void AP_AOA::timer()
{
    if (!_dev) {
        _healthy = false;
        return;
    }

    uint8_t buf[2] = {0,0};
    if (!_dev->read_registers(SENSERION_I2C_REG, buf, 2)) {
        _healthy = false;
        return;
    }

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    float pressureAoA = float(raw) / 240.0f;

    AP_Airspeed *airspeed = AP::airspeed();
    if (!airspeed) {
        _healthy = false;
        return;
    }

    float q = 0.0f;
    bool success = airspeed->get_differential_pressure(q);
    if (success == false) {
        _healthy = false;
        return;
    }

    // Use global constrain function
    float AoA = constrain_float((pressureAoA / q) * (10.0f / 0.42f), -90.0f, 90.0f);

    if (fabsf(pressureAoA) >= maxDp) {
        AoA = 999;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "AOA Max DP warning");
    }

    WITH_SEMAPHORE(sem);
    _last_AoA = AoA;
    _last_pressureAOA = pressureAoA;

    
    gcs().send_named_float("AOA", _last_AoA); //lets try to use this for now 
    //This will appear in Mission Planner Flight Data → Status → Named Floats.


    // mavlink_message_t msg;
    // mavlink_msg_scaled_pressure2_pack(
    //     hal.mavlink->get_system_id(),
    //     hal.mavlink->get_component_id(),
    //     &msg,
    //     hal.scheduler->micros(),    // time_boot_ms
    //     _last_AoA * 100.0f,        // abs_pressure (Pa x 100)
    //     0,                          // diff_pressure (unused)
    //     0,                          // temperature (unused)
    //     0                           // temperature differential (unused)
    // );
    // hal.mavlink->send_message(&msg);
  




    _healthy = true;
}

#endif  // AP_AIRSPEED_ENABLED


