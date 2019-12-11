/*
  SITL handling

  This simulates an analog airspeed sensor

  Andrew Tridgell November 2011
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "SITL_State.h"
#include <SITL/SITL.h>
#include <AP_Math/AP_Math.h>
#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace std;
extern const AP_HAL::HAL& hal;

using namespace HALSITL;

void SITL_State::_get_arspd_fault()
{
    srand(time(NULL));
    switch (_sitl->arspd_fault_type) {
        case SITL::SITL::ARSPD_FAULT_CLOGGED:
            cout<<"ARSPD_FAULT_CLOGGED"<<endl;
            break;

        case SITL::SITL::ARSPD_FAULT_SMOOTHY:
            cout<<"ARSPD_FAULT_SMOOTHY"<<endl;

            _sitl->arspd_fault_value += 0.01 * (rand() % 101);
            break;

        case SITL::SITL::ARSPD_FAULT_CONST:
        default:
        cout<<"ARSPD_FAULT_CONST"<<endl;
            break;
        }
}

/*
  convert airspeed in m/s to an airspeed sensor value
 */
void SITL_State::_update_airspeed(float airspeed)
{
    const float airspeed_ratio = 1.9936f;
    const float airspeed_offset = 2013.0f;

    float airspeed2 = airspeed;

    const uint64_t current_time = AP_HAL::millis();

    if (_sitl->arspd_fault_type != 0 && current_time % 5000 == 0)
    {
        if (airspeed2 > 0)
            _get_arspd_fault();
    }
    airspeed2 += _sitl->arspd_fault_value;
    
    // Check sensor failure
    airspeed = is_zero(_sitl->arspd_fail) ? airspeed : _sitl->arspd_fail;
    airspeed2 = is_zero(_sitl->arspd2_fail) ? airspeed2 : _sitl->arspd2_fail;
    cout<<"arspd_fault_type ="<<_sitl->arspd_fault_type<<endl;
    cout<<"arspd_fault_value  = "<<_sitl->arspd_fault_value<<endl;
    cout<<"ARSPD        = "<<airspeed<<endl;
    cout<<"ARSPD2       = "<<airspeed2<<endl;
    cout<<"ARSPD state  = "<<_sitl->state.airspeed<<endl;

    // Add noise
    airspeed = airspeed + (_sitl->arspd_noise * rand_float());
    airspeed2 = airspeed2 + (_sitl->arspd_noise * rand_float());



    if (!is_zero(_sitl->arspd_fail_pressure)) {
        // compute a realistic pressure report given some level of trapper air pressure in the tube and our current altitude
        // algorithm taken from https://en.wikipedia.org/wiki/Calibrated_airspeed#Calculation_from_impact_pressure
        float tube_pressure = fabsf(_sitl->arspd_fail_pressure - _barometer->get_pressure() + _sitl->arspd_fail_pitot_pressure);
        airspeed = 340.29409348 * sqrt(5 * (pow((tube_pressure / SSL_AIR_PRESSURE + 1), 2.0/7.0) - 1.0));
    }
    if (!is_zero(_sitl->arspd2_fail_pressure)) {
        // compute a realistic pressure report given some level of trapper air pressure in the tube and our current altitude
        // algorithm taken from https://en.wikipedia.org/wiki/Calibrated_airspeed#Calculation_from_impact_pressure
        float tube_pressure = fabsf(_sitl->arspd2_fail_pressure - _barometer->get_pressure() + _sitl->arspd2_fail_pitot_pressure);
        airspeed2 = 340.29409348 * sqrt(5 * (pow((tube_pressure / SSL_AIR_PRESSURE + 1), 2.0/7.0) - 1.0));
    }

    float airspeed_pressure = (airspeed * airspeed) / airspeed_ratio;
    float airspeed2_pressure = (airspeed2 * airspeed2) / airspeed_ratio;

    cout<<"pressure = "<<airspeed_pressure<<endl; cout<<"pressure = "<<airspeed2_pressure<<endl;

    // flip sign here for simulating reversed pitot/static connections
    if (_sitl->arspd_signflip) airspeed_pressure *= -1;
    if (_sitl->arspd_signflip) airspeed2_pressure *= -1;

    float airspeed_raw = airspeed_pressure + airspeed_offset;
    float airspeed2_raw = airspeed2_pressure + airspeed_offset;
    cout<<"airspeed raw "<<airspeed_raw<<endl;
    if (airspeed_raw / 4 > 0xFFFF) {
        airspeed_pin_value = 0xFFFF;
        return;
    }
    if (airspeed2_raw / 4 > 0xFFFF) {
        airspeed_2_pin_value = 0xFFFF;
        return;
    }
    // add delay
    const uint32_t now = AP_HAL::millis();
  
    uint32_t best_time_delta_wind = 200;  // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index_wind = 0;  // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - last_store_time_wind >= 10) {  // store data every 10 ms.
        last_store_time_wind = now;
        if (store_index_wind > wind_buffer_length - 1) {  // reset buffer index if index greater than size of buffer
            store_index_wind = 0;
        }
        buffer_wind[store_index_wind].data = airspeed_raw;  // add data to current index
        buffer_wind[store_index_wind].time = last_store_time_wind;  // add time to current index
        buffer_wind_2[store_index_wind].data = airspeed2_raw;  // add data to current index
        buffer_wind_2[store_index_wind].time = last_store_time_wind;  // add time to current index
        store_index_wind = store_index_wind + 1;  // increment index
    }

    // return delayed measurement
    delayed_time_wind = now - _sitl->wind_delay;  // get time corresponding to delay
    // find data corresponding to delayed time in buffer
    for (uint8_t i = 0; i <= wind_buffer_length - 1; i++) {
        // find difference between delayed time and time stamp in buffer
        time_delta_wind = abs(
                (int32_t)(delayed_time_wind - buffer_wind[i].time));
        // if this difference is smaller than last delta, store this time
        if (time_delta_wind < best_time_delta_wind) {
            best_index_wind = i;
            best_time_delta_wind = time_delta_wind;
        }
    }
    if (best_time_delta_wind < 200) {  // only output stored state if < 200 msec retrieval error
        airspeed_raw = buffer_wind[best_index_wind].data;
        airspeed2_raw = buffer_wind_2[best_index_wind].data;
    }

    airspeed_pin_value = airspeed_raw / 4;
    airspeed_2_pin_value = airspeed2_raw / 4;

    cout<<"airsp pi = "<<airspeed_pin_value<<endl;
    cout<<"airsp2 pi = "<<airspeed_2_pin_value<<endl;
}

#endif
