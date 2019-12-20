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

#ifndef ENABLE
    #define ENABLE   0
    #define CONST    1
    #define MULTIPLY 2
    #define CLOGGED  3
#endif

#define current_time() AP_HAL::millis()

using namespace std;
extern const AP_HAL::HAL& hal;

using namespace HALSITL;

class test
{
public:
    static test * getInstance()
    {
        if (!p_instance) {
            p_instance = new test();
        }
        return p_instance;
    }

    void update_time()
    {
        first_prev_call_time = current_time();
        second_prev_call_time = current_time();
    }


    float get_fault_clogged(float fault)
    {
        if (first_prev_call_time == 0) {
            update_time();
        }
        first_arspd_fault += ((current_time() - first_prev_call_time)/1000.f)*(fault);
        update_time();
        return first_arspd_fault;
    }

private:
    test()
    {
        first_arspd_fault = 0;
        second_arspd_fault = 0;
        first_prev_call_time = 0;
        second_prev_call_time = 0;
        is_first_call = true;
        isSecondWasCalled = true;
    };
    static test * p_instance;
    float first_arspd_fault;
    float second_arspd_fault;
    uint32_t first_prev_call_time;
    uint32_t second_prev_call_time;
    bool is_first_call;
    bool isSecondWasCalled;
};

static uint32_t previos_call_time;
static float fault_static = 0;
static bool isFirstCall = true;

/*static uint32_t previos_call_time2;
static float fault_static2 = 0;
static bool isFirstCall2 = true;*/

float SITL_State::add_clogged(float airspeed, float fault)
{
    fault_static += ((current_time() - previos_call_time)/1000.f)*(fault);
    return add_sum(airspeed, fault_static);
}

float SITL_State::add_sum(float airspeed, float fault)
{
    return airspeed + fault;
}

float SITL_State::add_multiply(float airspeed, float fault)
{
    return airspeed * fault;
}

float SITL_State::add_fault(float airspeed, float fault, float(*func)(float, float))
{
    return func(airspeed, fault);
}

typedef float(*func_fault)(float, float);

func_fault func_type_fault[] =
{
    HALSITL::SITL_State::add_sum, HALSITL::SITL_State::add_multiply, HALSITL::SITL_State::add_clogged
};

float SITL_State::_get_arspd_fault(float airspeed, int fault_type, float fault)
{
    switch (fault_type) {

        case MULTIPLY:
            cout<<"ARSPD_FAULT_MULTIPLY"<<endl;
            airspeed = SITL_State::add_fault(airspeed, fault, func_type_fault[fault_type - 1]);
            break;

        case CLOGGED:
            cout<<"ARSPD_FAULT_CLOGGED"<<endl;

            if (isFirstCall == true) {
                previos_call_time = current_time();
                isFirstCall = false;
            }

            airspeed = SITL_State::add_fault(airspeed, fault, func_type_fault[fault_type - 1]);
            //previos_call_time = current_time();
            break;

        case CONST:
            cout<<"ARPSD CONST"<<endl;
            
            break;
            airspeed = SITL_State::add_fault(airspeed, fault, func_type_fault[fault_type - 1]);

        default:
            break;
        }
        if (fault_type != CLOGGED)
            fault_static = 0;
        previos_call_time = current_time();
    return airspeed;
}
typedef struct
{
    int type;
    float fault;
    float current_fault;
    uint32_t time_prev;
} arspd_data;
void arspd_data_init(arspd_data& p, int fault_type, float fault)
{
    if (p.type != fault_type) {
        if (p.type == CLOGGED) {
            p.current_fault = 0;
            p.time_prev = 0;
        }
            
        p.type = fault_type;
    }
    p.fault = fault;
}
void arspd_clogged(arspd_data& p);
float get_arspd_fault(arspd_data& p, float airspeed)
{
    switch (p.type) {
        case CONST:
            airspeed += p.fault;
            break;
        case MULTIPLY:
            airspeed *= p.fault;
            break;
        case CLOGGED:
            arspd_clogged(p);
            airspeed += p.current_fault;
            break;
        default:
            break;
    }
    return airspeed;
}

void arspd_clogged(arspd_data& p)
{
    if (p.time_prev == 0)
        p.time_prev = current_time();
    p.current_fault += ((current_time() - p.time_prev)/1000.f)*(p.fault);
    p.time_prev = current_time();
}
test* test::p_instance = nullptr;
test* t = test::getInstance();
static arspd_data p_arspd_data;

/*
  convert airspeed in m/s to an airspeed sensor value
 */
void SITL_State::_update_airspeed(float airspeed)
{
    cout<<"CURRENT TIME= "<<current_time()<<"\n";
    const float airspeed_ratio = 1.9936f;
    const float airspeed_offset = 2013.0f;


    arspd_data_init(p_arspd_data, _sitl->arspd_fault_type, _sitl->arspd_fault_value);
    //arspd_clogged(p_arspd_data);
    //cout<<"FROM ARSTRUCT type = "<<p_arspd_data.type<<endl;
    cout<<"\nFROM STRUCT fault = "<<get_arspd_fault(p_arspd_data, airspeed)<<endl;
    cout<<"\n"<<endl;

    float true_airspeed = airspeed;
    float airspeed2 = airspeed;
    
    //t->set(current_time());
    cout<<"FAULT CLOGGED FROM TEST = "<<t->get_fault_clogged(_sitl->arspd_fault_value)<<endl;

    //if (isFirstCall == false && _sitl->arspd_fault_type != CLOGGED)
    //    previos_call_time = current_time();

    /*if (isFirstCall2 == false && _sitl->arspd2_fault_type != CLOGGED)
        previos_call_time2 = current_time();*/
        
    //if (_sitl->arspd_fault_type != 0)
    //{
        airspeed = _get_arspd_fault(airspeed, _sitl->arspd_fault_type , _sitl->arspd_fault_value);
        cout<<"ARSPD2 VALUE FAULT TESTING = "<<airspeed<<endl;
        cout<<"ARSPD CLEAR TESTING = "<<airspeed<<endl;
    //}

    /*if (_sitl->arspd2_fault_type != 0)
    {
        airspeed2 = _get_arspd_fault(airspeed2, _sitl->arspd2_fault_type , _sitl->arspd2_fault_value);
        cout<<"ARSPD2 VALUE FAULT TESTING = "<<airspeed2<<endl;
        cout<<"ARSPD CLEAR TESTING = "<<airspeed<<endl;
    }*/
    //previos_call_time = current_time();

    


    // switch for adding fault airspeed or airspeed2
    /*if (_sitl->arspd_fault_type == 2)
    {
        //airspeed += _sitl->arspd_fault_value;
        fault_static += SITL_State::add_fault(airspeed, _sitl->arspd_fault_value, func_type_fault[_sitl->arspd_fault_type]); //((current_time - previos_call_time)/1000)*(_sitl->arspd_fault_value);
        previos_call_time = current_time;
    }*/
    //airspeed2 += _sitl->arspd_fault_value;

    cout<<"PREVIOS FAULT VALUE = "<<fault_static<<endl;
    //cout<<"CURR_TIME VALUE = "<<current_time<<endl;

    // Check sensor failure
    airspeed = is_zero(_sitl->arspd_fail) ? airspeed : _sitl->arspd_fail;
    airspeed2 = is_zero(_sitl->arspd2_fail) ? airspeed2 : _sitl->arspd2_fail;
    cout<<"arspd_fault_type ="<<(int)_sitl->arspd_fault_type<<endl;
    cout<<"arspd2_fault_type  = "<<(int)_sitl->arspd2_fault_type<<endl;
    cout<<"ARSPD        = "<<airspeed<<endl;
    cout<<"ARSPD2       = "<<airspeed2<<endl;
    cout<<"ARSPD state  = "<<_sitl->state.airspeed<<endl;
    cout<<"ARSPD true value ="<<true_airspeed<<endl;
    cout<<"ARSPD fault value ="<<_sitl->arspd_fault_value<<endl;
    cout<<"ARSPD2 fault value = "<<_sitl->arspd2_fault_value<<endl;

    //cout<<"ARSPD POLIMORF = "<<SITL_State::add_fault(true_airspeed, _sitl->arspd_fault_value, func_type_fault[_sitl->arspd_fault_type])<<endl;
    
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

}

#endif
