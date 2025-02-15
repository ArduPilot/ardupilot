#include "AP_RangeFinder_URM09.h"

#if AP_RANGEFINDER_URM09_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

AP_RangeFinder_URM09::AP_RangeFinder_URM09(RangeFinder::RangeFinder_State &_state,
                                                           AP_RangeFinder_Params &_params,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev))
{
}

/*
   detect if a URM09 rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_URM09::detect(RangeFinder::RangeFinder_State &_state,
																AP_RangeFinder_Params &_params,
                                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_URM09 *sensor
        = NEW_NOTHROW AP_RangeFinder_URM09(_state, _params, std::move(dev));
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_URM09::_init(void)
{
    _dev->get_semaphore()->take_blocking();
    // set mode to passive
    // AP_RANGE_FINDER_URM09_MEASURE_RANG_500
    // params.max_distance is AP_FLoat
    //uint8_t set_mode = (uint8_t)(AP_RANGE_FINDER_URM09_MEASURE_RANG_500 | AP_RANGE_FINDER_URM09_MEASURE_MODE_PASSIVE);

    //_dev->write_register(eCFG_INDEX, set_mode); // 7 -
    //_dev->
    //uint8_t cmd[] = {AP_RANGE_FINDER_URM09_MEASURE_RANG_300, AP_RANGE_FINDER_URM09_MEASURE_MODE_PASSIVE};
    //_dev->transfer(cmd, sizeof(cmd), nullptr, 0);
    uint8_t sensor_mode = (uint8_t) (AP_RANGE_FINDER_URM09_MEASURE_MODE_PASSIVE | AP_RANGE_FINDER_URM09_MEASURE_RANG_300);
    bool ret = _dev->write_register(AP_RANGE_FINDER_URM09_eCFG_INDEX, sensor_mode);
    if(!ret){
        hal.console->printf("URM09: Couldn't write to reg: %u value %u\n",AP_RANGE_FINDER_URM09_eCFG_INDEX, sensor_mode);
    }
    if (!start_reading()) {
        _dev->get_semaphore()->give();
        return false;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(100);

    uint16_t reading_cm;
    if (!get_reading(reading_cm)) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(100000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_URM09::_timer, void));

    return true;
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_URM09::start_reading()
{
    //register 8, val 1
    //uint8_t cmd[1] = {AP_RANGE_FINDER_URM09_CMD_DISTANCE_MEASURE};
    // send command to take reading
    uint8_t cmd[2] = {AP_RANGE_FINDER_URM09_eCMD_INDEX, AP_RANGE_FINDER_URM09_CMD_DISTANCE_MEASURE};
    _dev->transfer(&cmd[0], 1,nullptr, 0);
    bool ret = _dev->transfer(&cmd[1], 1,nullptr, 0);
    //_dev->write_register(AP_RANGE_FINDER_URM09_eCMD_INDEX, AP_RANGE_FINDER_URM09_CMD_DISTANCE_MEASURE);
    return ret;
    //return _dev->transfer(cmd, sizeof(cmd), nullptr, 0);
}

// read - return last value measured by sensor
bool AP_RangeFinder_URM09::get_reading(uint16_t &reading_cm)
{
    //be16_t val;
    uint8_t msb = 0;
    uint8_t lsb = 0;
    
    // take range reading and read back results
    //uint8_t dist = eDIST_H_INDEX;
    //bool ret = _dev->transfer(nullptr, 0, (uint8_t *) &val, sizeof(val));
    //bool ret = _dev->read((uint8_t *) &val, sizeof(val));

    uint8_t cmd[2] = {AP_RANGE_FINDER_URM09_eDIST_H_INDEX, AP_RANGE_FINDER_URM09_eDIST_L_INDEX};
    _dev->transfer(&cmd[0], 1,nullptr, 0);
    bool ret = _dev->transfer(nullptr, 0, (uint8_t *) &msb, sizeof(msb)); 
    //_dev->read_registers(AP_RANGE_FINDER_URM09_eDIST_H_INDEX, (uint8_t *) &msb, 1);
    if(!ret){
        hal.console->printf("URM09: No data on High\n");
    }

    // _dev->read_registers(AP_RANGE_FINDER_URM09_eDIST_L_INDEX, (uint8_t *) &lsb, 1);

    _dev->transfer(&cmd[1], 1,nullptr, 0);
    ret = _dev->transfer(nullptr, 0, (uint8_t *) &lsb, sizeof(lsb)); 
    if(!ret){
        hal.console->printf("URM09: No data on Low\n");
    }
    if (ret) {
        // combine results into distance
        reading_cm = ((uint16_t) msb << 6) | (lsb & 0x3f);
    }

    // trigger a new reading
    start_reading();

    return ret;
}

/*
  timer called at 10Hz
*/
void AP_RangeFinder_URM09::_timer(void)
{
    uint16_t d;
    if (get_reading(d)) {
        WITH_SEMAPHORE(_sem);
        distance = d;
        new_distance = true;
        state.last_reading_ms = AP_HAL::millis();
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_URM09::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (new_distance) {
        state.distance_m = distance * 0.01f;
        new_distance = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        // if no updates for 0.3 seconds set no-data
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_URM09_ENABLED
