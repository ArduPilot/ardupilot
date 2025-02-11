#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_URM09_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

#define AP_RANGE_FINDER_URM09_DEFAULT_ADDR   0x11
#define AP_RANGE_FINDER_URM09_MEASURE_MODE_AUTOMATIC  0x80           ///< automatic mode
#define AP_RANGE_FINDER_URM09_MEASURE_MODE_PASSIVE    0x00           ///< passive mode
#define AP_RANGE_FINDER_URM09_CMD_DISTANCE_MEASURE    0x01           ///< passive mode configure registers
#define AP_RANGE_FINDER_URM09_MEASURE_RANG_500        0x20           ///< Ranging from 500 
#define AP_RANGE_FINDER_URM09_MEASURE_RANG_300        0x10           ///< Ranging from 300 
#define AP_RANGE_FINDER_URM09_MEASURE_RANG_150        0x00   
#define AP_RANGE_FINDER_URM09_eSLAVEADDR_INDEX        0x00
#define AP_RANGE_FINDER_URM09_ePID_INDEX              0x01
#define AP_RANGE_FINDER_URM09_eVERSION_INDEX          0x02
#define AP_RANGE_FINDER_URM09_eDIST_H_INDEX           0x03         /**< High distance eight digits */
#define AP_RANGE_FINDER_URM09_eDIST_L_INDEX           0x04        /**< Low  temperature eight digits */
#define AP_RANGE_FINDER_URM09_eCFG_INDEX              0x07
#define AP_RANGE_FINDER_URM09_eCMD_INDEX              0x08

class AP_RangeFinder_URM09 : public AP_RangeFinder_Backend
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    // constructor
    AP_RangeFinder_URM09(RangeFinder::RangeFinder_State &_state,
    								AP_RangeFinder_Params &_params,
                                 AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool _init(void);
    void _timer(void);

    uint16_t distance;
    bool new_distance;
    uint8_t mode;
    
    // start a reading
    bool start_reading(void);
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

};

#endif  // #if AP_RANGEFINDER_URM09_ENABLED

