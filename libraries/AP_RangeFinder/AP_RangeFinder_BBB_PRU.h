#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BBB_PRU_ENABLED

#include "AP_RangeFinder_Backend.h"

#define PRU0_CTRL_BASE 0x4a322000

#define PRU0_IRAM_BASE 0x4a334000
#define PRU0_IRAM_SIZE 0x2000

#define PRU0_DRAM_BASE 0x4a300000
#define PRU0_DRAM_SIZE 0x2000

struct range {
        uint32_t distance;
	uint32_t status;
};

class AP_RangeFinder_BBB_PRU : public AP_RangeFinder_Backend
{
public:
    /*
        Constructor:
        The constructor also initialises the rangefinder. Note that this
        constructor is not called until detect() returns true, so we
        already know that we should setup the rangefinder
    */
    using AP_RangeFinder_Backend::AP_RangeFinder_Backend;

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

};

#endif  // AP_RANGEFINDER_BBB_PRU_ENABLED
