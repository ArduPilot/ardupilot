#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"


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
    // constructor
    AP_RangeFinder_BBB_PRU(RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect();

    // update state
    void update(void);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

};
