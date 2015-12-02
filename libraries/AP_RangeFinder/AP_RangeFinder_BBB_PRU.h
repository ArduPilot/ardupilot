#ifndef __AP_RANGEFINDER_PRU_H__
#define __AP_RANGEFINDER_PRU_H__

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"


#define PRU0_CTRL_BASE 0x4a322000

#define PRU0_IRAM_BASE 0x4a334000
#define PRU0_IRAM_SIZE 0x2000

#define PRU0_DRAM_BASE 0x4a300000

struct range {
        uint32_t distance;
	uint32_t status;
};

class AP_RangeFinder_BBB_PRU : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_BBB_PRU(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);

private:

};
#endif  // __AP_RANGEFINDER_PRU_H__
