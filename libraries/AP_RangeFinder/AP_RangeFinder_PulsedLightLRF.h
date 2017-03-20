#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

/* Connection diagram
 *
 *        ------------------------------------------------------------------------------------
 *        |           J2-1(LED) J2-2(5V) J2-3(Enable) J2-4(Ref Clk) J2-5(GND) J2-6(GND)      |
 *        |                                                                                  |
 *        |                                                                                  |
 *        |                                      J1-3(I2C Clk) J1-2(I2C Data) J1-1(GND)      |
 *        ------------------------------------------------------------------------------------
 */

class AP_RangeFinder_PulsedLightLRF : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(uint8_t bus, RangeFinder &ranger, uint8_t instance,
                                          RangeFinder::RangeFinder_State &_state,
                                          RangeFinder::RangeFinder_Type rftype);

    // update state
    void update(void) override {}


private:
    // constructor
    AP_RangeFinder_PulsedLightLRF(uint8_t bus, RangeFinder &ranger, uint8_t instance,
                                  RangeFinder::RangeFinder_State &_state,
                                  RangeFinder::RangeFinder_Type rftype);

    // start a reading
    bool init(void);
    void timer(void);
    bool lidar_transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);
    
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    uint8_t sw_version;
    uint8_t hw_version;
    uint8_t check_reg_counter;
    bool v2_hardware;
    uint16_t last_distance_cm;
    RangeFinder::RangeFinder_Type rftype;
    
    enum { PHASE_MEASURE, PHASE_COLLECT } phase;
};
