#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_UPFLOW_Tx_ENABLED

#include "AP_OpticalFlow_Backend.h"
#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_UPFLOW_Tx : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_UPFLOW_Tx(AP_OpticalFlow &_frontend, AP_HAL::UARTDriver *uart);

    // initialise the sensor
    void init() override;

    // read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // detect if the sensor is available
    static AP_OpticalFlow_UPFLOW_Tx *detect(AP_OpticalFlow &_frontend);

private:
    struct PACKED UpixelsOpticalFlow {
        int16_t		flow_x_integral;        //unit:10^-4 radians multiply by 10^-4 to get radians
        int16_t		flow_y_integral;        //unit:10^-4 radians multiply by 10^-4 to get radians
        uint16_t   	integration_timespan;   //dt in us
        uint16_t   	ground_distance;        //reserved, always 999
        uint8_t    	opt_valid;               //0 for not valid, 245 for valid.
        uint8_t    	tof_valid;              //0 for not valid, 0x64 (100) for 100% valid.
    };
    AP_HAL::UARTDriver *uart;               // uart connected to flow sensor
    struct UpixelsOpticalFlow updata;       // struct for received data
    uint16_t recv_count;                    // amount of bytes received
    uint8_t sum;                            //checksum
    Vector2f gyro_sum;                      // sum of gyro sensor values since last frame from flow sensor
    uint16_t gyro_sum_count;                // number of gyro sensor values in sum
};

typedef struct{
    uint16_t   	ground_distance;        //reserved, always 999
    uint8_t    	tof_valid;              //0 for not valid, 0x64 (100) for 100% valid.
    bool        if_opt_ok;
}UPFLOW_TOF;

UPFLOW_TOF* get_upflow_tof();

#endif // AP_OPTICALFLOW_UPFLOW_Tx_ENABLED
