#pragma once
/*
  driver for the invensensev3 range of IMUs
  These are the ICM-4 series of IMUs
 */

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_Invensensev3 : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_Invensensev3();

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

    /* update accel and gyro state */
    bool update() override;
    void accumulate() override;

    void start() override;

    enum class Invensensev3_Type : uint8_t {
        ICM40609 = 0,
        ICM42688,
        ICM42605,
        ICM40605,
        IIM42652,
    };

    // acclerometers on Invensense sensors will return values up to 32G
    const uint16_t multiplier_accel = INT16_MAX/(32*GRAVITY_MSS);

private:
    AP_InertialSensor_Invensensev3(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation);

    /* Initialize sensor*/
    bool hardware_init();
    bool check_whoami();

    void set_filter_and_scaling(void);
    void fifo_reset();

    /* Read samples from FIFO */
    void read_fifo();

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    uint8_t register_read(uint8_t reg);
    void register_write(uint8_t reg, uint8_t val, bool checked=false);

    bool accumulate_samples(const struct FIFOData *data, uint8_t n_samples);

    // instance numbers of accel and gyro data
    uint8_t gyro_instance;
    uint8_t accel_instance;

    // reset FIFO configure1 register
    uint8_t fifo_config1;

    // temp scaling for FIFO temperature
    float temp_sensitivity;
    const float temp_zero = 25; // degC
    
    const enum Rotation rotation;

    float accel_scale;

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    // which sensor type this is
    enum Invensensev3_Type inv3_type;

    // buffer for fifo read
    struct FIFOData *fifo_buffer;

    float temp_filtered;
    LowPassFilter2pFloat temp_filter;
};
