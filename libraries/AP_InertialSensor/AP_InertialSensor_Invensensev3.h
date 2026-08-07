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
    // get a startup banner to output to the GCS
    bool get_output_banner(char* banner, uint8_t banner_len) override;

    enum class Invensensev3_Type : uint8_t {
        ICM40609 = 0, // No HiRes
        ICM42688, // HiRes 19bit
        ICM42605, // No HiRes
        ICM40605, // No HiRes
        IIM42652, // HiRes 19bit
        IIM42653, // HiRes 19bit
        ICM42670, // HiRes 19bit
        ICM45686  // HiRes 20bit
    };

    // acclerometers on Invensense sensors will return values up to 32G
    const uint16_t multiplier_accel = INT16_MAX/(32*GRAVITY_MSS);

protected:
    void set_primary(bool _is_primary) override;

private:
    AP_InertialSensor_Invensensev3(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation);

    /* Initialize sensor*/
    bool hardware_init();
    bool check_whoami();

    void set_filter_and_scaling(void);
    void set_filter_and_scaling_icm42670(void);
    void set_filter_and_scaling_icm456xy(void);
    void fifo_reset();
    uint16_t calculate_fast_sampling_backend_rate(uint16_t base_backend_rate, uint16_t max_backend_rate) const;

    /* Read samples from FIFO */
    void read_fifo();

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    uint8_t register_read(uint8_t reg);
    void register_write(uint8_t reg, uint8_t val, bool checked=false);

    uint8_t register_read_bank(uint8_t bank, uint8_t reg);
    void register_write_bank(uint8_t bank, uint8_t reg, uint8_t val);
    uint8_t register_read_bank_icm456xy(uint16_t bank_addr, uint16_t reg);
    void register_write_bank_icm456xy(uint16_t bank_addr, uint16_t reg, uint8_t val);

    bool accumulate_samples(const struct FIFOData *data, uint8_t n_samples);
    bool accumulate_highres_samples(const struct FIFODataHighRes *data, uint8_t n_samples);

    // get the gyro backend rate in Hz at which the FIFO is being read
    uint16_t get_gyro_backend_rate_hz() const override {
        return backend_rate_hz;
    }

    // reset FIFO configure1 register
    uint8_t fifo_config1;

    // temp scaling for FIFO temperature
    float temp_sensitivity;
    const float temp_zero = 25; // degC
    
    const enum Rotation rotation;

    static constexpr float SCALE_RANGE_16BIT = 32768; // 2^15;
    // HiRes support is either 20bit (19bit accel) or 19bit (18bit accel)
    static constexpr float SCALE_RANGE_20BIT = 524288; // 2^19;
    static constexpr float SCALE_RANGE_19BIT = 262144; // 2^18;

    /*
      gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==0)
    */
    static constexpr float GYRO_SCALE_2000DPS = radians(1) / (SCALE_RANGE_16BIT / 2000.0);
    /*
      gyro as 8.2 LSB/DPS at scale factor of +/- 4000dps (FS_SEL==0)
    */
    static constexpr float GYRO_SCALE_4000DPS = radians(1) / (SCALE_RANGE_16BIT / 4000.0);
    /*
      highres gyro is always 131 LSB/DPS modified by the data size transmitted
    */
    static constexpr float GYRO_SCALE_HIGHRES_2000DPS = radians(1) / (SCALE_RANGE_20BIT / 2000.0);
    static constexpr float GYRO_SCALE_HIGHRES_4000DPS = radians(1) / (SCALE_RANGE_20BIT / 4000.0);
    /*
      Accel scale 16g (2048 LSB/g)
    */
    static constexpr float ACCEL_SCALE_16G = (GRAVITY_MSS / (SCALE_RANGE_16BIT / 16));
    /*
      Accel scale 32g (1024 LSB/g)
    */
    static constexpr float ACCEL_SCALE_32G = (GRAVITY_MSS / (SCALE_RANGE_16BIT / 32));
    /*
      highres accel is 16384 LSB/g on 45686 amd 8192 LSB/g on all others
    */
    static constexpr float ACCEL_SCALE_HIGHRES_16G = (GRAVITY_MSS / (SCALE_RANGE_20BIT / 16));
    static constexpr float ACCEL_SCALE_HIGHRES_32G = (GRAVITY_MSS / (SCALE_RANGE_20BIT / 32));

    float accel_scale = ACCEL_SCALE_16G;
    float gyro_scale = GYRO_SCALE_2000DPS;

    // are we doing more than 1kHz sampling?
    bool fast_sampling;
#if HAL_INS_HIGHRES_SAMPLE
    bool highres_sampling;
#endif

    // what rate are we generating samples into the backend for gyros and accels?
    uint16_t backend_rate_hz;
    // pre-calculated backend period
    uint32_t backend_period_us;

    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;

    // which sensor type this is
    enum Invensensev3_Type inv3_type;

    // buffer for fifo read
    void* fifo_buffer;

    float temp_filtered;
    LowPassFilter2pFloat temp_filter;
    uint32_t sampling_rate_hz;
};
