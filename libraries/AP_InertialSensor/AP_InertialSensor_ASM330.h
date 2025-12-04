#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/* enable debug to see a register dump on startup */
#define AP_INERTIALSENSOR_AMS330_DEBUG_ENABLED 0

class AP_InertialSensor_ASM330 : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_ASM330() { }
    void start(void) override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);
private:
    AP_InertialSensor_ASM330(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                             enum Rotation rotation);

    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    struct PACKED sensor_raw_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    enum gyro_scale {
        G_SCALE_125DPS = 0,
        G_SCALE_250DPS,
        G_SCALE_500DPS,
        G_SCALE_1000DPS,
        G_SCALE_2000DPS,
    };

    enum accel_scale {
        A_SCALE_2G = 0,
        A_SCALE_4G,
        A_SCALE_8G,
        A_SCALE_16G
    };

    bool init_sensor();
    bool hardware_init();

    uint8_t register_read(uint8_t reg);
    void register_write(uint8_t reg, uint8_t val, bool checked=false);

    bool chip_reset();
    void fifo_reset();

    void common_init();
    void fifo_init();
    void gyro_init(enum gyro_scale scale);
    void accel_init(enum accel_scale scale);

    uint16_t get_count_fifo_unread_data();

    void set_gyro_scale(enum gyro_scale scale);
    void set_accel_scale(enum accel_scale scale);

    void poll_data();

    void update_transaction_g(struct sensor_raw_data raw_data);
    void update_transaction_x(struct sensor_raw_data raw_data);

    #if AP_INERTIALSENSOR_AMS330_DEBUG_ENABLED
    void dump_registers();
    #endif

    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    float gyro_scale;
    float accel_scale;
    enum Rotation rot;

    float temperature_degc;
    uint8_t temperature_counter;
    LowPassFilter2pFloat temperature_filter{100.0f, 1.0f};
};
