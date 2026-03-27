/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#ifndef LSM6DSV_DEFAULT_ROTATION
#define LSM6DSV_DEFAULT_ROTATION ROTATION_NONE
#endif

class AP_InertialSensor_LSM6DSV : public AP_InertialSensor_Backend {
public:
    enum class LSM6DSV_Type : uint8_t {
        LSM6DSV16X,
    };

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation=LSM6DSV_DEFAULT_ROTATION);

    void start() override;
    bool update() override;
    bool get_output_banner(char* banner, uint8_t banner_len) override;

private:
    enum class SampleSourceMode : uint8_t {
        Polling,
        FIFO,
    };

    enum class FifoTag : uint8_t {
        Empty = 0x00,
        GyroNC = 0x01,
        AccelNC = 0x02,
        Temperature = 0x03,
        Timestamp = 0x04,
        CfgChange = 0x05,
        AccelNC_T2 = 0x06,
        AccelNC_T1 = 0x07,
        Accel2xC = 0x08,
        Accel3xC = 0x09,
        GyroNC_T2 = 0x0A,
        GyroNC_T1 = 0x0B,
        Gyro2xC = 0x0C,
        Gyro3xC = 0x0D,
        RouteExt = 0x1D,
        Unsupported = 0xFF,
    };

    struct SampleFrame {
        Vector3f gyro;
        Vector3f accel;
        int16_t raw_temp;
        bool has_raw_temp;
    };

    struct SourceFrame {
        SampleFrame sample;
        uint8_t gyro_status;
        uint8_t accel_status;
    };

    struct FifoFrame {
        SourceFrame source;
        FifoTag tag;
        uint16_t unread_words;
        uint8_t tag_count;
    };

    AP_InertialSensor_LSM6DSV(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    bool init();
    bool hardware_init();
    bool check_whoami();
    bool reset_device();
    bool configure_gyro();
    bool configure_accel();
    bool configure_primary_fifo();
    bool wait_for_data_ready();

    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
    bool write_register(uint8_t reg, uint8_t value, bool checked=false);
    bool fetch_primary_sample(SampleFrame &sample);
    bool read_sample(SampleFrame &sample);
    bool read_status_registers(uint8_t &gyro_status, uint8_t &accel_status, uint32_t now_us);
    bool sample_ready_for_route(uint8_t gyro_status, uint8_t accel_status, uint32_t now_us);
    bool fetch_current_sample(SampleFrame &sample, uint32_t now_us);
    bool fetch_polling_frame(SampleFrame &sample, uint32_t now_us);
    bool fetch_source_frame(SourceFrame &frame, uint32_t now_us);
    bool read_fifo_status(FifoFrame &frame, uint32_t now_us);
    bool read_fifo_words_block(uint16_t n_words, uint32_t now_us);
    bool consume_fifo_word(FifoFrame &frame, SampleFrame &sample, const uint8_t *raw_word);
    uint16_t drain_fifo(uint32_t now_us);
    void publish_gyro_sample(SampleFrame &sample);
    void publish_accel_sample(SampleFrame &sample);
    void publish_current_sample(SampleFrame &sample, uint8_t gyro_status);
    void check_register_monitor();
    void update_temperature(uint8_t status, const int16_t *raw_temp=nullptr);
    uint8_t odr_code_for_rate(uint16_t rate_hz) const;
    uint16_t calculate_backend_rate(uint16_t base_rate_hz) const;

    SampleSourceMode active_sample_source() const;
    static bool fifo_tag_supported_for_primary(FifoTag tag);
    static FifoTag decode_fifo_tag(uint8_t raw_tag);
    static uint8_t decode_fifo_tag_count(uint8_t raw_tag);

    void poll_data();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;
    enum Rotation _rotation;
    float _accel_scale;
    float _gyro_scale;
    uint8_t _whoami;
    uint8_t _temperature_counter;
    uint16_t _backend_rate_hz;
    uint32_t _backend_period_us;
    bool _fast_sampling = false;
    LSM6DSV_Type _lsm6dsv_type;
    uint8_t *_fifo_buffer;
};
