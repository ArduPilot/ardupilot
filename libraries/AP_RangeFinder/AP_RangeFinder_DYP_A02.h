#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_DYP_A02_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

// DYP-A02 传感器有效量程 (cm)
#define DYP_A02_RANGE_MIN_CM  3
#define DYP_A02_RANGE_MAX_CM  450

// 安全距离范围 (cm)：读数超出 [MIN, MAX] 时判定为车辆故障（暂定，可后续调参）
#define DYP_A02_SAFE_MIN_CM   10
#define DYP_A02_SAFE_MAX_CM   50

/**
 * @brief Driver for DYP-A02-V2.0 UART automatic-output ultrasonic rangefinder
 *
 *  DYP A02 series (A02YYUW etc.) UART auto-output protocol.
 *  Sensor continuously transmits fixed 4-byte frames; no host command required.
 *
 *  UART settings: 9600 baud, 8 data bits, 1 stop bit, no parity (8N1), TTL level
 *
 *  Byte     Type        Name            Description
 *  --------------------------------------------------------------------------------------------------------------
 *  0        uint8_t     header          frame header, fixed 0xFF
 *  1        uint8_t     data_h          distance high byte (bits 15-8)
 *  2        uint8_t     data_l          distance low byte (bits 7-0)
 *  3        uint8_t     sum             checksum, low 8 bits of (header + data_h + data_l)
 *
 *  Distance (mm) = (data_h << 8) | data_l
 *  Distance (m)  = distance_mm * 0.001
 *
 *  Checksum: sum = (header + data_h + data_l) & 0xFF
 *
 *  Example frame (distance = 1953 mm = 195.3 cm):
 *    header = 0xFF, data_h = 0x07, data_l = 0xA1, sum = 0xA7
 *    sum = (0xFF + 0x07 + 0xA1) & 0xFF = 0xA7
 *    distance = 0x07A1 = 1953 mm
 */
class AP_RangeFinder_DYP_A02 : public AP_RangeFinder_Backend_Serial
{
public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_DYP_A02(_state, _params);
    }

    // 给定距离 (cm) 是否超出安全范围；true=应判为故障
    static bool distance_outside_safe_range_cm(uint16_t distance_cm);

    // 当前实例：无有效读数或超出安全距离；true=应判为故障
    bool has_safety_fault() const;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 9600;
    }

    int16_t max_distance_cm() const override {
        return MIN(params.max_distance_cm, DYP_A02_RANGE_MAX_CM);
    }

    int16_t min_distance_cm() const override {
        return MAX(params.min_distance_cm, DYP_A02_RANGE_MIN_CM);
    }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    bool get_reading(float &reading_m) override;
    bool find_frame_header(uint8_t start);

    uint8_t buffer[4];
    uint8_t buffer_used;
};

#endif
