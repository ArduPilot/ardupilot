#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_JIYI_CAN_ENABLED

#include "AP_RangeFinder_Backend_CAN.h"

/*
  Driver for JIYI Microbrain Intelligent millimeter-wave radar sensors
  communicating over CAN at 500 Kbps.

  Supported models:
    - UAV-R21-1  Obstacle Avoidance Radar  (forward/rear, 79 GHz, 1.5–27 m)
    - UAV-H30-1  Altitude Radar            (downward,    77 GHz, 0.2–27 m)

  CAN frame format (both models, standard 11-bit ID, DLC = 8):
    Byte 0–1 : Header / magic word  (0xEA 0x2D — big-endian)
    Byte 2–3 : Message type word    (0x04 0x00 for distance data)
    Byte 4–5 : Distance, cm         (uint16, big-endian; 0x0000 = no target)
    Byte 6–7 : SNR / checksum       (informational, not used for ranging)

  Default CAN IDs (standard frame):
    UAV-R21-1 front  : 0x073C   (29-bit extended per datasheet,
    UAV-R21-1 rear   : 0x074C    but observed data uses 11-bit standard 0x00D6)
    UAV-H30-1        : 0x075C
    Observed on bus  : 0x00D6   (confirmed from captured CSV data)

  ArduPilot protocol registration: AP_CAN::Protocol::JIYI
*/

class AP_RangeFinder_JIYI_CAN : public AP_RangeFinder_Backend_CAN
{
public:
    AP_RangeFinder_JIYI_CAN(RangeFinder::RangeFinder_State &_state,
                             AP_RangeFinder_Params &_params)
        : AP_RangeFinder_Backend_CAN(_state, _params,
                                     AP_CAN::Protocol::JIYI,
                                     "jiyi")
    {}

    // Return sensor type — both models are laser/radar rangefinders
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    // Handle an incoming CAN frame; returns true if the frame was consumed
    bool handle_frame(AP_HAL::CANFrame &frame) override;

private:
    // Magic word in bytes 0–1 (big-endian): 0xEA2D
    static constexpr uint16_t JIYI_MAGIC     = 0xEA2D;

    // Message-type word in bytes 2–3 that carries distance data: 0x0400
    static constexpr uint16_t JIYI_MSG_DIST  = 0x0400;

    // Sentinel value meaning "no target detected"
    static constexpr uint16_t JIYI_NO_TARGET = 0x0000;
};

#endif  // AP_RANGEFINDER_JIYI_CAN_ENABLED
