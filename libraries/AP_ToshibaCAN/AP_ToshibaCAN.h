/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>

#define TOSHIBACAN_MAX_NUM_ESCS 12

class AP_ToshibaCAN : public AP_HAL::CANProtocol {
public:
    AP_ToshibaCAN();
    ~AP_ToshibaCAN();

    /* Do not allow copies */
    AP_ToshibaCAN(const AP_ToshibaCAN &other) = delete;
    AP_ToshibaCAN &operator=(const AP_ToshibaCAN&) = delete;

    // Return ToshibaCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_ToshibaCAN *get_tcan(uint8_t driver_index);

    // initialise ToshibaCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;

    // called from SRV_Channels
    void update();

    // send ESC telemetry messages over MAVLink
    void send_esc_telemetry_mavlink(uint8_t mav_chan);

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(uavcan::CanFrame &out_frame, uavcan::MonotonicTime timeout);

    // read frame on CAN bus, returns true on success
    bool read_frame(uavcan::CanFrame &recv_frame, uavcan::MonotonicTime timeout);

    bool _initialized;
    char _thread_name[9];
    uint8_t _driver_index;
    uavcan::ICanDriver* _can_driver;
    const uavcan::CanFrame* _select_frames[uavcan::MaxCanIfaces] { };

    // PWM output
    HAL_Semaphore _rc_out_sem;
    uint16_t _scaled_output[TOSHIBACAN_MAX_NUM_ESCS];
    uint16_t update_count;          // counter increments each time main thread updates outputs
    uint16_t update_count_buffered; // counter when outputs copied to buffer before before sending to ESCs
    uint16_t update_count_sent;     // counter of outputs successfully sent
    uint8_t send_stage;             // stage of sending algorithm (each stage sends one frame to ESCs)

    // telemetry data (rpm, voltage)
    HAL_Semaphore _telem_sem;
    struct telemetry_info_t {
        uint16_t rpm;
        uint16_t millivolts;
        uint16_t count;
        bool new_data;
    } _telemetry[TOSHIBACAN_MAX_NUM_ESCS];
    uint32_t _telemetry_req_ms;     // system time (in milliseconds) to request data from escs (updated at 10hz)

    // bitmask of which escs seem to be present
    uint16_t _esc_present_bitmask;

    // structure for sending motor lock command to ESC
    union motor_lock_cmd_t {
        struct PACKED {
            uint8_t motor2:4;
            uint8_t motor1:4;
            uint8_t motor4:4;
            uint8_t motor3:4;
            uint8_t motor6:4;
            uint8_t motor5:4;
            uint8_t motor8:4;
            uint8_t motor7:4;
            uint8_t motor10:4;
            uint8_t motor9:4;
            uint8_t motor12:4;
            uint8_t motor11:4;
        };
        uint8_t data[6];
    };

    // structure for sending turn rate command to ESC
    union motor_rotation_cmd_t {
        struct PACKED {
            uint16_t motor4;
            uint16_t motor3;
            uint16_t motor2;
            uint16_t motor1;
        };
        uint8_t data[8];
    };

    // structure for requesting data from ESC
    union motor_request_data_cmd_t {
        struct PACKED {
            uint8_t motor2:4;
            uint8_t motor1:4;
            uint8_t motor4:4;
            uint8_t motor3:4;
            uint8_t motor6:4;
            uint8_t motor5:4;
            uint8_t motor8:4;
            uint8_t motor7:4;
            uint8_t motor10:4;
            uint8_t motor9:4;
            uint8_t motor12:4;
            uint8_t motor11:4;
        };
        uint8_t data[6];
    };

    // structure for replies from ESC of data1 (rpm and voltage)
    union motor_reply_data1_t {
        struct PACKED {
            uint8_t rxng:1;
            uint8_t state:7;
            uint16_t rpm;
            uint16_t reserved;
            uint16_t millivolts;
            uint8_t position_est_error;
        };
        uint8_t data[8];
    };

    // frames to be sent
    uavcan::CanFrame unlock_frame;
    uavcan::CanFrame mot_rot_frame1;
    uavcan::CanFrame mot_rot_frame2;
    uavcan::CanFrame mot_rot_frame3;
};
