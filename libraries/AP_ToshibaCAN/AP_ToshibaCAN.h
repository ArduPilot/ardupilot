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

    // return a bitmask of escs that are "present" which means they are responding to requests.  Bitmask matches RC outputs
    uint16_t get_present_mask() const { return _esc_present_bitmask; }

    // return total usage time in seconds
    uint32_t get_usage_seconds(uint8_t esc_id) const;

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(uavcan::CanFrame &out_frame, uavcan::MonotonicTime timeout);

    // read frame on CAN bus, returns true on success
    bool read_frame(uavcan::CanFrame &recv_frame, uavcan::MonotonicTime timeout);

    // update esc_present_bitmask
    void update_esc_present_bitmask();

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
        int16_t rpm;                // rpm
        uint16_t voltage_cv;        // voltage in centi-volts
        uint16_t current_ca;        // current in centi-amps
        uint16_t esc_temp;          // esc temperature in degrees
        uint16_t motor_temp;        // motor temperature in degrees
        uint16_t count;             // total number of packets sent
        uint32_t usage_sec;         // motor's total usage in seconds
        uint32_t last_update_ms;    // system time telemetry was last update (used to calc total current)
        float current_tot_mah;      // total current in mAh
        bool new_data;              // true if new telemetry data has been filled in but not logged yet
    } _telemetry[TOSHIBACAN_MAX_NUM_ESCS];
    uint32_t _telemetry_req_ms;     // system time (in milliseconds) to request data from escs (updated at 10hz)
    uint8_t _telemetry_temp_req_counter;    // counter used to trigger temp data requests from ESCs (10x slower than other telem data)
    uint8_t _telemetry_usage_req_counter;   // counter used to trigger usage data requests from ESCs (100x slower than other telem data)
    const float centiamp_ms_to_mah = 1.0f / 360000.0f;  // for converting centi-amps milliseconds to mAh

    // variables for updating bitmask of responsive escs
    uint16_t _esc_present_bitmask;      // bitmask of which escs seem to be present
    uint16_t _esc_present_bitmask_recent;   // bitmask of escs that have responded in the last second
    uint32_t _esc_present_update_ms;    // system time _esc_present_bitmask was last updated

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

    // helper function to create motor_request_data_cmd_t
    motor_request_data_cmd_t get_motor_request_data_cmd(uint8_t request_id) const;

    // structure for replies from ESC of data1 (rpm and voltage)
    union motor_reply_data1_t {
        struct PACKED {
            uint8_t rxng:1;         // RX No Good. "1" if ESC encountered error receiving a message since MOTOR_DATA1 was last sent
            uint8_t stepout:1;      // "1" if a "step out" has occured since MOTOR_DATA1 was last sent
            uint8_t state:6;
            int16_t rpm;
            uint16_t current_ma;    // current in milliamps
            uint16_t voltage_mv;    // voltage in millivolts
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
