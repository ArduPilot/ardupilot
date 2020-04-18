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

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/utility/RingBuffer.h>

#define TELEM_PAYLOAD_STATUS_CAPACITY          5 // size of the message buffer queue (max number of messages waiting to be sent)

// for fair scheduler
#define TELEM_TIME_SLOT_MAX               15
//#define TELEM_DEBUG

class AP_RCTelemetry {
public:
    AP_RCTelemetry(uint8_t time_slots) : _time_slots(time_slots) {}
    virtual ~AP_RCTelemetry() {};

    /* Do not allow copies */
    AP_RCTelemetry(const AP_RCTelemetry &other) = delete;
    AP_RCTelemetry &operator=(const AP_RCTelemetry&) = delete;

    // add statustext message to message queue
    void queue_message(MAV_SEVERITY severity, const char *text);

    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    uint32_t sensor_status_flags() const;

protected:
    void run_wfq_scheduler();
    // set an entry in the scheduler table
    void set_scheduler_entry(uint8_t slot, uint32_t weight, uint32_t min_period_ms) {
        _scheduler.packet_weight[slot] = weight;
        _scheduler.packet_min_period[slot] = min_period_ms;
    }
    // add an entry to the scheduler table
    void add_scheduler_entry(uint32_t weight, uint32_t min_period_ms) {
        set_scheduler_entry(_time_slots++, weight, min_period_ms);
    }
    // setup ready for passthrough operation
    virtual bool init(void);

    uint8_t _time_slots;

    struct
    {
        uint32_t last_poll_timer;
        uint32_t avg_packet_counter;
        uint32_t packet_timer[TELEM_TIME_SLOT_MAX];
        uint32_t packet_weight[TELEM_TIME_SLOT_MAX];
        uint32_t packet_min_period[TELEM_TIME_SLOT_MAX];
        uint8_t avg_packet_rate;
#ifdef TELEM_DEBUG
        uint8_t packet_rate[TELEM_TIME_SLOT_MAX];
#endif
    } _scheduler;

    struct {
        HAL_Semaphore sem;
        ObjectBuffer<mavlink_statustext_t> queue{TELEM_PAYLOAD_STATUS_CAPACITY};
        mavlink_statustext_t next;
        bool available;
    } _statustext;

private:
    uint32_t check_sensor_status_timer;
    uint32_t check_ekf_status_timer;

    // passthrough WFQ scheduler
    virtual void setup_wfq_scheduler() = 0;
    virtual bool get_next_msg_chunk(void) { return false; }
    virtual bool is_packet_ready(uint8_t idx, bool queue_empty) { return true; }
    virtual void process_packet(uint8_t idx) = 0;
    virtual void adjust_packet_weight(bool queue_empty) {};

    void update_avg_packet_rate();

    // methods to convert flight controller data to FrSky SPort Passthrough (OpenTX) format
    void check_sensor_status_flags(void);
    void check_ekf_status(void);
};
