#include "GCS.h"

#pragma once

class GCS_Frontend {
    friend class Plane; // for access to gcs[], needed for params' var_info
    friend class Copter; // for access to gcs[], needed for params' var_info
    friend class Rover; // for access to gcs[], needed for params' var_info
    friend class Tracker; // for access to gcs[], needed for params' var_info

public:

    FUNCTOR_TYPEDEF(run_cli_fn, void, AP_HAL::UARTDriver*);

    GCS_Frontend(DataFlash_Class &DataFlash) :
        _DataFlash(DataFlash) { }

    virtual void setup_uarts(AP_SerialManager &serial_manager);

    void reset_cli_timeout();
    void println_allcli(const char *msg);

    uint32_t last_radio_status_remrssi_ms();

    void data_stream_send(void);
    void send_text(MAV_SEVERITY severity, const char *str);
    void send_statustext(mavlink_channel_t chan);
    void send_message(enum ap_message id);
    void send_mission_item_reached_message(uint16_t mission_index);
    void send_text_fmt(MAV_SEVERITY severity, const char *fmt, va_list arg_list);
    void send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...);
    bool try_send_message(enum ap_message id);

    bool first_initialised();

    void set_run_cli_func(run_cli_fn func) {
        _run_cli_func = func;
    }
    void update(void);

    void delay_cb();

protected:

    const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
    GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

    DataFlash_Class &_DataFlash;
    run_cli_fn _run_cli_func = NULL;
};
