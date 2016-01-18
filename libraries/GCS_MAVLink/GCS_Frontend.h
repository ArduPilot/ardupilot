#include "GCS.h"

#pragma once

class Parameters; // we can't include Parameters.h as it is in the vehicle dir

class GCS_Frontend {
    friend class Plane; // for access to gcs[], needed for params' var_info
    friend class Copter; // for access to gcs[], needed for params' var_info
    friend class Rover; // for access to gcs[], needed for params' var_info
    friend class Tracker; // for access to gcs[], needed for params' var_info
    friend class GCS_MAVLINK; // for access to telem_delay()

public:

    FUNCTOR_TYPEDEF(run_cli_fn, void, AP_HAL::UARTDriver*);

    GCS_Frontend(DataFlash_Class &DataFlash, Parameters &g) :
        _DataFlash(DataFlash),
        _g(g) { }

    virtual void setup_uarts(AP_SerialManager &serial_manager);

    void reset_cli_timeout();
    void println_allcli(const char *msg);

    uint32_t last_radio_status_remrssi_ms();

    void data_stream_send(void);
    void send_text(MAV_SEVERITY severity, const char *str);
    void send_statustext(mavlink_channel_t chan);
    void send_message(enum ap_message id);
    void send_mission_item_reached_message(uint16_t mission_index);
    void send_text_fmt(MAV_SEVERITY severity, const char *fmt, va_list arg_list, bool activeonly = false);
    void send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...);
    bool try_send_message(enum ap_message id);
    void send_param_value(const char *param_name, ap_var_type param_type, float param_value, bool activeonly);

    // "active" link functions.  A link is "active" if we have
    // received any packets on it.  If a link is *NOT* active, there
    // may still be a GCS on that link - just something which does not
    // transmit mavlink.  An example of this might be a HUD device.

    // send a text message to all active GCS.
    void send_text_fmt_active(MAV_SEVERITY severity, const char *fmt, va_list arg_list);
    void send_text_fmt_active(MAV_SEVERITY severity, const char *fmt, ...);
    void send_text_active(MAV_SEVERITY severity, const char *str);

    // send a parameter value to all active GCS
    // note that this does not currently use the defered queuing mechanism
    void send_param_value_active(const char *param_name, ap_var_type param_type, float param_value);

    void send_home(const Location &home);

    bool first_initialised();

    void set_run_cli_func(run_cli_fn func);
    void update(void);

    void delay_cb();

    virtual uint8_t num_gcs() const { return _num_gcs; }
    virtual GCS_MAVLINK& gcs(const uint8_t i) = 0;

    // return a bitmap of active channels. Used by libraries to loop
    // over active channels to send to all active channels note that
    // "active" here means, "have received a mavlink packet from", not
    // necessarily that there is a GCS on this channel
    uint8_t active_channel_mask(void) const { return mavlink_active; }

protected:

    // indicate that a channel has seen mavlink traffic:
    void set_channel_active(mavlink_channel_t chan);

    // FIXME: these pure virtual functions should be replaced with
    // some sort of GCS_ parameter object
    virtual uint32_t telem_delay() const = 0;

    const uint8_t _num_gcs = MAVLINK_COMM_NUM_BUFFERS;

    DataFlash_Class &_DataFlash;
    Parameters &_g;

    run_cli_fn _run_cli_func = NULL;

private:

    // bitmask of what mavlink channels are active
    uint8_t mavlink_active;

};

namespace GCS_Frontend_Static {
    GCS_Frontend& get_Frontend();
};
