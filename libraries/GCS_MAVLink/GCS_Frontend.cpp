#include "GCS_Frontend.h"

extern const AP_HAL::HAL& hal;

#define FOR_EACH_GCS(methodcall)                \
    do {                                        \
        uint8_t count = num_gcs();              \
        for (uint8_t i=0; i<count; i++) {       \
            gcs(i).methodcall;                  \
        }                                       \
    } while (0)

#define FOR_EACH_INITIALISED_GCS(methodcall)    \
    do {                                          \
        uint8_t count = num_gcs();                \
        for (uint8_t i=0; i<count; i++) {         \
            GCS_MAVLINK &a_gcs = gcs(i);          \
            if (a_gcs.initialised) {              \
                a_gcs.methodcall;              \
            }                                     \
        }                                         \
    } while (0)

/*
 *  look for incoming commands on the GCS links
 */
void GCS_Frontend::update(void)
{
    FOR_EACH_INITIALISED_GCS(update());
}

void GCS_Frontend::set_run_cli_func(run_cli_fn func)
{
    _run_cli_func = func;
    FOR_EACH_GCS(set_run_cli_func(_run_cli_func));
}

// TODO: add some sanity checking that we have 4 GCS objects!
void GCS_Frontend::setup_uarts(AP_SerialManager &serial_manager)
{
    // init the GCS
    gcs(0).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_Console, 0);

    // setup serial port for telem1
    gcs(1).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

    // setup serial port for telem2
    gcs(2).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 1);

    // setup serial port for fourth telemetry port (not used by default)
    gcs(3).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 2);

    FOR_EACH_GCS(set_frontend(this));
}

// TODO: work out whether we really need the *first* one initialised,
// or whether going through setup_uarts is sufficient (in which case,
// we will get a "bool initialised()")
bool GCS_Frontend::first_initialised()
{
    return gcs(0).initialised;
}

void GCS_Frontend::println_allcli(const char *msg)
{
    if (gcs(1).initialised && (gcs(1).get_uart() != NULL)) {
        gcs(1).get_uart()->println(msg);
    }
    if (num_gcs() > 2 && gcs(2).initialised && (gcs(2).get_uart() != NULL)) {
        gcs(2).get_uart()->println(msg);
    }
}

void GCS_Frontend::reset_cli_timeout()
{
    FOR_EACH_GCS(reset_cli_timeout());
}

// FIXME: move this variable into front end?
uint32_t GCS_Frontend::last_radio_status_remrssi_ms()
{
    return gcs(0).last_radio_status_remrssi_ms;
}

/*
 *  send data streams in the given rate range on both links
 */
void GCS_Frontend::data_stream_send(void)
{
    FOR_EACH_INITIALISED_GCS(data_stream_send());
}
/*
 *  send a message on all GCS links
 */
void GCS_Frontend::send_message(enum ap_message id)
{
    FOR_EACH_INITIALISED_GCS(send_message(id));
}

void GCS_Frontend::send_home(const Location &home)
{
    FOR_EACH_INITIALISED_GCS(send_home(home));
}

/*
 *  send a mission item reached message and load the index before the send attempt in case it may get delayed
 */
// TODO: pass through to a send_mission_item_reached_message on each instance
void GCS_Frontend::send_mission_item_reached_message(uint16_t mission_index)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        GCS_MAVLINK &a_gcs = gcs(i);
        if (a_gcs.initialised) {
            a_gcs.mission_item_reached_index = mission_index;
            a_gcs.send_message(MSG_MISSION_ITEM_REACHED);
        }
    }
}


void GCS_Frontend::send_text(MAV_SEVERITY severity, const char *str)
{
    FOR_EACH_INITIALISED_GCS(send_text(severity, str));

#if LOGGING_ENABLED == ENABLED
    _DataFlash.Log_Write_Message(str);
#endif
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void GCS_Frontend::send_text_fmt(MAV_SEVERITY severity, const char *fmt, va_list arg_list)
{
    GCS_MAVLINK &gcs0 = gcs(0);
    gcs0.pending_status.severity = (uint8_t)severity;
    hal.util->vsnprintf((char *)gcs0.pending_status.text,
                        sizeof(gcs0.pending_status.text), fmt, arg_list);
#if LOGGING_ENABLED == ENABLED
    _DataFlash.Log_Write_Message(gcs0.pending_status.text);
#endif
    gcs0.send_message(MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs(); i++) {
        GCS_MAVLINK &a_gcs = gcs(i);
        if (a_gcs.initialised) {
            a_gcs.pending_status = gcs0.pending_status;
            a_gcs.send_message(MSG_STATUSTEXT);
        }
    }
}

void GCS_Frontend::send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    send_text_fmt(severity, fmt, arg_list);
    va_end(arg_list);
}

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void GCS_Frontend::delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;

    if (!first_initialised()) {
        return;
    }

    uint32_t tnow = AP_HAL::millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        send_message(MSG_HEARTBEAT);
        send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        update();
        data_stream_send();
        send_message(MSG_RETRY_DEFERRED);
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        send_text(MAV_SEVERITY_INFO, "Initialising APM");
    }
}

GCS_Frontend &gcs = GCS_Frontend_Static::get_Frontend();
