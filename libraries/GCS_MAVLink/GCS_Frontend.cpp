#include "GCS_Frontend.h"

extern const AP_HAL::HAL& hal;

/*
 *  look for incoming commands on the GCS links
 */
void GCS_Frontend::update(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].update(_run_cli_func);
        }
    }
}

// TODO: add some sanity checking that we have 4 GCS objects!
void GCS_Frontend::setup_uarts(AP_SerialManager &serial_manager)
{
    // init the GCS
    gcs[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_Console, 0);

    // setup serial port for telem1
    gcs[1].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

    // setup serial port for telem2
    gcs[2].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 1);

    // setup serial port for fourth telemetry port (not used by default)
    gcs[3].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 2);

    for (uint8_t i=0; i<4; i++) {
        gcs[i].set_frontend(this);
    }
}

// TODO: work out whether we really need the *first* one initialised,
// or whether going through setup_uarts is sufficient (in which case,
// we will get a "bool initialised()")
bool GCS_Frontend::first_initialised()
{
    return gcs[0].initialised;
}

void GCS_Frontend::println_allcli(const char *msg)
{
    if (gcs[1].initialised && (gcs[1].get_uart() != NULL)) {
        gcs[1].get_uart()->println(msg);
    }
    if (num_gcs > 2 && gcs[2].initialised && (gcs[2].get_uart() != NULL)) {
        gcs[2].get_uart()->println(msg);
    }
}

void GCS_Frontend::reset_cli_timeout()
{
    for (uint8_t i=0; i<num_gcs; i++) {
        gcs[i].reset_cli_timeout();
    }
}

// FIXME: move this variable into front end?
uint32_t GCS_Frontend::last_radio_status_remrssi_ms()
{
    return gcs[0].last_radio_status_remrssi_ms;
}

/*
 *  send data streams in the given rate range on both links
 */
void GCS_Frontend::data_stream_send(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].data_stream_send();
        }
    }
}
/*
 *  send a message on all GCS links
 */
void GCS_Frontend::send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_message(id);
        }
    }
}

/*
 *  send a mission item reached message and load the index before the send attempt in case it may get delayed
 */
void GCS_Frontend::send_mission_item_reached_message(uint16_t mission_index)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].mission_item_reached_index = mission_index;
            gcs[i].send_message(MSG_MISSION_ITEM_REACHED);
        }
    }
}


void GCS_Frontend::send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
    mavlink_msg_statustext_send(chan, s->severity, s->text);
}

void GCS_Frontend::send_text(MAV_SEVERITY severity, const char *str)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_text(severity, str);
        }
    }
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
    gcs[0].pending_status.severity = (uint8_t)severity;
    hal.util->vsnprintf((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
#if LOGGING_ENABLED == ENABLED
    _DataFlash.Log_Write_Message(gcs[0].pending_status.text);
#endif
    gcs[0].send_message(MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            gcs[i].send_message(MSG_STATUSTEXT);
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
