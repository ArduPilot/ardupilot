#include "DataFlash_Backend.h"

#include "DFMessageWriter.h"

extern const AP_HAL::HAL& hal;

DataFlash_Backend::DataFlash_Backend(DataFlash_Class &front,
                                     class DFMessageWriter_DFLogStart *writer) :
    _front(front),
    _startup_messagewriter(writer)
{
    writer->set_dataflash_backend(this);
}

uint8_t DataFlash_Backend::num_types() const
{
    return _front._num_types;
}

const struct LogStructure *DataFlash_Backend::structure(uint8_t num) const
{
    return _front.structure(num);
}

DataFlash_Backend::vehicle_startup_message_Log_Writer DataFlash_Backend::vehicle_message_writer() {
    return _front._vehicle_messages;
}

void DataFlash_Backend::periodic_10Hz(const uint32_t now)
{
}
void DataFlash_Backend::periodic_1Hz(const uint32_t now)
{
}
void DataFlash_Backend::periodic_fullrate(const uint32_t now)
{
}

void DataFlash_Backend::periodic_tasks()
{
    uint32_t now = AP_HAL::millis();
    if (now - _last_periodic_1Hz > 1000) {
        periodic_1Hz(now);
        _last_periodic_1Hz = now;
    }
    if (now - _last_periodic_10Hz > 100) {
        periodic_10Hz(now);
        _last_periodic_10Hz = now;
    }
    periodic_fullrate(now);
}


void DataFlash_Backend::internal_error() {
    _internal_errors++;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    abort();
#endif
}

void DataFlash_Backend::set_mission(const AP_Mission *mission) {
    _startup_messagewriter->set_mission(mission);
}

// this method can be overridden to do extra things with your buffer.
// for example, in DataFlash_MAVLink we may push messages into the UART.
void DataFlash_Backend::push_log_blocks() {
    WriteMoreStartupMessages();
}

// returns true if all format messages have been written, and thus it is OK
// for other messages to go out to the log
bool DataFlash_Backend::WriteBlockCheckStartupMessages()
{
    if (_startup_messagewriter->fmt_done()) {
        return true;
    }

    if (_writing_startup_messages) {
        // we have been called by a messagewriter, so writing is OK
        return true;
    }

    // we're not writing startup messages, so this must be some random
    // caller hoping to write blocks out.  Push out log blocks - we
    // might end up clearing the buffer.....
    push_log_blocks();
    if (_startup_messagewriter->fmt_done()) {
        return true;
    }

    // sorry!  currently busy writing out startup messages...
    return false;
}

// source more messages from the startup message writer:
void DataFlash_Backend::WriteMoreStartupMessages()
{

    if (_startup_messagewriter->finished()) {
        return;
    }

    _writing_startup_messages = true;
    _startup_messagewriter->process();
    _writing_startup_messages = false;
}
