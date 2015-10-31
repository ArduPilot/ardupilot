#include "DataFlash_Backend.h"

extern const AP_HAL::HAL& hal;

DataFlash_Backend::DataFlash_Backend(const struct LogStructure *structure,
                                     uint8_t num_types,
                                     DFMessageWriter *writer) :
    _structures(structure),
    _num_types(num_types),
    _startup_messagewriter(writer),
    _logging_started(false),
    _writes_enabled(false),
    _writing_preface_messages(false),
    _last_periodic_1Hz(0),
    _last_periodic_10Hz(0)
{
    writer->set_dataflash_backend(this);
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

#include <stdio.h>

void DataFlash_Backend::periodic_tasks()
{
    uint32_t now = hal.scheduler->millis();
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
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    abort();
#endif
}

bool DataFlash_Backend::WriteBlockCheckPrefaceMessages()
{
    if (_startup_messagewriter == NULL) {
        internal_error();
        return false;
    }

    if (_startup_messagewriter->finished()) {
        return true;
    }

    if (_writing_preface_messages) {
        // we have been called by a messagewriter, so writing is OK
        return true;
    }

    // we didn't come from push_log_blocks, so this is some random
    // caller hoping to write blocks out.  Push out log blocks - we might
    // end up clearing the buffer.....

    push_log_blocks();
    if (_startup_messagewriter->finished()) {
        return true;
    }

    // sorry!  currently busy writing out startup messages...
    dropped++;
    return false;
}

void DataFlash_Backend::WriteMorePrefaceMessages()
{

    if (_startup_messagewriter == NULL) {
        internal_error();
        return;
    }
    if (_startup_messagewriter->finished()) {
        return;
    }

    // don't allow the blockwriter to produce more than <n>
    // messages - just to prevent any infinite loop.  At time of
    // writing, startup seems to be ~600 messages(!)
    const uint16_t limit = 1024;
    uint16_t i = 0;
    // 300 bytes should fit any message.  Possibly we need a tristate
    // instead of a boolean; 0 == all done, -1 ==call me again,
    // insufficient space, 1== call me again, more messages

    _writing_preface_messages = true;
    while (bufferspace_available() > 300) {
        if (i++ >= limit) {
            internal_error();
            _startup_messagewriter->abort();
            break;
        }
        _startup_messagewriter->process();
        if (_startup_messagewriter->finished()) {
            _startup_messagewriter->abort();
            break;
        }
    }
    _writing_preface_messages = false;
}

