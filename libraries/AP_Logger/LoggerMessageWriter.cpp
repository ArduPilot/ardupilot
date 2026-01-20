#include "AP_Logger_config.h"

#if HAL_LOGGING_ENABLED

#include "AP_Common/AP_FWVersion.h"
#include "LoggerMessageWriter.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_Logger.h"
#include <AP_RCProtocol/AP_RCProtocol.h>

#if HAL_LOGGER_FENCE_ENABLED
    #include <AC_Fence/AC_Fence.h>
#endif

#if HAL_LOGGER_RALLY_ENABLED
#include <AP_Rally/AP_Rally.h>
#endif

#define FORCE_VERSION_H_INCLUDE
#include "ap_version.h"
#undef FORCE_VERSION_H_INCLUDE

// the message writers are mainly called at the end of periodic_fullrate() with a budget of 300us
// until they are finished calls to Write() will fail so we want to take up as much of
// the budget as possible in order that logging can begin in earnest as early as possible
#define MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US 50

extern const AP_HAL::HAL& hal;

/* LogStartup - these are simple state machines which allow us to
 * trickle out messages to the log files
 */

void LoggerMessageWriter::reset()
{
    _finished = false;
}

bool LoggerMessageWriter::out_of_time_for_writing_messages() const
{
#if AP_SCHEDULER_ENABLED
    return AP::scheduler().time_available_usec() < MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US;
#else
    return false;
#endif
}

void LoggerMessageWriter_DFLogStart::reset()
{
    LoggerMessageWriter::reset();

    _fmt_done = false;
    _params_done = false;
    _writesysinfo.reset();
#if AP_MISSION_ENABLED
    _writeentiremission.reset();
#endif
#if HAL_LOGGER_RALLY_ENABLED
    _writeallrallypoints.reset();
#endif
#if HAL_LOGGER_FENCE_ENABLED
    _writeallpolyfence.reset();
#endif

    stage = Stage::FORMATS;
    next_format_to_send = 0;
    _next_unit_to_send = 0;
    _next_multiplier_to_send = 0;
    _next_format_unit_to_send = 0;
    param_default = AP_Logger::quiet_nanf();
    ap = AP_Param::first(&token, &type, &param_default);
}

bool LoggerMessageWriter_DFLogStart::out_of_time_for_writing_messages_df() const
{
    if (stage == Stage::FORMATS) {
        // write out the FMT messages as fast as we can
#if AP_SCHEDULER_ENABLED
        return AP::scheduler().time_available_usec() == 0;
#else
        return false;
#endif
    }
    return LoggerMessageWriter::out_of_time_for_writing_messages();
}

/*
  check if we've taken too long in a process() stage
  return true if we should stop processing now as we are out of time
 */
bool LoggerMessageWriter_DFLogStart::check_process_limit(uint32_t start_us)
{
    const uint32_t limit_us = 1000U;
    if (AP_HAL::micros() - start_us > limit_us) {
        return true;
    }
    return false;
}

void LoggerMessageWriter_DFLogStart::process()
{
    if (out_of_time_for_writing_messages_df()) {
        return;
    }
    // allow any stage to run for max 1ms, to prevent a long loop on arming
    const uint32_t start_us = AP_HAL::micros();

    switch(stage) {
    case Stage::FORMATS:
        // write log formats so the log is self-describing
        while (next_format_to_send < _logger_backend->num_types()) {
            const auto &s { _logger_backend->structure(next_format_to_send) };
            if (_logger_backend->have_emitted_format_for_type((LogMessages)s->msg_type)) {
                next_format_to_send++;
                continue;
            }
            if (!_logger_backend->Write_Format(s)) {
                return; // call me again!
            }
            next_format_to_send++;
            if (check_process_limit(start_us)) {
                return; // call me again!
            }
        }
        _fmt_done = true;
        stage = Stage::PARMS;
        FALLTHROUGH;

    case Stage::PARMS: {
        while (ap) {
            if (!_logger_backend->Write_Parameter(ap, token, type, param_default)) {
                return;
            }
            param_default = AP_Logger::quiet_nanf();
            ap = AP_Param::next_scalar(&token, &type, &param_default);
            if (check_process_limit(start_us)) {
                return; // call me again!
            }
        }

        _params_done = true;
        stage = Stage::UNITS;
        }
        FALLTHROUGH;

    case Stage::UNITS:
        while (_next_unit_to_send < _logger_backend->num_units()) {
            if (!_logger_backend->Write_Unit(_logger_backend->unit(_next_unit_to_send))) {
                return; // call me again!
            }
            _next_unit_to_send++;
            if (check_process_limit(start_us)) {
                return; // call me again!
            }
        }
        stage = Stage::MULTIPLIERS;
        FALLTHROUGH;

    case Stage::MULTIPLIERS:
        while (_next_multiplier_to_send < _logger_backend->num_multipliers()) {
            if (!_logger_backend->Write_Multiplier(_logger_backend->multiplier(_next_multiplier_to_send))) {
                return; // call me again!
            }
            _next_multiplier_to_send++;
            if (check_process_limit(start_us)) {
                return; // call me again!
            }
        }
        stage = Stage::UNITS;
        FALLTHROUGH;

    case Stage::FORMAT_UNITS:
        while (_next_format_unit_to_send < _logger_backend->num_types()) {
            if (!_logger_backend->Write_Format_Units(_logger_backend->structure(_next_format_unit_to_send))) {
                return; // call me again!
            }
            _next_format_unit_to_send++;
            if (check_process_limit(start_us)) {
                return; // call me again!
            }
        }
        stage = Stage::RUNNING_SUBWRITERS;
        FALLTHROUGH;

    case Stage::RUNNING_SUBWRITERS: {
        LoggerMessageWriter *subwriters[] {
            &_writesysinfo,
#if AP_MISSION_ENABLED
            &_writeentiremission,
#endif
#if HAL_LOGGER_RALLY_ENABLED
            &_writeallrallypoints,
#endif
#if HAL_LOGGER_FENCE_ENABLED
            &_writeallpolyfence,
#endif
        };
        for (auto *sw : subwriters) {
            if (!sw->finished()) {
                sw->process();
                if (!sw->finished()) {
                    return;
                }
            }
        }
        stage = Stage::VEHICLE_MESSAGES;
        FALLTHROUGH;
    }
    case Stage::VEHICLE_MESSAGES:
        // we guarantee 200 bytes of space for the vehicle startup
        // messages.  This allows them to be simple functions rather
        // than e.g. LoggerMessageWriter-based state machines
        if (_logger_backend->vehicle_message_writer()) {
            if (_logger_backend->bufferspace_available() < 200) {
                return;
            }
            (_logger_backend->vehicle_message_writer())();
        }
        stage = Stage::DONE;
        FALLTHROUGH;

    case Stage::DONE:
        break;
    }

    _finished = true;
}

#if AP_MISSION_ENABLED
bool LoggerMessageWriter_DFLogStart::writeentiremission()
{
    if (stage != Stage::DONE) {
        return false;
    }
    stage = Stage::RUNNING_SUBWRITERS;
    _finished = false;
    _writeentiremission.reset();
    return true;
}
#endif

#if HAL_LOGGER_RALLY_ENABLED
bool LoggerMessageWriter_DFLogStart::writeallrallypoints()
{
    if (stage != Stage::DONE) {
        return false;
    }
    stage = Stage::RUNNING_SUBWRITERS;
    _finished = false;
    _writeallrallypoints.reset();
    return true;
}
#endif

#if HAL_LOGGER_FENCE_ENABLED
bool LoggerMessageWriter_DFLogStart::writeallfence()
{
    if (stage != Stage::DONE) {
        return false;
    }
    stage = Stage::RUNNING_SUBWRITERS;
    _finished = false;
    _writeallpolyfence.reset();
    return true;
}
#endif

void LoggerMessageWriter_WriteSysInfo::reset()
{
    LoggerMessageWriter::reset();
    stage = Stage::FIRMWARE_STRING;
}

void LoggerMessageWriter_WriteSysInfo::process() {
    const AP_FWVersion &fwver = AP::fwversion();

    switch(stage) {

    case Stage::FIRMWARE_STRING:
#ifdef AP_CUSTOM_FIRMWARE_STRING
        // also log original firmware string if different
        if (! _logger_backend->Write_MessageF("%s [%s]",
                                              fwver.fw_string,
                                              fwver.fw_string_original)) {
            return; // call me again
        }
#else
        if (! _logger_backend->Write_Message(fwver.fw_string)) {
            return; // call me again
        }
#endif
        stage = Stage::GIT_VERSIONS;
        FALLTHROUGH;

    case Stage::GIT_VERSIONS:
        if (fwver.middleware_name && fwver.os_name) {
            if (! _logger_backend->Write_MessageF("%s: %s %s: %s",
                                                        fwver.middleware_name,
                                                        fwver.middleware_hash_str,
                                                        fwver.os_name,
                                                        fwver.os_hash_str)) {
                return; // call me again
            }
        } else if (fwver.os_name) {
            if (! _logger_backend->Write_MessageF("%s: %s",
                                                        fwver.os_name,
                                                        fwver.os_hash_str)) {
                return; // call me again
            }
        }
        stage = Stage::VER;
        FALLTHROUGH;

    case Stage::VER: {
        if (!_logger_backend->Write_VER()) {
            return;
        }
        stage = Stage::SYSTEM_ID;
        FALLTHROUGH;
    }
    case Stage::SYSTEM_ID:
        char sysid[50];
        if (hal.util->get_system_id(sysid)) {
            if (! _logger_backend->Write_Message(sysid)) {
                return; // call me again
            }
        }
        stage = Stage::PARAM_SPACE_USED;
        FALLTHROUGH;

    case Stage::PARAM_SPACE_USED:
        if (! _logger_backend->Write_MessageF("Param space used: %u/%u", AP_Param::storage_used(), AP_Param::storage_size())) {
            return; // call me again
        }
        stage = Stage::RC_PROTOCOL;
        FALLTHROUGH;

    case Stage::RC_PROTOCOL: {
#if CONFIG_HAL_BOARD != HAL_BOARD_LINUX
#if AP_RCPROTOCOL_ENABLED
        const char *prot = AP::RC().detected_protocol_name();
#else
        const char *prot = nullptr;
#endif
#else  // this is not hal-chibios
        const char *prot = hal.rcin->protocol();
#endif
        if (prot == nullptr) {
            prot = "None";
        }
        if (! _logger_backend->Write_MessageF("RC Protocol: %s", prot)) {
            return; // call me again
        }
        stage = Stage::RC_OUTPUT;
        FALLTHROUGH;
    }
    case Stage::RC_OUTPUT: {
        char banner_msg[50];
        if (hal.rcout->get_output_mode_banner(banner_msg, sizeof(banner_msg))) {
            if (!_logger_backend->Write_Message(banner_msg)) {
                return; // call me again
            }
        }
        break;
    }
    }

    _finished = true;  // all done!
}

#if HAL_LOGGER_RALLY_ENABLED
void LoggerMessageWriter_WriteAllRallyPoints::process()
{
    const AP_Rally *_rally = AP::rally();
    if (_rally == nullptr) {
        _finished = true;
        return;
    }

    switch(stage) {

    case Stage::WRITE_NEW_RALLY_MESSAGE:
        if (! _logger_backend->Write_Message("New rally")) {
            return; // call me again
        }
        stage = Stage::WRITE_ALL_RALLY_POINTS;
        FALLTHROUGH;

    case Stage::WRITE_ALL_RALLY_POINTS:
        while (_rally_number_to_send < _rally->get_rally_total()) {
            if (out_of_time_for_writing_messages()) {
                return;
            }
            RallyLocation rallypoint;
            if (_rally->get_rally_point_with_index(_rally_number_to_send, rallypoint)) {
                if (!_logger_backend->Write_RallyPoint(
                        _rally->get_rally_total(),
                        _rally_number_to_send,
                        rallypoint)) {
                    return; // call me again
                }
            }
            _rally_number_to_send++;
        }
        stage = Stage::DONE;
        FALLTHROUGH;

    case Stage::DONE:
        break;
    }

    _finished = true;
}

void LoggerMessageWriter_WriteAllRallyPoints::reset()
{
    LoggerMessageWriter::reset();
    stage = Stage::WRITE_NEW_RALLY_MESSAGE;
    _rally_number_to_send = 0;
}
#endif  // HAL_LOGGER_RALLY_ENABLED

void LoggerMessageWriter_WriteEntireMission::process() {
    const AP_Mission *_mission = AP::mission();
    if (_mission == nullptr) {
        _finished = true;
        return;
    }

    switch(stage) {

    case Stage::WRITE_NEW_MISSION_MESSAGE:
        if (! _logger_backend->Write_Message("New mission")) {
            return; // call me again
        }
        stage = Stage::WRITE_MISSION_ITEMS;
        FALLTHROUGH;

    case Stage::WRITE_MISSION_ITEMS: {
        AP_Mission::Mission_Command cmd;
        while (_mission_number_to_send < _mission->num_commands()) {
            if (out_of_time_for_writing_messages()) {
                return;
            }
            // upon failure to write the mission we will re-read from
            // storage; this could be improved.
            if (_mission->read_cmd_from_storage(_mission_number_to_send,cmd)) {
                if (!_logger_backend->Write_Mission_Cmd(*_mission, cmd, LOG_CMD_MSG)) {
                    return; // call me again
                }
            }
            _mission_number_to_send++;
        }
        stage = Stage::DONE;
        FALLTHROUGH;
    }

    case Stage::DONE:
        break;
    }

    _finished = true;
}

void LoggerMessageWriter_WriteEntireMission::reset()
{
    LoggerMessageWriter::reset();
    stage = Stage::WRITE_NEW_MISSION_MESSAGE;
    _mission_number_to_send = 0;
}

#if HAL_LOGGER_FENCE_ENABLED
#if APM_BUILD_TYPE(APM_BUILD_Replay)
// dummy methods to allow build with Replay
void LoggerMessageWriter_Write_Polyfence::process() { };
void LoggerMessageWriter_Write_Polyfence::reset() { };
#else
void LoggerMessageWriter_Write_Polyfence::process() {

    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        _finished = true;
        return;
    }

    switch(stage) {

    case Stage::WRITE_NEW_FENCE_MESSAGE:
        if (!_logger_backend->Write_Message("New fence")) {
            return; // call me again
        }
        stage = Stage::WRITE_FENCE_ITEMS;
        FALLTHROUGH;

    case Stage::WRITE_FENCE_ITEMS: {
        while (_fence_number_to_send < fence->polyfence().num_stored_items()) {
            if (out_of_time_for_writing_messages()) {
                return;
            }

            // upon failure to write the fence we will re-read from
            // storage; this could be improved.
            AC_PolyFenceItem fenceitem {};
            if (fence->polyfence().get_item(_fence_number_to_send, fenceitem)) {
                if (!_logger_backend->Write_FencePoint(fence->polyfence().num_stored_items(), _fence_number_to_send, fenceitem)) {
                    return; // call me again
                }
            }
            _fence_number_to_send++;
        }
        stage = Stage::DONE;
        FALLTHROUGH;
    }

    case Stage::DONE:
        break;
    }

    _finished = true;
}

void LoggerMessageWriter_Write_Polyfence::reset()
{
    LoggerMessageWriter::reset();
    stage = Stage::WRITE_NEW_FENCE_MESSAGE;
    _fence_number_to_send = 0;
}
#endif // !APM_BUILD_TYPE(APM_BUILD_Replay)
#endif // AP_FENCE_ENABLED

#endif  // HAL_LOGGING_ENABLED
