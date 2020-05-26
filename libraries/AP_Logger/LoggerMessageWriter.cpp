#include "AP_Common/AP_FWVersion.h"
#include "LoggerMessageWriter.h"
#include <AP_Scheduler/AP_Scheduler.h>

#define FORCE_VERSION_H_INCLUDE
#include "ap_version.h"
#undef FORCE_VERSION_H_INCLUDE

#define MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US 200

extern const AP_HAL::HAL& hal;

/* LogStartup - these are simple state machines which allow us to
 * trickle out messages to the log files
 */

void LoggerMessageWriter::reset()
{
    _finished = false;
}

void LoggerMessageWriter_DFLogStart::reset()
{
    LoggerMessageWriter::reset();

    _fmt_done = false;
    _writesysinfo.reset();
    _writeentiremission.reset();
    _writeallrallypoints.reset();

    stage = Stage::FORMATS;
    next_format_to_send = 0;
    _next_unit_to_send = 0;
    _next_multiplier_to_send = 0;
    _next_format_unit_to_send = 0;
    ap = AP_Param::first(&token, &type);
}

void LoggerMessageWriter_DFLogStart::process()
{
    switch(stage) {
    case Stage::FORMATS:
        // write log formats so the log is self-describing
        while (next_format_to_send < _logger_backend->num_types()) {
            if (AP::scheduler().time_available_usec() < MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US) {
                return;
            }
            if (!_logger_backend->Write_Format(_logger_backend->structure(next_format_to_send))) {
                return; // call me again!
            }
            next_format_to_send++;
        }
        _fmt_done = true;
        stage = Stage::UNITS;
        FALLTHROUGH;

    case Stage::UNITS:
        while (_next_unit_to_send < _logger_backend->num_units()) {
            if (AP::scheduler().time_available_usec() < MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US) {
                return;
            }
            if (!_logger_backend->Write_Unit(_logger_backend->unit(_next_unit_to_send))) {
                return; // call me again!
            }
            _next_unit_to_send++;
        }
        stage = Stage::MULTIPLIERS;
        FALLTHROUGH;

    case Stage::MULTIPLIERS:
        while (_next_multiplier_to_send < _logger_backend->num_multipliers()) {
            if (AP::scheduler().time_available_usec() < MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US) {
                return;
            }
            if (!_logger_backend->Write_Multiplier(_logger_backend->multiplier(_next_multiplier_to_send))) {
                return; // call me again!
            }
            _next_multiplier_to_send++;
        }
        stage = Stage::UNITS;
        FALLTHROUGH;

    case Stage::FORMAT_UNITS:
        while (_next_format_unit_to_send < _logger_backend->num_types()) {
            if (AP::scheduler().time_available_usec() < MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US) {
                return;
            }
            if (!_logger_backend->Write_Format_Units(_logger_backend->structure(_next_format_unit_to_send))) {
                return; // call me again!
            }
            _next_format_unit_to_send++;
        }
        stage = Stage::PARMS;
        FALLTHROUGH;

    case Stage::PARMS:
        while (ap) {
            if (AP::scheduler().time_available_usec() < MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US) {
                return;
            }
            if (!_logger_backend->Write_Parameter(ap, token, type)) {
                return;
            }
            ap = AP_Param::next_scalar(&token, &type);
        }

        stage = Stage::RUNNING_SUBWRITERS;
        FALLTHROUGH;

    case Stage::RUNNING_SUBWRITERS:
        if (!_writesysinfo.finished()) {
            _writesysinfo.process();
            if (!_writesysinfo.finished()) {
                return;
            }
        }
        if (!_writeentiremission.finished()) {
            _writeentiremission.process();
            if (!_writeentiremission.finished()) {
                return;
            }
        }
        if (!_writeallrallypoints.finished()) {
            _writeallrallypoints.process();
            if (!_writeallrallypoints.finished()) {
                return;
            }
        }
        stage = Stage::VEHICLE_MESSAGES;
        FALLTHROUGH;

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

void LoggerMessageWriter_WriteSysInfo::reset()
{
    LoggerMessageWriter::reset();
    stage = Stage::FIRMWARE_STRING;
}

void LoggerMessageWriter_WriteSysInfo::process() {
    const AP_FWVersion &fwver = AP::fwversion();

    switch(stage) {

    case Stage::FIRMWARE_STRING:
        if (! _logger_backend->Write_Message(fwver.fw_string)) {
            return; // call me again
        }
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
        stage = Stage::SYSTEM_ID;
        FALLTHROUGH;

    case Stage::SYSTEM_ID:
        char sysid[40];
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

    case Stage::RC_PROTOCOL:
        const char *prot = hal.rcin->protocol();
        if (prot == nullptr) {
            prot = "None";
        }
        if (! _logger_backend->Write_MessageF("RC Protocol: %s", prot)) {
            return; // call me again
        }
    }

    _finished = true;  // all done!
}

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
            if (AP::scheduler().time_available_usec() < MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US) {
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
            if (AP::scheduler().time_available_usec() < MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US) {
                return;
            }
            // upon failure to write the mission we will re-read from
            // storage; this could be improved.
            if (_mission->read_cmd_from_storage(_mission_number_to_send,cmd)) {
                if (!_logger_backend->Write_Mission_Cmd(*_mission, cmd)) {
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
