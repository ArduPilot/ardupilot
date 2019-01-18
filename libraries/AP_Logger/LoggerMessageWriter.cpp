#include "AP_Common/AP_FWVersion.h"
#include "LoggerMessageWriter.h"

#define FORCE_VERSION_H_INCLUDE
#include "ap_version.h"
#undef FORCE_VERSION_H_INCLUDE

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

    stage = ls_blockwriter_stage_init;
    next_format_to_send = 0;
    _next_unit_to_send = 0;
    _next_multiplier_to_send = 0;
    _next_format_unit_to_send = 0;
    ap = AP_Param::first(&token, &type);
}

void LoggerMessageWriter_DFLogStart::process()
{
    switch(stage) {
    case ls_blockwriter_stage_init:
        stage = ls_blockwriter_stage_formats;
        FALLTHROUGH;

    case ls_blockwriter_stage_formats:
        // write log formats so the log is self-describing
        while (next_format_to_send < _dataflash_backend->num_types()) {
            if (!_dataflash_backend->Log_Write_Format(_dataflash_backend->structure(next_format_to_send))) {
                return; // call me again!
            }
            next_format_to_send++;
        }
        _fmt_done = true;
        stage = ls_blockwriter_stage_parms;
        FALLTHROUGH;

    case ls_blockwriter_stage_units:
        while (_next_unit_to_send < _dataflash_backend->num_units()) {
            if (!_dataflash_backend->Log_Write_Unit(_dataflash_backend->unit(_next_unit_to_send))) {
                return; // call me again!
            }
            _next_unit_to_send++;
        }
        stage = ls_blockwriter_stage_multipliers;
        FALLTHROUGH;

    case ls_blockwriter_stage_multipliers:
        while (_next_multiplier_to_send < _dataflash_backend->num_multipliers()) {
            if (!_dataflash_backend->Log_Write_Multiplier(_dataflash_backend->multiplier(_next_multiplier_to_send))) {
                return; // call me again!
            }
            _next_multiplier_to_send++;
        }
        stage = ls_blockwriter_stage_units;
        FALLTHROUGH;

    case ls_blockwriter_stage_format_units:
        while (_next_format_unit_to_send < _dataflash_backend->num_types()) {
            if (!_dataflash_backend->Log_Write_Format_Units(_dataflash_backend->structure(_next_format_unit_to_send))) {
                return; // call me again!
            }
            _next_format_unit_to_send++;
        }
        stage = ls_blockwriter_stage_parms;
        FALLTHROUGH;

    case ls_blockwriter_stage_parms:
        while (ap) {
            if (!_dataflash_backend->Log_Write_Parameter(ap, token, type)) {
                return;
            }
            ap = AP_Param::next_scalar(&token, &type);
        }

        stage = ls_blockwriter_stage_sysinfo;
        FALLTHROUGH;

    case ls_blockwriter_stage_sysinfo:
        _writesysinfo.process();
        if (!_writesysinfo.finished()) {
            return;
        }
        stage = ls_blockwriter_stage_write_entire_mission;
        FALLTHROUGH;

    case ls_blockwriter_stage_write_entire_mission:
        _writeentiremission.process();
        if (!_writeentiremission.finished()) {
            return;
        }
        stage = ls_blockwriter_stage_vehicle_messages;
        FALLTHROUGH;

    case ls_blockwriter_stage_vehicle_messages:
        // we guarantee 200 bytes of space for the vehicle startup
        // messages.  This allows them to be simple functions rather
        // than e.g. LoggerMessageWriter-based state machines
        if (_dataflash_backend->vehicle_message_writer()) {
            if (_dataflash_backend->bufferspace_available() < 200) {
                return;
            }
            (_dataflash_backend->vehicle_message_writer())();
        }
        stage = ls_blockwriter_stage_done;
        FALLTHROUGH;

    case ls_blockwriter_stage_done:
        break;
    }

    _finished = true;
}

void LoggerMessageWriter_WriteSysInfo::reset()
{
    LoggerMessageWriter::reset();
    stage = ws_blockwriter_stage_init;
}

void LoggerMessageWriter_WriteSysInfo::process() {
    const AP_FWVersion &fwver = AP::fwversion();

    switch(stage) {

    case ws_blockwriter_stage_init:
        stage = ws_blockwriter_stage_firmware_string;
        FALLTHROUGH;

    case ws_blockwriter_stage_firmware_string:
        if (! _dataflash_backend->Log_Write_Message(fwver.fw_string)) {
            return; // call me again
        }
        stage = ws_blockwriter_stage_git_versions;
        FALLTHROUGH;

    case ws_blockwriter_stage_git_versions:
        if (fwver.middleware_name && fwver.os_name) {
            if (! _dataflash_backend->Log_Write_MessageF("%s: %s %s: %s",
                                                        fwver.middleware_name,
                                                        fwver.middleware_hash_str,
                                                        fwver.os_name,
                                                        fwver.os_hash_str)) {
                return; // call me again
            }
        } else if (fwver.os_name) {
            if (! _dataflash_backend->Log_Write_MessageF("%s: %s",
                                                        fwver.os_name,
                                                        fwver.os_hash_str)) {
                return; // call me again
            }
        }
        stage = ws_blockwriter_stage_system_id;
        FALLTHROUGH;

    case ws_blockwriter_stage_system_id:
        char sysid[40];
        if (hal.util->get_system_id(sysid)) {
            if (! _dataflash_backend->Log_Write_Message(sysid)) {
                return; // call me again
            }
        }
    }

    _finished = true;  // all done!
}

void LoggerMessageWriter_WriteEntireMission::process() {
    const AP_Mission *_mission = AP::mission();
    if (_mission == nullptr) {
        _finished = true;
        return;
    }

    switch(stage) {

    case em_blockwriter_stage_init:
        if (_mission == nullptr) {
            stage = em_blockwriter_stage_done;
            break;
        } else {
            stage = em_blockwriter_stage_write_new_mission_message;
        }
        FALLTHROUGH;

    case em_blockwriter_stage_write_new_mission_message:
        if (! _dataflash_backend->Log_Write_Message("New mission")) {
            return; // call me again
        }
        stage = em_blockwriter_stage_write_mission_items;
        FALLTHROUGH;

    case em_blockwriter_stage_write_mission_items: {
        AP_Mission::Mission_Command cmd;
        while (_mission_number_to_send < _mission->num_commands()) {
            // upon failure to write the mission we will re-read from
            // storage; this could be improved.
            if (_mission->read_cmd_from_storage(_mission_number_to_send,cmd)) {
                if (!_dataflash_backend->Log_Write_Mission_Cmd(*_mission, cmd)) {
                    return; // call me again
                }
            }
            _mission_number_to_send++;
        }
        stage = em_blockwriter_stage_done;
        FALLTHROUGH;
    }

    case em_blockwriter_stage_done:
        break;
    }

    _finished = true;
}

void LoggerMessageWriter_WriteEntireMission::reset()
{
    LoggerMessageWriter::reset();
    stage = em_blockwriter_stage_init;
    _mission_number_to_send = 0;
}
