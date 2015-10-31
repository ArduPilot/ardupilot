#include "LogStartup.h"

extern const AP_HAL::HAL& hal;

/* LogStartup - these are simple state machines which allow us to
 * trickle out messages to the log files
 */


void DFMessageWriter::reset()
{
    _finished = false;
}

void DFMessageWriter_DFLogStart::reset()
{
    DFMessageWriter::reset();
    stage = ls_blockwriter_stage_init;
    next_format_to_send = 0;
    ap = AP_Param::first(&token, &type);
}

void DFMessageWriter_DFLogStart::process()
{
    if (_dataflash_backend == NULL) {
        // internal_error();
        return;
    }
    switch(stage) {
    case ls_blockwriter_stage_init:
        stage = ls_blockwriter_stage_formats;
        // fall through

    case ls_blockwriter_stage_formats:
        // write log formats so the log is self-describing
        while (next_format_to_send < _dataflash_backend->_num_types) {
            if (!_dataflash_backend->Log_Write_Format(&_dataflash_backend->_structures[next_format_to_send])) {
                return; // call me again!
            }
            // avoid corrupting the APM1/APM2 dataflash by writing too fast
            // hal.scheduler->delay(10);
            next_format_to_send++;
        }
        stage = ls_blockwriter_stage_parms;
        // fall through

    case ls_blockwriter_stage_parms:
        while (ap) {
            if (!_dataflash_backend->Log_Write_Parameter(ap, token, type)) {
                return;
            }
            ap = AP_Param::next_scalar(&token, &type);
        }

        // fall through
    }

    _finished = true;
}


void DFMessageWriter_WriteSysInfo::reset()
{
    DFMessageWriter::reset();
    stage = ws_blockwriter_stage_init;
}

void DFMessageWriter_WriteSysInfo::process() {
    switch(stage) {

    case ws_blockwriter_stage_init:
        stage = ws_blockwriter_stage_firmware_string;
        // fall through

    case ws_blockwriter_stage_firmware_string:
        if (! _dataflash_backend->Log_Write_Message_P(_firmware_string)) {
            return; // call me again
        }
        stage = ws_blockwriter_stage_git_versions;
        // fall through

    case ws_blockwriter_stage_git_versions:
#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        if (! _dataflash_backend->Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION))) {
            return; // call me again
        }
#endif
        stage = ws_blockwriter_stage_system_id;
        // fall through

    case ws_blockwriter_stage_system_id:
        char sysid[40];
        if (hal.util->get_system_id(sysid)) {
            if (! _dataflash_backend->Log_Write_Message(sysid)) {
                return; // call me again
            }
        }
        // fall through
    }

    _finished = true;  // all done!
}

// void DataFlash_Class::Log_Write_SysInfo(const prog_char_t *firmware_string)
// {
//     DFMessageWriter_WriteSysInfo writer(firmware_string);
//     writer->process();
// }
