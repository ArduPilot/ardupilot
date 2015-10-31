/* A quick explanation on what you're just about to debug:
 * - we are running a state machine to trickle messages out to a small buffer
 * - we register a callback on DataFlash; it keeps calling this callback until it returns true; other WriteBlocks are blocked while this is happening.
 * - this callback is expected to be calling WriteBlock itself to add things
 * - WriteBlock may return false - hopefully indicating no space - in which case "false" is returned indicating "call me again"
 * - we are *chaining* these monsters; DataFlash wants to write out FMT and PARM messages, so we call a blockwriter over there, and when it is done we write out our own messages...
 */

void DFMessageWriter_LogStartup::reset()
{
    stage = blockwriter_stage_init;
    _finished = false;
    _dflogstart->reset();
    _writesysinfo->reset();
}

void DFMessageWriter_LogStartup::set_dataflash_backend(DataFlash_Backend *backend)
{
    DFMessageWriter::set_dataflash_backend(backend);
    _dflogstart->set_dataflash_backend(backend);
    _writesysinfo->set_dataflash_backend(backend);
}

void DFMessageWriter_LogStartup::process()
{
    switch(stage) {
    case blockwriter_stage_init:
        stage = blockwriter_stage_run_dflogstart;
        // fall through

    case blockwriter_stage_run_dflogstart:
        _dflogstart->process();
        if (!_dflogstart->finished()) {
            return;
        }
        stage = blockwriter_stage_sysinfo;
        // fall through

    case blockwriter_stage_sysinfo:
        _writesysinfo->process();
        if (!_writesysinfo->finished()) {
            return;
        }

        stage = blockwriter_stage_frame;
        // fall through

    case blockwriter_stage_frame:
        if (!_dataflash_backend->Log_Write_Message_P(PSTR("Frame: " FRAME_CONFIG_STRING))) {
            return;
        }
        stage = blockwriter_stage_control_mode;
        // fall through

    case blockwriter_stage_control_mode:
        // log the flight mode
        if (!_dataflash_backend->Log_Write_Mode(control_mode)) {
            return;
        }
        // fall through
    }

    _finished = true;

    return;
}
