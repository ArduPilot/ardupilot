#ifndef LOG_STARTUP_H
#define LOG_STARTUP_H

#include <DFMessageWriter.h>

class DFMessageWriter_LogStartup : public DFMessageWriter {
public: // TODO: fix scopes
    DFMessageWriter_LogStartup(DataFlash_Class &DataFlashx, const prog_char_t *_firmware_string) :
        DFMessageWriter(DataFlashx)
        {
            _dflogstart = new DFMessageWriter_DFLogStart(DataFlashx);
            _writesysinfo = new DFMessageWriter_WriteSysInfo(DataFlashx, _firmware_string);
        }

    void process();
    void reset();

private:

    enum logging_start_blockwriter_stage {
        blockwriter_stage_init,
        blockwriter_stage_run_dflogstart,
        blockwriter_stage_sysinfo,
        blockwriter_stage_frame,
        blockwriter_stage_control_mode
    };

    logging_start_blockwriter_stage stage;

    // state for writing out log_start messages from DataFlash (FMT,PARM):
    DFMessageWriter_DFLogStart *_dflogstart;
    DFMessageWriter_WriteSysInfo *_writesysinfo;
};

#endif
