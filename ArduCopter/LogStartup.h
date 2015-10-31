#ifndef LOG_STARTUP_H
#define LOG_STARTUP_H

#include <DFMessageWriter.h>

class DFMessageWriter_LogStartup : public DFMessageWriter {
public: // TODO: fix scopes
    DFMessageWriter_LogStartup(const prog_char_t *_firmware_string) :
        DFMessageWriter(),
        stage(blockwriter_stage_init)
        {
            _dflogstart = new DFMessageWriter_DFLogStart();
            _writesysinfo = new DFMessageWriter_WriteSysInfo(_firmware_string);
        }
    void set_dataflash_backend(class DataFlash_Backend *dataflash_backend) override;

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

class DFMessageWriter_Factory_LogStartup : public DFMessageWriter_Factory
{
public:
    DFMessageWriter_Factory_LogStartup(DataFlash_Class &dataflash, const prog_char_t *firmware_string) :
        DFMessageWriter_Factory(dataflash),
        _firmware_string(firmware_string)
        { }

    DFMessageWriter_LogStartup *create() {
        return new DFMessageWriter_LogStartup(_firmware_string);
    }

private:
    const prog_char_t *_firmware_string;
};

#endif
