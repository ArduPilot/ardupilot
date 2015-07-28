#ifndef DF_LOGSTARTUP_H
#define DF_LOGSTARTUP_H

#include "DataFlash.h"

class DFMessageWriter {
public:
    DFMessageWriter(DataFlash_Class &DataFlash) :
        _DataFlash(DataFlash) { }

    virtual void reset() = 0;
    virtual void process() = 0;
    virtual bool finished() { return _finished; };
    void abort() { _finished = true; }

protected:
    bool _finished;
    DataFlash_Class &_DataFlash;
};


class DFMessageWriter_WriteSysInfo : public DFMessageWriter {
public:
    DFMessageWriter_WriteSysInfo(DataFlash_Class &DataFlash,
                                 const prog_char_t *firmware_string) :
        DFMessageWriter(DataFlash),
        _firmware_string(firmware_string)
        { }

    void reset();
    void process();

private:
    enum write_sysinfo_blockwriter_stage {
        ws_blockwriter_stage_init,
        ws_blockwriter_stage_firmware_string,
        ws_blockwriter_stage_git_versions,
        ws_blockwriter_stage_system_id
    };
    write_sysinfo_blockwriter_stage stage;

    const prog_char_t *_firmware_string;
};

class DFMessageWriter_DFLogStart : public DFMessageWriter {
public:
    DFMessageWriter_DFLogStart(DataFlash_Class &DataFlash) :
        DFMessageWriter(DataFlash)
        { }

    void reset();
    void process();

private:

    enum log_start_blockwriter_stage {
        ls_blockwriter_stage_init,
        ls_blockwriter_stage_formats,
        ls_blockwriter_stage_parms
    };

    log_start_blockwriter_stage stage;

    uint16_t next_format_to_send;
    AP_Param::ParamToken token;
    AP_Param *ap;
    enum ap_var_type type;
    uint16_t num_format_types;
    const struct LogStructure *_structures;
};

#endif

