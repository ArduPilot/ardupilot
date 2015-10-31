#ifndef DF_LOGSTARTUP_H
#define DF_LOGSTARTUP_H

#include "DataFlash_Backend.h"

class DFMessageWriter {
public:
    DFMessageWriter() :
        _finished(false)
        { }

    virtual void reset() = 0;
    virtual void process() = 0;
    virtual bool finished() { return _finished; };
    void abort() { _finished = true; }
    virtual void set_dataflash_backend(class DataFlash_Backend *dataflash_backend) {
        _dataflash_backend = dataflash_backend;
    }

protected:
    bool _finished;
    DataFlash_Backend *_dataflash_backend;
};


class DFMessageWriter_WriteSysInfo : public DFMessageWriter {
public:
    DFMessageWriter_WriteSysInfo(const prog_char_t *firmware_string) :
        DFMessageWriter(),
        stage(ws_blockwriter_stage_init),
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
    DFMessageWriter_DFLogStart() :
        DFMessageWriter(),
        stage(ls_blockwriter_stage_init),
        next_format_to_send(0)
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


class DFMessageWriter_Factory {
public:
    DFMessageWriter_Factory(DataFlash_Class &dataflash) :
        _dataflash(dataflash)
        { }
    virtual DFMessageWriter *create() = 0;

protected:
    DataFlash_Class &_dataflash;
};


#endif

