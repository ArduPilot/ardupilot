#pragma once

#include "VehicleType.h"
#include "DataFlashFileReader.h"
#include "LR_MsgHandler.h"
#include "Parameters.h"

class LogReader : public AP_LoggerFileReader
{
public:
    LogReader(struct LogStructure *log_structure, NavEKF2 &_ekf, NavEKF3 &_ekf3);

    VehicleType::vehicle_type vehicle;

    static bool check_user_param(const char *name);
    static bool set_parameter(const char *name, float value, bool force=false);

    bool handle_log_format_msg(const struct log_Format &f) override;
    bool handle_msg(const struct log_Format &f, uint8_t *msg) override;

    static bool in_list(const char *type, const char *list[]);

protected:

private:

    NavEKF2 &ekf2;
    NavEKF3 &ekf3;

    struct LogStructure *_log_structure;
    uint8_t _log_structure_count;

    class LR_MsgHandler *msgparser[LOGREADER_MAX_FORMATS] {};

    // mapping from original msgid to output msgid
    uint8_t mapped_msgid[256] {};

    // next available msgid for mapping
    uint8_t next_msgid = 1;

    void initialise_fmt_map();
    uint8_t map_fmt_type(const char *name, uint8_t intype);

    bool save_message_type(const char *name);
};

// some vars are difficult to get through the layers
struct globals {
    bool no_params;
};

extern struct globals globals;
