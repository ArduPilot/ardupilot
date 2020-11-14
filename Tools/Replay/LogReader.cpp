#include "LogReader.h"

#include "MsgHandler.h"
#include "Replay.h"

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>


#define DEBUG 1
#if DEBUG
# define debug(fmt, args...)     printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)
#endif

#define streq(x, y) (!strcmp(x, y))

extern struct user_parameter *user_parameters;

LogReader::LogReader(struct LogStructure *log_structure, NavEKF2 &_ekf2, NavEKF3 &_ekf3) :
    AP_LoggerFileReader(),
    _log_structure(log_structure),
    ekf2(_ekf2),
    ekf3(_ekf3)
{
}
// these names we emit from the code as normal, but are emitted using
// Log_Write.  Thus they are not present in LOG_COMMON_STRUCTURES.  A
// format will be written for this by the code itself the first time
// the message is emitted to the log.  However, we must not write the
// messages from the old log to the new log, so we need to keep a map
// of IDs to prune out...
static const char *log_write_names[] = {
    nullptr
};

static const char *generated_names[] = {
    NULL
};

/*
  see if a type is in a list of types
 */
bool LogReader::in_list(const char *type, const char *list[])
{
    if (list == NULL) {
        return false;
    }
    for (uint8_t i=0; list[i] != NULL; i++) {
        if (strcmp(type, list[i]) == 0) {
            return true;
        }
    }
    return false;
}

/*
  map from an incoming format type to an outgoing format type
 */
uint8_t LogReader::map_fmt_type(const char *name, uint8_t intype)
{
    if (intype == 128) {
        // everybody's favourite FMT message...
        return 128;
    }
    if (mapped_msgid[intype] != 0) {
        // already mapped
        return mapped_msgid[intype];
    }
    for (uint8_t n=next_msgid; n<255; n++) {
        bool already_mapped = false;
        for (uint16_t i=0; i<sizeof(mapped_msgid); i++) {
            if (mapped_msgid[i] == n) {
                // already mapped - must be one of our generated names
                already_mapped = true;
                break;
            }
        }
        if (already_mapped) {
            continue;
        }
        if (AP::logger().msg_type_in_use(n)) {
            continue;
        }
        mapped_msgid[intype] = n;
        next_msgid = n+1;
        if (next_msgid == 128) {
            // don't.  Just... don't...
            next_msgid += 1;
        }
        break;
    }
    if (mapped_msgid[intype] == 0) {
        ::printf("mapping failed\n");
        abort();
    }

    return mapped_msgid[intype];    
}

bool LogReader::handle_log_format_msg(const struct log_Format &f)
{
    // emit the output as we receive it:
    AP::logger().WriteBlock((void*)&f, sizeof(f));

	char name[5];
	memset(name, '\0', 5);
	memcpy(name, f.name, 4);
	// debug("Defining log format for type (%d) (%s)\n", f.type, name);

    struct LogStructure s = _log_structure[_log_structure_count++];
    // logger.set_num_types(_log_structure_count);

    if (in_list(name, log_write_names)) {
        // debug("%s is a Log_Write-written message\n", name);
    } else {
        if (!in_list(name, generated_names)) {
            s.msg_type = map_fmt_type(name, f.type);
            s.msg_len = f.length;
            s.name = f.name;
            s.format = f.format;
            s.labels = f.labels;
        }

        // emit the FMT to AP_Logger:
        struct log_Format pkt {};
        pkt.head1 = HEAD_BYTE1;
        pkt.head2 = HEAD_BYTE2;
        pkt.msgid = LOG_FORMAT_MSG;
        pkt.type = s.msg_type;
        pkt.length = s.msg_len;
        memcpy(pkt.name, s.name, sizeof(pkt.name));
        memcpy(pkt.format, s.format, sizeof(pkt.format));
        memcpy(pkt.labels, s.labels, sizeof(pkt.labels));
    }

    if (msgparser[f.type] != NULL) {
        return true;
    }

    // map from format name to a parser subclass:
	if (streq(name, "PARM")) {
        msgparser[f.type] = new LR_MsgHandler_PARM(formats[f.type]);
    } else if (streq(name, "RFRH")) {
        msgparser[f.type] = new LR_MsgHandler_RFRH(formats[f.type]);
    } else if (streq(name, "RFRF")) {
        msgparser[f.type] = new LR_MsgHandler_RFRF(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "RFRN")) {
        msgparser[f.type] = new LR_MsgHandler_RFRN(formats[f.type]);
    } else if (streq(name, "REV2")) {
	    msgparser[f.type] = new LR_MsgHandler_REV2(formats[f.type], ekf2);
	} else if (streq(name, "RSO2")) {
	    msgparser[f.type] = new LR_MsgHandler_RSO2(formats[f.type], ekf2);
	} else if (streq(name, "RWA2")) {
	    msgparser[f.type] = new LR_MsgHandler_RWA2(formats[f.type], ekf2);
	} else if (streq(name, "REV3")) {
	    msgparser[f.type] = new LR_MsgHandler_REV3(formats[f.type], ekf3);
	} else if (streq(name, "RSO3")) {
	    msgparser[f.type] = new LR_MsgHandler_RSO3(formats[f.type], ekf3);
	} else if (streq(name, "RWA3")) {
	    msgparser[f.type] = new LR_MsgHandler_RWA3(formats[f.type], ekf3);
	} else if (streq(name, "REY3")) {
	    msgparser[f.type] = new LR_MsgHandler_REY3(formats[f.type], ekf3);
	} else if (streq(name, "RISH")) {
	    msgparser[f.type] = new LR_MsgHandler_RISH(formats[f.type]);
	} else if (streq(name, "RISI")) {
	    msgparser[f.type] = new LR_MsgHandler_RISI(formats[f.type]);
    } else if (streq(name, "RASH")) {
	    msgparser[f.type] = new LR_MsgHandler_RASH(formats[f.type]);
	} else if (streq(name, "RASI")) {
	    msgparser[f.type] = new LR_MsgHandler_RASI(formats[f.type]);
	} else if (streq(name, "RBRH")) {
	    msgparser[f.type] = new LR_MsgHandler_RBRH(formats[f.type]);
	} else if (streq(name, "RBRI")) {
	    msgparser[f.type] = new LR_MsgHandler_RBRI(formats[f.type]);
	} else if (streq(name, "RRNH")) {
	    msgparser[f.type] = new LR_MsgHandler_RRNH(formats[f.type]);
	} else if (streq(name, "RRNI")) {
	    msgparser[f.type] = new LR_MsgHandler_RRNI(formats[f.type]);
	} else if (streq(name, "RGPH")) {
	    msgparser[f.type] = new LR_MsgHandler_RGPH(formats[f.type]);
	} else if (streq(name, "RGPI")) {
	    msgparser[f.type] = new LR_MsgHandler_RGPI(formats[f.type]);
    } else if (streq(name, "RGPJ")) {
        msgparser[f.type] = new LR_MsgHandler_RGPJ(formats[f.type]);
	} else if (streq(name, "RMGH")) {
	    msgparser[f.type] = new LR_MsgHandler_RMGH(formats[f.type]);
	} else if (streq(name, "RMGI")) {
	    msgparser[f.type] = new LR_MsgHandler_RMGI(formats[f.type]);
	} else if (streq(name, "RBCH")) {
	    msgparser[f.type] = new LR_MsgHandler_RBCH(formats[f.type]);
	} else if (streq(name, "RBCI")) {
	    msgparser[f.type] = new LR_MsgHandler_RBCI(formats[f.type]);
    } else if (streq(name, "RVOH")) {
        msgparser[f.type] = new LR_MsgHandler_RVOH(formats[f.type]);
    } else if (streq(name, "ROFH")) {
        msgparser[f.type] = new LR_MsgHandler_ROFH(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "REPH")) {
        msgparser[f.type] = new LR_MsgHandler_REPH(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "REVH")) {
        msgparser[f.type] = new LR_MsgHandler_REVH(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "RWOH")) {
        msgparser[f.type] = new LR_MsgHandler_RWOH(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "RBOH")) {
        msgparser[f.type] = new LR_MsgHandler_RBOH(formats[f.type], ekf2, ekf3);
	} else {
        // debug("  No parser for (%s)\n", name);
    }

    return true;
}

bool LogReader::handle_msg(const struct log_Format &f, uint8_t *msg) {
    // emit the output as we receive it:
    AP::logger().WriteBlock(msg, f.length);

    LR_MsgHandler *p = msgparser[f.type];
    if (p == NULL) {
        return true;
    }

    p->process_message(msg);

    return true;
}

/*
  see if a user parameter is set
 */
bool LogReader::check_user_param(const char *name)
{
    for (struct user_parameter *u=user_parameters; u; u=u->next) {
        if (strcmp(name, u->name) == 0) {
            return true;
        }
    }
    return false;
}

bool LogReader::set_parameter(const char *name, float value, bool force)
{
    if (!force && check_user_param(name)) {
        // ignore user set parameters
        return false;
    }
    enum ap_var_type var_type;
    AP_Param *vp = AP_Param::find(name, &var_type);
    if (vp == NULL) {
        // a lot of parameters will not be found - e.g. FORMAT_VERSION
        // and all of the vehicle-specific parameters, ....
        return false;
    }
    float old_value = 0;
    if (var_type == AP_PARAM_FLOAT) {
        old_value = ((AP_Float *)vp)->cast_to_float();
        ((AP_Float *)vp)->set(value);
    } else if (var_type == AP_PARAM_INT32) {
        old_value = ((AP_Int32 *)vp)->cast_to_float();
        ((AP_Int32 *)vp)->set(value);
    } else if (var_type == AP_PARAM_INT16) {
        old_value = ((AP_Int16 *)vp)->cast_to_float();
        ((AP_Int16 *)vp)->set(value);
    } else if (var_type == AP_PARAM_INT8) {
        old_value = ((AP_Int8 *)vp)->cast_to_float();
        ((AP_Int8 *)vp)->set(value);
    } else {
        AP_HAL::panic("What manner of evil is var_type=%u", var_type);
    }
    if (fabsf(old_value - value) > 1.0e-12) {
        ::printf("Changed %s to %.8f from %.8f\n", name, value, old_value);
    }
    return true;
}

