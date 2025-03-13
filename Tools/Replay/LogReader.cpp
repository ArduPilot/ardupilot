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
    ekf2(_ekf2),
    ekf3(_ekf3),
    _log_structure(log_structure)
{
}

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

bool LogReader::handle_log_format_msg(const struct log_Format &f)
{
    // emit the output as we receive it:
    AP::logger().WriteBlock((void*)&f, sizeof(f));

	char name[5];
	memset(name, '\0', 5);
	memcpy(name, f.name, 4);

    if (msgparser[f.type] != NULL) {
        return true;
    }

    // map from format name to a parser subclass:
	if (streq(name, "PARM")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_PARM(formats[f.type]);
    } else if (streq(name, "RFRH")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RFRH(formats[f.type]);
    } else if (streq(name, "RFRF")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RFRF(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "RFRN")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RFRN(formats[f.type]);
    } else if (streq(name, "REV2")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_REV2(formats[f.type], ekf2, ekf3);
	} else if (streq(name, "RSO2")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RSO2(formats[f.type], ekf2, ekf3);
	} else if (streq(name, "RWA2")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RWA2(formats[f.type], ekf2, ekf3);
	} else if (streq(name, "REV3")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_REV3(formats[f.type], ekf2, ekf3);
	} else if (streq(name, "RSO3")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RSO3(formats[f.type], ekf2, ekf3);
	} else if (streq(name, "RWA3")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RWA3(formats[f.type], ekf2, ekf3);
	} else if (streq(name, "REY3")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_REY3(formats[f.type], ekf2, ekf3);
	} else if (streq(name, "RISH")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RISH(formats[f.type]);
	} else if (streq(name, "RISI")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RISI(formats[f.type]);
    } else if (streq(name, "RASH")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RASH(formats[f.type]);
	} else if (streq(name, "RASI")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RASI(formats[f.type]);
	} else if (streq(name, "RBRH")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RBRH(formats[f.type]);
	} else if (streq(name, "RBRI")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RBRI(formats[f.type]);
	} else if (streq(name, "RRNH")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RRNH(formats[f.type]);
	} else if (streq(name, "RRNI")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RRNI(formats[f.type]);
	} else if (streq(name, "RGPH")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RGPH(formats[f.type]);
	} else if (streq(name, "RGPI")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RGPI(formats[f.type]);
    } else if (streq(name, "RGPJ")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RGPJ(formats[f.type]);
	} else if (streq(name, "RMGH")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RMGH(formats[f.type]);
	} else if (streq(name, "RMGI")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RMGI(formats[f.type]);
	} else if (streq(name, "RBCH")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RBCH(formats[f.type]);
	} else if (streq(name, "RBCI")) {
	    msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RBCI(formats[f.type]);
    } else if (streq(name, "RVOH")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RVOH(formats[f.type]);
    } else if (streq(name, "ROFH")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_ROFH(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "REPH")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_REPH(formats[f.type], ekf2, ekf3);
	} else if (streq(name, "RSLL")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RSLL(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "REVH")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_REVH(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "RWOH")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RWOH(formats[f.type], ekf2, ekf3);
    } else if (streq(name, "RBOH")) {
        msgparser[f.type] = NEW_NOTHROW LR_MsgHandler_RBOH(formats[f.type], ekf2, ekf3);
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

