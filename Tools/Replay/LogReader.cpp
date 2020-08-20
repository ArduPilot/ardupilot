#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>

#include "LogReader.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "MsgHandler.h"
#include "Replay.h"

#define DEBUG 1
#if DEBUG
# define debug(fmt, args...)     printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)
#endif

#define streq(x, y) (!strcmp(x, y))

extern const AP_HAL::HAL& hal;

const struct LogStructure running_codes_log_structure[] = {
    LOG_COMMON_STRUCTURES,
};

LogReader::LogReader(AP_AHRS &_ahrs,
                     AP_InertialSensor &_ins,
                     Compass &_compass,
                     AP_GPS &_gps,
                     AP_Airspeed &_airspeed,
                     AP_Logger &_logger,
                     struct LogStructure *log_structure,
                     uint8_t log_structure_count,
                     const char **&_nottypes):
    AP_LoggerFileReader(),
    vehicle(VehicleType::VEHICLE_UNKNOWN),
    ahrs(_ahrs),
    ins(_ins),
    compass(_compass),
    gps(_gps),
    airspeed(_airspeed),
    logger(_logger),
    accel_mask(7),
    gyro_mask(7),
    last_timestamp_usec(0),
    installed_vehicle_specific_parsers(false),
    _log_structure(log_structure),
    nottypes(_nottypes)
{
    if (log_structure_count != 0) {
        ::fprintf(stderr, "Do NOT put anything in the log_structure before passing it in here");
        abort(); // so there.
    }

    initialise_fmt_map();
}

struct log_Format deferred_formats[LOGREADER_MAX_FORMATS];

// some log entries (e.g. "NTUN") are used by the different vehicle
// types with wildy varying payloads.  We thus can't use the same
// parser for just any e.g. NTUN message.  We defer the registration
// of a parser for these messages until we know what model we're
// dealing with.
void LogReader::maybe_install_vehicle_specific_parsers() {
    if (! installed_vehicle_specific_parsers &&
	vehicle != VehicleType::VEHICLE_UNKNOWN) {
	switch(vehicle) {
	case VehicleType::VEHICLE_COPTER:
	    for (uint8_t i = 0; i<LOGREADER_MAX_FORMATS; i++) {
		if (deferred_formats[i].type != 0) {
		    msgparser[i] = new LR_MsgHandler_NTUN_Copter
			(deferred_formats[i], logger, last_timestamp_usec,
                         inavpos);
		}
	    }
	    break;
	case VehicleType::VEHICLE_PLANE:
	    break;
	case VehicleType::VEHICLE_ROVER:
	    break;
	case VehicleType::VEHICLE_UNKNOWN:
	    break;
	}
	installed_vehicle_specific_parsers = true;
    }
}

/*
  messages which we will be generating, so should be discarded.
  Additionally, FMT messages for messages NOT in this list will be
  passed straight through to the output file, whereas FMT messages for
  messages IN this list must come from LOG_BASE_STRUCTURES in
  LogStructure.h

  Note that there is an existing with FMTU messages, as these are
  emitted both from the common structures at log startup and also when
  Log_Write(...) is called for the first time for a particular format
  name.  Since we include it in generated_names FMTU messages will not
  be passed through from the source logs - but since the Replay code
  does call Log_Write(...) you will end up with a small selection of
  FMTU messages from that.

 */
static const char *generated_names[] = {
    "FMT",
    "FMTU",
    "NKF1", "NKF2", "NKF3", "NKF4", "NKF5", "NKF6", "NKF7", "NKF8", "NKF9", "NKF0",
    "NKQ1", "NKQ2",
    "XKF1", "XKF2", "XKF3", "XKF4", "XKF5", "XKF6", "XKF7", "XKF8", "XKF9", "XKF0",
    "XKQ1", "XKQ2", "XKFD", "XKV1", "XKV2",
    "AHR2",
    "ORGN",
    "POS",
    "CHEK",
    "IMT", "IMT2", "IMT3",
    "MAG", "MAG2",
    "BARO", "BAR2",
    "GPS","GPA",
    NULL,
};

// these names we emit from the code as normal, but are emitted using
// Log_Write.  Thus they are not present in LOG_COMMON_STRUCTURES.  A
// format will be written for this by the code itself the first time
// the message is emitted to the log.  However, we must not write the
// messages from the old log to the new log, so we need to keep a map
// of IDs to prune out...
static const char *log_write_names[] = {
    "NKA",
    "NKV",

    "NKT1",
    "NKT2",
    nullptr
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

void LogReader::initialise_fmt_map()
{
    for (const char **name = generated_names;
         *name !=nullptr;
         name++) {
        bool found = false;
        for (uint8_t n=0; n<ARRAY_SIZE(running_codes_log_structure); n++) {
            if (streq(*name, running_codes_log_structure[n].name)) {
                const uint8_t t = running_codes_log_structure[n].msg_type;
                mapped_msgid[t] = t;
                found = true;
                break;
            }
        }
        if (!found) {
            if (streq(*name, "CHEK")) {
                // HACK: CHEK is emitted using Log_Write, so doesn't
                // have a fixed address to pre-populate the fmt-map
                // with....
                continue;
            }
            ::fprintf(stderr, "Failed to find apparently-generated-name (%s) in COMMON_LOG_STRUCTURES\n", *name);
            if (strncmp(*name, "NK", 2)==0 || strncmp(*name, "XK", 2) == 0) {
                // cope with older logs
                continue;
            }
            abort();
        }
    }
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
        ::fprintf(stderr, "next_msgid=%u\n", next_msgid);
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
        break;
    }
    if (mapped_msgid[intype] == 0) {
        ::fprintf(stderr, "mapping failed\n");
        abort();
    }

    return mapped_msgid[intype];    
}

bool LogReader::save_message_type(const char *name)
{
    bool save_message = !in_list(name, generated_names);
    save_message = save_message && !in_list(name, log_write_names);
    if (save_chek_messages && strcmp(name, "CHEK") == 0) {
        save_message = true;
    }
    return save_message;
}

bool LogReader::handle_log_format_msg(const struct log_Format &f) 
{
	char name[5];
	memset(name, '\0', 5);
	memcpy(name, f.name, 4);
	debug("Defining log format for type (%d) (%s)\n", f.type, name);

    struct LogStructure s = _log_structure[_log_structure_count++];
    logger.set_num_types(_log_structure_count);

    if (in_list(name, log_write_names)) {
        debug("%s is a Log_Write-written message\n", name);
    } else {
        if (in_list(name, generated_names)) {
            debug("Log format for type (%d) (%s) taken from running code\n",
                  f.type, name);
            bool found = false;
            for (uint8_t n=0; n<ARRAY_SIZE(running_codes_log_structure); n++) {
                if (streq(name, running_codes_log_structure[n].name)) {
                    found = true;
                    memcpy(&s, &running_codes_log_structure[n], sizeof(LogStructure));
                    break;
                }
            }
            if (!found) {
                ::fprintf(stderr, "Expected to be able to emit an FMT for (%s), but no FMT message found in running code\n", name);
                abort();
            }
        } else {
            debug("Log format for type (%d) (%s) taken from log\n", f.type, name);
            // generate a LogStructure entry for this FMT
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
        strncpy(pkt.name, s.name, sizeof(pkt.name));
        strncpy(pkt.format, s.format, sizeof(pkt.format));
        strncpy(pkt.labels, s.labels, sizeof(pkt.labels));
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }

    if (msgparser[f.type] != NULL) {
        return true;
    }

	// map from format name to a parser subclass:
	if (streq(name, "PARM")) {
            msgparser[f.type] = new LR_MsgHandler_PARM
                (formats[f.type], logger,
                 last_timestamp_usec,
                 [this](const char *xname, const float xvalue) {
                    return set_parameter(xname, xvalue);
                 });
	} else if (streq(name, "GPS")) {
	    msgparser[f.type] = new LR_MsgHandler_GPS(formats[f.type],
                                                      logger,
                                                      last_timestamp_usec,
                                                      gps, ground_alt_cm);
	} else if (streq(name, "GPS2")) {
	    msgparser[f.type] = new LR_MsgHandler_GPS2(formats[f.type], logger,
                                                       last_timestamp_usec,
                                                       gps, ground_alt_cm);
	} else if (streq(name, "GPA")) {
	    msgparser[f.type] = new LR_MsgHandler_GPA(formats[f.type],
                                                      logger,
                                                      last_timestamp_usec,
                                                      gps);
	} else if (streq(name, "GPA2")) {
	    msgparser[f.type] = new LR_MsgHandler_GPA2(formats[f.type], logger,
                                                       last_timestamp_usec,
                                                       gps);
	} else if (streq(name, "MSG")) {
	    msgparser[f.type] = new LR_MsgHandler_MSG(formats[f.type], logger,
                                                   last_timestamp_usec,
						   vehicle, ahrs);
	} else if (streq(name, "IMU")) {
	    msgparser[f.type] = new LR_MsgHandler_IMU(formats[f.type], logger,
                                                   last_timestamp_usec,
						   accel_mask, gyro_mask, ins);
	} else if (streq(name, "IMU2")) {
	    msgparser[f.type] = new LR_MsgHandler_IMU2(formats[f.type], logger,
                                                    last_timestamp_usec,
						    accel_mask, gyro_mask, ins);
	} else if (streq(name, "IMU3")) {
	    msgparser[f.type] = new LR_MsgHandler_IMU3(formats[f.type], logger,
                                                    last_timestamp_usec,
						    accel_mask, gyro_mask, ins);
	} else if (streq(name, "IMT")) {
	    msgparser[f.type] = new LR_MsgHandler_IMT(formats[f.type], logger,
                                                      last_timestamp_usec,
                                                      accel_mask, gyro_mask, use_imt, ins);
	} else if (streq(name, "IMT2")) {
	    msgparser[f.type] = new LR_MsgHandler_IMT2(formats[f.type], logger,
                                                       last_timestamp_usec,
                                                       accel_mask, gyro_mask, use_imt, ins);
	} else if (streq(name, "IMT3")) {
	    msgparser[f.type] = new LR_MsgHandler_IMT3(formats[f.type], logger,
                                                       last_timestamp_usec,
                                                       accel_mask, gyro_mask, use_imt, ins);
	} else if (streq(name, "SIM")) {
	  msgparser[f.type] = new LR_MsgHandler_SIM(formats[f.type], logger,
                                                 last_timestamp_usec,
						 sim_attitude);
	} else if (streq(name, "BARO")) {
	  msgparser[f.type] = new LR_MsgHandler_BARO(formats[f.type], logger,
                                                  last_timestamp_usec);
	} else if (streq(name, "ARM")) {
	  msgparser[f.type] = new LR_MsgHandler_ARM(formats[f.type], logger,
                                                  last_timestamp_usec);
	} else if (streq(name, "EV")) {
	  msgparser[f.type] = new LR_MsgHandler_Event(formats[f.type], logger,
                                                  last_timestamp_usec);
	} else if (streq(name, "AHR2")) {
	  msgparser[f.type] = new LR_MsgHandler_AHR2(formats[f.type], logger,
						  last_timestamp_usec,
                                                  ahr2_attitude);
	} else if (streq(name, "ATT")) {
	  // this parser handles *all* attitude messages - the common one,
	  // and also the rover/copter/plane-specific (old) messages
	  msgparser[f.type] = new LR_MsgHandler_ATT(formats[f.type], logger,
						 last_timestamp_usec,
                                                 attitude);
	} else if (streq(name, "MAG")) {
	  msgparser[f.type] = new LR_MsgHandler_MAG(formats[f.type], logger,
						 last_timestamp_usec, compass);
	} else if (streq(name, "MAG2")) {
	  msgparser[f.type] = new LR_MsgHandler_MAG2(formats[f.type], logger,
						 last_timestamp_usec, compass);
	} else if (streq(name, "NTUN")) {
	    // the label "NTUN" is used by rover, copter and plane -
	    // and they all look different!  creation of a parser is
	    // deferred until we receive a MSG log entry telling us
	    // which vehicle type to use.  Sucks.
	    memcpy(&deferred_formats[f.type], &formats[f.type],
                   sizeof(struct log_Format));
	} else if (streq(name, "ARSP")) { // plane-specific(?!)
	    msgparser[f.type] = new LR_MsgHandler_ARSP(formats[f.type], logger,
                                                    last_timestamp_usec,
                                                    airspeed);
	} else if (streq(name, "NKF1")) {
	    msgparser[f.type] = new LR_MsgHandler_NKF1(formats[f.type], logger,
                                                       last_timestamp_usec);
	} else if (streq(name, "CHEK")) {
	  msgparser[f.type] = new LR_MsgHandler_CHEK(formats[f.type], logger,
                                                     last_timestamp_usec,
                                                     check_state);
	} else if (streq(name, "PM")) {
	  msgparser[f.type] = new LR_MsgHandler_PM(formats[f.type], logger,
                                                   last_timestamp_usec);
	} else {
            debug("  No parser for (%s)\n", name);
	}

        return true;
}

bool LogReader::handle_msg(const struct log_Format &f, uint8_t *msg) {
    char name[5];
    memset(name, '\0', 5);
    memcpy(name, f.name, 4);

    if (save_message_type(name)) {
        // write this message through to output log, changing the ID
        // present in the input log to that used for the same message
        // name in the output log
        if (mapped_msgid[msg[2]] == 0) {
            printf("Unknown msgid %u\n", (unsigned)msg[2]);
            exit(1);
        }
        msg[2] = mapped_msgid[msg[2]];
        if (!in_list(name, nottypes)) {
            logger.WriteBlock(msg, f.length);        
        }
        // a MsgHandler would probably have found a timestamp and
        // caled stop_clock.  This runs IO, clearing logger's
        // buffer.
        hal.scheduler->stop_clock(last_timestamp_usec);
    }

    LR_MsgHandler *p = msgparser[f.type];
    if (p == NULL) {
	return true;
    }

    p->process_message(msg);

    maybe_install_vehicle_specific_parsers();

    return true;
}

bool LogReader::wait_type(const char *wtype)
{
    while (true) {
        char type[5];
        if (!update(type)) {
            return false;
        }
        if (streq(type,wtype)) {
            break;
        }
    }
    return true;
}

bool LogReader::set_parameter(const char *name, float value)
{
    enum ap_var_type var_type;
    AP_Param *vp = AP_Param::find(name, &var_type);
    if (vp == NULL) {
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
        // we don't support mavlink set on this parameter
        return false;
    }
    if (fabsf(old_value - value) > 1.0e-12) {
        ::printf("Changed %s to %.8f from %.8f\n", name, value, old_value);
    }
    return true;
}
