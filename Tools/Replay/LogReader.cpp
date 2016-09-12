#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <DataFlash/DataFlash.h>

#include "LogReader.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "MsgHandler.h"
#include "Replay.h"

#define DEBUG 0
#if DEBUG
# define debug(fmt, args...)     printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)
#endif

#define streq(x, y) (!strcmp(x, y))

extern const AP_HAL::HAL& hal;

const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_CHEK_MSG, sizeof(log_Chek),
      "CHEK", "QccCLLffff",  "TimeUS,Roll,Pitch,Yaw,Lat,Lng,Alt,VN,VE,VD" }
};

LogReader::LogReader(AP_AHRS &_ahrs, AP_InertialSensor &_ins, AP_Baro &_baro, Compass &_compass, AP_GPS &_gps, 
                     AP_Airspeed &_airspeed, DataFlash_Class &_dataflash, const char **&_nottypes):
    vehicle(VehicleType::VEHICLE_UNKNOWN),
    ahrs(_ahrs),
    ins(_ins),
    baro(_baro),
    compass(_compass),
    gps(_gps),
    airspeed(_airspeed),
    dataflash(_dataflash),
    accel_mask(7),
    gyro_mask(7),
    last_timestamp_usec(0),
    installed_vehicle_specific_parsers(false),
    nottypes(_nottypes)
{
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
			(deferred_formats[i], dataflash, last_timestamp_usec,
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

LR_MsgHandler_PARM *parameter_handler;

/*
  messages which we will be generating, so should be discarded
 */
static const char *generated_names[] = { "EKF1", "EKF2", "EKF3", "EKF4", "EKF5",
                                         "NKF1", "NKF2", "NKF3", "NKF4", "NKF5",
                                         "NKF6", "NKF7", "NKF8", "NKF9", "NKF10",
                                         "AHR2", "POS", "CHEK",
                                         "IMT", "IMT2",
                                         "MAG", "MAG2",
                                         "BARO", "BAR2",
                                         "GPS","GPA",
                                         "NKA", "NKV", NULL };

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
    if (mapped_msgid[intype] != 0) {
        // already mapped
        return mapped_msgid[intype];
    }
    // see if it is in our structure list
    for (uint8_t i=0; i<ARRAY_SIZE(log_structure); i++) {
        if (strcmp(name, log_structure[i].name) == 0) {
            mapped_msgid[intype] = log_structure[i].msg_type;
            return mapped_msgid[intype];
        }
    }
    // it is a new one, allocate an ID
    mapped_msgid[intype] = next_msgid++;
    return mapped_msgid[intype];    
}

bool LogReader::save_message_type(const char *name)
{
    bool save_message = !in_list(name, generated_names);
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

        if (save_message_type(name)) {
            /* 
               any messages which we won't be generating internally in
               replay should get the original FMT header
               We need to remap the type in the FMT header to avoid
               conflicts with our current table
            */
            struct log_Format f_mapped = f;
            f_mapped.type = map_fmt_type(name, f.type);
            dataflash.WriteBlock(&f_mapped, sizeof(f_mapped));
        }

        if (msgparser[f.type] != NULL) {
            return true;
        }

	// map from format name to a parser subclass:
	if (streq(name, "PARM")) {
            parameter_handler = new LR_MsgHandler_PARM(formats[f.type], dataflash,
                                                    last_timestamp_usec);
	    msgparser[f.type] = parameter_handler;
	} else if (streq(name, "GPS")) {
	    msgparser[f.type] = new LR_MsgHandler_GPS(formats[f.type],
                                                      dataflash,
                                                      last_timestamp_usec,
                                                      gps, ground_alt_cm);
	} else if (streq(name, "GPS2")) {
	    msgparser[f.type] = new LR_MsgHandler_GPS2(formats[f.type], dataflash,
                                                       last_timestamp_usec,
                                                       gps, ground_alt_cm);
	} else if (streq(name, "GPA")) {
	    msgparser[f.type] = new LR_MsgHandler_GPA(formats[f.type],
                                                      dataflash,
                                                      last_timestamp_usec,
                                                      gps);
	} else if (streq(name, "GPA2")) {
	    msgparser[f.type] = new LR_MsgHandler_GPA2(formats[f.type], dataflash,
                                                       last_timestamp_usec,
                                                       gps);
	} else if (streq(name, "MSG")) {
	    msgparser[f.type] = new LR_MsgHandler_MSG(formats[f.type], dataflash,
                                                   last_timestamp_usec,
						   vehicle, ahrs);
	} else if (streq(name, "IMU")) {
	    msgparser[f.type] = new LR_MsgHandler_IMU(formats[f.type], dataflash,
                                                   last_timestamp_usec,
						   accel_mask, gyro_mask, ins);
	} else if (streq(name, "IMU2")) {
	    msgparser[f.type] = new LR_MsgHandler_IMU2(formats[f.type], dataflash,
                                                    last_timestamp_usec,
						    accel_mask, gyro_mask, ins);
	} else if (streq(name, "IMU3")) {
	    msgparser[f.type] = new LR_MsgHandler_IMU3(formats[f.type], dataflash,
                                                    last_timestamp_usec,
						    accel_mask, gyro_mask, ins);
	} else if (streq(name, "IMT")) {
	    msgparser[f.type] = new LR_MsgHandler_IMT(formats[f.type], dataflash,
                                                      last_timestamp_usec,
                                                      accel_mask, gyro_mask, use_imt, ins);
	} else if (streq(name, "IMT2")) {
	    msgparser[f.type] = new LR_MsgHandler_IMT2(formats[f.type], dataflash,
                                                       last_timestamp_usec,
                                                       accel_mask, gyro_mask, use_imt, ins);
	} else if (streq(name, "IMT3")) {
	    msgparser[f.type] = new LR_MsgHandler_IMT3(formats[f.type], dataflash,
                                                       last_timestamp_usec,
                                                       accel_mask, gyro_mask, use_imt, ins);
	} else if (streq(name, "SIM")) {
	  msgparser[f.type] = new LR_MsgHandler_SIM(formats[f.type], dataflash,
                                                 last_timestamp_usec,
						 sim_attitude);
	} else if (streq(name, "BARO")) {
	  msgparser[f.type] = new LR_MsgHandler_BARO(formats[f.type], dataflash,
                                                  last_timestamp_usec, baro);
	} else if (streq(name, "ARM")) {
	  msgparser[f.type] = new LR_MsgHandler_ARM(formats[f.type], dataflash,
                                                  last_timestamp_usec);
	} else if (streq(name, "EV")) {
	  msgparser[f.type] = new LR_MsgHandler_Event(formats[f.type], dataflash,
                                                  last_timestamp_usec);
	} else if (streq(name, "AHR2")) {
	  msgparser[f.type] = new LR_MsgHandler_AHR2(formats[f.type], dataflash,
						  last_timestamp_usec,
                                                  ahr2_attitude);
	} else if (streq(name, "ATT")) {
	  // this parser handles *all* attitude messages - the common one,
	  // and also the rover/copter/plane-specific (old) messages
	  msgparser[f.type] = new LR_MsgHandler_ATT(formats[f.type], dataflash,
						 last_timestamp_usec,
                                                 attitude);
	} else if (streq(name, "MAG")) {
	  msgparser[f.type] = new LR_MsgHandler_MAG(formats[f.type], dataflash,
						 last_timestamp_usec, compass);
	} else if (streq(name, "MAG2")) {
	  msgparser[f.type] = new LR_MsgHandler_MAG2(formats[f.type], dataflash,
						 last_timestamp_usec, compass);
	} else if (streq(name, "NTUN")) {
	    // the label "NTUN" is used by rover, copter and plane -
	    // and they all look different!  creation of a parser is
	    // deferred until we receive a MSG log entry telling us
	    // which vehicle type to use.  Sucks.
	    memcpy(&deferred_formats[f.type], &formats[f.type],
                   sizeof(struct log_Format));
	} else if (streq(name, "ARSP")) { // plane-specific(?!)
	    msgparser[f.type] = new LR_MsgHandler_ARSP(formats[f.type], dataflash,
                                                    last_timestamp_usec,
                                                    airspeed);
	} else if (streq(name, "NKF1")) {
	    msgparser[f.type] = new LR_MsgHandler_NKF1(formats[f.type], dataflash,
                                                       last_timestamp_usec);
	} else if (streq(name, "CHEK")) {
	  msgparser[f.type] = new LR_MsgHandler_CHEK(formats[f.type], dataflash,
                                                     last_timestamp_usec,
                                                     check_state);
	} else if (streq(name, "PM")) {
	  msgparser[f.type] = new LR_MsgHandler_PM(formats[f.type], dataflash,
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
        if (mapped_msgid[msg[2]] == 0) {
            printf("Unknown msgid %u\n", (unsigned)msg[2]);
            exit(1);
        }
        msg[2] = mapped_msgid[msg[2]];
        if (!in_list(name, nottypes)) {
            dataflash.WriteBlock(msg, f.length);        
        }
        // a MsgHandler would probably have found a timestamp and
        // caled stop_clock.  This runs IO, clearing dataflash's
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
    if (parameter_handler == NULL) {
        ::printf("No parameter format message found");
        return false;
    }
    return parameter_handler->set_parameter(name, value);
}

/*
  called when the last FMT message has been processed
 */
void LogReader::end_format_msgs(void)
{
    // write out any formats we will be producing
    for (uint8_t i=0; generated_names[i]; i++) {
        for (uint8_t n=0; n<ARRAY_SIZE(log_structure); n++) {
            if (strcmp(generated_names[i], log_structure[n].name) == 0) {
                const struct LogStructure *s = &log_structure[n];
                struct log_Format pkt {};
                pkt.head1 = HEAD_BYTE1;
                pkt.head2 = HEAD_BYTE2;
                pkt.msgid = LOG_FORMAT_MSG;
                pkt.type = s->msg_type;
                pkt.length = s->msg_len;
                strncpy(pkt.name, s->name, sizeof(pkt.name));
                strncpy(pkt.format, s->format, sizeof(pkt.format));
                strncpy(pkt.labels, s->labels, sizeof(pkt.labels));
                dataflash.WriteCriticalBlock(&pkt, sizeof(pkt));
            }
        }
    }
}
