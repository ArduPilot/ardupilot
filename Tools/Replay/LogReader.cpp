#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Airspeed.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor.h>
#include <DataFlash.h>

#include "LogReader.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "MsgHandler.h"

#define streq(x, y) (!strcmp(x, y))

extern const AP_HAL::HAL& hal;

LogReader::LogReader(AP_AHRS &_ahrs, AP_InertialSensor &_ins, AP_Baro &_baro, Compass &_compass, AP_GPS &_gps, AP_Airspeed &_airspeed, DataFlash_Class &_dataflash) :
    vehicle(VehicleType::VEHICLE_UNKNOWN),
    fd(-1),
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
    installed_vehicle_specific_parsers(false)
{}

bool LogReader::open_log(const char *logfile)
{
    fd = ::open(logfile, O_RDONLY);
    if (fd == -1) {
        return false;
    }
    return true;
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
		    msgparser[i] = new MsgHandler_NTUN_Copter
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

MsgHandler_PARM *parameter_handler;

/*
  list of message types to be passed through to output log
 */
static const char *passthrough_types[] = { "MODE", "EV", "POWR", "RCIN", "RCOU", "MSG", 
                                           "CMD", "CAM", "CURR", "SIM", "TERR", "STRT", "PM", 
                                           "STAT", NULL };

/*
  see if a type is in a list of types
 */
bool LogReader::in_list(const char *type, const char *list[])
{
    for (uint8_t i=0; list[i] != NULL; i++) {
        if (strcmp(type, list[i]) == 0) {
            return true;
        }
    }
    return false;
}

bool LogReader::update(char type[5])
{
    uint8_t hdr[3];
    if (::read(fd, hdr, 3) != 3) {
        return false;
    }
    if (hdr[0] != HEAD_BYTE1 || hdr[1] != HEAD_BYTE2) {
        printf("bad log header\n");
        return false;
    }

    if (hdr[2] == LOG_FORMAT_MSG) {
        struct log_Format f;
        memcpy(&f, hdr, 3);
        if (::read(fd, &f.type, sizeof(f)-3) != sizeof(f)-3) {
            return false;
        }
        memcpy(&formats[f.type], &f, sizeof(formats[f.type]));
        strncpy(type, f.name, 4);
        type[4] = 0;

	char name[5];
	memset(name, '\0', 5);
	memcpy(name, f.name, 4);
	::printf("Defining log format for type (%d) (%s)\n", f.type, name);

        dataflash.WriteBlock(&f, sizeof(f));

	// map from format name to a parser subclass:
	if (streq(name, "PARM")) {
            parameter_handler = new MsgHandler_PARM(formats[f.type], dataflash,
                                                    last_timestamp_usec);
	    msgparser[f.type] = parameter_handler;
	} else if (streq(name, "GPS")) {
	    msgparser[f.type] = new MsgHandler_GPS(formats[f.type],
						   dataflash,
                                                   last_timestamp_usec,
                                                   gps, ground_alt_cm,
                                                   rel_altitude);
	} else if (streq(name, "GPS2")) {
	    msgparser[f.type] = new MsgHandler_GPS2(formats[f.type], dataflash,
                                                    last_timestamp_usec,
						    gps, ground_alt_cm,
						    rel_altitude);
	} else if (streq(name, "MSG")) {
	    msgparser[f.type] = new MsgHandler_MSG(formats[f.type], dataflash,
                                                   last_timestamp_usec,
						   vehicle, ahrs);
	} else if (streq(name, "IMU")) {
	    msgparser[f.type] = new MsgHandler_IMU(formats[f.type], dataflash,
                                                   last_timestamp_usec,
						   accel_mask, gyro_mask, ins);
	} else if (streq(name, "IMU2")) {
	    msgparser[f.type] = new MsgHandler_IMU2(formats[f.type], dataflash,
                                                    last_timestamp_usec,
						    accel_mask, gyro_mask, ins);
	} else if (streq(name, "IMU3")) {
	    msgparser[f.type] = new MsgHandler_IMU3(formats[f.type], dataflash,
                                                    last_timestamp_usec,
						    accel_mask, gyro_mask, ins);
	} else if (streq(name, "SIM")) {
	  msgparser[f.type] = new MsgHandler_SIM(formats[f.type], dataflash,
                                                 last_timestamp_usec,
						 sim_attitude);
	} else if (streq(name, "BARO")) {
	  msgparser[f.type] = new MsgHandler_BARO(formats[f.type], dataflash,
                                                  last_timestamp_usec, baro);
	} else if (streq(name, "ARM")) {
	  msgparser[f.type] = new MsgHandler_ARM(formats[f.type], dataflash,
                                                  last_timestamp_usec);
	} else if (streq(name, "EV")) {
	  msgparser[f.type] = new MsgHandler_Event(formats[f.type], dataflash,
                                                  last_timestamp_usec);
	} else if (streq(name, "AHR2")) {
	  msgparser[f.type] = new MsgHandler_AHR2(formats[f.type], dataflash,
						  last_timestamp_usec,
                                                  ahr2_attitude);
	} else if (streq(name, "ATT")) {
	  // this parser handles *all* attitude messages - the common one,
	  // and also the rover/copter/plane-specific (old) messages
	  msgparser[f.type] = new MsgHandler_ATT(formats[f.type], dataflash,
						 last_timestamp_usec,
                                                 attitude);
	} else if (streq(name, "MAG")) {
	  msgparser[f.type] = new MsgHandler_MAG(formats[f.type], dataflash,
						 last_timestamp_usec, compass);
	} else if (streq(name, "MAG2")) {
	  msgparser[f.type] = new MsgHandler_MAG2(formats[f.type], dataflash,
						 last_timestamp_usec, compass);
	} else if (streq(name, "NTUN")) {
	    // the label "NTUN" is used by rover, copter and plane -
	    // and they all look different!  creation of a parser is
	    // deferred until we receive a MSG log entry telling us
	    // which vehicle type to use.  Sucks.
	    memcpy(&deferred_formats[f.type], &formats[f.type],
                   sizeof(struct log_Format));
	} else if (streq(name, "ARSP")) { // plane-specific(?!)
	    msgparser[f.type] = new MsgHandler_ARSP(formats[f.type], dataflash,
                                                    last_timestamp_usec,
                                                    airspeed);
	} else {
            ::printf("  No parser for (%s)\n", name);
	}

        return true;
    }

    const struct log_Format &f = formats[hdr[2]];
    if (f.length == 0) {
        // can't just throw these away as the format specifies the
        // number of bytes in the message
        ::printf("No format defined for type (%d)\n", hdr[2]);
        exit(1);
    }

    uint8_t msg[f.length];

    memcpy(msg, hdr, 3);
    if (::read(fd, &msg[3], f.length-3) != f.length-3) {
        return false;
    }

    strncpy(type, f.name, 4);
    type[4] = 0;

    MsgHandler *p = msgparser[f.type];
    if (p == NULL) {
        if (in_list(type, passthrough_types)) {
            dataflash.WriteBlock(msg, f.length);
        }
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
