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

#define streq(x, y) (!strcmp(x, y))

extern const AP_HAL::HAL& hal;

LogReader::LogReader(AP_AHRS &_ahrs, AP_InertialSensor &_ins, AP_Baro &_baro, Compass &_compass, AP_GPS &_gps, AP_Airspeed &_airspeed, DataFlash_Class &_dataflash) :
    vehicle(VEHICLE_UNKNOWN),
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
    last_timestamp_usec(0)
{}

void fatal(const char *msg) {
    ::printf("%s",msg);
    ::printf("\n");
    exit(1);
}

char *xstrdup(const char *string)
{
    char *ret = strdup(string);
    if (ret == NULL) {
        perror("strdup");
        fatal("strdup failed");
    }
    return ret;
}

#define LOGREADER_MAX_FIELDS 30
class MsgParser {
public:
    // constructor - create a parser for a MavLink message format
    MsgParser(struct log_Format f);

    // field_value - retrieve the value of a field from the supplied message
    // these return false if the field was not found
    template<typename R>
    bool field_value(uint8_t *msg, const char *label, R &ret);

    bool field_value(uint8_t *msg, const char *label, Vector3f &ret);
    bool field_value(uint8_t *msg, const char *label,
		     char *buffer, uint8_t bufferlen);

    // retrieve a comma-separated list of all labels
    void string_for_labels(char *buffer, uint bufferlen);

private:

    void add_field(const char *_label, uint8_t _type, uint8_t _offset,
                   uint8_t length);

    template<typename R>
    void field_value_for_type_at_offset(uint8_t *msg, uint8_t type,
                                        uint8_t offset, R &ret);

    struct log_Format f; // the format we are a parser for
    struct format_field_info { // parsed field information
        char *label;
        uint8_t type;
        uint8_t offset;
        uint8_t length;
    };
    struct format_field_info field_info[LOGREADER_MAX_FIELDS];

    uint8_t next_field;
    size_t size_for_type_table[52]; // maps field type (e.g. 'f') to e.g 4 bytes

    struct format_field_info *find_field_info(const char *label);

    void parse_format_fields();
    void init_field_types();
    void add_field_type(char type, size_t size);
    uint8_t size_for_type(char type);

    ~MsgParser();
};

void MsgParser::add_field_type(char type, size_t size)
{
    size_for_type_table[(type > 'A' ? (type-'A') : (type-'a'))] = size;
}

uint8_t MsgParser::size_for_type(char type)
{
    return size_for_type_table[(uint8_t)(type > 'A' ? (type-'A') : (type-'a'))];
}

void MsgParser::init_field_types()
{
    add_field_type('b', sizeof(int8_t));
    add_field_type('c', sizeof(int16_t));
    add_field_type('e', sizeof(int32_t));
    add_field_type('f', sizeof(float));
    add_field_type('h', sizeof(int16_t));
    add_field_type('i', sizeof(int32_t));
    add_field_type('n', sizeof(char[4]));
    add_field_type('B', sizeof(uint8_t));
    add_field_type('C', sizeof(uint16_t));
    add_field_type('E', sizeof(uint32_t));
    add_field_type('H', sizeof(uint16_t));
    add_field_type('I', sizeof(uint32_t));
    add_field_type('L', sizeof(int32_t));
    add_field_type('M', sizeof(uint8_t));
    add_field_type('N', sizeof(char[16]));
    add_field_type('Z', sizeof(char[64]));
}

struct MsgParser::format_field_info *MsgParser::find_field_info(const char *label)
{
    for(uint8_t i=0; i<next_field; i++) {
        if (streq(field_info[i].label, label)) {
            return &field_info[i];
        }
    }
    return NULL;
}

MsgParser::MsgParser(struct log_Format fx) : next_field(0), f(fx)
{
    init_field_types();
    parse_format_fields();
}


void MsgParser::add_field(const char *_label, uint8_t _type, uint8_t _offset,
                          uint8_t _length)
{
    field_info[next_field].label = xstrdup(_label);
    field_info[next_field].type = _type;
    field_info[next_field].offset = _offset;
    field_info[next_field].length = _length;
    next_field++;
}

void MsgParser::parse_format_fields()
{
    char *labels = xstrdup(f.labels);
    char * arg = labels;
    uint8_t label_offset = 0;
    char *next_label;
    uint8_t msg_offset = 3; // 3 bytes for the header

    while ((next_label = strtok(arg, ",")) != NULL) {
	if (label_offset > strlen(f.format)) {
	    free(labels);
	    printf("too few field times for labels %s (format=%s) (labels=%s)\n",
		   f.name, f.format, f.labels);
	    exit(1);
	}
        uint8_t field_type = f.format[label_offset];
        uint8_t length = size_for_type(field_type);
        add_field(next_label, field_type, msg_offset, length);
        arg = NULL;
        msg_offset += length;
        label_offset++;
    }

    if (label_offset != strlen(f.format)) {
	    free(labels);
	    printf("too few labels for format (format=%s) (labels=%s)\n",
		   f.format, f.labels);
    }

    free(labels);
}


template<typename R>
inline void MsgParser::field_value_for_type_at_offset(uint8_t *msg,
                                                      uint8_t type,
                                                      uint8_t offset,
                                                      R &ret)
{
    /* we register the types - add_field_type - so can we do without
     * this switch statement somehow? */
    switch (type) {
    case 'B':
        ret = (R)(((uint8_t*)&msg[offset])[0]);
        break;
    case 'c':
    case 'h':
        ret = (R)(((int16_t*)&msg[offset])[0]);
        break;
    case 'H':
        ret = (R)(((uint16_t*)&msg[offset])[0]);
        break;
    case 'C':
        ret = (R)(((uint16_t*)&msg[offset])[0]);
        break;
    case 'f':
        ret = (R)(((float*)&msg[offset])[0]);
        break;
    case 'I':
    case 'E':
        ret = (R)(((uint32_t*)&msg[offset])[0]);
        break;
    case 'L':
    case 'e':
        ret = (R)(((int32_t*)&msg[offset])[0]);
        break;
    default:
        ::printf("Unhandled format type (%c)\n", type);
        exit(1);
    }
}

template<typename R>
bool MsgParser::field_value(uint8_t *msg, const char *label, R &ret)
{
    struct format_field_info *info = find_field_info(label);
    uint8_t offset = info->offset;
    if (offset == 0) {
        return false;
    }

    field_value_for_type_at_offset(msg, info->type, offset, ret);

    return true;
}

bool MsgParser::field_value(uint8_t *msg, const char *label, char *ret, uint8_t retlen)
{
    struct format_field_info *info = find_field_info(label);
    uint8_t offset = info->offset;
    if (offset == 0) {
        return false;
    }

    memset(ret, '\0', retlen);

    memcpy(ret, &msg[offset], (retlen < info->length) ? retlen : info->length);

    return true;
}


bool MsgParser::field_value(uint8_t *msg, const char *label, Vector3f &ret)
{
    const char *axes = "XYZ";
    uint8_t i;
    for(i=0; i<next_field; i++) {
	if (!strncmp(field_info[i].label, label, strlen(label)) &&
	    strlen(field_info[i].label) == strlen(label)+1) {
	    for (uint8_t j=0; j<3; j++) {
		if (field_info[i].label[strlen(label)] == axes[j]) {
                    field_value_for_type_at_offset(msg,
                                                   field_info[i].type,
                                                   field_info[i].offset,
                                                   ret[j]);
                    break; // break from finding-label loop
                }
            }
        }
        if (i == next_field) {
            return 0; // not found
        }
    }

    return true;
}


void MsgParser::string_for_labels(char *buffer, uint bufferlen)
{
    memset(buffer, '\0', bufferlen);
    bufferlen--;

    char *pos = buffer;
    for (uint8_t k=0; k<LOGREADER_MAX_FIELDS; k++) {
        if (field_info[k].label != NULL) {
            uint8_t remaining = bufferlen - (pos - buffer);
            uint8_t label_length = strlen(field_info[k].label);
            uint8_t required = label_length;
            if (pos != buffer) { // room for a comma
                required++;
            }
            if (required+1 > remaining) { // null termination
                break;
            }

            if (pos != buffer) {
                *pos++ = ',';
            }

            memcpy(pos, field_info[k].label, label_length);
            pos += label_length;
        }
    }
}

MsgParser::~MsgParser()
{
    for (uint8_t k=0; k<LOGREADER_MAX_FIELDS; k++) {
        if (field_info[k].label != NULL) {
            free(field_info[k].label);
        }
    }
}

bool LogReader::open_log(const char *logfile)
{
    fd = ::open(logfile, O_RDONLY);
    if (fd == -1) {
        return false;
    }
    return true;
}

void LogReader::update_plane(uint8_t type, uint8_t *data, uint16_t length)
{
    MsgParser *p = msgparser[type];

    switch (type) {
    case LOG_PLANE_COMPASS_MSG: {
        update_from_msg_compass(0, p, data);
        break;
    }

    case LOG_PLANE_ATTITUDE_MSG: {
        wait_timestamp_from_msg(p, data);
        attitude_from_msg(p, data, attitude, "Roll", "Pitch", "Yaw");
        break;
    }

    case LOG_PLANE_AIRSPEED_MSG:
    case LOG_ARSP_MSG: {
	wait_timestamp_from_msg(p, data);
	p = msgparser[LOG_ARSP_MSG]; // this actually uses a non-"old" structure
	airspeed.setHIL(require_field_float(p, data, "AirSpeed"),
			require_field_float(p, data, "DiffPress"),
			require_field_float(p, data, "Temp"));
        dataflash.Log_Write_Airspeed(airspeed);
        break;
    }
    }
}

void LogReader::update_rover(uint8_t type, uint8_t *data, uint16_t length)
{
    MsgParser *p = msgparser[type];

    switch (type) {
    case LOG_ROVER_COMPASS_MSG: {
        wait_timestamp_from_msg(p, data);
        update_from_msg_compass(0, p, data);
        break;
    }

    case LOG_ROVER_ATTITUDE_MSG: {
        wait_timestamp_from_msg(p, data);
        attitude_from_msg(p, data, attitude, "Roll", "Pitch", "Yaw");
        break;
    }
    }
}

void LogReader::update_copter(uint8_t type, uint8_t *data, uint16_t length)
{
    MsgParser *p = msgparser[type];

    switch (type) {
    case LOG_COPTER_COMPASS_MSG: {
        wait_timestamp_from_msg(p, data);
        update_from_msg_compass(0, p, data);
        break;
    }

    case LOG_COPTER_ATTITUDE_MSG: {
        wait_timestamp_from_msg(p, data);
        attitude_from_msg(p, data, attitude, "Roll", "Pitch", "Yaw");
        break;
    }

    case LOG_COPTER_NAV_TUNING_MSG: {
        inavpos = Vector3f(require_field_float(p, data, "PosX") * 0.01f,
			   require_field_float(p, data, "PosY") * 0.01f,
			   0);
        break;
    }
    }
}

bool LogReader::set_parameter(const char *name, float value)
{
    const char *ignore_parms[] = { "GPS_TYPE", "AHRS_EKF_USE" };
    for (uint8_t i=0; i<sizeof(ignore_parms)/sizeof(ignore_parms[0]); i++) {
        if (strncmp(name, ignore_parms[i], AP_MAX_NAME_SIZE) == 0) {
            ::printf("Ignoring set of %s to %f\n", name, value);
            return true;
        }
    }
    enum ap_var_type var_type;
    AP_Param *vp = AP_Param::find(name, &var_type);
    if (vp == NULL) {
        return false;
    }
    if (var_type == AP_PARAM_FLOAT) {
        ((AP_Float *)vp)->set(value);
        ::printf("Set %s to %f\n", name, value);
    } else if (var_type == AP_PARAM_INT32) {
        ((AP_Int32 *)vp)->set(value);
        ::printf("Set %s to %d\n", name, (int)value);
    } else if (var_type == AP_PARAM_INT16) {
        ((AP_Int16 *)vp)->set(value);
        ::printf("Set %s to %d\n", name, (int)value);
    } else if (var_type == AP_PARAM_INT8) {
        ((AP_Int8 *)vp)->set(value);
        ::printf("Set %s to %d\n", name, (int)value);
    } else {
        // we don't support mavlink set on this parameter
        return false;
    }            
    return true;
}

template <typename R>
void LogReader::require_field(class MsgParser *p, uint8_t *msg, const char *label, R &ret)
{
    if (! p->field_value(msg, label, ret)) {
        char all_labels[256];
        p->string_for_labels(all_labels, 256);
        ::printf("Field (%s) not found; options are (%s)\n", label, all_labels);
        exit(1);
    }
}

void LogReader::require_field(MsgParser *p, uint8_t *msg, const char *label, char *buffer, uint8_t bufferlen)
{
    if (! p->field_value(msg, label, buffer, bufferlen)) {
        char all_labels[256];
        p->string_for_labels(all_labels, 256);
        ::printf("Field (%s) not found; options are (%s)\n", label, all_labels);
        exit(1);
    }
}

// start convenience functions for getting values for fields.  Also,
// these are required in some places as otherwise we attempt to take
// references to bit-packed data structures...
uint8_t LogReader::require_field_uint8_t(class MsgParser *p, uint8_t *msg, const char *label)
{
    uint8_t ret;
    require_field(p, msg, label, ret);
    return ret;
}
uint16_t LogReader::require_field_uint16_t(class MsgParser *p, uint8_t *msg, const char *label)
{
    uint16_t ret;
    require_field(p, msg, label, ret);
    return ret;
}
int16_t LogReader::require_field_int16_t(class MsgParser *p, uint8_t *msg, const char *label)
{
    int16_t ret;
    require_field(p, msg, label, ret);
    return ret;
}

int32_t LogReader::require_field_int32_t(class MsgParser *p, uint8_t *msg, const char *label)
{
    int32_t ret;
    require_field(p, msg, label, ret);
    return ret;
}
float LogReader::require_field_float(class MsgParser *p, uint8_t *msg, const char *label)
{
    float ret;
    require_field(p, msg, label, ret);
    return ret;
}

void LogReader::wait_timestamp_from_msg(class MsgParser *p, uint8_t *msg)
{
    uint32_t timestamp;
    require_field(p, msg, "TimeMS", timestamp);
    wait_timestamp(timestamp);
}

void LogReader::update_from_msg_imu(uint8_t imu_offset, class MsgParser *p, uint8_t *msg)
{
    wait_timestamp_from_msg(p, msg);

    uint8_t this_imu_mask = 1 << imu_offset;

    if (gyro_mask & this_imu_mask) {
        Vector3f gyro;
        require_field(p, msg, "Gyr", gyro);
        ins.set_gyro(imu_offset, gyro);
    }
    if (accel_mask & this_imu_mask) {
        Vector3f accel2;
        require_field(p, msg, "Acc", accel2);
        ins.set_accel(imu_offset, accel2);
    }

    dataflash.Log_Write_IMU(ins);
}

void LogReader::location_from_msg(class MsgParser *p, uint8_t *msg,
                                  Location &loc,
                                  const char *label_lat,
                                  const char *label_long,
                                  const char *label_alt)
{
    loc.lat = require_field_int32_t(p, msg, label_lat);
    loc.lng = require_field_int32_t(p, msg, label_long);
    loc.alt = require_field_int32_t(p, msg, label_alt);
    loc.options = 0;
}

void LogReader::ground_vel_from_msg(class MsgParser *p, uint8_t *msg,
                                    Vector3f &vel,
                                    const char *label_speed,
                                    const char *label_course,
                                    const char *label_vz)
{
    uint32_t ground_speed;
    int32_t ground_course;
    require_field(p, msg, label_speed, ground_speed);
    require_field(p, msg, label_course, ground_course);
    vel[0] = ground_speed*0.01f*cosf(radians(ground_course*0.01f));
    vel[1] = ground_speed*0.01f*sinf(radians(ground_course*0.01f));
    vel[2] = require_field_float(p, msg, label_vz);
}

void LogReader::attitude_from_msg(class MsgParser *p, uint8_t *msg,
                                  Vector3f &att,
                                  const char *label_roll,
                                  const char *label_pitch,
                                  const char *label_yaw)
{
    att[0] = require_field_int16_t(p, msg, label_roll) * 0.01f;
    att[1] = require_field_int16_t(p, msg, label_pitch) * 0.01f;
    att[2] = require_field_uint16_t(p, msg, label_yaw) * 0.01f;
}

void LogReader::update_from_msg_gps(uint8_t gps_offset, class MsgParser *p, uint8_t *msg, bool responsible_for_relalt)
{
    uint32_t timestamp;
    require_field(p, msg, "T", timestamp);
    wait_timestamp(timestamp);

    Location loc;
    location_from_msg(p, msg, loc, "Lat", "Lng", "Alt");
    Vector3f vel;
    ground_vel_from_msg(p, msg, vel, "Spd", "GCrs", "VZ");

    uint8_t status = require_field_uint8_t(p, msg, "Status");
    gps.setHIL(gps_offset,
               (AP_GPS::GPS_Status)status,
               timestamp,
               loc,
               vel,
               require_field_uint8_t(p, msg, "NSats"),
               require_field_uint8_t(p, msg, "HDop"),
               require_field_float(p, msg, "VZ") != 0);
    if (status == AP_GPS::GPS_OK_FIX_3D && ground_alt_cm == 0) {
        ground_alt_cm = require_field_int32_t(p, msg, "Alt");
    }

    if (responsible_for_relalt) {
        // this could possibly check for the presence of "RelAlt" label?
        rel_altitude = 0.01f * require_field_int32_t(p, msg, "RelAlt");
    }

    dataflash.Log_Write_GPS(gps, gps_offset, rel_altitude);
}


void LogReader::update_from_msg_compass(uint8_t compass_offset, class MsgParser *p, uint8_t *msg)
{
    wait_timestamp_from_msg(p, msg);

    Vector3f mag;
    require_field(p, msg, "Mag", mag);
    Vector3f mag_offset;
    require_field(p, msg, "Ofs", mag_offset);

    compass.setHIL(mag - mag_offset);
    // compass_offset is which compass we are setting info for;
    // mag_offset is a vector indicating the compass' calibration...
    compass.set_offsets(compass_offset, mag_offset);

    dataflash.Log_Write_Compass(compass);
}

bool LogReader::update(uint8_t &type)
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
        type = f.type;

        msgparser[f.type] = new MsgParser(f);

	// ::printf("Defining log format for type (%d)\n", f.type);

        return true;
    }

    if (formats[hdr[2]].type == 0) {
        // no format message received for this type; we should
        // probably have a verbose option and warn here!
        return false;
    }
    const struct log_Format &f = formats[hdr[2]];
    
    uint8_t msg[f.length];
    memcpy(msg, hdr, 3);
    if (::read(fd, &msg[3], f.length-3) != f.length-3) {
        return false;
    }

    MsgParser *p = msgparser[f.type];

    switch (f.type) {
    case LOG_MESSAGE_MSG: {
        const uint8_t msg_text_len = 64;
        char msg_text[msg_text_len];
        require_field(p, msg, "Message", msg_text, msg_text_len);

        if (strncmp(msg_text, "ArduPlane", strlen("ArduPlane")) == 0) {
            vehicle = VEHICLE_PLANE;
            ::printf("Detected Plane\n");
            ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
            ahrs.set_fly_forward(true);
        } else if (strncmp(msg_text, "ArduCopter", strlen("ArduCopter")) == 0) {
            vehicle = VEHICLE_COPTER;
            ::printf("Detected Copter\n");
            ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);
            ahrs.set_fly_forward(false);
        } else if (strncmp(msg_text, "ArduRover", strlen("ArduRover")) == 0) {
            vehicle = VEHICLE_ROVER;
            ::printf("Detected Rover\n");
            ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);
            ahrs.set_fly_forward(true);
        }
        dataflash.Log_Write_Message(msg_text);
        break;
    }

    case LOG_IMU_MSG: {
        update_from_msg_imu(0, p, msg);
        break;
    }

    case LOG_IMU2_MSG: {
        update_from_msg_imu(1, p, msg);
        break;
    }

    case LOG_IMU3_MSG: {
        update_from_msg_imu(2, p, msg);
        break;
    }

    case LOG_GPS_MSG: {
        update_from_msg_gps(0,p, msg, true);
        break;
    }

    case LOG_GPS2_MSG: {
        // only LOG_GPS_MSG gives us relative altitude.  We still log
        // the relative altitude when we get a LOG_GPS2_MESSAGE - but
        // the value we use (probably) comes from the most recent
        // LOG_GPS_MESSAGE message!
        update_from_msg_gps(1, p, msg, false);
        break;
    }

    case LOG_SIMSTATE_MSG: {
        wait_timestamp_from_msg(p, msg);
        attitude_from_msg(p, msg, sim_attitude, "Roll", "Pitch", "Yaw");
        break;
    }

    case LOG_BARO_MSG: {
        wait_timestamp_from_msg(p, msg);
        baro.setHIL(0,
                    require_field_float(p, msg, "Press"),
                    require_field_int16_t(p, msg, "Temp") * 0.01f);
        dataflash.Log_Write_Baro(baro);
        break;
    }

    case LOG_PARAMETER_MSG: {
        const uint8_t parameter_name_len = AP_MAX_NAME_SIZE + 1; // null-term
        char parameter_name[parameter_name_len];

        require_field(p, msg, "Name", parameter_name, parameter_name_len);

        set_parameter(parameter_name, require_field_float(p, msg, "Value"));
        break;
    }

    case LOG_AHR2_MSG: {
        wait_timestamp_from_msg(p, msg);
        attitude_from_msg(p, msg, ahr2_attitude, "Roll", "Pitch", "Yaw");
        break;
    }

    case LOG_ATTITUDE_MSG: {
        wait_timestamp_from_msg(p, msg);
        attitude_from_msg(p, msg, attitude, "Roll", "Pitch", "Yaw");
        break;
    }

    case LOG_COMPASS_MSG: {
        update_from_msg_compass(0, p, msg);
        break;
    }

    default:
        if (vehicle == VEHICLE_PLANE) {
            update_plane(f.type, msg, f.length);
        } else if (vehicle == VEHICLE_COPTER) {
            update_copter(f.type, msg, f.length);
        } else if (vehicle == VEHICLE_ROVER) {
            update_rover(f.type, msg, f.length);
        }
        break;
    }

    type = f.type;

    return true;
}

void LogReader::wait_timestamp(uint32_t timestamp)
{
    uint64_t timestamp_usec = timestamp*1000UL;
    timestamp_usec = ((timestamp_usec + 1000) / 2500) * 2500;
    last_timestamp_usec = timestamp_usec;
    hal.scheduler->stop_clock(timestamp_usec);
}

bool LogReader::wait_type(uint8_t wtype)
{
    while (true) {
        uint8_t type;
        if (!update(type)) {
            return false;
        }
        if (wtype == type) {
            break;
        }
    }
    return true;
}

