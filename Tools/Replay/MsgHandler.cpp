#include <MsgHandler.h>

extern const AP_HAL::HAL& hal;

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

void MsgHandler::add_field_type(char type, size_t size)
{
    size_for_type_table[(type > 'A' ? (type-'A') : (type-'a'))] = size;
}

uint8_t MsgHandler::size_for_type(char type)
{
    return size_for_type_table[(uint8_t)(type > 'A' ? (type-'A') : (type-'a'))];
}

void MsgHandler::init_field_types()
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

struct MsgHandler::format_field_info *MsgHandler::find_field_info(const char *label)
{
    for(uint8_t i=0; i<next_field; i++) {
        if (streq(field_info[i].label, label)) {
            return &field_info[i];
        }
    }
    return NULL;
}

MsgHandler::MsgHandler(struct log_Format &_f, DataFlash_Class &_dataflash,
                       uint64_t &_last_timestamp_usec)
    : next_field(0), f(_f), dataflash(_dataflash), last_timestamp_usec(_last_timestamp_usec)
{
    init_field_types();
    parse_format_fields();
}


void MsgHandler::add_field(const char *_label, uint8_t _type, uint8_t _offset,
                          uint8_t _length)
{
    field_info[next_field].label = xstrdup(_label);
    field_info[next_field].type = _type;
    field_info[next_field].offset = _offset;
    field_info[next_field].length = _length;
    next_field++;
}

void MsgHandler::parse_format_fields()
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

bool MsgHandler::field_value(uint8_t *msg, const char *label, char *ret, uint8_t retlen)
{
    struct format_field_info *info = find_field_info(label);
    if (info == NULL) {
      ::printf("No info for (%s)\n",label);
      exit(1);
    }

    uint8_t offset = info->offset;
    if (offset == 0) {
        return false;
    }

    memset(ret, '\0', retlen);

    memcpy(ret, &msg[offset], (retlen < info->length) ? retlen : info->length);

    return true;
}


bool MsgHandler::field_value(uint8_t *msg, const char *label, Vector3f &ret)
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


void MsgHandler::string_for_labels(char *buffer, uint bufferlen)
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

MsgHandler::~MsgHandler()
{
    for (uint8_t k=0; k<LOGREADER_MAX_FIELDS; k++) {
        if (field_info[k].label != NULL) {
            free(field_info[k].label);
        }
    }
}

extern uint64_t last_timestamp_usec; // fixme!

void MsgHandler::wait_timestamp(uint32_t timestamp)
{
    uint64_t timestamp_usec = timestamp*1000UL;
    last_timestamp_usec = timestamp_usec;
    hal.scheduler->stop_clock(timestamp_usec);
}

void MsgHandler::location_from_msg(uint8_t *msg,
                                  Location &loc,
                                  const char *label_lat,
                                  const char *label_long,
                                  const char *label_alt)
{
    loc.lat = require_field_int32_t(msg, label_lat);
    loc.lng = require_field_int32_t(msg, label_long);
    loc.alt = require_field_int32_t(msg, label_alt);
    loc.options = 0;
}

void MsgHandler::ground_vel_from_msg(uint8_t *msg,
                                    Vector3f &vel,
                                    const char *label_speed,
                                    const char *label_course,
                                    const char *label_vz)
{
    uint32_t ground_speed;
    int32_t ground_course;
    require_field(msg, label_speed, ground_speed);
    require_field(msg, label_course, ground_course);
    vel[0] = ground_speed*0.01f*cosf(radians(ground_course*0.01f));
    vel[1] = ground_speed*0.01f*sinf(radians(ground_course*0.01f));
    vel[2] = require_field_float(msg, label_vz);
}

void MsgHandler::attitude_from_msg(uint8_t *msg,
				   Vector3f &att,
				   const char *label_roll,
				   const char *label_pitch,
				   const char *label_yaw)
{
    att[0] = require_field_int16_t(msg, label_roll) * 0.01f;
    att[1] = require_field_int16_t(msg, label_pitch) * 0.01f;
    att[2] = require_field_uint16_t(msg, label_yaw) * 0.01f;
}

void MsgHandler::require_field(uint8_t *msg, const char *label, char *buffer, uint8_t bufferlen)
{
    if (! field_value(msg, label, buffer, bufferlen)) {
        char all_labels[256];
        string_for_labels(all_labels, 256);
        ::printf("Field (%s) not found; options are (%s)\n", label, all_labels);
        exit(1);
    }
}

float MsgHandler::require_field_float(uint8_t *msg, const char *label)
{
    float ret;
    require_field(msg, label, ret);
    return ret;
}
uint8_t MsgHandler::require_field_uint8_t(uint8_t *msg, const char *label)
{
    uint8_t ret;
    require_field(msg, label, ret);
    return ret;
}
int32_t MsgHandler::require_field_int32_t(uint8_t *msg, const char *label)
{
    int32_t ret;
    require_field(msg, label, ret);
    return ret;
}
uint16_t MsgHandler::require_field_uint16_t(uint8_t *msg, const char *label)
{
    uint16_t ret;
    require_field(msg, label, ret);
    return ret;
}
int16_t MsgHandler::require_field_int16_t(uint8_t *msg, const char *label)
{
    int16_t ret;
    require_field(msg, label, ret);
    return ret;
}

void MsgHandler::wait_timestamp_from_msg(uint8_t *msg)
{
    uint32_t timestamp;
    require_field(msg, "TimeMS", timestamp);
    wait_timestamp(timestamp);
}



/*
 * subclasses to handle specific messages below here
*/

void MsgHandler_AHR2::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    attitude_from_msg(msg, ahr2_attitude, "Roll", "Pitch", "Yaw");
}


void MsgHandler_ARM::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    uint8_t ArmState = require_field_uint8_t(msg, "ArmState");
    hal.util->set_soft_armed(ArmState);
    printf("Armed state: %u at %lu\n", 
           (unsigned)ArmState,
           (unsigned long)hal.scheduler->millis());
    dataflash.WriteBlock(msg, f.length);
}


void MsgHandler_ARSP::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    airspeed.setHIL(require_field_float(msg, "Airspeed"),
		    require_field_float(msg, "DiffPress"),
		    require_field_float(msg, "Temp"));
    dataflash.WriteBlock(msg, f.length);
}


void MsgHandler_ATT::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    attitude_from_msg(msg, attitude, "Roll", "Pitch", "Yaw");
}


void MsgHandler_BARO::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    baro.setHIL(0,
		require_field_float(msg, "Press"),
		require_field_int16_t(msg, "Temp") * 0.01f);
    dataflash.WriteBlock(msg, f.length);
}


#define DATA_ARMED                          10
#define DATA_DISARMED                       11

void MsgHandler_Event::process_message(uint8_t *msg)
{
    uint8_t id = require_field_uint8_t(msg, "Id");
    if (id == DATA_ARMED) {
        hal.util->set_soft_armed(true);
        printf("Armed at %lu\n", 
               (unsigned long)hal.scheduler->millis());
    } else if (id == DATA_DISARMED) {
        hal.util->set_soft_armed(false);
        printf("Disarmed at %lu\n", 
               (unsigned long)hal.scheduler->millis());
    }
    dataflash.WriteBlock(msg, f.length);
}


void MsgHandler_GPS2::process_message(uint8_t *msg)
{
    // only LOG_GPS_MSG gives us relative altitude.  We still log
    // the relative altitude when we get a LOG_GPS2_MESSAGE - but
    // the value we use (probably) comes from the most recent
    // LOG_GPS_MESSAGE message!
    update_from_msg_gps(1, msg, false);
}


void MsgHandler_GPS_Base::update_from_msg_gps(uint8_t gps_offset, uint8_t *msg, bool responsible_for_relalt)
{
    uint32_t timestamp;
    require_field(msg, "T", timestamp);
    wait_timestamp(timestamp);

    Location loc;
    location_from_msg(msg, loc, "Lat", "Lng", "Alt");
    Vector3f vel;
    ground_vel_from_msg(msg, vel, "Spd", "GCrs", "VZ");

    uint8_t status = require_field_uint8_t(msg, "Status");
    gps.setHIL(gps_offset,
               (AP_GPS::GPS_Status)status,
               timestamp,
               loc,
               vel,
               require_field_uint8_t(msg, "NSats"),
               require_field_uint8_t(msg, "HDop"),
               require_field_float(msg, "VZ") != 0);
    if (status == AP_GPS::GPS_OK_FIX_3D && ground_alt_cm == 0) {
        ground_alt_cm = require_field_int32_t(msg, "Alt");
    }

    if (responsible_for_relalt) {
        // this could possibly check for the presence of "RelAlt" label?
        rel_altitude = 0.01f * require_field_int32_t(msg, "RelAlt");
    }

    dataflash.WriteBlock(msg, f.length);
}



void MsgHandler_GPS::process_message(uint8_t *msg)
{
    update_from_msg_gps(0, msg, true);
}


void MsgHandler_IMU2::process_message(uint8_t *msg)
{
  update_from_msg_imu(1, msg);
}


void MsgHandler_IMU3::process_message(uint8_t *msg)
{
  update_from_msg_imu(2, msg);
}


void MsgHandler_IMU_Base::update_from_msg_imu(uint8_t imu_offset, uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    uint8_t this_imu_mask = 1 << imu_offset;

    if (gyro_mask & this_imu_mask) {
        Vector3f gyro;
        require_field(msg, "Gyr", gyro);
        ins.set_gyro(imu_offset, gyro);
    }
    if (accel_mask & this_imu_mask) {
        Vector3f accel2;
        require_field(msg, "Acc", accel2);
        ins.set_accel(imu_offset, accel2);
    }

    dataflash.WriteBlock(msg, f.length);
}


void MsgHandler_IMU::process_message(uint8_t *msg)
{
    update_from_msg_imu(0, msg);
}


void MsgHandler_MAG2::process_message(uint8_t *msg)
{
    update_from_msg_compass(1, msg);
}


void MsgHandler_MAG_Base::update_from_msg_compass(uint8_t compass_offset, uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    Vector3f mag;
    require_field(msg, "Mag", mag);
    Vector3f mag_offset;
    require_field(msg, "Ofs", mag_offset);

    compass.setHIL(compass_offset, mag - mag_offset);
    // compass_offset is which compass we are setting info for;
    // mag_offset is a vector indicating the compass' calibration...
    compass.set_offsets(compass_offset, mag_offset);

    dataflash.WriteBlock(msg, f.length);
}



void MsgHandler_MAG::process_message(uint8_t *msg)
{
    update_from_msg_compass(0, msg);
}

#include <AP_AHRS.h>
#include <VehicleType.h>

void MsgHandler_MSG::process_message(uint8_t *msg)
{
    const uint8_t msg_text_len = 64;
    char msg_text[msg_text_len];
    require_field(msg, "Message", msg_text, msg_text_len);

    if (strncmp(msg_text, "ArduPlane", strlen("ArduPlane")) == 0) {
	vehicle = VehicleType::VEHICLE_PLANE;
	::printf("Detected Plane\n");
	ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
	ahrs.set_fly_forward(true);
    } else if (strncmp(msg_text, "ArduCopter", strlen("ArduCopter")) == 0 ||
	       strncmp(msg_text, "APM:Copter", strlen("APM:Copter")) == 0) {
	vehicle = VehicleType::VEHICLE_COPTER;
	::printf("Detected Copter\n");
	ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);
	ahrs.set_fly_forward(false);
    } else if (strncmp(msg_text, "ArduRover", strlen("ArduRover")) == 0) {
	vehicle = VehicleType::VEHICLE_ROVER;
	::printf("Detected Rover\n");
	ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);
	ahrs.set_fly_forward(true);
    }
    dataflash.Log_Write_Message(msg_text);
}


void MsgHandler_NTUN_Copter::process_message(uint8_t *msg)
{
    inavpos = Vector3f(require_field_float(msg, "PosX") * 0.01f,
		       require_field_float(msg, "PosY") * 0.01f,
		       0);
}


bool MsgHandler::set_parameter(const char *name, float value)
{
    const char *ignore_parms[] = { "GPS_TYPE", "AHRS_EKF_USE", 
                                   "COMPASS_ORIENT", "COMPASS_ORIENT2",
                                   "COMPASS_ORIENT3"};
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

void MsgHandler_PARM::process_message(uint8_t *msg)
{
    const uint8_t parameter_name_len = AP_MAX_NAME_SIZE + 1; // null-term
    char parameter_name[parameter_name_len];

    require_field(msg, "Name", parameter_name, parameter_name_len);

    set_parameter(parameter_name, require_field_float(msg, "Value"));
}


void MsgHandler_SIM::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    attitude_from_msg(msg, sim_attitude, "Roll", "Pitch", "Yaw");
}

