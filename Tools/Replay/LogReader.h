
enum log_messages {
    // plane specific messages
    LOG_PLANE_ATTITUDE_MSG = 9,
    LOG_PLANE_COMPASS_MSG  = 11,
    LOG_PLANE_COMPASS2_MSG  = 15,
    LOG_PLANE_AIRSPEED_MSG  = 17,

    // copter specific messages
    LOG_COPTER_ATTITUDE_MSG = 1,
    LOG_COPTER_COMPASS_MSG  = 15,
    LOG_COPTER_NAV_TUNING_MSG = 5,

    // rover specific messages
    LOG_ROVER_ATTITUDE_MSG = 8,
    LOG_ROVER_COMPASS_MSG  = 10,
};


class LogReader 
{
public:
    LogReader(AP_AHRS &_ahrs, AP_InertialSensor &_ins, AP_Baro &_baro, Compass &_compass, AP_GPS &_gps, AP_Airspeed &_airspeed, DataFlash_Class &_dataflash);
    bool open_log(const char *logfile);
    bool update(uint8_t &type);
    bool wait_type(uint8_t type);

    const Vector3f &get_attitude(void) const { return attitude; }
    const Vector3f &get_ahr2_attitude(void) const { return ahr2_attitude; }
    const Vector3f &get_inavpos(void) const { return inavpos; }
    const Vector3f &get_sim_attitude(void) const { return sim_attitude; }
    const float &get_relalt(void) const { return rel_altitude; }

    enum vehicle_type { VEHICLE_UNKNOWN, VEHICLE_COPTER, VEHICLE_PLANE, VEHICLE_ROVER };

    vehicle_type vehicle;

    bool set_parameter(const char *name, float value);

    void set_accel_mask(uint8_t mask) { accel_mask = mask; }
    void set_gyro_mask(uint8_t mask) { gyro_mask = mask; }

    uint64_t last_timestamp_us(void) const { return last_timestamp_usec; }

private:
    int fd;
    AP_AHRS &ahrs;
    AP_InertialSensor &ins;
    AP_Baro &baro;
    Compass &compass;
    AP_GPS &gps;
    AP_Airspeed &airspeed;
    DataFlash_Class &dataflash;

    void update_from_msg_imu(uint8_t imu_offset, class MsgParser *p, uint8_t *data);
    void update_from_msg_gps(uint8_t imu_offset, class MsgParser *p, uint8_t *data, bool responsible_for_relalt);
    void update_from_msg_compass(uint8_t gps_offset, class MsgParser *p, uint8_t *msg);
    void wait_timestamp_from_msg(class MsgParser *p, uint8_t *data);

    uint8_t accel_mask;
    uint8_t gyro_mask;

    uint32_t ground_alt_cm;

#define LOGREADER_MAX_FORMATS 255 // must be >= highest MESSAGE
    struct log_Format formats[LOGREADER_MAX_FORMATS];
    class MsgParser *msgparser[LOGREADER_MAX_FORMATS];

    void force_add_format(struct log_Format *formats, MsgParser **parsers,
			  uint8_t type, const char *format, const char *labels);

    // support for the old PLANE formats
    void init_old_parsers_plane();
#define LOGREADER_MAX_FORMATS_PLANE 20
    struct log_Format formats_plane[LOGREADER_MAX_FORMATS_PLANE];
    class MsgParser *msgparser_plane[LOGREADER_MAX_FORMATS_PLANE];
    // support for the old ROVER formats
    void init_old_parsers_rover();
#define LOGREADER_MAX_FORMATS_ROVER 20
    struct log_Format formats_rover[LOGREADER_MAX_FORMATS_ROVER];
    class MsgParser *msgparser_rover[LOGREADER_MAX_FORMATS_ROVER];
    // support for the old COPTER formats
    void init_old_parsers_copter();
#define LOGREADER_MAX_FORMATS_COPTER 20
    struct log_Format formats_copter[LOGREADER_MAX_FORMATS_COPTER];
    class MsgParser *msgparser_copter[LOGREADER_MAX_FORMATS_COPTER];

    template <typename R>
      void require_field(class MsgParser *p, uint8_t *msg, const char *label, R &ret);
    void require_field(class MsgParser *p, uint8_t *data, const char *label, char *buffer, uint8_t bufferlen);

    // convenience wrappers around require_field
    uint16_t require_field_uint16_t(class MsgParser *p, uint8_t *data, const char *label);
    int16_t require_field_int16_t(class MsgParser *p, uint8_t *data, const char *label);
    uint8_t require_field_uint8_t(class MsgParser *p, uint8_t *data, const char *label);
    int32_t require_field_int32_t(class MsgParser *p, uint8_t *data, const char *label);
    float require_field_float(class MsgParser *p, uint8_t *data, const char *label);
    void location_from_msg(class MsgParser *p, uint8_t *data,
                           Location &loc,
                           const char *label_lat,
                           const char *label_long,
                           const char *label_alt);
    void ground_vel_from_msg(class MsgParser *p, uint8_t *data,
                             Vector3f &vel,
                             const char *label_speed,
                             const char *label_course,
                             const char *label_vz);
    void attitude_from_msg(class MsgParser *p, uint8_t *data,
                           Vector3f &att,
                           const char *label_roll,
                           const char *label_pitch,
                           const char *label_yaw);
    // end convenience wrappers

    Vector3f attitude;
    Vector3f ahr2_attitude;
    Vector3f sim_attitude;
    Vector3f inavpos;
    float rel_altitude;
    uint64_t last_timestamp_usec;

    void wait_timestamp(uint32_t timestamp);

    void update_plane(uint8_t type, uint8_t *data, uint16_t length);
    void update_copter(uint8_t type, uint8_t *data, uint16_t length);
    void update_rover(uint8_t type, uint8_t *data, uint16_t length);
};
