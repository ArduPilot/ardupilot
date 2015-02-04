
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
    LogReader(AP_AHRS &_ahrs, AP_InertialSensor &_ins, AP_Baro &_baro, AP_Compass_HIL &_compass, AP_GPS &_gps, AP_Airspeed &_airspeed, DataFlash_Class &_dataflash);
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

private:
    int fd;
    AP_AHRS &ahrs;
    AP_InertialSensor &ins;
    AP_Baro &baro;
    AP_Compass_HIL &compass;
    AP_GPS &gps;
    AP_Airspeed &airspeed;
    DataFlash_Class &dataflash;

    uint8_t accel_mask;
    uint8_t gyro_mask;

    uint32_t ground_alt_cm;

#define LOGREADER_MAX_FORMATS 100
    uint8_t num_formats;
    struct log_Format formats[LOGREADER_MAX_FORMATS];

    Vector3f attitude;
    Vector3f ahr2_attitude;
    Vector3f sim_attitude;
    Vector3f inavpos;
    float rel_altitude;

    void wait_timestamp(uint32_t timestamp);

    void process_plane(uint8_t type, uint8_t *data, uint16_t length);
    void process_copter(uint8_t type, uint8_t *data, uint16_t length);
    void process_rover(uint8_t type, uint8_t *data, uint16_t length);
};
