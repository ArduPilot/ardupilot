#include <VehicleType.h>

class LogReader
{
public:
    LogReader(AP_AHRS &_ahrs, AP_InertialSensor &_ins, AP_Baro &_baro, Compass &_compass, AP_GPS &_gps, AP_Airspeed &_airspeed, DataFlash_Class &_dataflash);
    bool open_log(const char *logfile);
    bool update(char type[5]);
    bool wait_type(const char *type);

    const Vector3f &get_attitude(void) const { return attitude; }
    const Vector3f &get_ahr2_attitude(void) const { return ahr2_attitude; }
    const Vector3f &get_inavpos(void) const { return inavpos; }
    const Vector3f &get_sim_attitude(void) const { return sim_attitude; }
    const float &get_relalt(void) const { return rel_altitude; }

    VehicleType::vehicle_type vehicle;

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

    void update_from_msg_compass(uint8_t gps_offset, class MsgHandler *p, uint8_t *msg);
    void wait_timestamp_from_msg(class MsgHandler *p, uint8_t *data);

    uint8_t accel_mask;
    uint8_t gyro_mask;

    uint32_t ground_alt_cm;

#define LOGREADER_MAX_FORMATS 255 // must be >= highest MESSAGE
    struct log_Format formats[LOGREADER_MAX_FORMATS];
    class MsgHandler *msgparser[LOGREADER_MAX_FORMATS];

    template <typename R>
    void require_field(class MsgHandler *p, uint8_t *msg, const char *label, R &ret);
    void require_field(class MsgHandler *p, uint8_t *data, const char *label, char *buffer, uint8_t bufferlen);

    // convenience wrappers around require_field
    uint16_t require_field_uint16_t(class MsgHandler *p, uint8_t *data, const char *label);
    int16_t require_field_int16_t(class MsgHandler *p, uint8_t *data, const char *label);
    uint8_t require_field_uint8_t(class MsgHandler *p, uint8_t *data, const char *label);
    int32_t require_field_int32_t(class MsgHandler *p, uint8_t *data, const char *label);
    float require_field_float(class MsgHandler *p, uint8_t *data, const char *label);
    uint8_t require_field_uint8_t(uint8_t *msg, const char *label);

    void location_from_msg(class MsgHandler *p, uint8_t *data,
                           Location &loc,
                           const char *label_lat,
                           const char *label_long,
                           const char *label_alt);
    void ground_vel_from_msg(class MsgHandler *p, uint8_t *data,
                             Vector3f &vel,
                             const char *label_speed,
                             const char *label_course,
                             const char *label_vz);
    void attitude_from_msg(class MsgHandler *p, uint8_t *data,
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

    void update_rover(uint8_t type, uint8_t *data, uint16_t length);

    bool installed_vehicle_specific_parsers;
    void maybe_install_vehicle_specific_parsers();

    bool in_list(const char *type, const char *list[]);
};
