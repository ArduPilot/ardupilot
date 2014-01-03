
enum log_messages {
    LOG_NTUN_MSG     = 2,
    LOG_ATTITUDE_MSG = 10,
    LOG_COMPASS_MSG  = 12
};


class LogReader 
{
public:
    LogReader(AP_InertialSensor &_ins, AP_Baro_HIL &_baro, AP_Compass_HIL &_compass, GPS *&_gps);
    bool open_log(const char *logfile);
    bool update(uint8_t &type);
    bool wait_type(uint8_t type);
    const Vector3f &get_attitude(void) const { return attitude; }
    const Vector3f &get_sim_attitude(void) const { return sim_attitude; }

private:
    int fd;
    AP_InertialSensor &ins;
    AP_Baro_HIL &baro;
    AP_Compass_HIL &compass;
    GPS *&gps;

    uint32_t ground_alt_cm;

    uint8_t num_formats;
    struct log_Format formats[32];

    Vector3f attitude;
    Vector3f sim_attitude;

    void wait_timestamp(uint32_t timestamp);
};
