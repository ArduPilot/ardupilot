#pragma once

#include <AP_GPS/AP_GPS.h>

#include <AP_Logger/LogStructure.h>

class AP_DAL_GPS {
public:

    /// GPS status codes
    enum GPS_Status : uint8_t {
        NO_GPS = 0,                     ///< No GPS connected/detected
        NO_FIX = 1,                     ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = 2,              ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = 3,              ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS = 4,           ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK_FLOAT = 5, ///< Receiving valid messages and 3D RTK Float
        GPS_OK_FIX_3D_RTK_FIXED = 6, ///< Receiving valid messages and 3D RTK Fixed
    };

    AP_DAL_GPS();

    GPS_Status status(uint8_t sensor_id) const {
        return (GPS_Status)_RGPI[sensor_id].status;
    }
    GPS_Status status() const {
        return status(primary_sensor());
    }
    const Location &location(uint8_t instance) const {
        return tmp_location[instance];
    }
    bool have_vertical_velocity(uint8_t instance) const {
        return _RGPI[instance].have_vertical_velocity;
    }
    bool have_vertical_velocity() const {
        return have_vertical_velocity(primary_sensor());
    }
    bool horizontal_accuracy(uint8_t instance, float &hacc) const {
        hacc = _RGPJ[instance].hacc;
        return _RGPI[instance].horizontal_accuracy_returncode;
    }
    bool horizontal_accuracy(float &hacc) const {
        return horizontal_accuracy(primary_sensor(),hacc);
    }

    bool vertical_accuracy(uint8_t instance, float &vacc) const {
        vacc = _RGPJ[instance].vacc;
        return _RGPI[instance].vertical_accuracy_returncode;
    }
    bool vertical_accuracy(float &vacc) const {
        return vertical_accuracy(primary_sensor(), vacc);
    }

    uint16_t get_hdop(uint8_t instance) const {
        return _RGPJ[instance].hdop;
    }
    uint16_t get_hdop() const {
        return get_hdop(primary_sensor());
    }

    uint32_t last_message_time_ms(uint8_t instance) const {
        return _RGPJ[instance].last_message_time_ms;
    }

    uint8_t num_sats(uint8_t instance) const {
        return _RGPI[instance].num_sats;
    }
    uint8_t num_sats() const {
        return num_sats(primary_sensor());
    }

    bool get_lag(uint8_t instance, float &lag_sec) const {
        lag_sec = _RGPI[instance].lag_sec;
        return _RGPI[instance].get_lag_returncode;
    }
    bool get_lag(float &lag_sec) const {
        return get_lag(primary_sensor(), lag_sec);
    }

    const Vector3f &velocity(uint8_t instance) const {
        return _RGPJ[instance].velocity;
    }
    const Vector3f &velocity() const {
        return velocity(primary_sensor());
    }

    bool speed_accuracy(uint8_t instance, float &sacc) const {
        sacc = _RGPJ[instance].sacc;
        return _RGPI[instance].speed_accuracy_returncode;
    }
    bool speed_accuracy(float &sacc) const {
        return speed_accuracy(primary_sensor(), sacc);
    }

    bool gps_yaw_deg(uint8_t instance, float &yaw_deg, float &accuracy_deg, uint32_t &time_ms) const {
        yaw_deg = _RGPJ[instance].yaw_deg;
        accuracy_deg = _RGPJ[instance].yaw_accuracy_deg;
        time_ms = _RGPJ[instance].yaw_deg_time_ms;
        return _RGPI[instance].gps_yaw_deg_returncode;
    }

    uint8_t num_sensors(void) const {
        return _RGPH.num_sensors;
    }

    uint8_t primary_sensor(void) const {
        return _RGPH.primary_sensor;
    }

    // TODO: decide if this really, really should be here!
    const Location &location() const {
        return location(_RGPH.primary_sensor);
    }

    // return a 3D vector defining the offset of the GPS antenna in meters relative to the body frame origin
    const Vector3f &get_antenna_offset(uint8_t instance) const {
        return _RGPI[instance].antenna_offset;
    }

    void start_frame();

    void handle_message(const log_RGPH &msg) {
        _RGPH = msg;
    }
    void handle_message(const log_RGPI &msg) {
        _RGPI[msg.instance] = msg;
    }
    void handle_message(const log_RGPJ &msg) {
        _RGPJ[msg.instance] = msg;

        tmp_location[msg.instance].lat = msg.lat;
        tmp_location[msg.instance].lng = msg.lng;
        tmp_location[msg.instance].alt = msg.alt;
    }

private:

    struct log_RGPH _RGPH;
    struct log_RGPI _RGPI[GPS_MAX_INSTANCES];
    struct log_RGPJ _RGPJ[GPS_MAX_INSTANCES];

    Location tmp_location[GPS_MAX_INSTANCES];
};
