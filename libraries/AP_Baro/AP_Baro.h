#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <Filter/Filter.h>
#include <Filter/DerivativeFilter.h>

// maximum number of sensor instances
#define BARO_MAX_INSTANCES 3

// maximum number of drivers. Note that a single driver can provide
// multiple sensor instances
#define BARO_MAX_DRIVERS 3

// timeouts for health reporting
#define BARO_TIMEOUT_MS                 500     // timeout in ms since last successful read
#define BARO_DATA_CHANGE_TIMEOUT_MS     2000    // timeout in ms since last successful read that involved temperature of pressure changing

class AP_Baro_Backend;

class AP_Baro
{
    friend class AP_Baro_Backend;
    friend class AP_Baro_SITL; // for access to sensors[]

public:
    // constructor
    AP_Baro();

    // barometer types
    typedef enum {
        BARO_TYPE_AIR,
        BARO_TYPE_WATER
    } baro_type_t;

    // initialise the barometer object, loading backend drivers
    void init(void);

    // update the barometer object, asking backends to push data to
    // the frontend
    void update(void);

    // healthy - returns true if sensor and derived altitude are good
    bool healthy(void) const { return healthy(_primary); }
    bool healthy(uint8_t instance) const { return sensors[instance].healthy && sensors[instance].alt_ok && sensors[instance].calibrated; }

    // check if all baros are healthy - used for SYS_STATUS report
    bool all_healthy(void) const;

    // pressure in Pascal. Divide by 100 for millibars or hectopascals
    float get_pressure(void) const { return get_pressure(_primary); }
    float get_pressure(uint8_t instance) const { return sensors[instance].pressure; }

    // temperature in degrees C
    float get_temperature(void) const { return get_temperature(_primary); }
    float get_temperature(uint8_t instance) const { return sensors[instance].temperature; }

    // accumulate a reading on sensors. Some backends without their
    // own thread or a timer may need this.
    void accumulate(void);

    // calibrate the barometer. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void calibrate(bool save=true);

    // update the barometer calibration to the current pressure. Can
    // be used for incremental preflight update of baro
    void update_calibration(void);

    // get current altitude in meters relative to altitude at the time
    // of the last calibrate() call
    float get_altitude(void) const { return get_altitude(_primary); }
    float get_altitude(uint8_t instance) const { return sensors[instance].altitude; }

    // get altitude difference in meters relative given a base
    // pressure in Pascal
    float get_altitude_difference(float base_pressure, float pressure) const;

    // get scale factor required to convert equivalent to true airspeed
    float get_EAS2TAS(void);

    // get air density / sea level density - decreases as altitude climbs
    float get_air_density_ratio(void);

    // get current climb rate in meters/s. A positive number means
    // going up
    float get_climb_rate(void);

    // ground temperature in degrees C
    // the ground values are only valid after calibration
    float get_ground_temperature(void) const;

    // ground pressure in Pascal
    // the ground values are only valid after calibration
    float get_ground_pressure(void) const { return get_ground_pressure(_primary); }
    float get_ground_pressure(uint8_t i)  const { return sensors[i].ground_pressure.get(); }

    // set the temperature to be used for altitude calibration. This
    // allows an external temperature source (such as a digital
    // airspeed sensor) to be used as the temperature source
    void set_external_temperature(float temperature);

    // get last time sample was taken (in ms)
    uint32_t get_last_update(void) const { return get_last_update(_primary); }
    uint32_t get_last_update(uint8_t instance) const { return sensors[instance].last_update_ms; }

    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    float get_external_temperature(void) const { return get_external_temperature(_primary); };
    float get_external_temperature(const uint8_t instance) const;

    // HIL (and SITL) interface, setting altitude
    void setHIL(float altitude_msl);

    // HIL (and SITL) interface, setting pressure, temperature, altitude and climb_rate
    // used by Replay
    void setHIL(uint8_t instance, float pressure, float temperature, float altitude, float climb_rate, uint32_t last_update_ms);

    // Set the primary baro
    void set_primary_baro(uint8_t primary) { _primary_baro.set_and_save(primary); };

    // Set the type (Air or Water) of a particular instance
    void set_type(uint8_t instance, baro_type_t type) { sensors[instance].type = type; };

    // Get the type (Air or Water) of a particular instance
    baro_type_t get_type(uint8_t instance) { return sensors[instance].type; };

    // HIL variables
    struct {
        float pressure;
        float temperature;
        float altitude;
        float climb_rate;
        uint32_t last_update_ms;
        bool updated:1;
        bool have_alt:1;
        bool have_last_update:1;
    } _hil;

    // register a new sensor, claiming a sensor slot. If we are out of
    // slots it will panic
    uint8_t register_sensor(void);

    // return number of registered sensors
    uint8_t num_instances(void) const { return _num_sensors; }

    // enable HIL mode
    void set_hil_mode(void) { _hil_mode = true; }

    // set baro drift amount
    void set_baro_drift_altitude(float alt) { _alt_offset = alt; }

    // get baro drift amount
    float get_baro_drift_offset(void) { return _alt_offset_active; }

    // simple atmospheric model
    static void SimpleAtmosphere(const float alt, float &sigma, float &delta, float &theta);

    // set a pressure correction from AP_TempCalibration
    void set_pressure_correction(uint8_t instance, float p_correction);
    
private:
    // how many drivers do we have?
    uint8_t _num_drivers;
    AP_Baro_Backend *drivers[BARO_MAX_DRIVERS];

    // how many sensors do we have?
    uint8_t _num_sensors;

    // what is the primary sensor at the moment?
    uint8_t _primary;

    struct sensor {
        baro_type_t type;                   // 0 for air pressure (default), 1 for water pressure
        uint32_t last_update_ms;        // last update time in ms
        uint32_t last_change_ms;        // last update time in ms that included a change in reading from previous readings
        bool healthy:1;                 // true if sensor is healthy
        bool alt_ok:1;                  // true if calculated altitude is ok
        bool calibrated:1;              // true if calculated calibrated successfully
        float pressure;                 // pressure in Pascal
        float temperature;              // temperature in degrees C
        float altitude;                 // calculated altitude
        AP_Float ground_pressure;
        float p_correction;
    } sensors[BARO_MAX_INSTANCES];

    AP_Float                            _alt_offset;
    float                               _alt_offset_active;
    AP_Int8                             _primary_baro; // primary chosen by user
    AP_Int8                             _ext_bus; // bus number for external barometer
    float                               _last_altitude_EAS2TAS;
    float                               _EAS2TAS;
    float                               _external_temperature;
    uint32_t                            _last_external_temperature_ms;
    DerivativeFilterFloat_Size7         _climb_rate_filter;
    AP_Float                            _specific_gravity; // the specific gravity of fluid for an ROV 1.00 for freshwater, 1.024 for salt water
    AP_Float                            _user_ground_temperature; // user override of the ground temperature used for EAS2TAS
    bool                                _hil_mode:1;
    float                               _guessed_ground_temperature; // currently ground temperature estimate using our best abailable source

    // when did we last notify the GCS of new pressure reference?
    uint32_t                            _last_notify_ms;

    bool _add_backend(AP_Baro_Backend *backend);
};
