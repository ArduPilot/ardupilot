
#ifndef __AP_HAL_AVR_SITL_STATE_H__
#define __AP_HAL_AVR_SITL_STATE_H__

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>

#include "../AP_Baro/AP_Baro.h"
#include "../AP_InertialSensor/AP_InertialSensor.h"
#include "../AP_Compass/Compass.h"
#include "../AP_OpticalFlow/AP_OpticalFlow.h"
#include "../AP_Terrain/AP_Terrain.h"
#include "../SITL/SITL.h"

class HAL_AVR_SITL;

class AVR_SITL::SITL_State {
    friend class AVR_SITL::SITLScheduler;
public:
    void init(int argc, char * const argv[]);

    enum vehicle_type {
	    ArduCopter,
	    APMrover2,
	    ArduPlane
    };

    int gps_pipe(void);
    int gps2_pipe(void);
    ssize_t gps_read(int fd, void *buf, size_t count);
    uint16_t pwm_output[11];
    uint16_t last_pwm_output[11];
    uint16_t pwm_input[8];
    bool new_rc_input;
    void loop_hook(void);
    uint16_t base_port(void) const { return _base_port; }

    // simulated airspeed, sonar and battery monitor
    uint16_t sonar_pin_value;    // pin 0
    uint16_t airspeed_pin_value; // pin 1
    uint16_t voltage_pin_value;  // pin 13
    uint16_t current_pin_value;  // pin 12

private:
    void _parse_command_line(int argc, char * const argv[]);
    void _set_param_default(char *parm);
    void _usage(void);
    void _sitl_setup(void);
    void _setup_fdm(void);
    void _setup_timer(void);
    void _setup_adc(void);

    float height_agl(void);
    void _update_barometer(float height);
    void _update_compass(float rollDeg, float pitchDeg, float yawDeg);
    void _update_flow(void);

    struct gps_data {
	    double latitude;
	    double longitude;
	    float altitude;
	    double speedN;
	    double speedE;
	    double speedD;
	    bool have_lock;
    };

    #define MAX_GPS_DELAY 100
    gps_data _gps_data[MAX_GPS_DELAY];

    bool _gps_has_basestation_position;
    gps_data _gps_basestation_data;
    void _gps_write(const uint8_t *p, uint16_t size);
    void _gps_send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size);
    void _update_gps_ubx(const struct gps_data *d);
    void _update_gps_mtk(const struct gps_data *d);
    void _update_gps_mtk16(const struct gps_data *d);
    void _update_gps_mtk19(const struct gps_data *d);
    uint16_t _gps_nmea_checksum(const char *s);
    void _gps_nmea_printf(const char *fmt, ...);
    void _update_gps_nmea(const struct gps_data *d);
    void _sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload);
    void _update_gps_sbp(const struct gps_data *d, bool sim_rtk);

    void _update_gps(double latitude, double longitude, float altitude,
			    double speedN, double speedE, double speedD, bool have_lock);

    void _update_ins(float roll, 	float pitch, 	float yaw,		// Relative to earth
			    double rollRate, 	double pitchRate,double yawRate,	// Local to plane
			    double xAccel, 	double yAccel, 	double zAccel,		// Local to plane
			    float airspeed,	float altitude);
    void _fdm_input(void);
    void _simulator_output(bool synthetic_clock_mode);
    void _apply_servo_filter(float deltat);
    uint16_t _airspeed_sensor(float airspeed);
    uint16_t _ground_sonar();
    float _gyro_drift(void);
    float _rand_float(void);
    Vector3f _rand_vec3f(void);

    pthread_t _fdm_thread_ctx;
    void _fdm_thread(void);
    int _fdm_pipe[2];
    uint64_t next_stop_clock;

    // internal state
    enum vehicle_type _vehicle;
    uint16_t _framerate;
    uint16_t _base_port;
    float _initial_height;
    struct sockaddr_in _rcout_addr;
    pid_t _parent_pid;
    uint32_t _update_count;
    bool _motors_on;

    AP_Baro *_barometer;
    AP_InertialSensor *_ins;
    SITLScheduler *_scheduler;
    Compass *_compass;
    OpticalFlow *_optical_flow;
    AP_Terrain *_terrain;

    int _sitl_fd;
    SITL *_sitl;
    uint16_t _rcout_port;
    uint16_t _simin_port;
    float _current;

    bool _synthetic_clock_mode;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#endif // __AP_HAL_AVR_SITL_STATE_H__

