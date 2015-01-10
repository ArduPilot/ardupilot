
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
#include "../AP_Compass/AP_Compass.h"
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
    static uint16_t pwm_output[11];
    static uint16_t last_pwm_output[11];
    static uint16_t pwm_input[8];
    static bool new_rc_input;
    static void loop_hook(void);
    uint16_t base_port(void) const { return _base_port; }

    // simulated airspeed, sonar and battery monitor
    static uint16_t sonar_pin_value;    // pin 0
    static uint16_t airspeed_pin_value; // pin 1
    static uint16_t voltage_pin_value;  // pin 13
    static uint16_t current_pin_value;  // pin 12

private:
    void _parse_command_line(int argc, char * const argv[]);
    void _set_param_default(char *parm);
    void _usage(void);
    void _sitl_setup(void);
    void _setup_fdm(void);
    void _setup_timer(void);
    void _setup_adc(void);

    // these methods are static as they are called
    // from the timer
    static float height_agl(void);
    static void _update_barometer(float height);
    static void _update_compass(float rollDeg, float pitchDeg, float yawDeg);
    static void _update_flow(void);

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
    static gps_data _gps_data[MAX_GPS_DELAY];

    static bool _gps_has_basestation_position;
    static gps_data _gps_basestation_data;
    static void _gps_write(const uint8_t *p, uint16_t size);
    static void _gps_send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size);
    static void _update_gps_ubx(const struct gps_data *d);
    static void _update_gps_mtk(const struct gps_data *d);
    static void _update_gps_mtk16(const struct gps_data *d);
    static void _update_gps_mtk19(const struct gps_data *d);
    static uint16_t _gps_nmea_checksum(const char *s);
    static void _gps_nmea_printf(const char *fmt, ...);
    static void _update_gps_nmea(const struct gps_data *d);
    static void _sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload);
    static void _update_gps_sbp(const struct gps_data *d, bool sim_rtk);

    static void _update_gps(double latitude, double longitude, float altitude,
			    double speedN, double speedE, double speedD, bool have_lock);

    static void _update_ins(float roll, 	float pitch, 	float yaw,		// Relative to earth
			    double rollRate, 	double pitchRate,double yawRate,	// Local to plane
			    double xAccel, 	double yAccel, 	double zAccel,		// Local to plane
			    float airspeed,	float altitude);
    static void _fdm_input(void);
    static void _simulator_output(void);
    static void _apply_servo_filter(float deltat);
    static uint16_t _airspeed_sensor(float airspeed);
    static uint16_t _ground_sonar();
    static float _gyro_drift(void);
    static float _rand_float(void);
    static Vector3f _rand_vec3f(void);

    // signal handlers
    static void _sig_fpe(int signum);
    static void _timer_handler(int signum);

    // internal state
    static enum vehicle_type _vehicle;
    static uint16_t _framerate;
    static uint16_t _base_port;
    float _initial_height;
    static struct sockaddr_in _rcout_addr;
    static pid_t _parent_pid;
    static uint32_t _update_count;
    static bool _motors_on;

    static AP_Baro *_barometer;
    static AP_InertialSensor *_ins;
    static SITLScheduler *_scheduler;
    static AP_Compass_HIL *_compass;
    static OpticalFlow *_optical_flow;
    static AP_Terrain *_terrain;

    static int _sitl_fd;
    static SITL *_sitl;
    static uint16_t _rcout_port;
    static uint16_t _simin_port;
    static float _current;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#endif // __AP_HAL_AVR_SITL_STATE_H__

