/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"
#include "UARTDriver.h"
#include "Scheduler.h"

#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/select.h>

#include <AP_Param.h>

extern const AP_HAL::HAL& hal;

using namespace AVR_SITL;

enum SITL_State::vehicle_type SITL_State::_vehicle;
uint16_t SITL_State::_framerate;
struct sockaddr_in SITL_State::_rcout_addr;
pid_t SITL_State::_parent_pid;
uint32_t SITL_State::_update_count;
bool SITL_State::_motors_on;
uint16_t SITL_State::airspeed_pin_value;

AP_Baro_BMP085_HIL *SITL_State::_barometer;
AP_InertialSensor_Stub *SITL_State::_ins;
SITLScheduler *SITL_State::_scheduler;
AP_Compass_HIL *SITL_State::_compass;

int SITL_State::_sitl_fd;
SITL *SITL_State::_sitl;
uint16_t SITL_State::pwm_output[11];
uint16_t SITL_State::pwm_input[8];
bool SITL_State::pwm_valid;

// catch floating point exceptions
void SITL_State::_sig_fpe(int signum)
{
	fprintf(stderr, "ERROR: Floating point exception\n");
	exit(1);
}

void SITL_State::_usage(void)
{
	fprintf(stdout, "Options:\n");
	fprintf(stdout, "\t-w          wipe eeprom and dataflash\n");
	fprintf(stdout, "\t-r RATE     set SITL framerate\n");
	fprintf(stdout, "\t-H HEIGHT   initial barometric height\n");
	fprintf(stdout, "\t-C          use console instead of TCP ports\n");
}

void SITL_State::_parse_command_line(int argc, char * const argv[])
{
	int opt;

	signal(SIGFPE, _sig_fpe);

	while ((opt = getopt(argc, argv, "swhr:H:C")) != -1) {
		switch (opt) {
		case 'w':
			AP_Param::erase_all();
			unlink("dataflash.bin");
			break;
		case 'r':
			_framerate = (unsigned)atoi(optarg);
			break;
		case 'H':
			_initial_height = atof(optarg);
			break;
		case 'C':
			AVR_SITL::SITLUARTDriver::_console = true;
			break;
		default:
			_usage();
			exit(1);
		}
	}

	fprintf(stdout, "Starting sketch '%s'\n", SKETCH);

	if (strcmp(SKETCH, "ArduCopter") == 0) {
		_vehicle = ArduCopter;
		if (_framerate == 0) {
			_framerate = 200;
		}
	} else if (strcmp(SKETCH, "APMrover2") == 0) {
		_vehicle = APMrover2;
		if (_framerate == 0) {
			_framerate = 50;
		}
		// set right default throttle for rover (allowing for reverse)
        pwm_input[2] = 1500;
	} else {
		_vehicle = ArduPlane;
		if (_framerate == 0) {
			_framerate = 50;
		}
	}

	_sitl_setup();
}


/*
  setup for SITL handling
 */
void SITL_State::_sitl_setup(void)
{
#ifndef __CYGWIN__
	_parent_pid = getppid();
#endif
	_rcout_addr.sin_family = AF_INET;
	_rcout_addr.sin_port = htons(_rcout_port);
	inet_pton(AF_INET, "127.0.0.1", &_rcout_addr.sin_addr);

	_setup_timer();
	_setup_fdm();
	fprintf(stdout, "Starting SITL input\n");

	// find the barometer object if it exists
	_sitl = (SITL *)AP_Param::find_object("SIM_");
	_barometer = (AP_Baro_BMP085_HIL *)AP_Param::find_object("GND_");
	_ins = (AP_InertialSensor_Stub *)AP_Param::find_object("INS_");
	_compass = (AP_Compass_HIL *)AP_Param::find_object("COMPASS_");

    if (_sitl != NULL) {
        // setup some initial values
        _update_barometer(_initial_height);
        _update_ins(0, 0, 0, 0, 0, 0, 0, 0, -9.8, 0);
        _update_compass(0, 0, 0);
        _update_gps(0, 0, 0, 0, 0, false);
    }
}


/*
  setup a SITL FDM listening UDP port
 */
void SITL_State::_setup_fdm(void)
{
	int one=1, ret;
	struct sockaddr_in sockaddr;

	memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
	sockaddr.sin_len = sizeof(sockaddr);
#endif
	sockaddr.sin_port = htons(_simin_port);
	sockaddr.sin_family = AF_INET;

	_sitl_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (_sitl_fd == -1) {
		fprintf(stderr, "SITL: socket failed - %s\n", strerror(errno));
		exit(1);
	}

	/* we want to be able to re-use ports quickly */
	setsockopt(_sitl_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	ret = bind(_sitl_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
	if (ret == -1) {
		fprintf(stderr, "SITL: bind failed on port %u - %s\n",
			(unsigned)ntohs(sockaddr.sin_port), strerror(errno));
		exit(1);
	}

	AVR_SITL::SITLUARTDriver::_set_nonblocking(_sitl_fd);
}


/*
  timer called at 1kHz
 */
void SITL_State::_timer_handler(int signum)
{
	static uint32_t last_update_count;
    static uint32_t last_pwm_input;

	static bool in_timer;

	if (in_timer || ((SITLScheduler *)hal.scheduler)->interrupts_are_blocked()) {
		return;
	}
	_scheduler->begin_atomic();

	in_timer = true;

#ifndef __CYGWIN__
	/* make sure we die if our parent dies */
	if (kill(_parent_pid, 0) != 0) {
		exit(1);
	}
#else
    
	static uint16_t count = 0;
	static uint32_t last_report;
        
    count++;
	if (millis() - last_report > 1000) {
		fprintf(stdout, "TH %u cps\n", count);
		count = 0;
		last_report = millis();
	}
#endif

    // simulate RC input at 50Hz
    if (hal.scheduler->millis() - last_pwm_input >= 20) {
        last_pwm_input = hal.scheduler->millis();
        pwm_valid = true;
    }

	/* check for packet from flight sim */
	_fdm_input();

	// send RC output to flight sim
	_simulator_output();

	if (_update_count == 0 && _sitl != NULL) {
		_update_gps(0, 0, 0, 0, 0, false);
		_scheduler->timer_event();
		_scheduler->end_atomic();
		in_timer = false;
		return;
	}

	if (_update_count == last_update_count) {
		_scheduler->timer_event();
		_scheduler->end_atomic();
		in_timer = false;
		return;
	}
	last_update_count = _update_count;

    if (_sitl != NULL) {
        _update_gps(_sitl->state.latitude, _sitl->state.longitude,
                    _sitl->state.altitude,
                    _sitl->state.speedN, _sitl->state.speedE, !_sitl->gps_disable);
        _update_ins(_sitl->state.rollDeg, _sitl->state.pitchDeg, _sitl->state.yawDeg,
                    _sitl->state.rollRate, _sitl->state.pitchRate, _sitl->state.yawRate,
                    _sitl->state.xAccel, _sitl->state.yAccel, _sitl->state.zAccel,
                    _sitl->state.airspeed);
        _update_barometer(_sitl->state.altitude);
        _update_compass(_sitl->state.rollDeg, _sitl->state.pitchDeg, _sitl->state.heading);
    }

	// trigger all APM timers. We do this last as it can re-enable
	// interrupts, which can lead to recursion
	_scheduler->timer_event();

	_scheduler->end_atomic();
	in_timer = false;
}


/*
  check for a SITL FDM packet
 */
void SITL_State::_fdm_input(void)
{
	ssize_t size;
	struct pwm_packet {
		uint16_t pwm[8];
	};
	union {
		struct sitl_fdm fg_pkt;
		struct pwm_packet pwm_pkt;
	} d;

	size = recv(_sitl_fd, &d, sizeof(d), MSG_DONTWAIT);
	switch (size) {
	case 132:
		static uint32_t last_report;
		static uint32_t count;

		if (d.fg_pkt.magic != 0x4c56414e) {
			fprintf(stdout, "Bad FDM packet - magic=0x%08x\n", d.fg_pkt.magic);
			return;
		}

		if (d.fg_pkt.latitude == 0 ||
		    d.fg_pkt.longitude == 0 ||
		    d.fg_pkt.altitude <= 0) {
			// garbage input
			return;
		}

        if (_sitl != NULL) {
            _sitl->state = d.fg_pkt;
        }
		_update_count++;

		count++;
		if (hal.scheduler->millis() - last_report > 1000) {
			//fprintf(stdout, "SIM %u FPS\n", count);
			count = 0;
			last_report = hal.scheduler->millis();
		}
		break;

	case 16: {
		// a packet giving the receiver PWM inputs
		uint8_t i;
		for (i=0; i<8; i++) {
			// setup the pwn input for the RC channel inputs
			if (d.pwm_pkt.pwm[i] != 0) {
				pwm_input[i] = d.pwm_pkt.pwm[i];
			}
		}
		break;
	}
	}

}

/*
  send RC outputs to simulator
 */
void SITL_State::_simulator_output(void)
{
	static uint32_t last_update;
	struct {
		uint16_t pwm[11];
		uint16_t speed, direction, turbulance;
	} control;
	/* this maps the registers used for PWM outputs. The RC
	 * driver updates these whenever it wants the channel output
	 * to change */
	uint8_t i;

	if (last_update == 0) {
		for (i=0; i<11; i++) {
			pwm_output[i] = 1000;
		}
		if (_vehicle == ArduPlane) {
			pwm_output[0] = pwm_output[1] = pwm_output[3] = 1500;
			pwm_output[7] = 1800;
		}
		if (_vehicle == APMrover2) {
			pwm_output[0] = pwm_output[1] = pwm_output[2] = pwm_output[3] = 1500;
			pwm_output[7] = 1800;
		}
	}

    if (_sitl == NULL) {
        return;
    }

	// output at chosen framerate
	if (last_update != 0 && hal.scheduler->millis() - last_update < 1000/_framerate) {
		return;
	}
	last_update = hal.scheduler->millis();

	for (i=0; i<11; i++) {
		if (pwm_output[i] == 0xFFFF) {
			control.pwm[i] = 0;
		} else {
			control.pwm[i] = pwm_output[i];
		}
	}

	if (_vehicle == ArduPlane) {
		// add in engine multiplier
		if (control.pwm[2] > 1000) {
			control.pwm[2] = ((control.pwm[2]-1000) * _sitl->engine_mul) + 1000;
			if (control.pwm[2] > 2000) control.pwm[2] = 2000;
		}
		_motors_on = ((control.pwm[2]-1000)/1000.0) > 0;
	} else if (_vehicle == APMrover2) {
		// add in engine multiplier
		if (control.pwm[2] != 1500) {
			control.pwm[2] = ((control.pwm[2]-1500) * _sitl->engine_mul) + 1500;
			if (control.pwm[2] > 2000) control.pwm[2] = 2000;
			if (control.pwm[2] < 1000) control.pwm[2] = 1000;
		}
		_motors_on = ((control.pwm[2]-1500)/500.0) != 0;
	} else {
		_motors_on = false;
		for (i=0; i<4; i++) {
			if ((control.pwm[i]-1000)/1000.0 > 0) {
				_motors_on = true;
			}
		}
	}

	// setup wind control
	control.speed      = _sitl->wind_speed * 100;
	float direction = _sitl->wind_direction;
	if (direction < 0) {
		direction += 360;
	}
	control.direction  = direction * 100;
	control.turbulance = _sitl->wind_turbulance * 100;

	// zero the wind for the first 15s to allow pitot calibration
	if (hal.scheduler->millis() < 15000) {
		control.speed = 0;
	}

	sendto(_sitl_fd, (void*)&control, sizeof(control), MSG_DONTWAIT, (const sockaddr *)&_rcout_addr, sizeof(_rcout_addr));
}


/*
  setup a timer used to prod the ISRs
 */
void SITL_State::_setup_timer(void)
{
	struct itimerval it;
	struct sigaction act;

	act.sa_handler = _timer_handler;
        act.sa_flags = SA_RESTART|SA_NODEFER;
        sigemptyset(&act.sa_mask);
        sigaddset(&act.sa_mask, SIGALRM);
        sigaction(SIGALRM, &act, NULL);

	it.it_interval.tv_sec = 0;
	it.it_interval.tv_usec = 1000; // 1KHz
	it.it_value = it.it_interval;

	setitimer(ITIMER_REAL, &it, NULL);
}

// generate a random float between -1 and 1
float SITL_State::_rand_float(void)
{
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

// generate a random Vector3f of size 1
Vector3f SITL_State::_rand_vec3f(void)
{
	Vector3f v = Vector3f(_rand_float(),
                          _rand_float(),
                          _rand_float());
	if (v.length() != 0.0) {
		v.normalize();
	}
	return v;
}


void SITL_State::init(int argc, char * const argv[])
{
    pwm_input[0] = pwm_input[1] = pwm_input[3] = 1500;
    pwm_input[4] = pwm_input[7] = 1800;
    pwm_input[2] = pwm_input[5] = pwm_input[6] = 1000;

    _scheduler = (SITLScheduler *)hal.scheduler;
	_parse_command_line(argc, argv);
}

// wait for serial input, or 100usec
void SITL_State::loop_hook(void)
{
    struct timeval tv;
    fd_set fds;
    int fd, max_fd = 0;

    FD_ZERO(&fds);
    fd = ((AVR_SITL::SITLUARTDriver*)hal.uartA)->_fd;
    if (fd != -1) {
        FD_SET(fd, &fds);
        max_fd = max(fd, max_fd);
    }
    fd = ((AVR_SITL::SITLUARTDriver*)hal.uartB)->_fd;
    if (fd != -1) {
        FD_SET(fd, &fds);
        max_fd = max(fd, max_fd);
    }
    fd = ((AVR_SITL::SITLUARTDriver*)hal.uartC)->_fd;
    if (fd != -1) {
        FD_SET(fd, &fds);
        max_fd = max(fd, max_fd);
    }
    tv.tv_sec = 0;
    tv.tv_usec = 100;
    fflush(stdout);
    fflush(stderr);
    select(max_fd+1, &fds, NULL, NULL, &tv);
}


#endif
