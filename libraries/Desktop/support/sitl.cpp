/*
  SITL handling

  This simulates the APM1 hardware sufficiently for the APM code to
  think it is running on real hardware

  Andrew Tridgell November 2011
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <math.h>
#include <APM_RC.h>
#include <wiring.h>
#include <AP_PeriodicProcess.h>
#include <AP_TimerProcess.h>
#include <AP_TimerAperiodicProcess.h>
#include "sitl_adc.h"
#include "sitl_rc.h"
#include "desktop.h"
#include "util.h"

#define FGIN_PORT 5501
#define FGOUT_PORT 5502

static int sitl_fd;
struct sockaddr_in fgout_addr;
static pid_t parent_pid;
struct ADC_UDR2 UDR2;
struct RC_ICR4 ICR4;
extern AP_TimerProcess timer_scheduler;
extern Arduino_Mega_ISR_Registry isr_registry;
static volatile uint32_t sim_input_count;


/*
  setup a FGear listening UDP port, using protocol from MAVLink.xml
 */
static void setup_fgear(void)
{
	int one=1, ret;
	struct sockaddr_in sockaddr;

	memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
	sockaddr.sin_len = sizeof(sockaddr);
#endif
	sockaddr.sin_port = htons(FGIN_PORT);
	sockaddr.sin_family = AF_INET;

	sitl_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sitl_fd == -1) {
		fprintf(stderr, "SITL: socket failed - %s\n", strerror(errno));
		exit(1);
	}

	/* we want to be able to re-use ports quickly */
	setsockopt(sitl_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	ret = bind(sitl_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
	if (ret == -1) {
		fprintf(stderr, "SITL: bind failed on port %u - %s\n",
			(unsigned)ntohs(sockaddr.sin_port), strerror(errno));
		exit(1);
	}

	set_nonblocking(sitl_fd);
}

/*
  check for a fgear packet
 */
static void sitl_fgear_input(void)
{
	ssize_t size;
	struct fg_mavlink {
		double latitude, longitude, altitude, heading,
			speedN, speedE,
			xAccel, yAccel, zAccel,
			rollRate, pitchRate, yawRate,
			rollDeg, pitchDeg, yawDeg,
			airspeed;
		uint32_t magic;
	};
	struct pwm_packet {
		uint16_t pwm[8];
	};
	union {
		struct fg_mavlink fg_pkt;
		struct pwm_packet pwm_pkt;
	} d;

	size = recv(sitl_fd, &d, sizeof(d), MSG_DONTWAIT);
	switch (size) {
	case 132:
		static uint32_t last_report;
		static uint32_t count;

		/* sigh, its big-endian */
		swap_doubles(&d.fg_pkt.latitude, 16);
		d.fg_pkt.magic = ntohl(d.fg_pkt.magic);
		if (d.fg_pkt.magic != 0x4c56414d) {
			printf("Bad fgear packet - magic=0x%08x\n", d.fg_pkt.magic);
			return;
		}

		if (d.fg_pkt.latitude == 0 ||
		    d.fg_pkt.longitude == 0 ||
		    d.fg_pkt.altitude <= 0) {
			// garbage input
			return;
		}

		sitl_update_gps(d.fg_pkt.latitude, d.fg_pkt.longitude,
				ft2m(d.fg_pkt.altitude),
				ft2m(d.fg_pkt.speedN), ft2m(d.fg_pkt.speedE), true);
		sitl_update_adc(d.fg_pkt.rollDeg, d.fg_pkt.pitchDeg, d.fg_pkt.heading, kt2mps(d.fg_pkt.airspeed));
		sitl_update_barometer(ft2m(d.fg_pkt.altitude));
		sitl_update_compass(d.fg_pkt.heading, d.fg_pkt.rollDeg, d.fg_pkt.pitchDeg, d.fg_pkt.heading);
		count++;
		sim_input_count++;
		if (millis() - last_report > 1000) {
			printf("SIM %u FPS\n", count);
			count = 0;
			last_report = millis();
		}
		break;

	case 16: {
		// a packet giving the receiver PWM inputs
		uint8_t i;
		for (i=0; i<8; i++) {
			// setup the ICR4 register for the RC channel
			// inputs
			if (d.pwm_pkt.pwm[i] != 0) {
				ICR4.set(i, d.pwm_pkt.pwm[i]);
			}
		}
		break;
	}
	}

}

/*
  send RC outputs to simulator for a quadcopter
 */
static void sitl_quadcopter_output(uint16_t pwm[8])
{
	struct fg_output {
		float throttle[4];
		uint16_t pwm[8];
	} pkt;
	for (uint8_t i=0; i<8; i++) {
		pkt.pwm[i] = htonl(pwm[i]);
	}
	for (uint8_t i=0; i<4; i++) {
		pkt.throttle[i] = swap_float((pwm[i]-1000) / 1000.0);
	}
	sendto(sitl_fd, &pkt, sizeof(pkt), MSG_DONTWAIT, (const sockaddr *)&fgout_addr, sizeof(fgout_addr));
}

/*
  send RC outputs to simulator for a plane
 */
static void sitl_plane_output(uint16_t pwm[8])
{
	double servo[4];

	servo[0] = (((int)pwm[0]) - 1500)/500.0;
	servo[1] = (((int)pwm[1]) - 1500)/500.0;
	servo[2] = (((int)pwm[3]) - 1500)/500.0;
	servo[3] = (pwm[2] - 1000) / 1000.0;
	swap_doubles(servo, 4);
	sendto(sitl_fd, &servo, sizeof(servo), MSG_DONTWAIT, (const sockaddr *)&fgout_addr, sizeof(fgout_addr));
}


/*
  send RC outputs to simulator for a quadcopter
 */
static void sitl_simulator_output(void)
{
	static uint32_t last_update;
	uint16_t pwm[8];
	/* this maps the registers used for PWM outputs. The RC
	 * driver updates these whenever it wants the channel output
	 * to change */
	uint16_t *reg[11] = { &OCR5B, &OCR5C, &OCR1B, &OCR1C,
			      &OCR4C, &OCR4B, &OCR3C, &OCR3B,
			      &OCR5A, &OCR1A, &OCR3A };
	uint8_t i;

	if (last_update == 0) {
		if (desktop_state.quadcopter) {
			for (i=0; i<8; i++) {
				(*reg[i]) = 1000*2;
			}
		} else {
			(*reg[0]) = (*reg[1]) = (*reg[3]) = 1500*2;
			(*reg[2]) = (*reg[4]) = (*reg[6]) = 1000*2;
			(*reg[5]) = (*reg[7]) = 1800*2;
		}
	}

	// output at chosen framerate
	if (millis() - last_update < 1000/desktop_state.framerate) {
		return;
	}
	last_update = millis();

	for (i=0; i<8; i++) {
		// the registers are 2x the PWM value
		pwm[i] = (*reg[i])/2;
	}

	if (desktop_state.quadcopter) {
		sitl_quadcopter_output(pwm);
	} else {
		sitl_plane_output(pwm);
	}

}

/*
  timer called at 1kHz
 */
static void timer_handler(int signum)
{
	/* make sure we die if our parent dies */
	if (kill(parent_pid, 0) != 0) {
		exit(1);
	}

	/* check for packet from flight sim */
	sitl_fgear_input();

	// trigger all timers
	timer_scheduler.run();

	// trigger RC input
	if (isr_registry._registry[ISR_REGISTRY_TIMER4_CAPT]) {
		isr_registry._registry[ISR_REGISTRY_TIMER4_CAPT]();
	}

	// send RC output to flight sim
	sitl_simulator_output();

	if (sim_input_count == 0) {
		sitl_update_gps(0, 0, 0, 0, 0, false);
	}
}


/*
  setup a timer used to prod the ISRs
 */
static void setup_timer(void)
{
	struct itimerval it;
	struct sigaction act;

	act.sa_handler = timer_handler;
        act.sa_flags = SA_RESTART|SA_NODEFER;
        sigemptyset(&act.sa_mask);
        sigaddset(&act.sa_mask, SIGALRM);
        sigaction(SIGALRM, &act, NULL);

	it.it_interval.tv_sec = 0;
	it.it_interval.tv_usec = 1000; // 1KHz
	it.it_value = it.it_interval;

	setitimer(ITIMER_REAL, &it, NULL);
}


/*
  setup for SITL handling
 */
void sitl_setup(void)
{
	parent_pid = getppid();

	fgout_addr.sin_family = AF_INET;
	fgout_addr.sin_port = htons(FGOUT_PORT);
	inet_pton(AF_INET, "127.0.0.1", &fgout_addr.sin_addr);

	setup_timer();
	setup_fgear();
	sitl_setup_adc();
	printf("Starting SITL input\n");

	// setup some initial values
	sitl_update_barometer(desktop_state.initial_height);
	sitl_update_adc(0, 0, 0, 0);
	sitl_update_compass(0, 0, 0, 0);
	sitl_update_gps(0, 0, 0, 0, 0, false);
}
