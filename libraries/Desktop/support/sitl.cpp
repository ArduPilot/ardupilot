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
#include "sitl_adc.h"
#include "sitl_rc.h"
#include "desktop.h"
#include "util.h"

/*
  the sitl_fdm packet is received by the SITL build from the flight
  simulator. This is used to feed the internal sensor emulation
 */
struct sitl_fdm {
	// little-endian packet format
	double latitude, longitude; // degrees
	double altitude;  // MSL
	double heading;   // degrees
	double speedN, speedE; // m/s
	double xAccel, yAccel, zAccel;       // m/s/s in body frame
	double rollRate, pitchRate, yawRate; // degrees/s/s in earth frame
	double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
	double airspeed; // m/s
	uint32_t magic; // 0x4c56414e
};


#define SIMIN_PORT 5501
#define RCOUT_PORT 5502

static int sitl_fd;
struct sockaddr_in rcout_addr;
static pid_t parent_pid;
struct ADC_UDR2 UDR2;
struct RC_ICR4 ICR4;
extern AP_TimerProcess timer_scheduler;
extern Arduino_Mega_ISR_Registry isr_registry;

static struct sitl_fdm sim_state;
static uint32_t update_count;


/*
  setup a SITL FDM listening UDP port
 */
static void setup_fdm(void)
{
	int one=1, ret;
	struct sockaddr_in sockaddr;

	memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
	sockaddr.sin_len = sizeof(sockaddr);
#endif
	sockaddr.sin_port = htons(SIMIN_PORT);
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
  check for a SITL FDM packet
 */
static void sitl_fdm_input(void)
{
	ssize_t size;
	struct pwm_packet {
		uint16_t pwm[8];
	};
	union {
		struct sitl_fdm fg_pkt;
		struct pwm_packet pwm_pkt;
	} d;

	size = recv(sitl_fd, &d, sizeof(d), MSG_DONTWAIT);
	switch (size) {
	case 132:
		static uint32_t last_report;
		static uint32_t count;

		if (d.fg_pkt.magic != 0x4c56414e) {
			printf("Bad FDM packet - magic=0x%08x\n", d.fg_pkt.magic);
			return;
		}

		if (d.fg_pkt.latitude == 0 ||
		    d.fg_pkt.longitude == 0 ||
		    d.fg_pkt.altitude <= 0) {
			// garbage input
			return;
		}

		sim_state = d.fg_pkt;
		update_count++;

		count++;
		if (millis() - last_report > 1000) {
			//printf("SIM %u FPS\n", count);
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

// used for noise generation in the ADC code
// motor speed in revolutions per second
float sitl_motor_speed[4] = {0,0,0,0};

/*
  send RC outputs to simulator
 */
static void sitl_simulator_output(void)
{
	static uint32_t last_update;
	uint16_t pwm[11];
	/* this maps the registers used for PWM outputs. The RC
	 * driver updates these whenever it wants the channel output
	 * to change */
	uint16_t *reg[11] = { &OCR5B, &OCR5C, &OCR1B, &OCR1C,
			      &OCR4C, &OCR4B, &OCR3C, &OCR3B,
			      &OCR5A, &OCR1A, &OCR3A };
	uint8_t i;

	if (last_update == 0) {
		for (i=0; i<11; i++) {
			(*reg[i]) = 1000*2;
		}
		if (!desktop_state.quadcopter) {
			(*reg[0]) = (*reg[1]) = (*reg[3]) = 1500*2;
			(*reg[7]) = 1800*2;
		}
	}

	// output at chosen framerate
	if (millis() - last_update < 1000/desktop_state.framerate) {
		return;
	}
	last_update = millis();

	for (i=0; i<11; i++) {
		// the registers are 2x the PWM value
		pwm[i] = (*reg[i])/2;
	}

	if (!desktop_state.quadcopter) {
		// 400kV motor, 16V
		sitl_motor_speed[0] = ((pwm[2]-1000)/1000.0) * 400 * 16 / 60.0;
	} else {
		// 850kV motor, 16V
		for (i=0; i<4; i++) {
			sitl_motor_speed[i] = ((pwm[i]-1000)/1000.0) * 850 * 12 / 60.0;
		}
	}

	sendto(sitl_fd, (void*)pwm, sizeof(pwm), MSG_DONTWAIT, (const sockaddr *)&rcout_addr, sizeof(rcout_addr));
}

/*
  timer called at 1kHz
 */
static void timer_handler(int signum)
{
	static uint32_t last_update_count;

	/* make sure we die if our parent dies */
	if (kill(parent_pid, 0) != 0) {
		exit(1);
	}

	/* check for packet from flight sim */
	sitl_fdm_input();

	// trigger all timers
	timer_scheduler.run();

	// trigger RC input
	if (isr_registry._registry[ISR_REGISTRY_TIMER4_CAPT]) {
		isr_registry._registry[ISR_REGISTRY_TIMER4_CAPT]();
	}

	// send RC output to flight sim
	sitl_simulator_output();

	if (update_count == 0) {
		sitl_update_gps(0, 0, 0, 0, 0, false);
		return;
	}

	if (update_count == last_update_count) {
		return;
	}
	last_update_count = update_count;

	sitl_update_gps(sim_state.latitude, sim_state.longitude,
			sim_state.altitude,
			sim_state.speedN, sim_state.speedE, true);
	sitl_update_adc(sim_state.rollDeg, sim_state.pitchDeg, sim_state.yawDeg,
			sim_state.rollRate, sim_state.pitchRate, sim_state.yawRate,
			sim_state.xAccel, sim_state.yAccel, sim_state.zAccel,
			sim_state.airspeed);
	sitl_update_barometer(sim_state.altitude);
	sitl_update_compass(sim_state.heading, sim_state.rollDeg, sim_state.pitchDeg, sim_state.heading);
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

	rcout_addr.sin_family = AF_INET;
	rcout_addr.sin_port = htons(RCOUT_PORT);
	inet_pton(AF_INET, "127.0.0.1", &rcout_addr.sin_addr);

	setup_timer();
	setup_fdm();
	sitl_setup_adc();
	printf("Starting SITL input\n");

	// setup some initial values
	sitl_update_barometer(desktop_state.initial_height);
	sitl_update_adc(0, 0, 0, 0, 0, 0, 0, 0, -9.8, 0);
	sitl_update_compass(0, 0, 0, 0);
	sitl_update_gps(0, 0, 0, 0, 0, false);
}
