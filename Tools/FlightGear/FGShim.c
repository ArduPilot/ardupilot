// -*- tab-width: 8; Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

//
// Copyright (c) 2010 Michael Smith. All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//	  notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//	  notice, this list of conditions and the following disclaimer in the
//	  documentation and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.	IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.
//

//
// This is a serial port proxy shim for use with MAVLink-capable MAV
// controllers.  It allows the controller to interact with FlightGear
// and control a MAV inside the simulation.
//
// In addition to bridging between MAVLink and FlightGear, MAVLink data
// is forwarded to QGroundControl if it is running on the local system.
//
// The controller must be willing to send the RC_CHANNELS_SCALED 
// stream in response to a request, and it should be able to operate
// using the data supplied by the GPS_RAW and RAW_IMU messages.
//

#include <sys/errno.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <sys/uio.h>

#include <netinet/in.h>

#include <err.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#pragma pack(1)
#include "mavlink_types.h"

mavlink_system_t         mavlink_system;
void                    comm_send_ch(mavlink_channel_t chan, uint8_t ch);
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "ardupilotmega/mavlink.h"

#define TARGET_SYSTEM           7       /* XXX what should these really be? */
#define TARGET_COMPONENT        1

/*
 * Binary packet as exchanged with FG.
 *
 * Described in MAVLink.xml
 */
struct fgIMUData {
        // GPS
        double          latitude;
        double          longitude;
        double          altitude;
        double          heading;
        double          velocityN;
        double          velocityE;

        // IMU
        double          accelX;
        double          accelY;
        double          accelZ;
        double          rateRoll;
        double          ratePitch;
        double          rateYaw;

        // trailer
#define MSG_MAGIC       0x4c56414d
        uint32_t        magic;
} __attribute__((packed));

struct fgControlData {
        double          aileron;
        double          elevator;
        double          rudder;
        double          throttle;
} __attribute__((packed));

/* utility functions for transforming FG blobs */
void            swap32(void *p);
void            swap64(void *p);

/* diagnostic tool */
void            hexdump(void *p, size_t s);

/* local port addresses for communication with FlightGear - these are the FG defaults */
#define IMU_LISTEN_PORT         5501
#define CTRL_SEND_PORT          5500

/* local port addresses for communication with QGroundControl */
#define QGCS_LISTEN_PORT        14551
#define QGCS_SEND_PORT          14550

/*
 * debug logs
 */
#define FAC_MAIN        0
#define FAC_CTRL        1
#define FAC_IMU         2
#define FAC_QGCS        3
char *fac_names[] = {"", "CTRL: ", "IMU:  ", "QGCS: "};

#define log(_fac, _fmt, _args...)                                       \
        do {                                                            \
                fprintf(stderr, "%s" _fmt "\n", fac_names[FAC_##_fac], ##_args); \
        } while(0)

#define debug(_fac, _lvl, _fmt, _args...)                               \
        do {                                                            \
                if (gDebug >= _lvl) fprintf(stderr, "%s" _fmt "\n", fac_names[FAC_##_fac], ##_args); \
        } while(0)

int             gShouldQuit;                    /* set to 1 if threads should exit */
int             gDebug;                         /* set to 1 if -d is passed */

void            usage(void);                    /* print usage message */
void            shouldQuit(int sig);            /* signal that the program should quit */

/* 
 * Thread handling aircraft control instructions from the MAV controller to FlightGear.
 * Also forwards MAVLink packets to QGroundControl.
 */
pthread_t       gCTRLThread;
void            *ctrlThread(void *arg);
void            ctrlHandleMessage(mavlink_message_t *msg);

/* thread handling pseudo-IMU data from FlightGear to the MAV controller */
pthread_t       gIMUThread;
void            *imuThread(void *arg);
int             imuGetMessage(int sock, struct fgIMUData *msg);
void            imuSendPacket(struct fgIMUData *msg);

/* thread handling MAVLink traffic from QGroundControl to the MAV controller */
pthread_t       gQGCSThread;
void            *qgsThread(void *arg);

int             fgSock;         /* socket used for flightgear comms */
struct sockaddr_in fgAddr = {sizeof(fgAddr), AF_INET};

int             qgcsSock;        /* socket used for qgroundcontrol comms */
struct sockaddr_in qgcsAddr = {sizeof(qgcsAddr), AF_INET};

int             port;           /* serial port connected to MAV controller */
pthread_mutex_t portMutex;      /* lock that must be held while writing to the serial port */

int
main(int argc, char *argv[])
{
        int                     ch;
        struct sockaddr_in      sin = {sizeof(sin), AF_INET};
        int                     opt;
        struct termios          t;

        /* handle arguments */
        while ((ch = getopt(argc, argv, "ds:")) != -1) {
                switch(ch) {
                case 'd':
                        gDebug++;
                        break;

                case '?':
                default:
                        usage();
                }
        }
        argc -= optind;
        argv += optind;

        if (pthread_mutex_init(&portMutex, NULL))
                errx(1, "port mutex");

        /*
         * set up the flightgear socket
         */
        fgSock = socket(AF_INET, SOCK_DGRAM, 0);
        if (fgSock < 0)
                err(1, "IMU socket");
        sin.sin_port = htons(IMU_LISTEN_PORT);
        sin.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        fgAddr.sin_port = htons(CTRL_SEND_PORT);
        fgAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        /* bind the listening side of the socket */
        for (;;) {
                if (bind(fgSock, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
                        if (errno != EADDRINUSE)
                                err(1, "IMU socket bind");
                        log(IMU, "socket in use, waiting...");
                        sleep(5);
                } else {
                        break;
                }
        }
        
        /* set socket options */
        opt = 1;
        if (setsockopt(fgSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
                err(1, "IMU setsockopt SO_REUSEADDR");

        /*
         * set up the qgroundcontrol socket
         */
        qgcsSock = socket(AF_INET, SOCK_DGRAM, 0);
        if (qgcsSock < 0)
                err(1, "QGroundControl socket");
        qgcsAddr.sin_port = htons(QGCS_SEND_PORT);
        qgcsAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        sin.sin_port = htons(QGCS_LISTEN_PORT);
        sin.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        /* bind the listening side of the socket */
        for (;;) {
                if (bind(qgcsSock, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
                        if (errno != EADDRINUSE)
                                err(1, "QGroundControl socket bind");
                        log(QGCS, "socket in use, waiting...");
                        sleep(5);
                } else {
                        break;
                }
        }
        
        /* set socket options */
        opt = 1;
        if (setsockopt(qgcsSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
                err(1, "QGroundControl setsockopt SO_REUSEADDR");

        /*
         * set up the serial port at 57600
         */
        if (argc < 1)
                errx(1, "missing serial port name");
        if (0 >= (port = open(argv[0], O_RDWR | O_NONBLOCK)))
                err(1, "could not open port %s", argv[0]);
        if (tcgetattr(port, &t))
                err(1, "tcgetattr");
        cfmakeraw(&t);
        t.c_cflag |= CLOCAL;
        cfsetspeed(&t, 57600);
        
        if (tcsetattr(port, TCSANOW, &t))
                err(1, "tcsetattr");

        /* start worker threads */
        if (pthread_create(&gIMUThread, NULL, imuThread, NULL))
                err(1, "IMU thread");
        if (pthread_create(&gCTRLThread, NULL, ctrlThread, NULL))
                err(1, "CTRL thread");
        if (pthread_create(&gQGCSThread, NULL, qgsThread, NULL))
                err(1, "QGS thread");

        /* don't install new handlers until the threads are ready */
        signal(SIGHUP, shouldQuit);
        siginterrupt(SIGHUP, 1);
        signal(SIGINT, shouldQuit);
        siginterrupt(SIGINT, 1);
        signal(SIGTERM, shouldQuit);
        siginterrupt(SIGTERM, 1);
        signal(SIGQUIT, shouldQuit);
        siginterrupt(SIGQUIT, 1);
        
        pthread_join(gIMUThread, NULL);
        log(IMU, "service terminated");
        pthread_join(gCTRLThread, NULL);
        log(CTRL, "service terminated");
        pthread_join(gQGCSThread, NULL);
        log(QGCS, "service terminated");

        close(port);
        close(fgSock);
        close(qgcsSock);

        return(0);
}

#if __linux__
#define getprogname program_invocation_short_name
#endif

void
usage()
{
        fprintf(stderr, "usage: %s [-d[-d]]\n", getprogname());
        exit(1);
}

void
shouldQuit(int sig)
{
        if (!gShouldQuit) {
                fputs("\b\b  \b\b", stderr);    /* overwrite tty's ^C */
                if (sig)
                        debug(MAIN, 1, "caught signal %d, cleaning up", sig);
                pthread_kill(gIMUThread, SIGINT);
                pthread_kill(gCTRLThread, SIGINT);
                pthread_kill(gQGCSThread, SIGINT);
                gShouldQuit = 1;
        }
}

/*
 * IMU thread listens for datagrams from FlightGear and forwards them out
 * the serial port.
 */
void *
imuThread(void *arg)
{
        struct fgIMUData        msg;

        /* do not take signals on this thread */
        pthread_sigmask(SIG_BLOCK, NULL, NULL);

        log(IMU, "Listening for FlightGear binary data on port %d", IMU_LISTEN_PORT);

        /* handle IMU messages */
        while (!gShouldQuit) {
                if (imuGetMessage(fgSock, &msg))
                        imuSendPacket(&msg);
        }

        return(NULL);
}

int
imuGetMessage(int fgSock, struct fgIMUData *msg)
{
        ssize_t         received;

        /* get a message */
        received = recvfrom(fgSock, msg, sizeof(*msg), MSG_WAITALL, NULL, NULL);
        if (received != sizeof(*msg)) {
                if (received < 0) {
                        log(IMU, "receive error: %s", strerror(errno));
                shouldQuit(0);
                return(0);
                } else {
                        if (0 == received) {
                                log(IMU, "Received zero-length data packet, check that MAVLink.xml is correctly installed.");
                                shouldQuit(0);
                        } else {
                                log(IMU, "received %ld instead of %lu", received, sizeof(*msg));
                        }
                        return(0);
                }
        }
        swap32(&msg->magic);
        if (msg->magic != MSG_MAGIC) {
                log(IMU, "bad magic 0x%08x not 0x%08x", msg->magic, MSG_MAGIC);
                return(0);
        }

        /* endian-swap message fields */
        swap64(&msg->latitude);
        swap64(&msg->longitude);
        swap64(&msg->altitude);
        swap64(&msg->heading);
        swap64(&msg->velocityN);
        swap64(&msg->velocityE);
        swap64(&msg->accelX);
        swap64(&msg->accelY);
        swap64(&msg->accelZ);
        swap64(&msg->rateRoll);
        swap64(&msg->ratePitch);
        swap64(&msg->rateYaw);

        return(1);
}

void
imuSendPacket(struct fgIMUData *msg)
{
        struct timeval  tv;
        uint64_t        usec;
        int             i;

        gettimeofday(&tv, NULL);
        usec = (uint64_t)tv.tv_sec + tv.tv_usec;

        pthread_mutex_lock(&portMutex);

        debug(IMU, 2, "***");
        debug(IMU, 2, "lat %f  lon %f  alt %f", msg->latitude, msg->longitude, msg->altitude);
        debug(IMU, 2, "head %f  velocityN %f  velocityE %f", msg->heading, msg->velocityN, msg->velocityE);
        debug(IMU, 2, "accX %f  accY %f  accZ %f", msg->accelX, msg->accelY, msg->accelZ);
        debug(IMU, 2, "roll %f  pitch %f  yaw %f", msg->rateRoll, msg->ratePitch, msg->rateYaw);


#define ft2m(_x)        ((_x) * 0.3408)                         /* feet to metres */
#define dps2mrps(_x)    ((_x) * 17.453293)                      /* degrees per second to milliradians per second */
#define fpss2mg(_x)     ((_x) * 1000/ 32.2)                     /* feet per second per second to milligees */
#define rad2deg(_x)     fmod((((_x) * 57.29578) + 360), 360)    /* radians to degrees */

        mavlink_msg_gps_raw_send(0,
                                 usec,
                                 3,                     /* 3D fix */
                                 msg->latitude,
                                 msg->longitude,
                                 ft2m(msg->altitude),
                                 0,                     /* no uncertainty */
                                 0,                     /* no uncertainty */
                                 ft2m(sqrt((msg->velocityN * msg->velocityN) +
                                           (msg->velocityE * msg->velocityE))),
                                 rad2deg(atan2(msg->velocityE, msg->velocityN)));

        mavlink_msg_raw_imu_send(0,
                                 usec,
                                 fpss2mg(msg->accelX),
                                 fpss2mg(msg->accelY),
                                 fpss2mg(msg->accelZ),
                                 dps2mrps(msg->rateRoll),
                                 dps2mrps(msg->ratePitch),
                                 dps2mrps(msg->rateYaw),
                                 0,     /* xmag */
                                 0,     /* ymag */
                                 0);    /* zmag */

        pthread_mutex_unlock(&portMutex);

}

void *
ctrlThread(void *arg)
{
        fd_set                  readset, errorset;
        mavlink_message_t       msg;
        mavlink_status_t        status;
        int                     result;

        /* do not take signals on this thread */
        pthread_sigmask(SIG_BLOCK, NULL, NULL);
        
        log(CTRL, "Listening for MAVLink packets and sending to %d", CTRL_SEND_PORT);

        /* loop handling data arriving on the serial port */
        while (!gShouldQuit) {

                /* wait for data or an exceptional condition */
                FD_ZERO(&readset);
                FD_ZERO(&errorset);
                FD_SET(port, &readset);
                FD_SET(port, &errorset);
                if (select(port + 1, &readset, NULL, &errorset, NULL) < 0) {
                        if (errno != EINTR)
                                log(CTRL, "select error: %s", strerror(errno));
                        goto abort;
                } 

                /* exception? */
                if (FD_ISSET(port, &errorset)) {
                        log(CTRL, "serial port error");
                        goto abort;
                }

                /* no data available for reading? */
                if (!FD_ISSET(port, &readset))
                        continue;

                /* read data in chunks and process */
                for (;;) {
                        uint8_t buf[128];       /* 11ms @ 115200 */
                        ssize_t cnt;
                        uint8_t *ch;
                        
                        /* port is nonblocking, cnt may be as small as 1 byte */
                        cnt = read(port, buf, sizeof(buf));
                        if (cnt < 0) {
                                /* have we run out of data? */
                                if (errno == EWOULDBLOCK)
                                        break;
                                
                                /* some other error */
                                log(CTRL, "serial port error");
                                goto abort;
                        } 
                        
                        /* port closed for some reason */
                        if (cnt == 0) {
                                log(CTRL, "serial port closed");
                                goto abort;
                        }
                        
                        /* process characters, possibly handle messages */
                        for (ch = buf; cnt; ch++, cnt--)
                                if (mavlink_parse_char(0, *ch, &msg, &status))
                                        ctrlHandleMessage(&msg);
                }
        }

abort:
        shouldQuit(0);
        return(NULL);
}

void
ctrlHandleMessage(mavlink_message_t *msg)
{
        struct fgControlData            cdata;
        time_t                          now;
        static time_t                   lastRCMessage;
        uint8_t                         buf[1024];
        int                             len;

        time(&now);

        switch(msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
                /*
                 * If we have received a heartbeat, we are connected
                 * to something.  If we haven't seen an RC_CHANNELS_SCALED
                 * message in the last couple of seconds, ask for it to
                 * be added to the stream.
                 */
                if ((now - lastRCMessage) > 2) {
                        debug(CTRL, 1, "got heartbeat, requesting RC channel datastream");
                        pthread_mutex_lock(&portMutex);
                        mavlink_msg_request_data_stream_send(0,
                                                             TARGET_SYSTEM,
                                                             TARGET_COMPONENT, 
                                                             MAV_DATA_STREAM_RAW_CONTROLLER,
                                                             10, /* 10Hz enough? */
                                                             1); /* start */
                        pthread_mutex_unlock(&portMutex);
                        /* suppress re-sending for 2 seconds */
                        lastRCMessage = now;
                }
                break;
                
                
        case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:

                /* build the control message for FG */
                /* XXX ArduPilotMega channel ordering */
                cdata.aileron  = (double)mavlink_msg_rc_channels_scaled_get_chan1_scaled(msg) / 10000;
                cdata.elevator = (double)mavlink_msg_rc_channels_scaled_get_chan2_scaled(msg) / 10000;
                cdata.throttle = (double)mavlink_msg_rc_channels_scaled_get_chan3_scaled(msg) / 10000;
                cdata.rudder   = (double)mavlink_msg_rc_channels_scaled_get_chan4_scaled(msg) / 10000;

                debug(CTRL, 2, "%+6.4f %+6.4f %+6.4f %+6.4f", cdata.aileron, cdata.elevator, cdata.throttle, cdata.rudder);

                /* swap and send it to FG */
                swap64(&cdata.aileron);
                swap64(&cdata.elevator);
                swap64(&cdata.throttle);
                swap64(&cdata.rudder);
                sendto(fgSock, &cdata, sizeof(cdata), 0, (struct sockaddr *)&fgAddr, sizeof(fgAddr));

                /* update our timestamp so that we don't re-request the stream */
                lastRCMessage = now;
        }

        /* pass the message on to QGCS */
        len = mavlink_msg_to_send_buffer(buf, msg);
        sendto(qgcsSock, buf, len, 0, (struct sockaddr *)&qgcsAddr, sizeof(qgcsAddr));
}


/*
 * QGS thread listens for datagrams from QGroundControl and forwards them out
 * the serial port.
 */
void *
qgsThread(void *arg)
{
        struct fgIMUData        msg;
        ssize_t                 received;
        uint8_t                 buf[1024];
        int                     sent, result;

        /* do not take signals on this thread */
        pthread_sigmask(SIG_BLOCK, NULL, NULL);

        log(QGCS, "Listening for QGroundControl data on port %d", QGCS_LISTEN_PORT);

        while (!gShouldQuit) {

                /* get a message */
                received = recvfrom(qgcsSock, buf, sizeof(buf), MSG_WAITALL, NULL, NULL);

                /* XXX might want to intercept messages here that would turn off data that we need */

                /* and forward it */
                pthread_mutex_lock(&portMutex);
                sent = 0;
                while (sent < received) {
                        if (gShouldQuit)
                                break;
                        result = write(port, buf + sent, received - sent);
                        if (result < 0) {
                                if (result == EAGAIN)
                                        continue;
                                warn("serial passthrough write failed");
                                goto abort;
                        }
                        if (result == 0) {
                                warnx("serial port closed");
                                goto abort;
                        }
                        sent += result;
                }
                pthread_mutex_unlock(&portMutex);
        }

 abort:
        shouldQuit(0);
        return(NULL);
}


void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
        if (write(port, &ch, 1) != 1)
                warn("serial packet write failed");
}

void
swap32(void *p)
{
        *(int32_t *)p = htonl(*(int32_t *)p);
}

union temp64 {
                int64_t ll;
                int32_t l[2];
};

void
swap64(void *p)
{
        union temp64  *f, t;

        f = (union temp64 *)p;

        t.l[0] = htonl(f->l[1]);
        t.l[1] = htonl(f->l[0]);

        f->ll = t.ll;
}

void
hexdump(void *p, size_t s)
{
        int     i, j, lim;
        uint8_t *c = (uint8_t *)p;

        for (i = 0; i < s; i += 32) {
                printf("%04x:", i);
                lim = s - i;
                if (lim > 32)
                        lim = 32;
                for (j = 0; j < lim; j++)
                        printf(" %02x", *c++);
                printf("\n");
        }
}
