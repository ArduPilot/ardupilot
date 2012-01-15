// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

// test harness for the APM_BinComm bits

#include <stdint.h>
#include <err.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#include "WProgram.h"

#include "../APM_BinComm.h"

static void     handler(void *arg, uint8_t messageId, uint8_t messageVersion, void *messageData);

BinComm::MessageHandler handlers[] = {
        {BinComm::MSG_ANY,       handler,        NULL},
        {BinComm::MSG_NULL,      0,              0}
};

Stream          port;
BinComm         comm(handlers, &port);

int             port_fd;

unsigned int
millis(void)
{
        return 0;
}

void
Stream::write(uint8_t val)
{
        ::write(port_fd, &val, 1);
}

int
Stream::available(void)
{
        return(1);
}

int
Stream::read(void)
{
        int     ret;
        uint8_t c;

        switch(::read(port_fd, &c, 1)) {
        case 1:
                return c;
        case 0:
                errx(1, "device disappeared");

        default:
                // almost certainly EWOULDBLOCK
                return -1;
        }
}

void
handler(void *arg, uint8_t messageId, uint8_t messageVersion, void *messageData)
{

        if (messageId == BinComm::MSG_HEARTBEAT) {
                struct BinComm::msg_heartbeat *m = (struct BinComm::msg_heartbeat *)messageData;
                printf("Heartbeat: mode %u  time %u  voltage %u  command %u\n",
                       m->flightMode, m->timeStamp, m->batteryVoltage, m->commandIndex);
        } else
        if (messageId == BinComm::MSG_ATTITUDE) {
                struct BinComm::msg_attitude *m = (struct BinComm::msg_attitude *)messageData;
                printf("Attitude: pitch %d  roll %d  yaw %d\n",
                       m->pitch, m->roll, m->yaw);
        } else
        if (messageId == BinComm::MSG_LOCATION) {
                struct BinComm::msg_location *m = (struct BinComm::msg_location *)messageData;
                printf("Location: lat %d  long %d  altitude %d  groundspeed %d  groundcourse %d  time %u\n",
                       m->latitude, m->longitude, m->altitude, m->groundSpeed, m->groundCourse, m->timeOfWeek);
        } else
        if (messageId == BinComm::MSG_STATUS_TEXT) {
                struct BinComm::msg_status_text *m = (struct BinComm::msg_status_text *)messageData;
                printf("Message %d: %-50s\n", m->severity, m->text);
        } else {
                warnx("received message %d,%d", messageId, messageVersion);
        }
}

int
main(int argc, char *argv[])
{
        struct termios  t;

        if (2 > argc)
                errx(1, "BinCommTest <port>");
        if (0 >= (port_fd = open(argv[1], O_RDWR | O_NONBLOCK)))
                err(1, "could not open port %s", argv[1]);
        if (tcgetattr(port_fd, &t))
                err(1, "tcgetattr");
        cfsetspeed(&t, 115200);
        if (tcsetattr(port_fd, TCSANOW, &t))
                err(1, "tcsetattr");

        // spin listening
        for (;;) {
                comm.update();
        }
}
