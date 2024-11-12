/*
  socketcan transport for SITL CAN
 */
#include "CAN_SocketCAN.h"

#if HAL_NUM_CAN_IFACES && HAL_CAN_WITH_SOCKETCAN

#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <errno.h>
#include <stdlib.h>
#include "CAN_SocketCAN.h"

/*
  initialise socketcan transport
 */
bool CAN_SocketCAN::init(uint8_t instance)
{
    struct sockaddr_can addr {};
    struct ifreq ifr {};
    int ret;

    fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (fd < 0) {
        goto fail;
    }

    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "vcan%u", instance);
    ret = ioctl(fd, SIOCGIFINDEX, &ifr);
    if (ret == -1) {
        goto fail;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    ret = bind(fd, (struct sockaddr*)&addr, sizeof(addr));
    if (ret == -1) {
        goto fail;
    }

    return true;

fail:
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
    return false;
}

/*
  send a CAN frame
 */
bool CAN_SocketCAN::send(const AP_HAL::CANFrame &frame)
{
    if (frame.canfd) {
        // not supported on socketcan
        return false;
    }
    struct can_frame transmit_frame {};
    transmit_frame.can_id = frame.id;
    transmit_frame.can_dlc = frame.dlc;

    const uint8_t data_length = AP_HAL::CANFrame::dlcToDataLength(frame.dlc);
    memcpy(transmit_frame.data, frame.data, data_length);

    return ::write(fd, &transmit_frame, sizeof(transmit_frame)) == sizeof(transmit_frame);
}

/*
  receive a CAN frame
 */
bool CAN_SocketCAN::receive(AP_HAL::CANFrame &frame)
{
    struct can_frame receive_frame;
    const ssize_t ret = ::read(fd, &receive_frame, sizeof(receive_frame));
    if (ret != sizeof(receive_frame)) {
        return false;
    }

    // run constructor to initialise
    new(&frame) AP_HAL::CANFrame(receive_frame.can_id, receive_frame.data, receive_frame.can_dlc, false);

    if (sem_handle != nullptr) {
        sem_handle->signal();
    }
    return true;
}

#endif // HAL_NUM_CAN_IFACES
