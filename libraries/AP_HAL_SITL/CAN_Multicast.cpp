/*
  multicast UDP transport for SITL CAN
 */
#include "CAN_Multicast.h"

#if HAL_NUM_CAN_IFACES

#include <net/if.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <AP_Math/crc.h>

#define MCAST_ADDRESS_BASE "239.65.82.0"
#define MCAST_PORT 57732U
#define MCAST_MAGIC 0x2934U
#define MCAST_FLAG_CANFD 0x0001
#define MCAST_MAX_PKT_LEN 74 // 64 byte data + 10 byte header

struct PACKED mcast_pkt {
    uint16_t magic;
    uint16_t crc;
    uint16_t flags;
    uint32_t message_id;
    uint8_t data[MCAST_MAX_PKT_LEN-10];
};

/*
  initialise multicast transport
 */
bool CAN_Multicast::init(uint8_t instance)
{
    // setup incoming multicast socket
    char address[] = MCAST_ADDRESS_BASE;

    address[strlen(address)-1] = '0' + instance;
    return sock.connect(address, MCAST_PORT);
}

/*
  send a CAN frame
 */
bool CAN_Multicast::send(const AP_HAL::CANFrame &frame)
{
    struct mcast_pkt pkt {};
    pkt.magic = MCAST_MAGIC;
    pkt.flags = 0;
#if HAL_CANFD_SUPPORTED
    if (frame.canfd) {
        pkt.flags |= MCAST_FLAG_CANFD;
    }
#endif
    pkt.message_id = frame.id;
    const uint8_t data_length = AP_HAL::CANFrame::dlcToDataLength(frame.dlc);
    memcpy(pkt.data, frame.data, data_length);
    pkt.crc = crc16_ccitt((uint8_t*)&pkt.flags, data_length+6, 0xFFFFU);

    return sock.send((void*)&pkt, data_length+10) == data_length+10;
}

/*
  receive a CAN frame
 */
bool CAN_Multicast::receive(AP_HAL::CANFrame &frame)
{
    struct mcast_pkt pkt;
    ssize_t ret = sock.recv((void*)&pkt, sizeof(pkt), 0);
    if (ret < 10) {
        return false;
    }
    if (pkt.magic != MCAST_MAGIC) {
        return false;
    }
    if (pkt.crc != crc16_ccitt((uint8_t*)&pkt.flags, ret-4, 0xFFFFU)) {
        return false;
    }

    // run constructor to initialise
    new(&frame) AP_HAL::CANFrame(pkt.message_id, pkt.data, ret-10, (pkt.flags & MCAST_FLAG_CANFD) != 0);

    if (sem_handle != nullptr) {
        sem_handle->signal();
    }

    return true;
}

#endif // HAL_NUM_CAN_IFACES
