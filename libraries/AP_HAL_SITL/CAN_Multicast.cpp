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
    struct sockaddr_in sockaddr {};
    struct ip_mreq mreq {};
    int one = 1;
    int ret;

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    address[strlen(address)-1] = '0' + instance;

    sockaddr.sin_port = htons(MCAST_PORT);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(address);

    fd_in = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_in == -1) {
        goto fail;
    }
    ret = fcntl(fd_in, F_SETFD, FD_CLOEXEC);
    if (ret == -1) {
        goto fail;
    }
    if (setsockopt(fd_in, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == -1) {
        goto fail;
    }

    // close on exec, to allow reboot
    fcntl(fd_in, F_SETFD, FD_CLOEXEC);

#if defined(__CYGWIN__) || defined(__CYGWIN64__) || defined(CYGWIN_BUILD)
    /*
      on cygwin you need to bind to INADDR_ANY then use the multicast
      IP_ADD_MEMBERSHIP to get on the right address
     */
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
    
    ret = bind(fd_in, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        goto fail;
    }

    mreq.imr_multiaddr.s_addr = inet_addr(address);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    ret = setsockopt(fd_in, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    if (ret == -1) {
        goto fail;
    }

    // setup outgoing socket
    fd_out = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_out == -1) {
        goto fail;
    }
    ret = fcntl(fd_out, F_SETFD, FD_CLOEXEC);
    if (ret == -1) {
        goto fail;
    }

    ret = connect(fd_out, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        goto fail;
    }
    
    return true;

fail:
    if (fd_in != -1) {
        (void)close(fd_in);
        fd_in = -1;
    }
    if (fd_out != -1) {
        (void)close(fd_out);
        fd_out = -1;
    }
    return false;
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

    return ::send(fd_out, (void*)&pkt, data_length+10, 0) == data_length+10;
}

/*
  receive a CAN frame
 */
bool CAN_Multicast::receive(AP_HAL::CANFrame &frame)
{
    struct mcast_pkt pkt;
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);
    ssize_t ret = ::recvfrom(fd_in, (void*)&pkt, sizeof(pkt), MSG_DONTWAIT, (struct sockaddr *)&src_addr, &src_len);
    if (ret < 10) {
        return false;
    }
    if (pkt.magic != MCAST_MAGIC) {
        return false;
    }
    if (pkt.crc != crc16_ccitt((uint8_t*)&pkt.flags, ret-4, 0xFFFFU)) {
        return false;
    }

    // ensure it isn't a packet we sent
    struct sockaddr_in send_addr;
    socklen_t send_len = sizeof(send_addr);
    if (getsockname(fd_out, (struct sockaddr *)&send_addr, &send_len) != 0) {
        return false;
    }
    if (src_addr.sin_port == send_addr.sin_port &&
        src_addr.sin_family == send_addr.sin_family &&
        src_addr.sin_addr.s_addr == send_addr.sin_addr.s_addr) {
        return false;
    }

    // run constructor to initialise
    new(&frame) AP_HAL::CANFrame(pkt.message_id, pkt.data, ret-10, (pkt.flags & MCAST_FLAG_CANFD) != 0);

    return true;
}

#endif // HAL_NUM_CAN_IFACES
