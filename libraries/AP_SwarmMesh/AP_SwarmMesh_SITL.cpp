/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_SwarmMesh_SITL.h"

#if AP_SWARMMESH_SITL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <sys/socket.h>

// multicast group shared by all SITL swarm instances
#define SWARMMESH_MCAST_ADDRESS "239.65.83.0"
#define SWARMMESH_MCAST_PORT    57733U

// Large kernel receive buffer for the multicast socket. With the full sysid range (254 nodes) every instance receives ~N x stream-rate packets/s.
// The default ~256KB SO_RCVBUF overflows between drains and the kernel silently discards ~half the traffic (peer tables converge to only ~half the swarm).
// A few MB absorbs the scheduling jitter. Bounded by kern.ipc.maxsockbuf.
#define SWARMMESH_SITL_RCVBUF (4 * 1024 * 1024)

AP_SwarmMesh_SITL::AP_SwarmMesh_SITL(AP_SwarmMesh &frontend) :
    AP_SwarmMesh_Backend(frontend),
    _sock(true)
{
    _sock_ok = _sock.connect(SWARMMESH_MCAST_ADDRESS, SWARMMESH_MCAST_PORT);
    if (_sock_ok) {
        const int fd = _sock.get_read_fd();
        if (fd >= 0) {
            int rcvbuf = SWARMMESH_SITL_RCVBUF;
            setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        }
    }
}

bool AP_SwarmMesh_SITL::transport_ready() const
{
    return _sock_ok;
}

uint32_t AP_SwarmMesh_SITL::transport_txspace()
{
    return 0xFFFFU;
}

// bridge UDP datagram recv into a byte-stream buffer that parse_byte() drains one byte at a time
uint32_t AP_SwarmMesh_SITL::transport_available()
{
    if (_rx_buf_pos < _rx_buf_len) {
        return _rx_buf_len - _rx_buf_pos;
    }
    // buffer exhausted, try to pull the next datagram (non-blocking)
    const ssize_t n = _sock.recv(_rx_buf, sizeof(_rx_buf), 0);
    if (n <= 0) {
        return 0;
    }
    _rx_buf_len = (uint16_t)n;
    _rx_buf_pos = 0;
    return _rx_buf_len;
}

int16_t AP_SwarmMesh_SITL::transport_read()
{
    if (_rx_buf_pos >= _rx_buf_len) {
        return -1;
    }
    return _rx_buf[_rx_buf_pos++];
}

void AP_SwarmMesh_SITL::transport_write(const uint8_t *buf, uint16_t len)
{
    _sock.send(buf, len);
}

#endif  // AP_SWARMMESH_SITL_ENABLED
