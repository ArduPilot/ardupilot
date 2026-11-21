#include "RCOutput_ATAL_Loopback.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

using namespace Linux;

void RCOutput_ATAL_Loopback::init()
{
    socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
}

void RCOutput_ATAL_Loopback::set_freq(uint32_t, uint16_t freq_hz)
{
    frequency_hz = freq_hz;
}

uint16_t RCOutput_ATAL_Loopback::get_freq(uint8_t)
{
    return frequency_hz;
}

void RCOutput_ATAL_Loopback::enable_ch(uint8_t) {}
void RCOutput_ATAL_Loopback::disable_ch(uint8_t) {}

void RCOutput_ATAL_Loopback::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= channel_count) {
        return;
    }
    values[ch] = period_us;
    if (!corked) {
        push();
    }
}

uint16_t RCOutput_ATAL_Loopback::read(uint8_t ch)
{
    return ch < channel_count ? values[ch] : 900;
}

void RCOutput_ATAL_Loopback::read(uint16_t *period_us, uint8_t len)
{
    memcpy(period_us, values, std::min<uint8_t>(len, channel_count) * sizeof(values[0]));
}

void RCOutput_ATAL_Loopback::cork()
{
    corked = true;
}

void RCOutput_ATAL_Loopback::push()
{
    corked = false;
    if (socket_fd < 0) {
        return;
    }

    // Raw SERVO channel order. Mapping to ESP32 IDs belongs in the eventual
    // actuator adapter, after hardware and SERVOx_FUNCTION are verified.
    char packet[128];
    int size = snprintf(packet, sizeof(packet), "ATAL_RCOUT,1,%u",
                        AP_HAL::get_HAL().util->get_soft_armed() ? 1U : 0U);
    for (uint16_t value : values) {
        size += snprintf(packet + size, sizeof(packet) - size, ",%u", value);
    }
    const sockaddr_in endpoint{AF_INET, htons(14660), {htonl(INADDR_LOOPBACK)}, {0}};
    sendto(socket_fd, packet, size, MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&endpoint), sizeof(endpoint));
}
