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
/*
  this is a driver for RC output in the QFLIGHT board. Output goes via
  a UART with a CRC. See libraries/RC_Channel/examples/RC_UART for an
  example of the other end of this protocol
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include "RCOutput_qflight.h"
#include <AP_HAL_Linux/qflight/qflight_util.h>
#include <AP_HAL_Linux/qflight/qflight_dsp.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCOutput_QFLIGHT::init()
{
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&RCOutput_QFLIGHT::timer_update, void));
}

void RCOutput_QFLIGHT::set_device_path(const char *_device)
{
    device = _device;
}


void RCOutput_QFLIGHT::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    // no support for changing frequency yet
}

uint16_t RCOutput_QFLIGHT::get_freq(uint8_t ch)
{
    // return fixed fake value - no control of frequency over the UART
    return 490;
}

void RCOutput_QFLIGHT::enable_ch(uint8_t ch)
{
    if (ch >= channel_count) {
        return;
    }
    enable_mask |= 1U<<ch;
}

void RCOutput_QFLIGHT::disable_ch(uint8_t ch)
{
    if (ch >= channel_count) {
        return;
    }
    enable_mask &= ~1U<<ch;
}

void RCOutput_QFLIGHT::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= channel_count) {
        return;
    }
    period[ch] = period_us;
    if (!corked) {
        need_write = true;
    }
}

uint16_t RCOutput_QFLIGHT::read(uint8_t ch)
{
    if (ch >= channel_count) {
        return 0;
    }
    return period[ch];
}

void RCOutput_QFLIGHT::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

void RCOutput_QFLIGHT::timer_update(void)
{
    /*
      we defer the open to the time to ensure all RPC calls are made
      from the same thread
     */
    if (fd == -1 && device != nullptr) {
        int ret = qflight_UART_open(device, &fd);
        printf("Opened ESC UART %s ret=%d fd=%d\n",
               device, ret, (int)fd);
        if (fd != -1) {
            qflight_UART_set_baudrate(fd, baudrate);
        }
    }
    if (!need_write || fd == -1) {
        return;
    }
    /*
      this implements the PWM over UART prototocol. 
     */
    struct PACKED {
        uint8_t magic = 0xF7;
        uint16_t period[channel_count];
        uint16_t crc;
    } frame;
    memcpy(frame.period, period, sizeof(period));
    frame.crc = crc_calculate((uint8_t*)frame.period, channel_count*2);
    int32_t nwritten;
    qflight_UART_write(fd, (uint8_t *)&frame, sizeof(frame), &nwritten);
    need_write = false;

    check_rc_in();
}

/*
  we accept RC input from the UART and treat it as RC overrides. This
  is an lazy way to allow an RCOutput driver to do RCInput. See the
  RC_UART example for the other end of this protocol
 */
void RCOutput_QFLIGHT::check_rc_in(void)
{
    const uint8_t magic = 0xf6;
    while (nrcin_bytes != sizeof(rcu.bytes)) {
        int32_t nread;
        if (qflight_UART_read(fd, rcu.bytes, sizeof(rcu.bytes)-nrcin_bytes, &nread) != 0 || nread <= 0) {
            return;
        }
        nrcin_bytes += nread;
        if (rcu.rcin.magic != magic) {
            for (uint8_t i=1; i<nrcin_bytes; i++) {
                if (rcu.bytes[i] == magic) {
                    memmove(&rcu.bytes[0], &rcu.bytes[i], nrcin_bytes-i);
                    nrcin_bytes = nrcin_bytes - i;
                    return;
                }
            }
            nrcin_bytes = 0;
            return;
        }
    }
    if (nrcin_bytes == sizeof(rcu.bytes)) {
        if (rcu.rcin.magic == 0xf6 &&
            crc_calculate((uint8_t*)rcu.rcin.rcin, sizeof(rcu.rcin.rcin)) == rcu.rcin.crc) {
            bool have_data = false;
            for (uint8_t i=0; i<8; i++) {
                if (rcu.rcin.rcin[i] != 0) {
                    have_data = true;
                    break;
                }
            }
            if (have_data) {
                hal.rcin->set_overrides((int16_t*)rcu.rcin.rcin, 8);
            }
        }
        nrcin_bytes = 0;
    }
}

void RCOutput_QFLIGHT::cork(void)
{
    corked = true;
}

void RCOutput_QFLIGHT::push(void)
{
    corked = false;
    need_write = true;
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

