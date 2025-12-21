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
  uploader for IOMCU partly based on px4io_uploader.cpp from px4
 */
/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_IO_MCU

#include "AP_IOMCU.h"
#include <AP_ROMFS/AP_ROMFS.h>
#include <AP_Math/crc.h>

#define debug(fmt, args ...)  do { hal.console->printf("IOMCU: " fmt "\n", ## args); } while(0)

extern const AP_HAL::HAL &hal;

#ifndef AP_IOMCU_BOOTLOADER_BAUDRATE
#define AP_IOMCU_BOOTLOADER_BAUDRATE 115200
#endif

#ifndef AP_IOMCU_CHIBIOS_BOOTLOADER
#define AP_IOMCU_CHIBIOS_BOOTLOADER 0
#endif

/*
  upload a firmware to the IOMCU
 */
bool AP_IOMCU::upload_fw(void)
{
    // set baudrate for bootloader
    uart.begin(AP_IOMCU_BOOTLOADER_BAUDRATE, 256, 256);

    bool ret = false;

    /* look for the bootloader for 150 ms */
    for (uint8_t i = 0; i < 15; i++) {
        ret = sync();
        if (ret) {
            break;
        }
        hal.scheduler->delay(10);
    }

    if (!ret) {
        debug("IO update failed sync");
        return false;
    }

    uint32_t bl_rev;
#if AP_IOMCU_CHIBIOS_BOOTLOADER
    uint32_t unused;
#endif

    if (!get_info(INFO_BL_REV, bl_rev)
#if AP_IOMCU_CHIBIOS_BOOTLOADER
    || !get_info(INFO_BOARD_ID, unused) ||
    !get_info(INFO_FLASH_SIZE, unused)
#endif
    ) {
        debug("Err: failed to get bootloader info");
        return false;
    }
    if (bl_rev > BL_REV) {
        debug("Err: unsupported bootloader revision %u", unsigned(bl_rev));
        return false;
    }
    debug("found bootloader revision: %u", unsigned(bl_rev));

#if AP_IOMCU_CHIBIOS_BOOTLOADER
    if (bl_rev > 2) {
        // verify the CRC of the IO firmware
        if (verify_rev3(fw_size)) {
            // already up to date, no need to update, just reboot
            reboot();
            return true;
        }
    }
#endif

    ret = erase();
    if (!ret) {
        debug("erase failed");
        return false;
    }

    ret = program(fw_size);
    if (!ret) {
        debug("program failed");
        return false;
    }

    if (bl_rev <= 2) {
        ret = verify_rev2(fw_size);
    } else {
        ret = verify_rev3(fw_size);
    }

    if (!ret) {
        debug("verify failed");
        return false;
    }

    ret = reboot();

    if (!ret) {
        debug("reboot failed");
        return false;
    }

    debug("update complete");

    // sleep for enough time for the IO chip to boot
    hal.scheduler->delay(100);

    return true;
}

/*
  receive a byte from the IO bootloader
 */
bool AP_IOMCU::recv_byte_with_timeout(uint8_t *c, uint32_t timeout_ms)
{
    uint32_t start = AP_HAL::millis();
    do {
        int16_t v = uart.read();
        if (v >= 0) {
            *c = uint8_t(v);
            return true;
        }
        hal.scheduler->delay_microseconds(50);
    } while (AP_HAL::millis() - start < timeout_ms);

    return false;
}

/*
  receive multiple bytes from the bootloader
 */
bool AP_IOMCU::recv_bytes(uint8_t *p, uint32_t count)
{
    bool ret = true;

    while (count--) {
        ret = recv_byte_with_timeout(p++, 5000);
        if (!ret) {
            break;
        }
    }

    return ret;
}

/*
  discard any pending bytes
 */
void AP_IOMCU::drain(void)
{
    uint8_t c;
    bool ret;

    do {
        ret = recv_byte_with_timeout(&c, 40);
    } while (ret);
}

/*
  send a byte to the bootloader
 */
bool AP_IOMCU::send(uint8_t c)
{
    if (uart.write(c) != 1) {
        return false;
    }
    return true;
}

/*
  send a buffer to the bootloader
 */
bool AP_IOMCU::send(const uint8_t *p, uint32_t count)
{
    bool ret = true;

    while (count--) {
        ret = send(*p++);
        if (!ret) {
            break;
        }
    }

    return ret;
}

/*
  wait for bootloader protocol sync
 */
bool AP_IOMCU::get_sync(uint32_t timeout_ms)
{
    uint8_t c[2];
    bool ret;

    ret = recv_byte_with_timeout(c, timeout_ms);
    if (!ret) {
        return false;
    }

    ret = recv_byte_with_timeout(c + 1, timeout_ms);
    if (!ret) {
        return ret;
    }

    if ((c[0] != PROTO_INSYNC) || (c[1] != PROTO_OK)) {
        debug("bad sync 0x%02x,0x%02x", c[0], c[1]);
        return false;
    }

    return true;
}

/*
  drain then get sync with bootloader
 */
bool AP_IOMCU::sync()
{
    drain();

    /* complete any pending program operation */
    for (uint32_t i = 0; i < (PROG_MULTI_MAX + 6); i++) {
        send(0);
    }

    send(PROTO_GET_SYNC);
    send(PROTO_EOC);
    return get_sync();
}

/*
  get bootloader version
 */
bool AP_IOMCU::get_info(uint8_t param, uint32_t &val)
{
    bool ret;

    send(PROTO_GET_DEVICE);
    send(param);
    send(PROTO_EOC);

    ret = recv_bytes((uint8_t *)&val, sizeof(val));
    if (!ret) {
        return ret;
    }

    return get_sync();
}

/*
  erase IO firmware
 */
bool AP_IOMCU::erase()
{
    debug("erase...");
    send(PROTO_CHIP_ERASE);
    send(PROTO_EOC);
// The timeout for erase is increased for larger flash sizes on the IOMCU
#if AP_IOMCU_FW_FLASH_SIZE > 1024*1024 // 1MB
    return get_sync(20000); // 20s timeout for erase
#else
    return get_sync(10000); // 10s timeout for erase
#endif
}

/*
  send new firmware to bootloader
 */
bool AP_IOMCU::program(uint32_t size)
{
    bool ret = false;
    uint32_t sent = 0;

    if (size & 3) {
        return false;
    }

    debug("programming %u bytes...", (unsigned)size);

    while (sent < size) {
        /* get more bytes to program */
        uint32_t n = size - sent;
        if (n > PROG_MULTI_MAX) {
            n = PROG_MULTI_MAX;
        }

        send(PROTO_PROG_MULTI);
        send(n);
        send(&fw[sent], n);
        send(PROTO_EOC);

        ret = get_sync(1000);
        if (!ret) {
            debug("Failed at %u", (unsigned)sent);
            return false;
        }

        sent += n;
    }
    debug("upload OK");
    return true;
}

/*
  verify firmware for a rev2 bootloader
 */
bool AP_IOMCU::verify_rev2(uint32_t size)
{
    bool ret;
    size_t sent = 0;

    debug("verify...");

    send(PROTO_CHIP_VERIFY);
    send(PROTO_EOC);
    ret = get_sync();
    if (!ret) {
        return ret;
    }

    while (sent < size) {
        /* get more bytes to verify */
        uint32_t n = size - sent;
        if (n > 4) {
            n = 4;
        }

        send(PROTO_READ_MULTI);
        send(n);
        send(PROTO_EOC);


        for (uint8_t i = 0; i<n; i++) {
            uint8_t c;
            ret = recv_byte_with_timeout(&c, 5000);
            if (!ret) {
                debug("%d: got %d waiting for bytes", sent + i, ret);
                return ret;
            }
            if (c != fw[sent+i]) {
                debug("%d: got 0x%02x expected 0x%02x", sent + i, c, fw[sent+i]);
                return false;
            }
        }

        sent += n;

        ret = get_sync();
        if (!ret) {
            debug("timeout waiting for post-verify sync");
            return ret;
        }
    }

    return true;
}

/*
  verify firmware for a rev3 bootloader
 */
bool AP_IOMCU::verify_rev3(uint32_t fw_size_local)
{
    bool ret;
    uint32_t sum = 0;
    uint32_t crc = 0;
    uint32_t fw_size_remote;
    const uint8_t fill_blank = 0xff;

    debug("verify...");

    ret = get_info(INFO_FLASH_SIZE, fw_size_remote);
    send(PROTO_EOC);

    if (!ret) {
        debug("could not read firmware size");
        return ret;
    }

    sum = crc32_small(0, fw, fw_size_local);

    /* fill the rest of CRC with 0xff */
    for (uint32_t i=0; i<fw_size_remote - fw_size_local; i++) {
        sum = crc32_small(sum, &fill_blank, 1);
    }

    /* request CRC from IO */
    send(PROTO_GET_CRC);
    send(PROTO_EOC);

    ret = recv_bytes((uint8_t *)(&crc), sizeof(crc));
    if (!ret) {
        debug("did not receive CRC checksum");
        return ret;
    }

    /* compare the CRC sum from the IO with the one calculated */
    if (sum != crc) {
        debug("CRC wrong: received: 0x%x, expected: 0x%x", (unsigned)crc, (unsigned)sum);
#if AP_IOMCU_CHIBIOS_BOOTLOADER
        get_sync();
#endif
        return false;
    }

    crc_is_ok = true;

#if AP_IOMCU_CHIBIOS_BOOTLOADER
    return get_sync();
#else
    return true;
#endif
}

/*
  reboot IO MCU
 */
bool AP_IOMCU::reboot()
{
    send(PROTO_REBOOT);
    hal.scheduler->delay(200);
    send(PROTO_EOC);
    hal.scheduler->delay(200);
    return true;
}

#endif // HAL_WITH_IO_MCU
