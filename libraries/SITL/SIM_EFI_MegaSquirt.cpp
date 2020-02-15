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
  simulate MegaSquirt EFI system
*/

#include "SIM_Aircraft.h"
#include <SITL/SITL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>

using namespace SITL;

static uint32_t CRC32_MS(const uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0;
    while (len--) {
        crc ^= ~0U;
        crc = crc_crc32(crc, buf++, 1);
        crc ^= ~0U;
    }
    return crc;
}

void EFI_MegaSquirt::update()
{
    auto sitl = AP::sitl();
    if (!sitl || sitl->efi_type == SITL::EFI_TYPE_NONE) {
        return;
    }
    if (!connected) {
        connected = sock.connect("127.0.0.1", 5763);
    }
    if (!connected) {
        return;
    }
    float rpm = sitl->state.rpm[0];

    table7.rpm = rpm;
    table7.fuelload = 20;
    table7.dwell = 2.0;
    table7.baro_hPa = 1000;
    table7.map_hPa = 895;
    table7.mat_cF = 3013;
    table7.fuelPressure = 6280;
    table7.throttle_pos = 580;
    table7.ct_cF = 3940;
    table7.afr_target1 = 148;

    if (!sock.pollin(0)) {
        return;
    }

    // receive command
    while (ofs < sizeof(r_command)) {
        if (sock.recv(&buf[ofs], 1, 0) != 1) {
            break;
        }
        switch (ofs) {
        case 0:
            if (buf[ofs] == 0) {
                ofs++;
            }
            break;
        case 1:
            if (buf[ofs] != 7) {
                ofs = 0;
            } else {
                ofs++;
            }
            break;
        case 2:
            if (buf[ofs] != 0x72) {
                ofs = 0;
            } else {
                ofs++;
            }
            break;
        case 3:
            if (buf[ofs] != 0x00) {
                ofs = 0;
            } else {
                ofs++;
            }
            break;
        case 4:
            if (buf[ofs] != 0x07) {
                ofs = 0;
            } else {
                ofs++;
            }
            break;
        case 5 ... 12:
            ofs++;
            break;
        }
    }
    if (ofs >= sizeof(r_command)) {
        // check CRC
        uint32_t crc = CRC32_MS(&buf[2], sizeof(r_command)-6);
        uint32_t crc2 = be32toh(r_command.crc);
        if (crc == crc2) {
            send_table();
        } else {
            printf("BAD EFI CRC: 0x%08x 0x%08x\n", crc, crc2);
        }
        ofs = 0;
    }
}

/*
  send table response
 */
void EFI_MegaSquirt::send_table(void)
{
    uint16_t table_offset = be16toh(r_command.table_offset);
    uint16_t table_size = be16toh(r_command.table_size);

    if (table_offset >= sizeof(table7)) {
        printf("EFI_MS: bad table_offset %u\n", table_offset);
        return;
    }
    if (table_size+table_offset > sizeof(table7)) {
        table_size = sizeof(table7) - table_offset;
    }

    uint16_t len = htobe16(table_size+1);
    uint8_t outbuf[1+table_size];
    outbuf[0] = 0;
    swab(table_offset+(const uint8_t *)&table7, &outbuf[1], table_size);

    sock.send(&len, sizeof(len));
    sock.send(outbuf, sizeof(outbuf));
    uint32_t crc = htobe32(CRC32_MS(outbuf, sizeof(outbuf)));
    sock.send((const uint8_t *)&crc, sizeof(crc));
}
