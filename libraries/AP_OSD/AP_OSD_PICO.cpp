/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_OSD_PICO.h"

#if AP_OSD_PICO_ENABLED

#include <GCS_MAVLink/GCS.h>

/*
  The PIO1 broker decides between this and the LED driver by reading OSD_TYPE
  directly, because it has to answer before AP_OSD exists - RCOutput asks
  during init_ardupilot(). It cannot include this header without the HAL
  depending on AP_OSD, so the value is duplicated there and pinned here.
 */
static_assert((int)AP_OSD::osd_types::OSD_PICO == 6,
              "RP2350_pio1.cpp hardcodes OSD_TYPE 6 for the PIO OSD");

AP_OSD_Backend *AP_OSD_PICO::probe(AP_OSD &osd)
{
    AP_OSD_PICO *backend = NEW_NOTHROW AP_OSD_PICO(osd);
    if (backend == nullptr) {
        return nullptr;
    }
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

bool AP_OSD_PICO::init(void)
{
    /*
      Font first, then the hardware. The other way round, a font failure
      returns false, probe() deletes the backend, and PIO1, a DMA channel and
      26 KB of heap stay claimed by an object that no longer exists.

      load_font_data() tries the microSD before ROMFS and falls back to font 0
      if the requested one is missing. The layout is the same 54 bytes per
      character the MAX7456 wants; only the pixel encoding differs, and the
      driver converts through a lookup table.
     */
    FileData *fd = load_font_data(_osd.font_num);
    if (fd == nullptr) {
        return false;
    }
    if (fd->length != 256UL * OSD_PICO_GLYPH_BYTES) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "OSD: font is %u bytes, wanted %u",
                      unsigned(fd->length), unsigned(256UL * OSD_PICO_GLYPH_BYTES));
        delete fd;
        return false;
    }
    /*
      PAL is only the answer when no camera is attached at boot: init()
      counts fields and switches to NTSC if that is what is arriving.
     */
    if (!driver.init(true)) {
        delete fd;
        return false;
    }
    /*
      fd is deliberately not freed: the driver blits straight out of it rather
      than keeping a converted copy, so it has to outlive init(). 13.8 KB of
      ROMFS held for the life of the boot.
     */
    driver.set_font(fd->data);
    driver.clear();
    driver.flush();
    return true;
}

void AP_OSD_PICO::write(uint8_t x, uint8_t y, const char *text)
{
    driver.write(x, y, text);
}

void AP_OSD_PICO::clear(void)
{
    AP_OSD_Backend::clear();
    driver.clear();
}

void AP_OSD_PICO::flush(void)
{
    driver.flush();
}

#endif  // AP_OSD_PICO_ENABLED
