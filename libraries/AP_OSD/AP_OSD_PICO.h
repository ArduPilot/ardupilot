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
/*
 * Analog OSD on RP2350, overlaid on the camera feed by PIO rather than by a
 * MAX7456. The scan-out lives in AP_HAL_ChibiOS/OSD_pico; this is the
 * character layer, which is all AP_OSD needs to know about.
 */
#pragma once

#include "AP_OSD_config.h"
#include <AP_OSD/AP_OSD_Backend.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/OSD_pico.h>
#endif

#ifndef AP_OSD_PICO_ENABLED
#define AP_OSD_PICO_ENABLED 0
#endif

#if AP_OSD_PICO_ENABLED

class AP_OSD_PICO : public AP_OSD_Backend
{
public:
    static AP_OSD_Backend *probe(AP_OSD &osd);

    void write(uint8_t x, uint8_t y, const char *text) override;
    bool init() override;
    void flush() override;
    void clear() override;

    /*
      Asks whether a second backend of the given type can run alongside this
      one, so it is false for the ones that do the same job. PICO is another
      bitmap character overlay, like MAX7456 and SITL, so those three are
      mutually exclusive; the MSP backends drive a different display and can
      coexist.
     */
    bool is_compatible_with_backend_type(AP_OSD::osd_types type) const override {
        switch (type) {
        case AP_OSD::osd_types::OSD_PICO:
        case AP_OSD::osd_types::OSD_MAX7456:
        case AP_OSD::osd_types::OSD_SITL:
            return false;
        case AP_OSD::osd_types::OSD_NONE:
        case AP_OSD::osd_types::OSD_TXONLY:
        case AP_OSD::osd_types::OSD_MSP:
        case AP_OSD::osd_types::OSD_MSP_DISPLAYPORT:
            return true;
        }
        return false;
    }

    AP_OSD::osd_types get_backend_type() const override {
        return AP_OSD::osd_types::OSD_PICO;
    }

private:
    using AP_OSD_Backend::AP_OSD_Backend;

    ChibiOS::OSD_pico driver;
};

#endif  // AP_OSD_PICO_ENABLED
