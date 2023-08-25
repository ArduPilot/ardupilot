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

#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_BATTERY_BALANCE

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

#ifndef AP_PERIPH_BATTERY_BALANCE_NUMCELLS_DEFAULT
#define AP_PERIPH_BATTERY_BALANCE_NUMCELLS_DEFAULT 0
#endif

#ifndef AP_PERIPH_BATTERY_BALANCE_RATE_DEFAULT
#define AP_PERIPH_BATTERY_BALANCE_RATE_DEFAULT 1
#endif

#ifndef AP_PERIPH_BATTERY_BALANCE_CELL1_PIN_DEFAULT
#define AP_PERIPH_BATTERY_BALANCE_CELL1_PIN_DEFAULT 1
#endif

#ifndef AP_PERIPH_BATTERY_BALANCE_ID_DEFAULT
#define AP_PERIPH_BATTERY_BALANCE_ID_DEFAULT 0
#endif


const AP_Param::GroupInfo BattBalance::var_info[] {
    // @Param: _NUM_CELLS
    // @DisplayName: Number of battery cells
    // @Description: Number of battery cells to monitor
    // @Range: 0 64
    AP_GROUPINFO("_NUM_CELLS", 1, BattBalance, num_cells, AP_PERIPH_BATTERY_BALANCE_NUMCELLS_DEFAULT),

    // @Param: _ID
    // @DisplayName: Battery ID
    // @Description: Battery ID to match against other batteries
    // @Range: 0 127
    AP_GROUPINFO("_ID", 2, BattBalance, id, AP_PERIPH_BATTERY_BALANCE_ID_DEFAULT),
        
    // @Param: _RATE
    // @DisplayName: Send Rate
    // @Description: Rate to send cell information
    // @Range: 0 20
    AP_GROUPINFO("_RATE", 3, BattBalance, rate, AP_PERIPH_BATTERY_BALANCE_RATE_DEFAULT),

    // @Param: _CELL1_PIN
    // @DisplayName: First analog pin
    // @Description: Analog pin of the first cell. Later cells must be sequential
    // @Range: 0 127
    AP_GROUPINFO("_CELL1_PIN", 4, BattBalance, cell1_pin, AP_PERIPH_BATTERY_BALANCE_CELL1_PIN_DEFAULT),
        
    AP_GROUPEND
};

BattBalance::BattBalance(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Periph_FW::batt_balance_update()
{
    const int8_t ncell = battery_balance.num_cells;
    if (ncell <= 0) {
        return;
    }

    // allocate cell sources if needed
    if (battery_balance.cells == nullptr) {
        battery_balance.cells = new AP_HAL::AnalogSource*[ncell];
        if (battery_balance.cells == nullptr) {
            return;
        }
        battery_balance.cells_allocated = ncell;
        for (uint8_t i=0; i<battery_balance.cells_allocated; i++) {
            battery_balance.cells[i] = hal.analogin->channel(battery_balance.cell1_pin + i);
        }
    }

    const uint32_t now = AP_HAL::millis();
    if (now - battery_balance.last_send_ms < 1000.0/battery_balance.rate.get()) {
        return;
    }
    battery_balance.last_send_ms = now;

    // allocate space for the packet. This is a large
    // packet that won't fit on the stack, so dynamically allocate
    auto *pkt = new ardupilot_equipment_power_BatteryInfoAux;
    uint8_t *buffer = new uint8_t[ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_MAX_SIZE];
    if (pkt == nullptr || buffer == nullptr) {
        delete pkt;
        return;
    }

    pkt->voltage_cell.len = battery_balance.cells_allocated;
    float last_cell = 0;
    for (uint8_t i=0; i<battery_balance.cells_allocated; i++) {
        auto *chan = battery_balance.cells[i];
        if (chan == nullptr) {
            continue;
        }
        const float v = chan->voltage_average();
        pkt->voltage_cell.data[i] = v - last_cell;
        last_cell = v;
    }
    pkt->max_current = nanf("");
    pkt->nominal_voltage = nanf("");
    pkt->battery_id = uint8_t(battery_balance.id);

    // encode and send message:
    const uint16_t total_size = ardupilot_equipment_power_BatteryInfoAux_encode(pkt, buffer, !periph.canfdout());

    canard_broadcast(ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_SIGNATURE,
                     ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     buffer,
                     total_size);

    delete pkt;
    delete buffer;
}

#endif  // HAL_PERIPH_ENABLE_BATTERY_BALANCE

