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
  serial options support, for serial over DroneCAN
 */

#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_SERIAL_OPTIONS

#include "serial_options.h"

SerialOptionsDev::SerialOptionsDev(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

const AP_Param::GroupInfo SerialOptionsDev::var_info[] {

    // @Param: OPTIONS
    // @DisplayName: Serial options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    AP_GROUPINFO("OPTIONS", 1, SerialOptionsDev, options, 0),

    // @Param: RTSCTS
    // @DisplayName: Serial1 flow control
    // @Description: Enable flow control. You must have the RTS and CTS pins available on the port. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    AP_GROUPINFO("RTSCTS",  2, SerialOptionsDev, rtscts, float(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE)),

    AP_GROUPEND
};

#endif  // HAL_PERIPH_ENABLE_SERIAL_OPTIONS
