/* PS3 Teensy HID Gamepad
 * Copyright (C) 2010 Josh Kropf <josh@slashdev.ca>
 *
 * Based on works by:
 *   grunskis <http://github.com/grunskis/gamepad>
 *   Toodles <http://forums.shoryuken.com/showthread.php?t=131230>
 *
 *  Modified to use with Joypad Interface Driver for URUS and APM.
 *  Copyright (c) 2017-2018 Hiroshi Takey <htakey@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* USB Keyboard Example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_keyboard.html
 * Copyright (c) 2009 PJRC.COM, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#if (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_APM16U)

#include <stdint.h>
#include <avr/pgmspace.h>

#pragma pack(push, 1)
#include "usb_gamepad_def_shared.h"
#pragma pack(pop)

const usb_string_descriptor_struct string0 PROGMEM = {
	4,
	3,
	{0x0409}
};

const usb_string_descriptor_struct string1 PROGMEM = {
	sizeof(STR_MANUFACTURER),
	3,
	STR_MANUFACTURER
};

const usb_string_descriptor_struct string2 PROGMEM = {
	sizeof(STR_PRODUCT),
	3,
	STR_PRODUCT
};


#endif
