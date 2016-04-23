/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Bryan Newbold.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file boards.h
 * @author Bryan Newbold <bnewbold@leaflabs.com>,
 *         Marti Bolivar <mbolivar@leaflabs.com>
 * @brief Board-specific pin information.
 *
 * To add a new board type, add a new pair of files to
 * /wirish/boards/, update the section below with a new "BOARD" type,
 * and update /wirish/rules.mk to include your boards/your_board.cpp
 * file in the top-level Makefile build.
 */

#ifndef _BOARDS_H_
#define _BOARDS_H_

#include "hal_types.h"

#include "wirish_types.h"

/* Set of all possible pin names; not all boards have all these (note
 * that we use the Dx convention since all of the Maple's pins are
 * "digital" pins (e.g. can be used with digitalRead() and
 * digitalWrite()), but not all of them are connected to ADCs. */
enum {
    D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15, D16,
    D17, D18, D19, D20, D21, D22, D23, D24, D25, D26, D27, D28, D29, D30, D31,
    D32, D33, D34, D35, D36, D37, D38, D39, D40, D41, D42, D43, D44, D45, D46,
    D47, D48, D49, D50, D51, D52, D53, D54, D55, D56, D57, D58, D59, D60, D61,
    D62, D63, D64, D65, D66, D67, D68, D69, D70, D71, D72, D73, D74, D75, D76,
    D77, D78, D79, D80, D81, D82, D83, D84, D85, D86, D87, D88, D89, D90, D91,
    D92, D93, D94, D95, D96, D97, D98, D99, D100, D101, D102, D103, D104, D105,
    D106, D107, D108, D109, D110, D111, };

/**
 * @brief Maps each Maple pin to a corresponding stm32_pin_info.
 * @see stm32_pin_info
 */
extern const stm32_pin_info PIN_MAP[];

/**
 * @brief Pins capable of PWM output.
 *
 * Its length is BOARD_NR_PWM_PINS.
 */
extern const uint8 boardPWMPins[];

/**
 * @brief Array of pins capable of analog input.
 *
 * Its length is BOARD_NR_ADC_PINS.
 */
extern const uint8 boardADCPins[];

/**
 * @brief Pins which are connected to external hardware.
 *
 * For example, on Maple boards, it always at least includes
 * BOARD_LED_PIN.  Its length is BOARD_NR_USED_PINS.
 */
extern const uint8 boardUsedPins[];

/**
 * @brief Generic board initialization function.
 *
 * This function is called before main().  It ensures that the clocks
 * and peripherals are configured properly for use with wirish, then
 * calls boardInit().
 *
 * @see boardInit()
 */
void init(void);

/**
 * @brief Board-specific initialization function.
 *
 * This function is called from init() after all generic board
 * initialization has been performed.  Each board is required to
 * define its own.
 *
 * @see init()
 */
extern void boardInit(void);

/**
 * @brief Test if a pin is used for a special purpose on your board.
 * @param pin Pin to test
 * @return true if the given pin is in boardUsedPins, and false otherwise.
 * @see boardUsedPins
 */
//bool boardUsesPin(uint8 pin);

/* Include the appropriate private header from boards/: */

/* FIXME HACK put boards/ before these paths once IDE uses make. */

#if defined(BOARD_laserlab_MP32V1F1)
#include "laserlab_MP32V1F1.h"
#elif defined(BOARD_laserlab_MP32V3F1)
#include "laserlab_MP32V3F1.h"
#elif defined(BOARD_laserlab_MP32V1F4)
#include "laserlab_MP32V1F4.h"
#elif defined(BOARD_revomini_MP32V1F4)
#include "revomini_MP32V1F4.h"
#else
/*
 * TODO turn this into a warning so people can:
 *
 * #include "my_board_config.h"
 * #include "wirish.h"
 *
 * This will enable third-party board support without requiring that
 * anybody hack around in libmaple itself.
 */
#error "Board type has not been selected correctly."
#endif

/* Set derived definitions */

#define CLOCK_SPEED_MHZ                 CYCLES_PER_MICROSECOND
#define CLOCK_SPEED_HZ                  (CLOCK_SPEED_MHZ * 1000000UL)

#endif
