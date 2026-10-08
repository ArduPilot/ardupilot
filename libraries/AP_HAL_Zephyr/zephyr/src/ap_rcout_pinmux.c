/*
 * Pad-ownership switching for FMU_CH1-8 (the ap_rcout_mux arbiter node in
 * the board DTS): PWM (FlexPWM alternates) vs DShot (FlexIO alternates),
 * applied at runtime by RCOutput.cpp per the ArduPilot output parameters.
 *
 * A C file, not C++, deliberately: PINCTRL_DT_DEFINE expands the NXP SoC's
 * pinctrl_soc.h initializers, which narrow an int constant into uint32_t -
 * fine in the C driver files those macros were written for, a hard
 * -Wnarrowing error in a C++ TU (measured 2026-08-11: compiling this from
 * RCOutput.cpp failed exactly there). Same pattern as
 * rt1176_gpio_mux_fixup.c / rt1176_romapi_flash.c in this directory.
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

#include <zephyr/devicetree.h>

#if DT_NODE_EXISTS(DT_NODELABEL(ap_rcout_mux))

/* Custom pinctrl state id for the arbiter node's "dshot" state, because Zephyr
 * has no standard state name for a second alternate on the same pads. */
#define PINCTRL_STATE_DSHOT 2U

#include <zephyr/drivers/pinctrl.h>

#include "ap_rcout_pinmux.h"

/* PINCTRL_DT_DEFINE emits static (TU-local) objects on this build (no
 * CONFIG_PINCTRL_DYNAMIC), so this cannot collide with any driver's own
 * pinctrl definitions. */
PINCTRL_DT_DEFINE(DT_NODELABEL(ap_rcout_mux));

int ap_rcout_pinmux_apply_pwm(void)
{
	return pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(ap_rcout_mux)),
				   PINCTRL_STATE_DEFAULT);
}

int ap_rcout_pinmux_apply_dshot(void)
{
	return pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(ap_rcout_mux)),
				   PINCTRL_STATE_DSHOT);
}

#endif /* ap_rcout_mux */
