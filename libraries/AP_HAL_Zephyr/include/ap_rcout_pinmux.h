/*
 * FMU_CH1-8 pad-ownership switching (PWM vs DShot) - see
 * zephyr/src/ap_rcout_pinmux.c for why this is a C-side facility and the
 * ap_rcout_mux node in the board DTS for the arbitration model.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Mux the FMU_CH1-8 pad bank to its FlexPWM alternates (the boot/default
 * state). Returns 0 on success (pinctrl_apply_state's own convention). */
int ap_rcout_pinmux_apply_pwm(void);

/* Mux the FMU_CH1-8 pad bank to its FlexIO alternates for DShot. */
int ap_rcout_pinmux_apply_dshot(void);

#ifdef __cplusplus
}
#endif
