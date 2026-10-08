/*
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
 *
 * Code by @davidbuzz and Claude
 */

/* XBARA1 route for RC PPM hardware capture: IOMUX_XBAR_INOUT12 -> QTIMER input.
 * Without it the pad reaches no timer and capture never fires. */

#include <zephyr/init.h>
#include <zephyr/kernel.h>   /* pulls the arch sys_io accessors on Cortex-M */
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>

#if defined(CONFIG_AP_RCIN_PWM_CAPTURE)

#define RT1176_CCM_LPCG43_DIRECT  0x40CC6560UL  /* XBAR1 clock gate, see header */
#define RT1176_CCM_LPCG70_DIRECT  0x40CC68C0UL  /* QTIMER1 clock gate: LPCG 70
	(fsl_clock.h:574 kCLOCK_Qtimer1=70) at CCM_BASE+0x6000+70*0x20. The
	Zephyr pwm_mcux_qtmr driver never turns this on (the SDK's own
	CLOCK_EnableClock is compiled out under Zephyr), and with the gate
	closed every QTMR register write silently no-ops - bench-measured:
	whole channel register file read back zero after driver init. */
#define RT1176_XBARA1_SEL69       0x4003C08AUL  /* selects for outputs 138/139 */
#define XBAR_IN_IOMUX_INOUT12     12U
#define XBAR_OUT138_LOW_BYTE_MASK 0x00FFU
/* THE LINK THE ENUM TABLES DON'T MENTION, found only in the RM's XBAR chapter:
 * the input and output enums are numbered independently, so the obvious pairing
 * is wrong. */
#define RT1176_IOMUXC_GPR_GPR12   0x400E4030UL
#define GPR12_QTIMER1_TRM0_INPUT_SEL_XBAR (1UL << 8)

static int rt1176_rcin_xbar_fixup(void)
{
	/* clock the crossbar and QTIMER1 before touching either (the
	   readback check below catches a still-gated crossbar) */
	sys_write32(sys_read32(RT1176_CCM_LPCG43_DIRECT) | 1U,
		    RT1176_CCM_LPCG43_DIRECT);
	sys_write32(sys_read32(RT1176_CCM_LPCG70_DIRECT) | 1U,
		    RT1176_CCM_LPCG70_DIRECT);

	/* QTIMER1 TMR0 input source = crossbar, not the pad daisy */
	sys_write32(sys_read32(RT1176_IOMUXC_GPR_GPR12) |
		    GPR12_QTIMER1_TRM0_INPUT_SEL_XBAR,
		    RT1176_IOMUXC_GPR_GPR12);

	uint16_t sel = sys_read16(RT1176_XBARA1_SEL69);
	sel = (uint16_t)((sel & ~XBAR_OUT138_LOW_BYTE_MASK) | XBAR_IN_IOMUX_INOUT12);
	sys_write16(sel, RT1176_XBARA1_SEL69);

	const uint16_t verify = sys_read16(RT1176_XBARA1_SEL69);
	if ((verify & XBAR_OUT138_LOW_BYTE_MASK) != XBAR_IN_IOMUX_INOUT12) {
		printk("RCIN_XBAR: route IN12->OUT138 FAILED (SEL69=0x%04x)\n", verify);
		return -EIO;
	}
	printk("RCIN_XBAR: IN12 -> QTIMER1_TIMER0 routed (SEL69=0x%04x)\n", verify);
	return 0;
}

/* APPLICATION level: after pinctrl/clock init, before RCInput::init() runs
   from the AP startup path. */
SYS_INIT(rt1176_rcin_xbar_fixup, APPLICATION, 41);

#endif /* CONFIG_AP_RCIN_PWM_CAPTURE */
