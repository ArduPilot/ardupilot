/*
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
/* i.MX RT1176: route pads muxed to GPIO_MUXn back to the standard GPIOn
 * controller. Without this the SPI chip selects land on a controller nothing
 * drives, so every transfer completes with nothing on the bus. */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>

#if defined(CONFIG_SOC_MIMXRT1176_CM7) || defined(CONFIG_SOC_SERIES_IMXRT11XX)

/* AP_ prefix is REQUIRED, not cosmetic: <zephyr/kernel.h> transitively defines
 * the unprefixed names, and the collision is silent. */
#define AP_IOMUXC_GPR_BASE 0x400E4000U  // IOMUXC_GPR block, RT1176 RM
#define AP_IOMUXC_GPR(n)   (AP_IOMUXC_GPR_BASE + ((n) * 4U))

/* GPR40..GPR43: the GPIO_MUX2 / GPIO_MUX3 instance-selection registers. */
#define GPR_GPIO_MUX2_SEL_LOW   40
#define GPR_GPIO_MUX2_SEL_HIGH  41
#define GPR_GPIO_MUX3_SEL_LOW   42
#define GPR_GPIO_MUX3_SEL_HIGH  43

/* Kept as globals so the pre-fix values stay readable over SWD after boot -
 * this bug is invisible from the console, so leave the evidence behind. */
uint32_t g_gpio_mux_sel_before[4];
uint32_t g_gpio_mux_sel_after[4];

static const uint8_t gpio_mux_sel_regs[4] = {
	GPR_GPIO_MUX2_SEL_LOW,
	GPR_GPIO_MUX2_SEL_HIGH,
	GPR_GPIO_MUX3_SEL_LOW,
	GPR_GPIO_MUX3_SEL_HIGH,
};

/* Exposed so the boot SPI diagnostic can re-apply this and print the before/
 * after values to the console. The bug is otherwise completely invisible: it
 * produces a healthy-looking SPI bus whose chip select simply never moves. */
void rt1176_gpio_mux_apply(uint32_t *before, uint32_t *after)
{
	for (int i = 0; i < 4; i++) {
		uint32_t b = sys_read32(AP_IOMUXC_GPR(gpio_mux_sel_regs[i]));

		sys_write32(0U, AP_IOMUXC_GPR(gpio_mux_sel_regs[i]));

		uint32_t a = sys_read32(AP_IOMUXC_GPR(gpio_mux_sel_regs[i]));

		if (before != NULL) {
			before[i] = b;
		}
		if (after != NULL) {
			after[i] = a;
		}
	}
}

static int rt1176_gpio_mux_fixup(void)
{
	rt1176_gpio_mux_apply(g_gpio_mux_sel_before, g_gpio_mux_sel_after);
	return 0;
}

/* PRE_KERNEL_2 rather than PRE_KERNEL_1, so it runs after the pinctrl the pads
 * are otherwise given. */
SYS_INIT(rt1176_gpio_mux_fixup, PRE_KERNEL_2, 1);

/* SECOND, LATE APPLICATION - REQUIRED, NOT BELT-AND-BRACES: something between
 * PRE_KERNEL_2 and APPLICATION re-muxes these pads, so the fixup must run twice. */
static int rt1176_gpio_mux_fixup_late(void)
{
	rt1176_gpio_mux_apply(NULL, NULL);
	return 0;
}

SYS_INIT(rt1176_gpio_mux_fixup_late, APPLICATION, 40);

#endif /* RT1176 */
