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
/* i.MX RT1176: fix eDMA0's Group0-vs-Group1 fixed-priority arbitration. The
 * reset default starves Group1 entirely, so channels there never win the bus. */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>

#if defined(CONFIG_SOC_MIMXRT1176_CM7) || defined(CONFIG_SOC_SERIES_IMXRT11XX)

#define AP_EDMA0_BASE 0x40070000U  // edma0 dma-controller, matches the board DTS `reg`
#define AP_EDMA0_CR   (AP_EDMA0_BASE + 0x0U)  // CR: Control register, offset 0x0

#define AP_DMA_CR_ERGA_MASK 0x8U  // ERGA: Enable Round Robin Group Arbitration (bit 3)

/* Kept as globals so the before/after values stay readable over SWD after
 * boot - this fix is otherwise invisible from the console. */
uint32_t g_edma0_cr_before;
uint32_t g_edma0_cr_after;

static int rt1176_edma_arbitration_fixup(void)
{
	uint32_t cr = sys_read32(AP_EDMA0_CR);

	g_edma0_cr_before = cr;

	sys_write32(cr | AP_DMA_CR_ERGA_MASK, AP_EDMA0_CR);

	g_edma0_cr_after = sys_read32(AP_EDMA0_CR);

	return 0;
}

SYS_INIT(rt1176_edma_arbitration_fixup, POST_KERNEL, 0);

#endif /* RT1176 */
