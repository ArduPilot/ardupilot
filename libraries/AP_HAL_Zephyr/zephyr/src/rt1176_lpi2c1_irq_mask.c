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
/*
  RT1176: mask LPI2C1's interrupt before its driver enables it.

  LPI2C1 (irq 32, see nxp_rt11xx.dtsi) enables its interrupt at the end of its
  POST_KERNEL/50 init. That bus goes to the GPS1 connector, unconnected on the
  bench, and floating SDA/SCL triggers a spurious interrupt during init that
  hangs the boot. Masking at PRE_KERNEL_1/99 gets in first.

  This was previously unguarded in uart_probe_diag.c and so ran `irq_disable(32)`
  on every SoC we build - a different interrupt entirely on STM32H7 and Xtensa.
  It is now compiled only for the RT11xx series.
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#define LPI2C1_IRQ 32

static int mask_lpi2c1_irq_early(void)
{
	irq_disable(LPI2C1_IRQ);
	return 0;
}
SYS_INIT(mask_lpi2c1_irq_early, PRE_KERNEL_1, 99);

/* Force z_impl_k_busy_wait's object into the link: with LPI2C1 disabled nothing
   else references it and the link fails. Carried over from the checkpoint that
   used to host it, which is now optional. */
static int force_busy_wait_link(void)
{
	k_busy_wait(1);
	return 0;
}
SYS_INIT(force_busy_wait_link, APPLICATION, 0);
