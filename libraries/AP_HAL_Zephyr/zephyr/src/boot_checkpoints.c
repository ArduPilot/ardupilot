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
  Boot-stage checkpoint tracing: one line per SYS_INIT level, so the last line
  printed names the level that hung. Written to find an RT1176 early-boot
  crash-reset loop that logging could not report, because CONFIG_LOG_MODE_DEFERRED
  needs a log thread to flush and a board that resets first flushes nothing.

  Diagnostic only, off by default - see CONFIG_AP_BOOT_CHECKPOINTS.
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include "ap_diag_console.h"

/* In the BOOTLOADER image these checkpoints are suppressed: both images compile
 * this file, so every capture otherwise showed the block twice. */
#ifdef AP_ZEPHYR_BOOTLOADER_BUILD
#define CKPT_PUTS(s) do { } while (0)
#else
#define CKPT_PUTS(s) ap_diag_puts(s)
#endif

static int checkpoint_a(void)
{
	CKPT_PUTS("CKPT_A pre_kernel_2/0");
	return 0;
}
/* PRE_KERNEL_2/1, not /0: ap_diag_console_init() takes /0 and must run first. */
SYS_INIT(checkpoint_a, PRE_KERNEL_2, 1);

static int checkpoint_b(void)
{
	CKPT_PUTS("CKPT_B pre_kernel_2/99");
	return 0;
}
SYS_INIT(checkpoint_b, PRE_KERNEL_2, 99);

#define POST_KERNEL_CKPT(name, prio) \
	static int name(void) { CKPT_PUTS("CKPT_" #name " post_kernel/" #prio); return 0; } \
	SYS_INIT(name, POST_KERNEL, prio)

POST_KERNEL_CKPT(ckpt_pk00, 0);
POST_KERNEL_CKPT(ckpt_pk10, 10);
POST_KERNEL_CKPT(ckpt_pk20, 20);
POST_KERNEL_CKPT(ckpt_pk30, 30);
POST_KERNEL_CKPT(ckpt_pk40, 40);
POST_KERNEL_CKPT(ckpt_pk45, 45);
POST_KERNEL_CKPT(ckpt_pk50, 50);
POST_KERNEL_CKPT(ckpt_pk55, 55);
POST_KERNEL_CKPT(ckpt_pk60, 60);
POST_KERNEL_CKPT(ckpt_pk70, 70);
POST_KERNEL_CKPT(ckpt_pk80, 80);
POST_KERNEL_CKPT(ckpt_pk90, 90);
POST_KERNEL_CKPT(ckpt_pk99, 99);

static int checkpoint_e(void)
{
	CKPT_PUTS("CKPT_E application/0");
	return 0;
}
SYS_INIT(checkpoint_e, APPLICATION, 0);
