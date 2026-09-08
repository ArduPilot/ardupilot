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
#pragma once

#include <stdint.h>

/*
  Synchronous console output via raw uart_poll_out(): no logging subsystem, no
  log thread, no buffering. Usable from PRE_KERNEL_1 onwards and from a fault
  handler, where CONFIG_LOG_MODE_DEFERRED output would be lost because nothing
  ever runs to flush it. Owned by ap_fault_handler.c.
 */
#ifdef __cplusplus
extern "C" {
#endif

void ap_diag_puts(const char *s);
void ap_diag_puthex(const char *label, uint32_t val);

#ifdef __cplusplus
}
#endif
