/*
 * Veto WFI in Zephyr's idle thread.
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

/* On mr_vmu_rt1176 a WFI gates the whole CM7 clock domain - SysTick and DWT both
 * freeze - so the kernel clock stops whenever all threads sleep. Veto the WFI. */

#include <stdbool.h>

bool z_arm_on_enter_cpu_idle(void)
{
	return false;
}
