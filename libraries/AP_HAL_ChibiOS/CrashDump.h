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
 */

#pragma once

#include <CrashCatcher.h>
#include <stddef.h>

bool crashdump_flash_start(const CrashCatcherInfo *info);
const CrashCatcherMemoryRegion *crashdump_flash_memory_regions(bool active);
void crashdump_flash_write(const void *memory,
                           CrashCatcherElementSizes element_size,
                           size_t element_count);
CrashCatcherReturnCodes crashdump_flash_end(CrashCatcherReturnCodes return_code,
        bool is_breakpoint);
