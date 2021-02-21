/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// minimal header for checking if AP_FS is available

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if HAL_OS_POSIX_IO || HAL_OS_FATFS_IO
#define HAVE_FILESYSTEM_SUPPORT 1
#else
#define HAVE_FILESYSTEM_SUPPORT 0
#endif
