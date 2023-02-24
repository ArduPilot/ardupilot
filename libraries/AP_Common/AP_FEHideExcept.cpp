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
/*
 *  AP_FEHideExcept.cpp - hide floating point exceptions
 *
 *  Links
 *
 *  https://stackoverflow.com/questions/37819235/how-do-you-enable-floating-point-exceptions-for-clang-in-os-x
 *  http://www-personal.umich.edu/~williams/archive/computation/fe-handling-example.c
 *  https://android.googlesource.com/platform/bionic/+/a147a1d/libm/arm64/fenv.c
 *
 */

#include "AP_FEHideExcept.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

FEHideExcept::FEHideExcept()
{
    feholdexcept(&envp);
}

FEHideExcept::~FEHideExcept()
{
    feclearexcept(FE_ALL_EXCEPT);
    feupdateenv(&envp);
}

#endif  // CONFIG_HAL_BOARD
