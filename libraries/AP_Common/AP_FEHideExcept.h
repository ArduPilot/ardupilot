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
///
/// @file   AP_FEHideExcept.h
/// @brief  Hide floating point exceptions
///

#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <fenv.h>

/// \brief Class to temporarily hide floating point exceptions
class FEHideExcept
{
public:
    FEHideExcept();
    ~FEHideExcept();

    // no-copy
    FEHideExcept(const FEHideExcept &) = delete;
    FEHideExcept(const FEHideExcept &&) = delete;
    FEHideExcept& operator=(const FEHideExcept &) = delete;
    FEHideExcept& operator=(const FEHideExcept &&) = delete;

private:
    fenv_t envp;
};

#endif  // CONFIG_HAL_BOARD
