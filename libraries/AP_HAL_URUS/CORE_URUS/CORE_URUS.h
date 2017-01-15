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
 *  Synthetic Hardware Abstraction layer (SHAL) for URUS System.
 *  Author: Hiroshi Takey, November 2016
 *
 *  URUS CORE SHAL was based and adapted from the TOP APM HAL
 */

#pragma once

#include "CORE_URUS_NAMESPACE.h"

#include "CoreUrusTimers.h"
#include "CoreUrusScheduler.h"

#include <stdint.h>

class NSCORE_URUS::CLCORE_URUS {
public:

    CLCORE_URUS(NSCORE_URUS::CLCoreUrusTimers* _timers)
    :
    timers(_timers)
    {}

    virtual void init_core() const = 0;

    NSCORE_URUS::CLCoreUrusTimers*  timers;
};

