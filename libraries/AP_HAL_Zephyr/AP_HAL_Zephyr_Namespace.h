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

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_DSP
#include "DSP.h"
#else
#include <AP_HAL_Empty/DSP.h>
#endif
#include <AP_HAL_Empty/Flash.h>
#include <AP_HAL_Empty/OpticalFlow.h>
#include <AP_HAL_Empty/WSPIDevice.h>

#include "AnalogIn.h"
#include "CANIface.h"
#include "GPIO.h"
#include "I2CDevice.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "SPIDevice.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "Util.h"

namespace Zephyr {
/* DSP.h defines Zephyr::DSP directly (a real CMSIS-DSP-backed
   implementation) when HAL_WITH_DSP is set - only alias to the no-op
   Empty:: stub when it isn't, so a board that flips HAL_WITH_DSP on can't
   silently end up with the empty stub instantiated under a "real"-looking
   name (see HAL_Zephyr_Class.cpp's `static DSP dspInstance;`). */
#if !HAL_WITH_DSP
using DSP = Empty::DSP;
#endif
using Flash = Empty::Flash;
using OpticalFlow = Empty::OpticalFlow;
using WSPIDevice = Empty::WSPIDevice;
using WSPIDeviceManager = Empty::WSPIDeviceManager;
}  // namespace Zephyr
