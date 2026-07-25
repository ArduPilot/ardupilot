/*
    ChibiOS - Copyright (C) 2016..2018 Rocco Marco Guglielmi

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    ex.h
 * @brief   EX main include file.
 *
 * @addtogroup EX_INFO
 * @details EX related info.
 * @{
 */

#ifndef EX_H
#define EX_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @brief   ChibiOS/EX identification macro.
 */
#define __CHIBIOS_EX__

/**
 * @brief   Stable release flag.
 */
#define CH_EX_STABLE            0

/**
 * @name    ChibiOS/EX version identification
 * @{
 */
/**
 * @brief   EX version string.
 */
#define CH_EX_VERSION           "1.3.0"

/**
 * @brief   EX version major number.
 */
#define CH_EX_MAJOR             1

/**
 * @brief   EX version minor number.
 */
#define CH_EX_MINOR             3

/**
 * @brief   EX version patch number.
 */
#define CH_EX_PATCH             0
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Late inclusions.                                                          */
/*===========================================================================*/

#include "ex_sensors.h"
#include "ex_accelerometer.h"
#include "ex_barometer.h"
#include "ex_compass.h"
#include "ex_displays.h"
#include "ex_gyroscope.h"
#include "ex_hygrometer.h"
#include "ex_rangefinder.h"
#include "ex_thermometer.h"

#endif /* EX_H */

/** @} */
