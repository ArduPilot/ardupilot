/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    chlicense.h
 * @brief   License Module macros and structures.
 *
 * @addtogroup chibios_license
 * @details This module contains all the definitions required for defining
 *          a licensing scheme for customers or public releases.
 * @{
 */

#ifndef CHLICENSE_H
#define CHLICENSE_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name   Allowed Features Levels
 * @{
 */
#define CH_FEATURES_BASIC                   0
#define CH_FEATURES_INTERMEDIATE            1
#define CH_FEATURES_FULL                    2
/** @} */

/**
 * @name    Deployment Options
 * @{
 */
#define CH_DEPLOY_UNLIMITED                -1
#define CH_DEPLOY_NONE                      0
/** @} */

/**
 * @name    Licensing Options
 * @{
 */
#define CH_LICENSE_GPL                      0
#define CH_LICENSE_GPL_EXCEPTION            1
#define CH_LICENSE_COMMERCIAL_FREE          2
#define CH_LICENSE_COMMERCIAL_DEV_1000      3
#define CH_LICENSE_COMMERCIAL_DEV_5000      4
#define CH_LICENSE_COMMERCIAL_FULL          5
#define CH_LICENSE_COMMERCIAL_RUNTIME       6
#define CH_LICENSE_PARTNER                  7
/** @} */

#include "chversion.h"
#include "chcustomer.h"
#if CH_LICENSE == CH_LICENSE_PARTNER
#include "chpartner.h"
#endif
#if CH_LICENSE == CH_LICENSE_COMMERCIAL_RUNTIME
#include "chruntime.h"
#endif

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on chversion.h.*/
#if !defined(__CHIBIOS__)
  #error "__CHIBIOS__ not defined in chversion.h"
#endif

#if !defined(CH_VERSION_STABLE)
  #error "CH_VERSION_STABLE not defined in chversion.h"
#endif

#if !defined(CH_VERSION_YEAR)
  #error "CH_VERSION_YEAR not defined in chversion.h"
#endif

#if !defined(CH_VERSION_MONTH)
  #error "CH_VERSION_MONTH not defined in chversion.h"
#endif

#if !defined(CH_VERSION_PATCH)
  #error "CH_VERSION_PATCH not defined in chversion.h"
#endif

#if !defined(CH_VERSION_NICKNAME)
  #error "CH_VERSION_NICKNAME not defined in chversion.h"
#endif

#if !defined(CH_VERSION_DATE)
  #error "CH_VERSION_DATE not defined in chversion.h"
#endif

#if (CH_VERSION_STABLE < 0) || (CH_VERSION_STABLE > 1)
  #error "invalid CH_VERSION_STABLE value in chversion.h"
#endif

#if (CH_VERSION_YEAR < 12) || (CH_VERSION_YEAR > 99)
  #error "invalid CH_VERSION_YEAR value in chversion.h"
#endif

#if (CH_VERSION_MONTH < 1) || (CH_VERSION_MONTH > 12)
  #error "invalid CH_VERSION_MONTH value in chversion.h"
#endif

#if (CH_VERSION_DATE < 201201) || (CH_VERSION_DATE > 209912)
  #error "invalid CH_VERSION_DATE value in chversion.h"
#endif

/* Checks on chcustomer.h.*/
#if !defined(CH_CUSTOMER_ID_STRING)
#error "CH_CUSTOMER_ID_STRING not defined in chcustomer.h"
#endif

#if !defined(CH_CUSTOMER_ID_CODE)
#error "CH_CUSTOMER_ID_CODE not defined in chcustomer.h"
#endif

#if !defined(CH_CUSTOMER_LICENSE_EOS_DATE)
#error "CH_CUSTOMER_LICENSE_EOS_DATE not defined in chcustomer.h"
#endif

#if !defined(CH_CUSTOMER_LICENSE_VERSION_YEAR)
#error "CH_CUSTOMER_LICENSE_VERSION_YEAR not defined in chcustomer.h"
#endif

#if !defined(CH_CUSTOMER_LICENSE_VERSION_MONTH)
#error "CH_CUSTOMER_LICENSE_VERSION_MONTH not defined in chcustomer.h"
#endif

#if !defined(CH_CUSTOMER_LICENSE_VERSION_DATE)
#error "CH_CUSTOMER_LICENSE_VERSION_DATE not defined in chcustomer.h"
#endif

#if !defined(CH_LICENSE)
#error "CH_LICENSE not defined in chcustomer.h"
#endif

#if (CH_CUSTOMER_LICENSE_EOS_DATE < 201201) ||                              \
    (CH_CUSTOMER_LICENSE_EOS_DATE > 209912)
#error "invalid CH_CUSTOMER_LICENSE_EOS_DATE value in chcustomer.h"
#endif

#if (CH_CUSTOMER_LICENSE_VERSION_YEAR < 12) ||                              \
    (CH_CUSTOMER_LICENSE_VERSION_YEAR > 99)
#error "invalid CH_CUSTOMER_LICENSE_VERSION_YEAR value in chcustomer.h"
#endif

#if (CH_CUSTOMER_LICENSE_VERSION_MONTH < 1) ||                              \
    (CH_CUSTOMER_LICENSE_VERSION_MONTH > 12)
#error "invalid CH_CUSTOMER_LICENSE_VERSION_MONTH value in chcustomer.h"
#endif

#if (CH_CUSTOMER_LICENSE_VERSION_DATE < 201201) ||                          \
    (CH_CUSTOMER_LICENSE_VERSION_DATE > 209912)
  #error "invalid CH_CUSTOMER_LICENSE_VERSION_DATE value in chcustomer.h"
#endif

/* Checks on licensed versions.*/
#if CH_VERSION_DATE > CH_CUSTOMER_LICENSE_VERSION_DATE
#error "this ChibiOS version is newer than your license, see chcustomer.h"
#endif

/* Checks on end-of-support date.*/
#if CH_VERSION_DATE > CH_CUSTOMER_LICENSE_EOS_DATE
#error "this ChibiOS version is beyond your End-Of-Support date, see chcustomer.h"
#endif

/* Defaults for GPL license.*/
#if (CH_LICENSE == CH_LICENSE_GPL) || defined(__DOXYGEN__)
/**
 * @brief   License identification string.
 * @details This string identifies the license in a machine-readable
 *          format.
 */
#define CH_LICENSE_TYPE_STRING              "GNU General Public License 3 (GPL3)"

/**
 * @brief   Customer identification string.
 * @details This information is only available for registered commercial users.
 */
#define CH_LICENSE_ID_STRING                "N/A"

/**
 * @brief   Customer code.
 * @details This information is only available for registered commercial users.
 */
#define CH_LICENSE_ID_CODE                  "N/A"

/**
 * @brief   Code modifiability restrictions.
 * @details This setting defines if the source code is user-modifiable or not.
 */
#define CH_LICENSE_MODIFIABLE_CODE          TRUE

/**
 * @brief   Code functionality restrictions.
 */
#define CH_LICENSE_FEATURES                 CH_FEATURES_FULL

/**
 * @brief   Code deploy restrictions.
 * @details This is the per-core deploy limit allowed under the current
 *          license scheme.
 */
#define CH_LICENSE_MAX_DEPLOY               CH_DEPLOY_UNLIMITED

#elif CH_LICENSE == CH_LICENSE_GPL_EXCEPTION
#define CH_LICENSE_TYPE_STRING              "GNU General Public License 3 (GPL3) + Exception"
#define CH_LICENSE_ID_STRING                "N/A"
#define CH_LICENSE_ID_CODE                  "N/A"
#define CH_LICENSE_MODIFIABLE_CODE          FALSE
#define CH_LICENSE_FEATURES                 CH_FEATURES_BASIC
#define CH_LICENSE_MAX_DEPLOY               CH_DEPLOY_UNLIMITED

#elif CH_LICENSE == CH_LICENSE_COMMERCIAL_FREE
#define CH_LICENSE_TYPE_STRING              "Zero Cost Registered License for 500 Cores"
#define CH_LICENSE_ID_STRING                "N/A"
#define CH_LICENSE_ID_CODE                  "2017-0000"
#define CH_LICENSE_MODIFIABLE_CODE          FALSE
#define CH_LICENSE_FEATURES                 CH_FEATURES_INTERMEDIATE
#define CH_LICENSE_MAX_DEPLOY               500

#elif CH_LICENSE == CH_LICENSE_COMMERCIAL_DEV_1000
#define CH_LICENSE_TYPE_STRING              "Developer Commercial License for 1000 Cores"
#define CH_LICENSE_ID_STRING                CH_CUSTOMER_ID_STRING
#define CH_LICENSE_ID_CODE                  CH_CUSTOMER_ID_CODE
#define CH_LICENSE_MODIFIABLE_CODE          TRUE
#define CH_LICENSE_FEATURES                 CH_FEATURES_FULL
#define CH_LICENSE_MAX_DEPLOY               1000

#elif CH_LICENSE == CH_LICENSE_COMMERCIAL_DEV_5000
#define CH_LICENSE_TYPE_STRING              "Developer Commercial License for 5000 Cores"
#define CH_LICENSE_ID_STRING                CH_CUSTOMER_ID_STRING
#define CH_LICENSE_ID_CODE                  CH_CUSTOMER_ID_CODE
#define CH_LICENSE_MODIFIABLE_CODE          TRUE
#define CH_LICENSE_FEATURES                 CH_FEATURES_FULL
#define CH_LICENSE_MAX_DEPLOY               5000

#elif CH_LICENSE == CH_LICENSE_COMMERCIAL_FULL
#define CH_LICENSE_TYPE_STRING              "Full Commercial License for Unlimited Deployment"
#define CH_LICENSE_ID_STRING                CH_CUSTOMER_ID_STRING
#define CH_LICENSE_ID_CODE                  CH_CUSTOMER_ID_CODE
#define CH_LICENSE_MODIFIABLE_CODE          TRUE
#define CH_LICENSE_FEATURES                 CH_FEATURES_FULL
#define CH_LICENSE_MAX_DEPLOY               CH_DEPLOY_UNLIMITED

#elif CH_LICENSE == CH_LICENSE_COMMERCIAL_RUNTIME
#define CH_LICENSE_TYPE_STRING              "Runtime Commercial License"
#define CH_LICENSE_ID_STRING                CH_CUSTOMER_ID_STRING
#define CH_LICENSE_ID_CODE                  CH_CUSTOMER_ID_CODE
#define CH_LICENSE_MODIFIABLE_CODE          TRUE
#define CH_LICENSE_FEATURES                 CH_FEATURES_FULL
#define CH_LICENSE_MAX_DEPLOY               CH_RUNTIME_MAX_DEPLOY

#elif CH_LICENSE == CH_LICENSE_PARTNER
#define CH_LICENSE_TYPE_STRING              "Partners Special Commercial License"
#define CH_LICENSE_ID_STRING                CH_CUSTOMER_ID_STRING
#define CH_LICENSE_ID_CODE                  CH_CUSTOMER_ID_CODE
#define CH_LICENSE_MODIFIABLE_CODE          CH_PARTNER_MODIFIABLE_CODE
#define CH_LICENSE_FEATURES                 CH_PARTNER_FEATURES
#define CH_LICENSE_MAX_DEPLOY               CH_PARTNER_MAX_DEPLOY

#else
#error "invalid licensing option"
#endif

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
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* CHLICENSE_H */

/** @} */
