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
 * @file    chcustomer.h
 * @brief   Customer-related info.
 *
 * @addtogroup chibios_customer
 * @details This module incapsulates licensee information, this is only
 *          meaningful for commercial licenses. It is a stub for public
 *          releases.
 * @{
 */

#ifndef CHCUSTOMER_H
#define CHCUSTOMER_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Customer readable identifier.
 */
#define CH_CUSTOMER_ID_STRING               "Santa, North Pole"

/**
 * @brief   Customer code.
 */
#define CH_CUSTOMER_ID_CODE                 "xxxx-yyyy"

/**
 * @brief   End-Of-Support date (yyyymm).
 */
#define CH_CUSTOMER_LICENSE_EOS_DATE        209912

/**
 * @brief   Licensed branch year.
 */
#define CH_CUSTOMER_LICENSE_VERSION_YEAR    99

/**
 * @brief   Licensed branch month.
 */
#define CH_CUSTOMER_LICENSE_VERSION_MONTH   12

/**
 * @brief   Current license.
 * @note    This setting is reserved to the copyright owner.
 * @note    Changing this setting invalidates the license.
 * @note    The license statement in the source headers is valid, applicable
 *          and binding regardless this setting.
 */
#define CH_LICENSE                          CH_LICENSE_GPL

/**
 * @name    Licensed Products
 * @{
 */
#define CH_CUSTOMER_LIC_RT                  TRUE
#define CH_CUSTOMER_LIC_NIL                 TRUE
#define CH_CUSTOMER_LIC_OSLIB               TRUE
#define CH_CUSTOMER_LIC_EX                  TRUE
#define CH_CUSTOMER_LIC_SB                  TRUE
#define CH_CUSTOMER_LIC_PORT_CM0            TRUE
#define CH_CUSTOMER_LIC_PORT_CM3            TRUE
#define CH_CUSTOMER_LIC_PORT_CM4            TRUE
#define CH_CUSTOMER_LIC_PORT_CM7            TRUE
#define CH_CUSTOMER_LIC_PORT_CM33           TRUE
#define CH_CUSTOMER_LIC_PORT_CM55           TRUE
#define CH_CUSTOMER_LIC_PORT_ARM79          TRUE
#define CH_CUSTOMER_LIC_PORT_E200Z0         TRUE
#define CH_CUSTOMER_LIC_PORT_E200Z2         TRUE
#define CH_CUSTOMER_LIC_PORT_E200Z3         TRUE
#define CH_CUSTOMER_LIC_PORT_E200Z4         TRUE
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/**
 * @brief   Licensed version date in numeric form (yyyymm).
 */
#define CH_CUSTOMER_LICENSE_VERSION_DATE                                    \
  (((CH_CUSTOMER_LICENSE_VERSION_YEAR + 2000) * 100) +                      \
   CH_CUSTOMER_LICENSE_VERSION_MONTH)

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

#endif /* CHCUSTOMER_H */

/** @} */
