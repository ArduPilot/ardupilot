/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file        drvoverlay.c
 * @brief       Generated VFS Template Driver source.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  DRVOVERLAY
 * @{
 */

#include "vfs.h"

#if (VFS_CFG_ENABLE_DRV_OVERLAY == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module local macros.                                                      */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/**
 * @brief       Global state of @p vfs_overlay_driver_c.
 */
static struct vfs_overlay_driver_static_struct vfs_overlay_driver_static;

/* Module code has been generated into an hand-editable file and included
   here.*/
#include "drvoverlay_impl.inc"

#endif /* VFS_CFG_ENABLE_DRV_OVERLAY == TRUE */

/** @} */
