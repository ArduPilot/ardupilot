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
 * @file        drvstreams.c
 * @brief       Generated VFS Streams Driver source.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  DRVSTREAMS
 * @{
 */

#include "vfs.h"

#if (VFS_CFG_ENABLE_DRV_STREAMS == TRUE) || defined(__DOXYGEN__)

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

/**
 * @brief       Structure representing the global state of @p
 *              vfs_streams_driver_c.
 */
struct vfs_streams_driver_static_struct {
  /**
   * @brief       Pool of directory nodes.
   */
  memory_pool_t             dir_nodes_pool;
  /**
   * @brief       Pool of file nodes.
   */
  memory_pool_t             file_nodes_pool;
  /**
   * @brief       Static storage of directory nodes.
   */
  vfs_streams_dir_node_c    dir_nodes[DRV_CFG_STREAMS_DIR_NODES_NUM];
  /**
   * @brief       Static storage of file nodes.
   */
  vfs_streams_file_node_c   file_nodes[DRV_CFG_STREAMS_FILE_NODES_NUM];
};

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/**
 * @brief       Global state of @p vfs_streams_driver_c.
 */
static struct vfs_streams_driver_static_struct vfs_streams_driver_static;

/* Module code has been generated into an hand-editable file and included
   here.*/
#include "drvstreams_impl.inc"

#endif /* VFS_CFG_ENABLE_DRV_STREAMS == TRUE */

/** @} */
