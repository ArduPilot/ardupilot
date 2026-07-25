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
 * @file        drvtmplfs.c
 * @brief       Generated VFS Template Driver source.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  DRVTMPLFS
 * @{
 */

#include "vfs.h"

#if (VFS_CFG_ENABLE_DRV_TEMPLATE == TRUE) || defined(__DOXYGEN__)

#include "ff.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module local macros.                                                      */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief       Global state of @p vfs_tmpl_driver_c.
 */
struct vfs_tmpl_driver_static_struct vfs_tmpl_driver_static;

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/**
 * @class       vfs_tmpl_dir_node_c
 * @extends     vfs_directory_node_c
 *
 *
 * @name        Class @p vfs_tmpl_dir_node_c structures
 * @{
 */

/**
 * @brief       Type of a VFS Tmpl directory node class.
 */
typedef struct vfs_tmpl_dir_node vfs_tmpl_dir_node_c;

/**
 * @brief       Class @p vfs_tmpl_dir_node_c virtual methods table.
 */
struct vfs_tmpl_dir_node_vmt {
  /* From base_object_c.*/
  void (*dispose)(void *ip);
  /* From referenced_object_c.*/
  void * (*addref)(void *ip);
  object_references_t (*release)(void *ip);
  /* From vfs_node_c.*/
  msg_t (*stat)(void *ip, vfs_stat_t *sp);
  /* From vfs_directory_node_c.*/
  msg_t (*first)(void *ip, vfs_direntry_info_t *dip);
  msg_t (*next)(void *ip, vfs_direntry_info_t *dip);
  /* From vfs_tmpl_dir_node_c.*/
};

/**
 * @brief       Structure representing a VFS Tmpl directory node class.
 */
struct vfs_tmpl_dir_node {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_tmpl_dir_node_vmt *vmt;
  /**
   * @brief       Number of references to the object.
   */
  object_references_t       references;
  /**
   * @brief       Driver handling this node.
   */
  vfs_driver_c              *driver;
  /**
   * @brief       Node mode information.
   */
  vfs_mode_t                mode;
};
/** @} */

/**
 * @class       vfs_tmpl_file_node_c
 * @extends     vfs_file_node_c
 * @implements  sequential_stream_i
 *
 *
 * @name        Class @p vfs_tmpl_file_node_c structures
 * @{
 */

/**
 * @brief       Type of a VFS Tmpl file node class.
 */
typedef struct vfs_tmpl_file_node vfs_tmpl_file_node_c;

/**
 * @brief       Class @p vfs_tmpl_file_node_c virtual methods table.
 */
struct vfs_tmpl_file_node_vmt {
  /* From base_object_c.*/
  void (*dispose)(void *ip);
  /* From referenced_object_c.*/
  void * (*addref)(void *ip);
  object_references_t (*release)(void *ip);
  /* From vfs_node_c.*/
  msg_t (*stat)(void *ip, vfs_stat_t *sp);
  /* From vfs_file_node_c.*/
  ssize_t (*read)(void *ip, uint8_t *buf, size_t n);
  ssize_t (*write)(void *ip, const uint8_t *buf, size_t n);
  msg_t (*setpos)(void *ip, vfs_offset_t offset, vfs_seekmode_t whence);
  vfs_offset_t (*getpos)(void *ip);
  sequential_stream_i * (*getstream)(void *ip);
  /* From vfs_tmpl_file_node_c.*/
};

/**
 * @brief       Structure representing a VFS Tmpl file node class.
 */
struct vfs_tmpl_file_node {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_tmpl_file_node_vmt *vmt;
  /**
   * @brief       Number of references to the object.
   */
  object_references_t       references;
  /**
   * @brief       Driver handling this node.
   */
  vfs_driver_c              *driver;
  /**
   * @brief       Node mode information.
   */
  vfs_mode_t                mode;
  /**
   * @brief       Implemented interface @p sequential_stream_i.
   */
  sequential_stream_i       stm;
};
/** @} */

/**
 * @brief       Global state of @p vfs_tmpl_driver_c.
 */
struct vfs_tmpl_driver_static_struct {
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
  vfs_tmpl_dir_node_c       dir_nodes[DRV_CFG_TMPL_DIR_NODES_NUM];
  /**
   * @brief       Static storage of file nodes.
   */
  vfs_tmpl_file_node_c      file_nodes[DRV_CFG_TMPL_FILE_NODES_NUM];
};

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/* Module code has been generated into an hand-editable file and included
   here.*/
#include "drvtmplfs_impl.inc"

#endif /* VFS_CFG_ENABLE_DRV_TEMPLATE == TRUE */

/** @} */
