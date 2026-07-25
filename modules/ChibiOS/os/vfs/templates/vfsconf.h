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
 * @file    templates/vfsconf.h
 * @brief   VFS configuration header.
 *
 * @addtogroup VFS_CONF
 * @{
 */

#ifndef VFSCONF_H
#define VFSCONF_H

#define _CHIBIOS_VFS_CONF_
#define _CHIBIOS_VFS_CONF_VER_1_0_

/*===========================================================================*/
/**
 * @name VFS general settings
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Maximum filename length.
 */
#if !defined(VFS_CFG_NAMELEN_MAX) || defined(__DOXYGEN__)
#define VFS_CFG_NAMELEN_MAX                 15
#endif

/**
 * @brief   Maximum paths length.
 */
#if !defined(VFS_CFG_PATHLEN_MAX) || defined(__DOXYGEN__)
#define VFS_CFG_PATHLEN_MAX                 1023
#endif

/**
 * @brief   Number of shared path buffers.
 */
#if !defined(VFS_CFG_PATHBUFS_NUM) || defined(__DOXYGEN__)
#define VFS_CFG_PATHBUFS_NUM                1
#endif

/** @} */

/*===========================================================================*/
/**
 * @name VFS drivers
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Enables the VFS Overlay Driver.
 */
#if !defined(VFS_CFG_ENABLE_DRV_OVERLAY) || defined(__DOXYGEN__)
#define VFS_CFG_ENABLE_DRV_OVERLAY          TRUE
#endif

/**
 * @brief   Enables the VFS Streams Driver.
 */
#if !defined(VFS_CFG_ENABLE_DRV_STREAMS) || defined(__DOXYGEN__)
#define VFS_CFG_ENABLE_DRV_STREAMS          TRUE
#endif

/**
 * @brief   Enables the VFS ChibiFS Driver.
 */
#if !defined(VFS_CFG_ENABLE_DRV_CHFS) || defined(__DOXYGEN__)
#define VFS_CFG_ENABLE_DRV_CHFS             FALSE
#endif

/**
 * @brief   Enables the VFS FatFS Driver.
 */
#if !defined(VFS_CFG_ENABLE_DRV_FATFS) || defined(__DOXYGEN__)
#define VFS_CFG_ENABLE_DRV_FATFS            TRUE
#endif

/**
 * @brief   Enables the VFS LittleFS Driver.
 */
#if !defined(VFS_CFG_ENABLE_DRV_LITTLEFS) || defined(__DOXYGEN__)
#define VFS_CFG_ENABLE_DRV_LITTLEFS         TRUE
#endif

/** @} */

/*===========================================================================*/
/**
 * @name Overlay driver settings
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Maximum number of overlay directories.
 */
#if !defined(DRV_CFG_OVERLAY_DRV_MAX) || defined(__DOXYGEN__)
#define DRV_CFG_OVERLAY_DRV_MAX             2
#endif

/**
 * @brief   Number of directory nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_OVERLAY_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_OVERLAY_DIR_NODES_NUM       1
#endif

/** @} */

/*===========================================================================*/
/**
 * @name Streams driver settings
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Number of directory nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_STREAMS_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_STREAMS_DIR_NODES_NUM       1
#endif

/**
 * @brief   Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_STREAMS_FILE_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_STREAMS_FILE_NODES_NUM      2
#endif

/** @} */

/*===========================================================================*/
/**
 * @name ChibiFS driver settings
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Number of directory nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_CHFS_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_CHFS_DIR_NODES_NUM          1
#endif

/**
 * @brief   Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_CHFS_FILE_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_CHFS_FILE_NODES_NUM         1
#endif

/** @} */

/*===========================================================================*/
/**
 * @name FatFS driver settings
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Maximum number of FatFS file systems mounted.
 */
#if !defined(DRV_CFG_FATFS_FS_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_FATFS_FS_NUM                1
#endif

/**
 * @brief   Number of directory nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_FATFS_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_FATFS_DIR_NODES_NUM         1
#endif

/**
 * @brief   Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_FATFS_FILE_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_FATFS_FILE_NODES_NUM        2
#endif

/** @} */

/*===========================================================================*/
/**
 * @name LittleFS driver settings
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Number of shared path buffers.
 */
#if !defined(DRV_CFG_LITTLEFS_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_LITTLEFS_DIR_NODES_NUM      2
#endif

/**
 * @brief   Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_LITTLEFS_FILE_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_LITTLEFS_FILE_NODES_NUM     2
#endif

/**
 * @brief   Number of info nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_LITTLEFS_INFO_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_LITTLEFS_INFO_NODES_NUM     1
#endif

/** @} */

#endif /* VFSCONF_H */

/** @} */
