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
 * @file    httpd_fatfs.h
 * @brief   HTTPD FatFS bindings macros and structures.
 * @addtogroup LWIP_HTTPD_FATFS
 * @{
 */

#ifndef HTTPD_FATFS_H
#define HTTPD_FATFS_H

#if !defined(LWIP_HTTPD_CUSTOM_FILES)
#error "LWIP_HTTPD_CUSTOM_FILES not defined"
#endif

#if !defined(LWIP_HTTPD_DYNAMIC_FILE_READ)
#error "LWIP_HTTPD_DYNAMIC_FILE_READ not defined"
#endif

#if !defined(LWIP_HTTPD_DYNAMIC_HEADERS)
#error "LWIP_HTTPD_DYNAMIC_HEADERS not defined"
#endif

#if !LWIP_HTTPD_CUSTOM_FILES
#error "LWIP_HTTPD_CUSTOM_FILES not enabled"
#endif

#if !LWIP_HTTPD_DYNAMIC_FILE_READ
#error "LWIP_HTTPD_DYNAMIC_FILE_READ not enabled"
#endif

#if !LWIP_HTTPD_DYNAMIC_HEADERS
#error "LWIP_HTTPD_DYNAMIC_HEADERS not enabled"
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void httpd_fatfs_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HTTPD_FATFS_H */

/** @} */
