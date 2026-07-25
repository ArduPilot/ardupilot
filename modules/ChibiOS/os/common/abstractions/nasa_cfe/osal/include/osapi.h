/*
** File: osapi.h
**
**      Copyright (c) 2004-2006, United States government as represented by the 
**      administrator of the National Aeronautics Space Administration.  
**      All rights reserved. This software was created at NASAs Goddard 
**      Space Flight Center pursuant to government contracts.
**
**      This is governed by the NASA Open Source Agreement and may be used, 
**      distributed and modified only pursuant to the terms of that agreement.
**
** Author:  Alan Cudmore - Code 582
**
** Purpose: Contains functions prototype definitions and variables declarations
**          for the OS Abstraction Layer, Core OS module
**
** $Revision: 1.10 $
**
** $Date: 2013/07/25 10:01:32GMT-05:00 $
**
** $Log: osapi.h  $
** Revision 1.10 2013/07/25 10:01:32GMT-05:00 acudmore 
** Added C++ support
** Revision 1.9 2010/11/12 12:00:17GMT-05:00 acudmore 
** replaced copyright character with (c) and added open source notice where needed.
** Revision 1.8 2010/03/08 15:57:20EST acudmore 
** include new OSAL version header file
** Revision 1.7 2009/08/10 14:01:10EDT acudmore 
** Reset OSAL version for trunk
** Revision 1.6 2009/08/10 13:55:49EDT acudmore 
** Updated OSAL version defines to 3.0
** Revision 1.5 2009/06/10 14:15:55EDT acudmore 
** Removed HAL include files. HAL code was removed from OSAL.
** Revision 1.4 2008/08/20 16:12:51EDT apcudmore 
** Updated timer error codes
** Revision 1.3 2008/08/20 15:46:27EDT apcudmore 
** Add support for timer API
** Revision 1.2 2008/06/20 15:13:43EDT apcudmore 
** Checked in new Module loader/symbol table functionality
** Revision 1.1 2008/04/20 22:36:02EDT ruperera 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
** Revision 1.6 2008/02/14 11:29:10EST apcudmore 
** Updated version define ( 2.11 )
** Revision 1.5 2008/02/07 11:31:58EST apcudmore 
** Fixed merge problem
** Revision 1.4 2008/02/07 11:07:29EST apcudmore 
** Added dynamic loader / Symbol lookup API 
**   -- API only, next release will have functionality
** Revision 1.2 2008/01/29 14:30:49EST njyanchik 
** I added code to all the ports that allow the values of both binary and counting semaphores to be
** gotten through the OS_*SemGetInfo API.
** Revision 1.1 2007/10/16 16:14:52EDT apcudmore 
** Initial revision
** Member added to project d:/mksdata/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
** Revision 1.2 2007/09/28 15:46:49EDT rjmcgraw 
** Updated version numbers to 5.0
** Revision 1.1 2007/08/24 13:43:25EDT apcudmore
** Initial revision
** Member added to project d:/mksdata/MKS-CFE-PROJECT/fsw/cfe-core/os/inc/project.pj
** Revision 1.9.1.1 2007/05/21 08:58:51EDT njyanchik
** The trunk version number has been updated to version 0.0
** Revision 1.9 2006/06/12 10:20:07EDT rjmcgraw
** Updated OS_MINOR_VERSION from 3 to 4
** Revision 1.8 2006/02/03 09:30:45EST njyanchik
** Changed version number to 2.3
** Revision 1.7 2006/01/20 11:56:16EST njyanchik
** Fixed header file information to match api document
** Revision 1.15  2005/11/09 13:35:49  nyanchik
** Revisions for 2.2 include:
** a new scheduler mapper for Linux and OS X
** addition of OS_printf function
** fixed issues that would cause warnings at compile time
**
**
*/

#ifndef _osapi_
#define _osapi_

#include "common_types.h"

#ifdef __cplusplus
   extern "C" {
#endif

#define OS_SUCCESS                     (0)
#define OS_ERROR                       (-1)
#define OS_INVALID_POINTER             (-2)
#define OS_ERROR_ADDRESS_MISALIGNED    (-3)
#define OS_ERROR_TIMEOUT               (-4)
#define OS_INVALID_INT_NUM             (-5)
#define OS_SEM_FAILURE                 (-6)
#define OS_SEM_TIMEOUT                 (-7)
#define OS_QUEUE_EMPTY                 (-8)
#define OS_QUEUE_FULL                  (-9)
#define OS_QUEUE_TIMEOUT               (-10)
#define OS_QUEUE_INVALID_SIZE          (-11)
#define OS_QUEUE_ID_ERROR              (-12)
#define OS_ERR_NAME_TOO_LONG           (-13)
#define OS_ERR_NO_FREE_IDS             (-14)
#define OS_ERR_NAME_TAKEN              (-15)
#define OS_ERR_INVALID_ID              (-16)
#define OS_ERR_NAME_NOT_FOUND          (-17)
#define OS_ERR_SEM_NOT_FULL            (-18)
#define OS_ERR_INVALID_PRIORITY        (-19)
#define OS_INVALID_SEM_VALUE           (-20)
#define OS_ERR_FILE                    (-27)
#define OS_ERR_NOT_IMPLEMENTED         (-28)
#define OS_TIMER_ERR_INVALID_ARGS      (-29)
#define OS_TIMER_ERR_TIMER_ID          (-30)
#define OS_TIMER_ERR_UNAVAILABLE       (-31)
#define OS_TIMER_ERR_INTERNAL          (-32)

/*
** Defines for Queue Timeout parameters
*/
#define OS_PEND   (0)
#define OS_CHECK (-1)

#include "osapi-version.h"

/*
** Include the configuration file
*/
#include "osconfig.h"

/*
** Include the OS API modules
*/
#include "osapi-os-core.h"
//#include "osapi-os-filesys.h"
//#include "osapi-os-net.h"
//#include "osapi-os-loader.h"
#include "osapi-os-timer.h"
#include "osapi-os-custom.h"

#ifdef __cplusplus
   }
#endif

#endif

