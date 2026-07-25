/******************************************************************************
** File: osconfig.h
** $Id: osconfig.h 1.2 2013/12/16 13:08:05GMT-05:00 acudmore Exp  $
**
** Purpose:
**   This header file contains the OS API  configuration parameters.
**
** Author:  A. Cudmore
**
** Notes:
**
** $Date: 2013/12/16 13:08:05GMT-05:00 $
** $Revision: 1.2 $
** $Log: osconfig.h  $
** Revision 1.2 2013/12/16 13:08:05GMT-05:00 acudmore 
** use OS_FS_PHYS_NAME_LEN macro instead of hard-coded value
** Revision 1.1 2013/07/19 14:05:44GMT-05:00 acudmore 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/MKS-OSAL-REPOSITORY/src/bsp/sis-rtems/config/project.pj
** Revision 1.8 2011/12/05 12:41:15GMT-05:00 acudmore 
** Removed OS_MEM_TABLE_SIZE parameter
** Revision 1.7 2009/07/14 14:24:53EDT acudmore 
** Added parameter for local path size.
** Revision 1.6 2009/07/07 14:01:02EDT acudmore 
** Changed OS_MAX_NUM_OPEN_FILES to 50 to preserve data/telmetry space
** Revision 1.5 2009/07/07 13:58:22EDT acudmore 
** Added OS_STATIC_LOADER define to switch between static and dynamic loaders.
** Revision 1.4 2009/06/04 11:43:43EDT rmcgraw 
** DCR8290:1 Increased settings for max tasks,queues,sems and modules
** Revision 1.3 2008/08/20 15:49:37EDT apcudmore 
** Add OS_MAX_TIMERS parameter for Timer API
** Revision 1.2 2008/06/20 15:17:56EDT apcudmore 
** Added conditional define for Module Loader API configuration
** Revision 1.1 2008/04/20 22:35:19EDT ruperera 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/MKS-OSAL-REPOSITORY/build/inc/project.pj
** Revision 1.6 2008/02/12 13:27:59EST apcudmore 
** New API updates:
**   - fixed RTEMS osapi compile error
**   - related makefile fixes
**   - header file parameter update
**
** Revision 1.1 2005/06/09 10:57:58EDT rperera
** Initial revision
**
******************************************************************************/

#ifndef _osconfig_
#define _osconfig_

/*
** Platform Configuration Parameters for the OS API
*/

#define OS_MAX_TASKS                64 /* Not used.*/
#define OS_MAX_QUEUES               64
#define OS_MAX_COUNT_SEMAPHORES     20
#define OS_MAX_BIN_SEMAPHORES       20
#define OS_MAX_MUTEXES              20

/*
** Maximum length for an absolute path name
*/
#define OS_MAX_PATH_LEN     64

/*
** Maximum length for a local or host path/filename.
**   This parameter can consist of the OSAL filename/path + 
**   the host OS physical volume name or path.
*/
#define OS_MAX_LOCAL_PATH_LEN (OS_MAX_PATH_LEN + OS_FS_PHYS_NAME_LEN)


/* 
** The maxium length allowed for a object (task,queue....) name 
*/
#define OS_MAX_API_NAME     20

/* 
** The maximum length for a file name 
*/
#define OS_MAX_FILE_NAME    20

/* 
** These defines are for OS_printf
*/
#define OS_BUFFER_SIZE 172
#define OS_BUFFER_MSG_DEPTH 100

/* This #define turns on a utility task that
 * will read the statements to print from
 * the OS_printf function. If you want OS_printf
 * to print the text out itself, comment this out 
 * 
 * NOTE: The Utility Task #defines only have meaning 
 * on the VxWorks operating systems
 */
 
#define OS_UTILITY_TASK_ON


#ifdef OS_UTILITY_TASK_ON 
    #define OS_UTILITYTASK_STACK_SIZE 2048
    /* some room is left for other lower priority tasks */
    #define OS_UTILITYTASK_PRIORITY   245
#endif


/* 
** the size of a command that can be passed to the underlying OS 
*/
#define OS_MAX_CMD_LEN 1000

/*
** This define will include the OS network API.
** It should be turned off for targtets that do not have a network stack or 
** device ( like the basic RAD750 vxWorks BSP )
*/
#undef OS_INCLUDE_NETWORK

/* 
** This is the maximum number of open file descriptors allowed at a time 
*/
#define OS_MAX_NUM_OPEN_FILES 50 

/* 
** This defines the filethe input command of OS_ShellOutputToFile
** is written to in the VxWorks6 port 
*/
#define OS_SHELL_CMD_INPUT_FILE_NAME "/ram/OS_ShellCmd.in"

/* 
** This define sets the queue implentation of the Linux port to use sockets 
** commenting this out makes the Linux port use the POSIX message queues.
*/
/* #define OSAL_SOCKET_QUEUE */

/*
** Module loader/symbol table is optional
*/
#undef OS_INCLUDE_MODULE_LOADER

#ifdef OS_INCLUDE_MODULE_LOADER
   /*
   ** This define sets the size of the OS Module Table, which keeps track of the loaded modules in 
   ** the running system. This define must be set high enough to support the maximum number of
   ** loadable modules in the system. If the the table is filled up at runtime, a new module load
   ** would fail.
   */
   #define OS_MAX_MODULES 10 

   /*
   ** The Static Loader define is used for switching between the Dynamic and Static loader implementations.
   */
   /* #define OS_STATIC_LOADER */

#endif


/*
** This define sets the maximum symbol name string length. It is used in implementations that 
** support the symbols and symbol lookup.
*/
#define OS_MAX_SYM_LEN 64


/*
** This define sets the maximum number of timers available
*/
#define OS_MAX_TIMERS         5

#endif
