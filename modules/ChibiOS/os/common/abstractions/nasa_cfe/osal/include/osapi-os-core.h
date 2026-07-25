/*
** File: osapi-os-core.h
**
**      Copyright (c) 2004-2006, United States government as represented by the 
**      administrator of the National Aeronautics Space Administration.  
**      All rights reserved. This software was created at NASAs Goddard 
**      Space Flight Center pursuant to government contracts.
**
**      This is governed by the NASA Open Source Agreement and may be used, 
**      distributed and modified only pursuant to the terms of that agreement.
**
** Author:  Ezra Yeheksli -Code 582/Raytheon
**
** Purpose: Contains functions prototype definitions and variables declarations
**          for the OS Abstraction Layer, Core OS module
**
** $Revision: 1.8 $ 
**
** $Date: 2013/07/25 10:02:00GMT-05:00 $
**
** $Log: osapi-os-core.h  $
** Revision 1.8 2013/07/25 10:02:00GMT-05:00 acudmore 
** removed circular include "osapi.h"
** Revision 1.7 2012/04/11 09:30:48GMT-05:00 acudmore 
** Added OS_printf_enable and OS_printf_disable
** Revision 1.6 2010/11/12 12:00:17EST acudmore 
** replaced copyright character with (c) and added open source notice where needed.
** Revision 1.5 2010/11/10 15:33:14EST acudmore 
** Updated IntAttachHandler prototype
** Revision 1.4 2010/03/08 12:06:28EST acudmore 
** added function pointer type to get rid of warnings
** Revision 1.3 2010/02/01 12:37:15EST acudmore 
** added return code to OS API init
** Revision 1.2 2009/08/04 10:49:09EDT acudmore 
**   
*/

#ifndef _osapi_core_
#define _osapi_core_

#include <stdarg.h>   /* for va_list */

/*difines constants for OS_BinSemCreate for state of semaphore  */
#define OS_SEM_FULL     1
#define OS_SEM_EMPTY    0

/* #define for enabling floating point operations on a task*/
#define OS_FP_ENABLED 1

/*  tables for the properties of objects */

/*tasks */
typedef struct
{
    char name [OS_MAX_API_NAME];
    uint32 creator;
    uint32 stack_size;
    uint32 priority;
    uint32 OStask_id;
}OS_task_prop_t;
    
/* queues */
typedef struct
{
    char name [OS_MAX_API_NAME];
    uint32 creator;
}OS_queue_prop_t;

/* Binary Semaphores */
typedef struct
{                     
    char name [OS_MAX_API_NAME];
    uint32 creator;
    int32  value;
}OS_bin_sem_prop_t;

/* Counting Semaphores */
typedef struct
{                     
    char name [OS_MAX_API_NAME];
    uint32 creator;
    int32 value;
}OS_count_sem_prop_t;

/* Mutexes */
typedef struct
{
    char name [OS_MAX_API_NAME];
    uint32 creator;
}OS_mut_sem_prop_t;


/* struct for OS_GetLocalTime() */

typedef struct 
{ 
    uint32 seconds; 
    uint32 microsecs;
}OS_time_t; 

/* heap info */
typedef struct
{
    uint32 free_bytes;
    uint32 free_blocks;
    uint32 largest_free_block;
}OS_heap_prop_t;


/* This typedef is for the OS_GetErrorName function, to ensure
 * everyone is making an array of the same length */

typedef char os_err_name_t[35];

/*
** These typedefs are for the task entry point
*/
typedef void osal_task;
typedef osal_task ((*osal_task_entry)(void));

/*
** Exported Functions
*/

/*
** Initialization of API
*/
int32 OS_API_Init (void);


/*
** Task API
*/

int32 OS_TaskCreate            (uint32 *task_id, const char *task_name, 
                                osal_task_entry function_pointer,
                                const uint32 *stack_pointer, 
                                uint32 stack_size,
                                uint32 priority, uint32 flags);

int32 OS_TaskDelete            (uint32 task_id); 
void OS_TaskExit               (void);
int32 OS_TaskInstallDeleteHandler(void *function_pointer);
int32 OS_TaskDelay             (uint32 millisecond);
int32 OS_TaskSetPriority       (uint32 task_id, uint32 new_priority);
int32 OS_TaskRegister          (void);
uint32 OS_TaskGetId            (void);
int32 OS_TaskGetIdByName       (uint32 *task_id, const char *task_name);
int32 OS_TaskGetInfo           (uint32 task_id, OS_task_prop_t *task_prop);          

/*
** Message Queue API
*/

/*
** Queue Create now has the Queue ID returned to the caller.
*/
int32 OS_QueueCreate           (uint32 *queue_id, const char *queue_name,
                                uint32 queue_depth, uint32 data_size, uint32 flags);
int32 OS_QueueDelete           (uint32 queue_id);
int32 OS_QueueGet              (uint32 queue_id, void *data, uint32 size, 
                                uint32 *size_copied, int32 timeout);
int32 OS_QueuePut              (uint32 queue_id, void *data, uint32 size, 
                                uint32 flags);
int32 OS_QueueGetIdByName      (uint32 *queue_id, const char *queue_name);
int32 OS_QueueGetInfo          (uint32 queue_id, OS_queue_prop_t *queue_prop);

/*
** Semaphore API
*/

int32 OS_BinSemCreate          (uint32 *sem_id, const char *sem_name, 
                                uint32 sem_initial_value, uint32 options);
int32 OS_BinSemFlush            (uint32 sem_id);
int32 OS_BinSemGive            (uint32 sem_id);
int32 OS_BinSemTake            (uint32 sem_id);
int32 OS_BinSemTimedWait       (uint32 sem_id, uint32 msecs);
int32 OS_BinSemDelete          (uint32 sem_id);
int32 OS_BinSemGetIdByName     (uint32 *sem_id, const char *sem_name);
int32 OS_BinSemGetInfo         (uint32 sem_id, OS_bin_sem_prop_t *bin_prop);

int32 OS_CountSemCreate          (uint32 *sem_id, const char *sem_name, 
                                uint32 sem_initial_value, uint32 options);
int32 OS_CountSemGive            (uint32 sem_id);
int32 OS_CountSemTake            (uint32 sem_id);
int32 OS_CountSemTimedWait       (uint32 sem_id, uint32 msecs);
int32 OS_CountSemDelete          (uint32 sem_id);
int32 OS_CountSemGetIdByName     (uint32 *sem_id, const char *sem_name);
int32 OS_CountSemGetInfo         (uint32 sem_id, OS_count_sem_prop_t *count_prop);

/*
** Mutex API
*/

int32 OS_MutSemCreate           (uint32 *sem_id, const char *sem_name, uint32 options);
int32 OS_MutSemGive             (uint32 sem_id);
int32 OS_MutSemTake             (uint32 sem_id);
int32 OS_MutSemDelete           (uint32 sem_id);  
int32 OS_MutSemGetIdByName      (uint32 *sem_id, const char *sem_name); 
int32 OS_MutSemGetInfo          (uint32 sem_id, OS_mut_sem_prop_t *mut_prop);

/*
** OS Time/Tick related API
*/

int32 OS_Milli2Ticks           (uint32 milli_seconds);
int32 OS_Tick2Micros           (void);
int32  OS_GetLocalTime         (OS_time_t *time_struct);
int32  OS_SetLocalTime         (OS_time_t *time_struct);  

/*
** Exception API
*/

int32 OS_ExcAttachHandler      (uint32 ExceptionNumber, 
                                void (*ExceptionHandler)(uint32, uint32 *,uint32), 
                                int32 parameter);
int32 OS_ExcEnable             (int32 ExceptionNumber);
int32 OS_ExcDisable            (int32 ExceptionNumber);

/*
** Floating Point Unit API
*/

int32 OS_FPUExcAttachHandler   (uint32 ExceptionNumber, void * ExceptionHandler ,
                                 int32 parameter);
int32 OS_FPUExcEnable          (int32 ExceptionNumber);
int32 OS_FPUExcDisable         (int32 ExceptionNumber);
int32 OS_FPUExcSetMask         (uint32 mask);
int32 OS_FPUExcGetMask         (uint32 *mask);

/*
** Interrupt API
*/
int32 OS_IntAttachHandler  (uint32 InterruptNumber, osal_task_entry InterruptHandler, int32 parameter);
int32 OS_IntUnlock         (int32 IntLevel);
int32 OS_IntLock           (void);

int32 OS_IntEnable         (int32 Level);
int32 OS_IntDisable        (int32 Level);

int32 OS_IntSetMask        (uint32 mask);
int32 OS_IntGetMask        (uint32 *mask);
int32 OS_IntAck             (int32 InterruptNumber);

/*
** Shared memory API 
*/
int32 OS_ShMemInit          (void);
int32 OS_ShMemCreate        (uint32 *Id, uint32 NBytes, char* SegName);
int32 OS_ShMemSemTake       (uint32 Id);
int32 OS_ShMemSemGive       (uint32 Id);
int32 OS_ShMemAttach        (uint32 * Address, uint32 Id);
int32 OS_ShMemGetIdByName   (uint32 *ShMemId, const char *SegName );

/*
** Heap API
*/
int32 OS_HeapGetInfo       (OS_heap_prop_t *heap_prop);

/*
** API for useful debugging function
*/
int32 OS_GetErrorName      (int32 error_num, os_err_name_t* err_name);


/* 
** Abstraction for printf statements 
*/
void OS_printf( const char *string, ...);
void OS_printf_disable(void);
void OS_printf_enable(void);

#endif
