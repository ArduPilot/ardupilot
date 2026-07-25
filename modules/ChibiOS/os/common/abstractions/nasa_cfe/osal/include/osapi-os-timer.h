/*
** File: osapi-os-timer.h
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
** Purpose: Contains functions prototype definitions and variable declarations
**          for the OS Abstraction Layer, Timer API
**
** $Revision: 1.5 $ 
**
** $Date: 2013/07/25 10:02:20GMT-05:00 $
**
** $Log: osapi-os-timer.h  $
** Revision 1.5 2013/07/25 10:02:20GMT-05:00 acudmore 
** removed circular include "osapi.h"
** Revision 1.4 2010/11/12 12:00:19GMT-05:00 acudmore 
** replaced copyright character with (c) and added open source notice where needed.
** Revision 1.3 2010/02/01 12:38:34EST acudmore 
** Added return code to OS_TimerAPIInit
** Revision 1.2 2008/08/26 13:52:52EDT apcudmore 
** removed linux specific define
** Revision 1.1 2008/08/20 16:12:07EDT apcudmore 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
** 
**  
*/

#ifndef _osapi_timer_
#define _osapi_timer_

/*
** Typedefs
*/
typedef void (*OS_TimerCallback_t)(uint32 timer_id);

typedef struct 
{
   char                name[OS_MAX_API_NAME];
   uint32              creator;
   uint32              start_time;
   uint32              interval_time;
   uint32              accuracy;

} OS_timer_prop_t;


/*
** Timer API
*/
int32  OS_TimerAPIInit     (void);

int32 OS_TimerCreate      (uint32 *timer_id, const char *timer_name, uint32 *clock_accuracy, OS_TimerCallback_t callback_ptr);
int32 OS_TimerSet         (uint32 timer_id, uint32 start_msec, uint32 interval_msec);
int32 OS_TimerDelete      (uint32 timer_id);

int32 OS_TimerGetIdByName (uint32 *timer_id, const char *timer_name);
int32 OS_TimerGetInfo     (uint32  timer_id, OS_timer_prop_t *timer_prop);

#endif
