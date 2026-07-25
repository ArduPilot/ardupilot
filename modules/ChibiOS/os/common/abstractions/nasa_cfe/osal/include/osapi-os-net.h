/*
** File: osapi-os-net.h
**
**      Copyright (c) 2004-2006, United States government as represented by the 
**      administrator of the National Aeronautics Space Administration.  
**      All rights reserved. This software was created at NASAs Goddard 
**      Space Flight Center pursuant to government contracts.
**
**      This is governed by the NASA Open Source Agreement and may be used, 
**      distributed and modified only pursuant to the terms of that agreement.
**
** Author:  Alan Cudmore Code 582
**
** Purpose: Contains functions prototype definitions and variables declarations
**          for the OS Abstraction Layer, Network Module
**
** $Revision: 1.2 $ 
**
** $Date: 2010/11/12 12:00:19GMT-05:00 $
**
** $Log: osapi-os-net.h  $
** Revision 1.2 2010/11/12 12:00:19GMT-05:00 acudmore 
** replaced copyright character with (c) and added open source notice where needed.
** Revision 1.1 2008/04/20 22:36:02EDT ruperera 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
** Revision 1.1 2007/10/16 16:14:52EDT apcudmore 
** Initial revision
** Member added to project d:/mksdata/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
** Revision 1.1 2007/08/24 13:43:25EDT apcudmore 
** Initial revision
** Member added to project d:/mksdata/MKS-CFE-PROJECT/fsw/cfe-core/os/inc/project.pj
** Revision 1.3 2006/01/20 11:56:18EST njyanchik 
** Fixed header file information to match api document
** Revision 1.4  2005/06/07 16:49:31  nyanchik
** changed returns code for osapi.c to all int32 from uint32
**
** Revision 1.3  2005/03/22 19:04:54  acudmore
** fixed uint type
**
** Revision 1.2  2005/03/22 18:59:33  acudmore
** updated prototype
**
** Revision 1.1  2005/03/22 18:58:51  acudmore
** added osapi network interface
**
** Revision 1.1  2005/03/15 18:26:32  nyanchik
** *** empty log message ***
**
**
** Date Written:
**
**    
*/
#ifndef _osapi_network_
#define _osapi_network_

int32 OS_NetworkGetID             (void);
int32 OS_NetworkGetHostName       (char *host_name, uint32 name_len);

#endif
