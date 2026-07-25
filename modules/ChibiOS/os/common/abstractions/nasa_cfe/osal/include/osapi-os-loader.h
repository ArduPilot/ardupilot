/*
** File: osapi-os-loader.h
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
**          for the OS Abstraction Layer, Object file loader API
**
** $Revision: 1.5 $ 
**
** $Date: 2013/07/25 10:02:08GMT-05:00 $
**
** $Log: osapi-os-loader.h  $
** Revision 1.5 2013/07/25 10:02:08GMT-05:00 acudmore 
** removed circular include "osapi.h"
** Revision 1.4 2010/11/12 12:00:18GMT-05:00 acudmore 
** replaced copyright character with (c) and added open source notice where needed.
** Revision 1.3 2010/02/01 12:38:06EST acudmore 
** added return code to OS_ModuleTableInit
** Revision 1.2 2008/06/20 15:13:43EDT apcudmore 
** Checked in new Module loader/symbol table functionality
** Revision 1.1 2008/04/20 22:36:02EDT ruperera 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
** Revision 1.1 2008/02/07 11:08:24EST apcudmore 
** Initial revision
** Member added to project d:/mksdata/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
** 
**  
*/

#ifndef _osapi_loader_
#define _osapi_loader_

/*
** Defines
*/


/*
** Typedefs
*/

typedef struct
{
   uint32 valid;
   uint32 code_address;
   uint32 code_size;
   uint32 data_address;
   uint32 data_size;
   uint32 bss_address;
   uint32 bss_size;
   uint32 flags;  
} OS_module_address_t;

typedef struct 
{
   int                 free;
   uint32              entry_point; 
   uint32              host_module_id;
   char                filename[OS_MAX_PATH_LEN];
   char                name[OS_MAX_API_NAME];
   OS_module_address_t addr;

} OS_module_record_t;

/*
** Loader API
*/
int32 OS_ModuleTableInit ( void );

int32 OS_SymbolLookup (uint32 *symbol_address, char *symbol_name );

int32 OS_SymbolTableDump ( char *filename, uint32 size_limit );

int32 OS_ModuleLoad ( uint32 *module_id, char *module_name, char *filename );

int32 OS_ModuleUnload ( uint32 module_id );

int32 OS_ModuleInfo ( uint32 module_id, OS_module_record_t *module_info );


#endif
