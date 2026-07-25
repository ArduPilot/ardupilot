/*
** File: osapi-os-filesys.h
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
**          for the OS Abstraction Layer, File System module
**
** $Revision: 1.11 $ 
**
** $Date: 2013/12/16 12:57:41GMT-05:00 $
**
** $Log: osapi-os-filesys.h  $
** Revision 1.11 2013/12/16 12:57:41GMT-05:00 acudmore 
** Added macros for Volume name length and physical device name length
** Revision 1.10 2013/07/29 12:05:48GMT-05:00 acudmore 
** Added define for device and volume name length
** Revision 1.9 2013/07/25 14:31:21GMT-05:00 acudmore 
** Added prototype and datatype for OS_GetFsInfo
** Revision 1.8 2011/12/05 12:04:21GMT-05:00 acudmore 
** Added OS_rewinddir API
** Revision 1.7 2011/04/05 16:01:12EDT acudmore 
** Added OS_CloseFileByName and OS_CloseAllFiles
** Revision 1.6 2010/11/15 11:04:38EST acudmore 
** Added OS_FileOpenCheck function.
** Revision 1.5 2010/11/12 12:00:18EST acudmore 
** replaced copyright character with (c) and added open source notice where needed.
** Revision 1.4 2010/02/01 12:28:57EST acudmore 
** Added OS_fsBytesFree API
** Revision 1.3 2010/01/25 14:44:26EST acudmore 
** renamed "new" variable to avoid C++ reserved name conflict.
** Revision 1.2 2009/07/14 15:16:05EDT acudmore 
** Added OS_TranslatePath to the API
** Revision 1.1 2008/04/20 22:36:01EDT ruperera 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
** Revision 1.1 2007/10/16 16:14:52EDT apcudmore 
** Initial revision
** Member added to project d:/mksdata/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
** Revision 1.1 2007/08/24 13:43:24EDT apcudmore 
** Initial revision
** Member added to project d:/mksdata/MKS-CFE-PROJECT/fsw/cfe-core/os/inc/project.pj
** Revision 1.17 2007/06/07 09:59:14EDT njyanchik 
** I replaced the second OS_cp definition with OS_mv
** Revision 1.16 2007/06/05 16:25:33EDT apcudmore 
** Increased Number of volume table entries from 10 to 14.
** Added 2 extra EEPROM disk mappings to RAD750 Volume table + 2 spares
** Added 4 spares to every other volume table.
** Revision 1.15 2007/05/25 09:17:56EDT njyanchik 
** I added the rmfs call to the OSAL and updated the unit test stubs to match
** Revision 1.14 2007/03/21 10:15:29EST njyanchik 
** I mistakenly put the wrong length in for the path in the OS_FDTableEntry structure, and I added 
** some code that will set and out of range file descriptors .IsValid flag to false in OS_FDGetInfo
** Revision 1.13 2007/03/06 11:52:46EST njyanchik 
** This change goes with the previous CP, I forgot to include it
** Revision 1.12 2007/02/28 14:57:45EST njyanchik 
** The updates for supporting copying and moving files are now supported
** Revision 1.11 2007/02/27 15:22:11EST njyanchik 
** This CP has the initial import of the new file descripor table mechanism
** Revision 1.10 2006/12/20 10:27:09EST njyanchik 
** This change package incorporates all the changes necessary for the addition
** of a new API to get the real physical drive undernieth a mount point
** Revision 1.9 2006/11/14 14:44:28GMT-05:00 njyanchik 
** Checks were added to the OS fs calls that look at the return of a function that
** changes the name of paths from abstracted to local path names.
** Revision 1.8 2006/10/30 16:12:19GMT-05:00 apcudmore 
** Updated Compact flash and RAM device names for vxWorks 6.2 changes. 
** Revision 1.7 2006/10/25 11:31:18EDT njyanchik 
** This CP incorporates changes to every bsp_voltab.c file. I increased the number
** entries in the volume table to 10. I also changed the #define in the os_filesys.h
** file for the number of entries to match.
** 
** This update also includes adding the prototype for OS_initfs in os_filesys.h
** Revision 1.6 2006/09/26 09:03:46GMT-05:00 njyanchik 
** Contains the initial import of the ES Shell commands interface
** Revision 1.5 2006/07/25 15:37:52EDT njyanchik 
** It turns out the both the FS app and the OSAL were incorrect where file descriptors are
** concerned. the file descriptors should be int32 across the board.
** Revision 1.4 2006/01/20 11:56:18EST njyanchik 
** Fixed header file information to match api document
** Revision 1.26  2005/07/12 17:13:56  nyanchik
** Moved the Volume table to a bsp table in the arch directories.
**
** Revision 1.2 2005/07/11 16:26:57EDT apcudmore 
** OSAPI 2.0 integration
** Revision 1.25  2005/07/06 16:11:17  nyanchik
** *** empty log message ***
**
** Revision 1.24  2005/07/05 18:34:55  nyanchik
** fixed issues found in code walkthrogh. Also removed the OS_Info* functions that are going in the BSP
**
** Revision 1.23  2005/06/17 19:46:34  nyanchik
** added new file system style to linux and rtems.
**
** Revision 1.22  2005/06/15 16:43:48  nyanchik
** added extra parenthesis for the .h file # defines
**
** Revision 1.21  2005/06/06 14:17:42  nyanchik
** added headers to osapi-os-core.h and osapi-os-filesys.h
**
** Revision 1.20  2005/06/02 18:04:24  nyanchik
** *** empty log message ***
**
** Revision 1.1  2005/03/15 18:26:32  nyanchik
** *** empty log message ***
**
**
** Date Written:
**
**    
*/

#ifndef _osapi_filesys_
#define _osapi_filesys_
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>

#define OS_READ_ONLY        0
#define OS_WRITE_ONLY       1
#define OS_READ_WRITE       2

#define OS_SEEK_SET         0
#define OS_SEEK_CUR         1
#define OS_SEEK_END         2

#define OS_CHK_ONLY         0
#define OS_REPAIR           1

#define FS_BASED            0
#define RAM_DISK            1
#define EEPROM_DISK         2
#define ATA_DISK            3


/*
** Number of entries in the internal volume table
*/
#define NUM_TABLE_ENTRIES 14

/*
** Length of a Device and Volume name 
*/
#define OS_FS_DEV_NAME_LEN  32
#define OS_FS_PHYS_NAME_LEN 64
#define OS_FS_VOL_NAME_LEN  32


/*
** Defines for File System Calls
*/
#define OS_FS_SUCCESS                    0
#define OS_FS_ERROR                    (-1)
#define OS_FS_ERR_INVALID_POINTER      (-2) 
#define OS_FS_ERR_PATH_TOO_LONG        (-3)
#define OS_FS_ERR_NAME_TOO_LONG        (-4)
#define OS_FS_UNIMPLEMENTED            (-5) 
#define OS_FS_ERR_DRIVE_NOT_CREATED    (-6)
#define OS_FS_ERR_DEVICE_NOT_FREE      (-7)
#define OS_FS_ERR_PATH_INVALID         (-8)
#define OS_FS_ERR_NO_FREE_FDS          (-9)
#define OS_FS_ERR_INVALID_FD           (-10)

/* This typedef is for the OS_FS_GetErrorName function, to ensure
 * everyone is making an array of the same length */

typedef char os_fs_err_name_t[35];


/*
** Internal structure of the OS volume table for 
** mounted file systems and path translation
*/
typedef struct
{
    char   DeviceName [OS_FS_DEV_NAME_LEN];
    char   PhysDevName [OS_FS_PHYS_NAME_LEN];
    uint32 VolumeType;
    uint8  VolatileFlag;
    uint8  FreeFlag;
    uint8  IsMounted;
    char   VolumeName [OS_FS_VOL_NAME_LEN];
    char   MountPoint [OS_MAX_PATH_LEN];
    uint32 BlockSize;

}OS_VolumeInfo_t;

typedef struct
{
    int32   OSfd;                   /* The underlying OS's file descriptor */
    char    Path[OS_MAX_PATH_LEN];  /* The path of the file opened */
    uint32   User;                  /* The task id of the task who opened the file*/
    uint8   IsValid;                /* Whether or not this entry is valid */
}OS_FDTableEntry;

typedef struct
{
   uint32   MaxFds;                /* Total number of file descriptors */
   uint32   FreeFds;               /* Total number that are free */
   uint32   MaxVolumes;            /* Maximum number of volumes */
   uint32   FreeVolumes;           /* Total number of volumes free */
} os_fsinfo_t; 

/* modified to posix calls, since all of the 
 * applicable OSes use the posix calls */

typedef struct stat         os_fstat_t;
typedef DIR*                os_dirp_t;
typedef struct dirent       os_dirent_t;
/* still don't know what this should be*/
typedef unsigned long int   os_fshealth_t; 

/*
 * Exported Functions
*/


/******************************************************************************
** Standard File system API
******************************************************************************/
/*
 * Initializes the File System functions 
*/

int32           OS_FS_Init(void);

/*
 * Creates a file specified by path
*/
int32           OS_creat  (const char *path, int32  access);

/*
 * Opend a file for reading/writing. Returns file descriptor
*/
int32           OS_open   (const char *path,  int32 access,  uint32 mode);

/*
 * Closes an open file.
*/
int32           OS_close  (int32  filedes);

/*
 * Reads nbytes bytes from file into buffer
*/
int32           OS_read   (int32  filedes, void *buffer, uint32 nbytes);

/*
 * Write nybytes bytes of buffer into the file
*/
int32           OS_write  (int32  filedes, void *buffer, uint32 nbytes);

/*
 * Changes the permissions of a file
*/
int32           OS_chmod  (const char *path, uint32 access);

/*
 * Returns file status information in filestats
*/
int32           OS_stat   (const char *path, os_fstat_t  *filestats);

/*
 * Seeks to the specified position of an open file 
*/
int32           OS_lseek  (int32  filedes, int32 offset, uint32 whence);

/*
 * Removes a file from the file system
*/
int32           OS_remove (const char *path);

/*
 * Renames a file in the file system
*/
int32           OS_rename (const char *old_filename, const char *new_filename);

/* 
 * copies a single file from src to dest
*/
int32 OS_cp (const char *src, const char *dest);

/* 
 * moves a single file from src to dest
*/
int32 OS_mv (const char *src, const char *dest);

/*
 * Copies the info of an open file to the structure
*/
int32 OS_FDGetInfo (int32 filedes, OS_FDTableEntry *fd_prop);

/*
** Check to see if a file is open
*/
int32 OS_FileOpenCheck(char *Filename);

/*
** Close all open files
*/
int32 OS_CloseAllFiles(void);

/*
** Close a file by filename
*/
int32 OS_CloseFileByName(char *Filename);


/******************************************************************************
** Directory API 
******************************************************************************/

/*
 * Makes a new directory
*/
int32           OS_mkdir   (const char *path, uint32 access);

/*
 * Opens a directory for searching
*/
os_dirp_t       OS_opendir (const char *path);

/*
 * Closes an open directory
*/
int32           OS_closedir(os_dirp_t directory);

/*
 * Rewinds an open directory
*/
void           OS_rewinddir(os_dirp_t directory);

/*
 * Reads the next object in the directory
*/
os_dirent_t *   OS_readdir (os_dirp_t directory);

/*
 * Removes an empty directory from the file system.
*/
int32           OS_rmdir   (const char *path);

/******************************************************************************
** System Level API 
******************************************************************************/
/*
 * Makes a file system
*/
int32           OS_mkfs        (char *address,char *devname, char *volname,
                                uint32 blocksize, uint32 numblocks);
/*
 * Mounts a file system
*/
int32           OS_mount       (const char *devname, char *mountpoint);

/*
 * Initializes an existing file system
*/
int32           OS_initfs      (char *address,char *devname, char *volname,
                                uint32 blocksize, uint32 numblocks);

/*
 * removes a file system 
*/
int32           OS_rmfs        (char *devname);

/*
 * Unmounts a mounted file system
*/
int32           OS_unmount     (const char *mountpoint);

/*
 * Returns the number of free blocks in a file system
*/
int32           OS_fsBlocksFree (const char *name);

/*
** Returns the number of free bytes in a file system 
** Note the 64 bit data type to support filesystems that
** are greater than 4 Gigabytes
*/
int32 OS_fsBytesFree (const char *name, uint64 *bytes_free);

/*
 * Checks the health of a file system and repairs it if neccesary
*/
os_fshealth_t   OS_chkfs       (const char *name, boolean repair);

/*
 * Returns in the parameter the physical drive underneith the mount point 
*/
int32       OS_FS_GetPhysDriveName  (char * PhysDriveName, char * MountPoint);

/*
** Translates a OSAL Virtual file system path to a host Local path
*/
int32       OS_TranslatePath ( const char *VirtualPath, char *LocalPath);

/*             
**  Returns information about the file system in an os_fsinfo_t
*/
int32       OS_GetFsInfo(os_fsinfo_t  *filesys_info);

/******************************************************************************
** Shell API
******************************************************************************/

/* executes the shell command passed into is and writes the output of that 
 * command to the file specified by the given OSAPI file descriptor */
int32 OS_ShellOutputToFile(char* Cmd, int32 OS_fd);
#endif
