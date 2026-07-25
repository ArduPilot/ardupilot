/*---------------------------------------------------------------------------
**
**  Filename:
**    $Id: common_types.h 1.9 2014/01/14 16:28:32GMT-05:00 acudmore Exp  $
**
**      Copyright (c) 2004-2006, United States government as represented by the 
**      administrator of the National Aeronautics Space Administration.  
**      All rights reserved. This software was created at NASAs Goddard 
**      Space Flight Center pursuant to government contracts.
**
**      This is governed by the NASA Open Source Agreement and may be used, 
**      distributed and modified only pursuant to the terms of that agreement. 
**
**  Purpose:
**	    Unit specification for common types.
**
**  Design Notes:
**         Assumes make file has defined processor family
**
**  References:
**     Flight Software Branch C Coding Standard Version 1.0a
**
**
**	Notes:
**
**
**  $Date: 2014/01/14 16:28:32GMT-05:00 $
**  $Revision: 1.9 $
**  $Log: common_types.h  $
**  Revision 1.9 2014/01/14 16:28:32GMT-05:00 acudmore 
**  Fixed typo in macro for x86-64
**  Revision 1.8 2013/08/09 13:58:04GMT-05:00 acudmore 
**  Added int64 type, added support for ARM arch, added 64 bit x86 arch, added arch check for GCC arch macros, added check for proper data type sizes
**  Revision 1.7 2013/07/25 10:01:29GMT-05:00 acudmore 
**  Added C++ support
**  Revision 1.6 2012/04/11 09:19:03GMT-05:00 acudmore 
**  added OS_USED attribute
**  Revision 1.5 2010/02/18 16:43:29EST acudmore 
**  Added SPARC processor section
**  Removed special characters from comments that cause problems with some tools.
**  Revision 1.4 2010/02/18 16:41:39EST acudmore 
**  Added a block of defines for GCC specific pragmas and extensions.
**  Removed RTEMS boolean related ifdefs
**  moved OS_PACK into the GCC specific block
**  Revision 1.3 2010/02/01 12:31:17EST acudmore 
**  Added uint64 type
**  Revision 1.2 2009/07/07 16:30:05EDT acudmore 
**  Removed conditinal comp. around boolean for m68k.
**  This will need to be done for all RTEMS targets
**  Revision 1.1 2009/06/02 10:04:58EDT acudmore 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/MKS-OSAL-REPOSITORY/src/os/inc/project.pj
**  Revision 1.1 2008/04/20 22:35:58EDT ruperera 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/MKS-OSAL-REPOSITORY/src/inc/project.pj
**  Revision 1.1 2007/10/16 16:14:49EDT apcudmore 
**  Initial revision
**  Member added to project d:/mksdata/MKS-OSAL-REPOSITORY/src/inc/project.pj
**  Revision 1.2 2006/06/08 14:28:32EDT David Kobe (dlkobe) 
**  Added NASA Open Source Legal Statement
**  Revision 1.1 2005/06/09 09:57:51GMT-05:00 rperera 
**  Initial revision
**  Member added to project d:/mksdata/MKS-CFE-REPOSITORY/cfe-core/inc/project.pj
**  Revision 1.6  2005/03/24 19:20:52  rmcgraw
**  Wrapped the boolean defintion for all three processors with #ifndef _USING_RTEMS_INCLUDES_
**
**  Revision 1.5  2005/03/10 16:59:08  acudmore
**  removed boolean prefix to TRUE and FALSE defintion to avoid vxWorks conflict.
**
**  Revision 1.4  2005/03/07 20:23:34  acudmore
**  removed duplicate boolean definition
**
**  Revision 1.3  2005/03/07 20:05:17  acudmore
**  updated with __PPC__ macro that gnu compiler uses
**
**  Revision 1.2  2005/03/04 16:02:44  acudmore
**  added coldfire architecture
**
**  Revision 1.1  2005/03/04 15:58:45  acudmore
**  Added common_types.h
**
**
**
**-------------------------------------------------------------------------*/

#ifndef _common_types_
#define _common_types_

#ifdef __cplusplus
   extern "C" {
#endif

/*
** Includes
*/

/*
** Macro Definitions
*/

/* 
** Condition = TRUE is ok, Condition = FALSE is error 
*/
#define CompileTimeAssert(Condition, Message) typedef char Message[(Condition) ? 1 : -1]


/*
** Define compiler specific macros
** The __extension__ compiler pragma is required
** for the uint64 type using GCC with the ANSI C90 standard.
** Other macros can go in here as needed, for example alignment 
** pragmas.
*/
#if defined (__GNUC__)
   #define _EXTENSION_  __extension__
   #define OS_PACK      __attribute__ ((packed))
   #define OS_ALIGN(n)  __attribute__((aligned(n)))
   #define OS_USED      __attribute__((used))
#else
   #define _EXTENSION_ 
   #define OS_PACK
   #define OS_ALIGN(n) 
   #define OS_USED 
#endif

#if defined(_ix86_) || defined (__i386__)
/* ----------------------- Intel x86 processor family -------------------------*/
  /* Little endian */
  #undef   _STRUCT_HIGH_BIT_FIRST_
  #define  _STRUCT_LOW_BIT_FIRST_

  typedef unsigned char                         boolean;
  typedef signed char                           int8;
  typedef short int                             int16;
  typedef long int                              int32;
 _EXTENSION_ typedef long long int              int64; 
  typedef unsigned char                         uint8;
  typedef unsigned short int                    uint16;
  typedef unsigned long int                     uint32;
  _EXTENSION_ typedef unsigned long long int    uint64;

  typedef unsigned long int                     cpuaddr;

#elif defined (_ix64_) || defined (__x86_64__) 
/* ----------------------- Intel/AMD x64 processor family -------------------------*/
  /* Little endian */
  #undef   _STRUCT_HIGH_BIT_FIRST_
  #define  _STRUCT_LOW_BIT_FIRST_

  typedef unsigned char                         boolean;
  typedef signed char                           int8;
  typedef short int                             int16;
  typedef int                                   int32;
  typedef long int                              int64;
  typedef unsigned char                         uint8;
  typedef unsigned short int                    uint16;
  typedef unsigned int                          uint32;
  typedef unsigned long int                     uint64;

  typedef unsigned long int                     cpuaddr;

#elif defined(__PPC__) || defined (__ppc__)
   /* ----------------------- Motorola Power PC family ---------------------------*/
   /* The PPC can be programmed to be big or little endian, we assume native */
   /* Big endian */
   #define _STRUCT_HIGH_BIT_FIRST_
   #undef  _STRUCT_LOW_BIT_FIRST_

   typedef unsigned char                        boolean;
   typedef signed char                          int8;
   typedef short int                            int16;
   typedef long int                             int32;
   _EXTENSION_ typedef long long int            int64;
   typedef unsigned char                        uint8;
   typedef unsigned short int                   uint16;
   typedef unsigned long int                    uint32;
   _EXTENSION_ typedef unsigned long long int   uint64;

  typedef unsigned long int                     cpuaddr;

#elif defined(_m68k_) || defined(__m68k__)
   /* ----------------------- Motorola m68k/Coldfire family ---------------------------*/
   /* Big endian */
   #define _STRUCT_HIGH_BIT_FIRST_
   #undef  _STRUCT_LOW_BIT_FIRST_

   typedef unsigned char                        boolean;
   typedef signed char                          int8;
   typedef short int                            int16;
   typedef long int                             int32;
   _EXTENSION_ typedef long long int            int64;
   typedef unsigned char                        uint8;
   typedef unsigned short int                   uint16;
   typedef unsigned long int                    uint32;
   _EXTENSION_ typedef unsigned long long int   uint64;

  typedef unsigned long int                     cpuaddr;

#elif defined (__ARM__) || defined(__arm__)
/* ----------------------- ARM processor family -------------------------*/
  /* Little endian */
  #undef   _STRUCT_HIGH_BIT_FIRST_
  #define  _STRUCT_LOW_BIT_FIRST_

  typedef unsigned char                         boolean;
  typedef signed char                           int8;
  typedef short int                             int16;
  typedef long int                              int32;
  _EXTENSION_ typedef long long int             int64;
  typedef unsigned char                         uint8;
  typedef unsigned short int                    uint16;
  typedef unsigned long int                     uint32;
  _EXTENSION_ typedef unsigned long long int    uint64;

  typedef unsigned long int                     cpuaddr;

#elif defined(__SPARC__) || defined (_sparc_)
   /* ----------------------- SPARC/LEON family ---------------------------*/
   /* SPARC Big endian */
   #define _STRUCT_HIGH_BIT_FIRST_
   #undef  _STRUCT_LOW_BIT_FIRST_

   typedef unsigned char                        boolean;
   typedef signed char                          int8;
   typedef short int                            int16;
   typedef long int                             int32;
   _EXTENSION_ typedef long long int            int64;
   typedef unsigned char                        uint8;
   typedef unsigned short int                   uint16;
   typedef unsigned long int                    uint32;
   _EXTENSION_ typedef unsigned long long int   uint64;

   typedef unsigned long int                     cpuaddr;

#else  /* not any of the above */
   #error undefined processor
#endif  /* processor types */

#ifndef NULL              /* pointer to nothing */
   #define NULL ((void *) 0)
#endif

#ifndef TRUE              /* Boolean true */
   #define TRUE (1)
#endif

#ifndef FALSE              /* Boolean false */
   #define FALSE (0)
#endif

/* 
** Check Sizes 
*/
CompileTimeAssert(sizeof(uint8)==1,  TypeUint8WrongSize);
CompileTimeAssert(sizeof(uint16)==2, TypeUint16WrongSize);
CompileTimeAssert(sizeof(uint32)==4, TypeUint32WrongSize);
CompileTimeAssert(sizeof(uint64)==8, TypeUint64WrongSize);
CompileTimeAssert(sizeof(int8)==1,   Typeint8WrongSize);
CompileTimeAssert(sizeof(int16)==2,  Typeint16WrongSize);
CompileTimeAssert(sizeof(int32)==4,  Typeint32WrongSize);
CompileTimeAssert(sizeof(int64)==8,  Typeint64WrongSize);

#ifdef __cplusplus
   }
#endif

#endif  /* _common_types_ */
