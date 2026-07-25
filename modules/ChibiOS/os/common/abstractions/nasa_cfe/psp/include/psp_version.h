/*
**  $Id: psp_version.h 1.2.2.3 2014/10/01 15:41:27GMT-05:00 sstrege Exp  $
**
**
**      Copyright (c) 2004-2006, United States government as represented by the 
**      administrator of the National Aeronautics Space Administration.  
**      All rights reserved. This software(cFE) was created at NASA's Goddard 
**      Space Flight Center pursuant to government contracts.
**
**      This software may be used only pursuant to a United States government 
**      sponsored project and the United States government may not be charged
**      for use thereof. 
**
**
**
**  Purpose:
**  Provide version identifiers for the cFE Platform Support Packages (PSP).
**
*/

#ifndef _psp_version_
#define _psp_version_


/*
** Macro Definitions
*/
#define CFE_PSP_MAJOR_VERSION          1
#define CFE_PSP_MINOR_VERSION          2
#define CFE_PSP_REVISION               0
#define CFE_PSP_MISSION_REV            0

/* For backwards compatibility */
#define CFE_PSP_SUBMINOR_VERSION       CFE_PSP_REVISION 


#endif  /* _psp_version_ */

