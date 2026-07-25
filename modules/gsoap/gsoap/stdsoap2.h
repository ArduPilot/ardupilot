/*
        stdsoap2.h 2.8.114

        gSOAP runtime engine

gSOAP XML Web services tools
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under ONE of the following licenses:
GPL or the gSOAP public license.
--------------------------------------------------------------------------------
Contributors:

Wind River Systems, Inc., for the following addition licensed under the gSOAP
public license:
  - vxWorks compatible, enabled with compiler option -DVXWORKS
--------------------------------------------------------------------------------
gSOAP public license.

The contents of this file are subject to the gSOAP Public License Version 1.3
(the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at
http://www.cs.fsu.edu/~engelen/soaplicense.html
Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
for the specific language governing rights and limitations under the License.

The Initial Developer of the Original Code is Robert A. van Engelen.
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc., All Rights Reserved.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#define GSOAP_VERSION 208114

#ifdef WITH_SOAPDEFS_H
# include "soapdefs.h"          /* include user-defined stuff in soapdefs.h */
#endif

#ifndef _THREAD_SAFE
# define _THREAD_SAFE
#endif

#ifndef OPENSERVER
# ifndef _REENTRANT
#  define _REENTRANT
# endif
#endif

#ifdef _WIN32
# ifndef WIN32
#  define WIN32
# endif
#endif

/* for legacy purposes we use WIN32 macro, even when WIN64 is supported */
#ifdef _WIN64
# ifndef WIN32
#  define WIN32
# endif
#endif

#ifdef WIN32
# ifdef SOAP_STD_EXPORTS        /* dllexport the API functions and classes */
#  ifndef SOAP_STD_API
#   define SOAP_STD_API __declspec(dllexport)
#  endif
#  ifndef SOAP_CMAC
#   define SOAP_CMAC SOAP_STD_API       /* export soap struct and generated classes */
#  endif
#  ifndef SOAP_FMAC1
#   define SOAP_FMAC1 SOAP_STD_API      /* export stdsoap2.cpp API */
#  endif
#  ifndef SOAP_FMAC3
#   define SOAP_FMAC3 SOAP_STD_API      /* export soapC.cpp serializers API */
#  endif
#  ifndef SOAP_FMAC5
#   define SOAP_FMAC5 SOAP_STD_API      /* export soapClient.cpp and soapServer.cpp API */
#  endif
# endif
#endif

#ifndef SOAP_FMAC1      /* stdsoap2.h declaration macro */
# define SOAP_FMAC1
#endif

#ifndef SOAP_FMAC2      /* stdsoap2.h declaration macro */
# define SOAP_FMAC2
#endif

#ifndef SOAP_FMAC3      /* (de)serializer declaration macro */
# define SOAP_FMAC3
#endif

#ifndef SOAP_FMAC3S     /* string converter for (de)serializer declaration macro */
# define SOAP_FMAC3S SOAP_FMAC3
#endif

#ifndef SOAP_FMAC4      /* (de)serializer declaration macro */
# define SOAP_FMAC4
#endif

#ifndef SOAP_FMAC4S     /* string converter for (de)serializer declaration macro */
# define SOAP_FMAC4S SOAP_FMAC4
#endif

#ifndef SOAP_FMAC5      /* stub/skeleton declaration macro */
# define SOAP_FMAC5
#endif

#ifndef SOAP_FMAC6      /* stub/skeleton declaration macro */
# define SOAP_FMAC6
#endif

#ifndef SOAP_CMAC       /* class declaration macro */
# define SOAP_CMAC
#endif

#ifndef SOAP_NMAC       /* namespace table declaration macro */
# define SOAP_NMAC
#endif

#ifndef SOAP_SOURCE_STAMP
# define SOAP_SOURCE_STAMP(str)
#endif

#ifndef STDSOAP_H
#define STDSOAP_H

#define SOAP_XSTRINGIFY(s) SOAP_STRINGIFY(s)
#define SOAP_STRINGIFY(s) #s

#if defined(SOAPDEFS_H) && !defined(WITH_SOAPDEFS_H)
# include SOAP_XSTRINGIFY(SOAPDEFS_H)           /* include user-defined "SOAPDEFS_H" */
#endif

#if defined(__vxworks) || defined(__VXWORKS__)
# ifndef VXWORKS
#  define VXWORKS
# endif
#endif

#ifdef _WIN32_WCE
# ifndef UNDER_CE
#  define UNDER_CE _WIN32_WCE
# endif
#endif

#ifdef UNDER_CE
# ifndef WIN32
#  define WIN32
# endif
#endif

#ifdef __BORLANDC__
# ifdef __clang__
#  ifdef _WIN32
#   ifndef WIN32
#    define WIN32
#   endif
#  endif
# else
#  ifdef __WIN32__
#   ifndef WIN32
#    define WIN32
#   endif
#  endif
# endif
#endif

#if defined(__CYGWIN__) || defined(__CYGWIN32__)
# ifndef CYGWIN
#  define CYGWIN
# endif
#endif

#ifdef __SYMBIAN32__ 
# define SYMBIAN
# undef WIN32
#endif

#if defined(__hpux)
# ifndef HP_UX
#  define HP_UX
# endif
#endif

#if defined(__digital__) && defined(__unix__)
# ifndef TRU64
#  define TRU64 
# endif
#endif

#ifdef __MVS__
# ifndef OS390
#  define OS390
# endif
#endif

#if defined(__sun) && defined(__SVR4)
# ifndef SUN_OS
#  define SUN_OS
# endif
#endif

#ifdef HAVE_CONFIG_H
# include "config.h"
#else
# if defined(UNDER_CE)
#  define SOAP_BUFLEN (2048)
#  define SOAP_PTRHASH (32)
#  define SOAP_IDHASH (19)
#  define SOAP_BLKLEN (32)
#  define SOAP_TAGLEN (256)
#  define SOAP_HDRLEN (1024)
#  define SOAP_MAXDIMS (4)
#  define HAVE_SSCANF
# elif defined(CYGWIN)
#  define HAVE_POLL
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_SYS_TIMEB_H
#  define HAVE_FTIME
#  define HAVE_RAND_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define HAVE_STRERROR_R
#  define HAVE_INTTYPES_H
#  define HAVE_LOCALE_H
#  define HAVE_SOCKLEN_T
# elif defined(WIN32)
#  ifdef __BORLANDC__
#   ifdef __clang__
#    define HAVE_STRRCHR
#    define HAVE_STRTOD
#    define HAVE_SSCANF
#    define HAVE_STRTOD_L
#    define HAVE_STRTOL
#    define HAVE_SYS_TIMEB_H
#    define HAVE_FTIME
#    define HAVE_GMTIME
#    define SOAP_LONG_FORMAT "%lld"
#    define SOAP_ULONG_FORMAT "%llu"
#   else
#    define HAVE_STRRCHR
#    define HAVE_STRTOD
#    define HAVE_SSCANF
#    define HAVE_STRTOD_L
#    define HAVE_STRTOL
#    define HAVE_SYS_TIMEB_H
#    define HAVE_FTIME
#    define HAVE_GMTIME
#    define SOAP_LONG_FORMAT "%I64d"
#    define SOAP_ULONG_FORMAT "%I64u"
#   endif
#  else
#   if _MSC_VER >= 1400
#    define HAVE_SNPRINTF
#   endif
#   define HAVE_STRRCHR
#   define HAVE_STRTOD
#   define HAVE_SSCANF
#   define HAVE_STRTOD_L
#   define HAVE_STRTOL
#   define HAVE_STRTOUL
#   if _MSC_VER >= 1300
#    define HAVE_STRTOLL         /* use _strtoi64 */
#    define HAVE_STRTOULL        /* use _strtoui64 */
#   endif
#   define HAVE_SYS_TIMEB_H
#   define HAVE_FTIME
#   define HAVE_GMTIME
#   define SOAP_LONG_FORMAT "%I64d"
#   define SOAP_ULONG_FORMAT "%I64u"
#   define HAVE_LOCALE_H
#  endif
# elif defined(__APPLE__)
#  define HAVE_POLL
#  define HAVE_SNPRINTF
#  define HAVE_STRLCPY
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOD_L
#  define HAVE_SSCANF_L
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_STRTOLL
#  define HAVE_STRTOULL
#  define HAVE_RAND_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_TM_GMTOFF
#  define HAVE_GETTIMEOFDAY
#  define HAVE_LOCALTIME_R
// #  define HAVE_STRERROR_R
#  define HAVE_TIMEGM
#  define HAVE_INTTYPES_H
#  define HAVE_LOCALE_H
#  define HAVE_XLOCALE_H
#  define HAVE_RANDOM
#  define HAVE_SOCKLEN_T
# elif defined(_AIX41)
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_LOCALE_H
# elif defined(_AIX)
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_SYS_TIMEB_H
#  define HAVE_FTIME
#  define HAVE_RAND_R
#  define HAVE_GETHOSTBYNAME_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define HAVE_LOCALE_H
#  define HAVE_SOCKLEN_T
# elif defined(HP_UX)
#  include <sys/_inttypes.h>
#  if 0 /* enable if __strtoll and __strtoull are available */
extern intmax_t __strtoll(const char*, char**, int);
extern intmax_t __strtoull(const char*, char**, int);
#  define strtoll __strtoll
#  define strtoull __strtoull
#  endif
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_STRTOLL
#  define HAVE_STRTOULL
#  define HAVE_SYS_TIMEB_H
#  define HAVE_FTIME
#  define HAVE_RAND_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define HAVE_ISNAN
#  define HAVE_LOCALE_H
# elif defined(FREEBSD) || defined(__FreeBSD__) || defined(OPENBSD)
#  define HAVE_POLL
#  define HAVE_SNPRINTF
#  define HAVE_STRLCPY
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOD_L
#  define HAVE_SSCANF_L
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_STRTOLL
#  define HAVE_STRTOULL
#  define HAVE_GETTIMEOFDAY
#  define HAVE_RAND_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define HAVE_STRERROR_R
#  define SOAP_LONG_FORMAT "%qd"
#  define SOAP_ULONG_FORMAT "%qu"
#  define HAVE_ISNAN
#  define HAVE_ISINF
#  define HAVE_LOCALE_H
#  define HAVE_XLOCALE_H
#  define HAVE_RANDOM
#  define HAVE_SOCKLEN_T
# elif defined(__VMS)
#  include <ioctl.h>
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_SYS_TIMEB_H
#  define HAVE_FTIME
#  define HAVE_RAND_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
# elif defined(TRU64)
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_GETTIMEOFDAY
#  define HAVE_RAND_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define __USE_STD_IOSTREAM
#  define SOAP_LONG_FORMAT "%ld"
#  define SOAP_ULONG_FORMAT "%lu"
#  define HAVE_LOCALE_H
# elif defined(MAC_CARBON)
#  define WITH_NOIO
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_SYS_TIMEB_H
#  define HAVE_FTIME
#  define HAVE_RAND_R
#  define HAVE_GETHOSTBYNAME_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define HAVE_STRERROR_R
# elif defined(SYMBIAN)
#  define WITH_LEAN
#  define WITH_NONAMESPACES
#  define HAVE_STRTOD   /* use STRTOD since sscanf doesn't seem to work */
#  include <e32def.h>
#  include <sys/ioctl.h>
# elif defined(VXWORKS)
#  ifdef _WRS_KERNEL
#   define _POSIX_THREADS 1
#  endif
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_GMTIME
#  define HAVE_LOCALTIME
#  define HAVE_MKTIME
# elif defined(OS390)
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_SYS_TIMEB_H
#  define HAVE_FTIME
#  define HAVE_RAND_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define HAVE_SOCKLEN_T
# elif defined(AS400)
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_SYS_TIMEB_H
#  define HAVE_FTIME
#  define HAVE_RAND_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
# elif defined(__QNX__) || defined(QNX)
/* QNX does not have a working version of strtof */
#  undef HAVE_STRTOF
#  define HAVE_POLL
#  define HAVE_SNPRINTF
#  define HAVE_STRLCPY
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_STRTOLL
#  define HAVE_STRTOULL
#  define HAVE_GETTIMEOFDAY
#  define HAVE_RAND_R
#  define HAVE_GETHOSTBYNAME_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define HAVE_STRERROR_R
#  define HAVE_SOCKLEN_T
# elif defined(SUN_OS)
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_SYS_TIMEB_H
#  define HAVE_FTIME
#  define HAVE_RAND_R
#  define HAVE_GETHOSTBYNAME_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
# elif defined(__GLIBC__) || defined(__GNU__) || defined(__GNUC__)
#  define HAVE_POLL
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_STRTOLL
#  define HAVE_STRTOULL
#  define HAVE_GETTIMEOFDAY
#  define HAVE_RAND_R
#  define HAVE_GETHOSTBYNAME_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define HAVE_STRERROR_R
#  define HAVE_TIMEGM
#  define HAVE_ISNAN
#  define HAVE_ISINF
#  if !defined(__GNUC__) || __GNUC__ >= 4 /* gcc 3 and earlier often refuse to compile _l functions */
#   define HAVE_STRTOD_L
#   define HAVE_SSCANF_L
#   define HAVE_LOCALE_H
#  endif
#  define HAVE_SOCKLEN_T
# else /* Default assumptions for supported library functions when not including config.h */
#  ifndef WITH_C_LOCALE
#   ifndef WITH_NO_C_LOCALE
#    define WITH_NO_C_LOCALE /* turn locale support off by default */
#   endif
#  endif
#  define HAVE_SNPRINTF
#  define HAVE_STRRCHR
#  define HAVE_STRTOD
#  define HAVE_SSCANF
#  define HAVE_STRTOL
#  define HAVE_STRTOUL
#  define HAVE_STRTOLL
#  define HAVE_STRTOULL
#  define HAVE_GETTIMEOFDAY
#  define HAVE_RAND_R
#  define HAVE_GETHOSTBYNAME_R
#  define HAVE_GMTIME_R
#  define HAVE_ASCTIME_R
#  define HAVE_LOCALTIME_R
#  define HAVE_STRERROR_R
#  define HAVE_LOCALE_H
# endif
#endif

#ifdef WITH_LEANER
# ifndef WITH_LEAN
#  define WITH_LEAN
# endif
#endif

/* gSOAP 2.7.15 and higher: always use FAST and retain CDATA in literal XML, unless WITH_LEAN or WITH_NOCDATA */
#ifndef WITH_LEAN
# ifndef WITH_FAST
#  define WITH_FAST
# endif
# ifndef WITH_NOCDATA
#  ifndef WITH_CDATA
#   define WITH_CDATA
#  endif
# endif
#endif

#if !defined(__cplusplus)
/* allowing empty struct/union in C is a GNU extension */
# if !defined(__GNU__) && !defined(__GNUC__)
#  define WITH_NOEMPTYSTRUCT
# endif
#endif

/* silence clang's C99 variadic macro warnings */
#ifdef __clang__ 
# pragma clang diagnostic ignored "-Wvariadic-macros" 
#endif 

#if defined(WITH_PURE_VIRTUAL)
# define SOAP_PURE_VIRTUAL = 0
# define SOAP_PURE_VIRTUAL_COPY = 0
#elif defined(WITH_DEFAULT_VIRTUAL)
# define SOAP_PURE_VIRTUAL { return SOAP_NO_METHOD; }
# define SOAP_PURE_VIRTUAL_COPY
#else
# define SOAP_PURE_VIRTUAL
# define SOAP_PURE_VIRTUAL_COPY
#endif

/* older OpenVMS TCP/IP stacks cannot handle 65536 bytes */
#ifdef __VMS
# ifndef SOAP_BUFLEN
#  define SOAP_BUFLEN (65535)
# endif
#endif

/* small buffer, to accelerate base64 and hex binary output */
#ifndef SOAP_BINARY_BUFLEN
# define SOAP_BINARY_BUFLEN (128)
#endif

/* if we have locale.h then we should use it WITH_C_LOCALE enabled to avoid decimal point conversion issues */
#ifdef HAVE_LOCALE_H
# ifndef WITH_NO_C_LOCALE
#  ifndef WITH_C_LOCALE
#   define WITH_C_LOCALE
#  endif
# endif
#endif

/* MinGW does not support uselocale() and xlocale.h and gettimeofday() */
#if defined(__MINGW32__) || defined(__MINGW64__)
# if !defined(WITH_NO_C_LOCALE)
#  define WITH_NO_C_LOCALE
# endif
# undef HAVE_GETTIMEOFDAY
#endif

/* user can set WITH_NO_C_LOCALE to force removal of locale (e.g. in case of compiler errors) */
#ifdef WITH_NO_C_LOCALE
# undef WITH_C_LOCALE
#endif

#ifndef WITH_NOSTDLIB
# include <stdlib.h>
# include <stddef.h>
# include <stdio.h>
# include <string.h>
# if !defined(HAVE_CONFIG_H) || defined(HAVE_CTYPE_H)
#  include <ctype.h>
# endif
# if !defined(HAVE_CONFIG_H) || defined(HAVE_LIMITS_H)
#  include <limits.h>   /* for MB_LEN_MAX */
# endif
# if !defined(HAVE_CONFIG_H) || defined(HAVE_FLOAT_H)
#  include <float.h>    /* for INFINITY */
# endif
# if !defined(HAVE_CONFIG_H) || defined(HAVE_MATH_H)
#  include <math.h>    /* for isnan() and isinf() */
# endif
#endif

/* force inclusion of xlocale.h when WITH_INCLUDE_XLOCALE_H is defined by the user for systems that require xlocale.h */
#ifdef WITH_INCLUDE_XLOCALE_H
# ifndef HAVE_XLOCALE_H
#  define HAVE_XLOCALE_H
# endif
#endif

#ifdef WITH_C_LOCALE
# include <locale.h>
# if defined(WIN32) && !defined(CYGWIN)
#  define SOAP_LOCALE_T _locale_t
#  define SOAP_LOCALE(soap) ((soap)->c_locale ? (soap)->c_locale : ((soap)->c_locale = _create_locale(LC_ALL, "C")))
#  define SOAP_FREELOCALE(soap) (void)((soap)->c_locale && (_free_locale((soap)->c_locale), ((soap)->c_locale = NULL)))
# else
#  if defined(HAVE_XLOCALE_H)
#   include <xlocale.h>
#  endif
#  define SOAP_LOCALE_T locale_t
#  define SOAP_LOCALE(soap) ((soap)->c_locale ? (soap)->c_locale : ((soap)->c_locale = newlocale(LC_ALL_MASK, "C", NULL)))
#  define SOAP_FREELOCALE(soap) (void)((soap)->c_locale && (freelocale((soap)->c_locale), ((soap)->c_locale = NULL)))
#  if defined(CYGWIN)
#   undef HAVE_STRTOF_L /* Cygwin does not support strtof_l strtod_l */
#   undef HAVE_STRTOD_L
#  endif
# endif
#else
# undef HAVE_STRTOF_L
# undef HAVE_STRTOD_L
# undef HAVE_SSCANF_L
#endif

#ifdef TANDEM_NONSTOP /* Support for Guardian */
# define SOAP_BUFLEN (32767)
/*# define WITH_NOSTDLIB */ /* uncommment to remove stdlib dependences */
# define WITH_NOIO      /* no IO dependences, e.g. remove TCP/IP */
# define int32_t int
# define int64_t long long
# define LONG64 long long
# define ULONG64 unsigned long long
# define DBL_PINFTY (1.1579208923716189e77)
# undef HAVE_GMTIME_R
# undef HAVE_ASCTIME_R
# undef HAVE_LOCALTIME_R
# undef HAVE_SNPRINTF
# define SOAP_BUFLEN (32767)
# define SOAP_SOCKET short
#pragma nolist
# include <sys\param.h>
# include <sys\socket.h>
# include <netinet\in.h>
# include <netdb.h>
# include <stdio.h>
# include <fcntl.h>                                           
# include <string.h>                                          
# include <stdlib.h>                                          
# include <memory.h>                                          
# include <errno.h>                                           
# include <cextdecs.h(TIME,FILE_CLOSE_,AWAITIOX,DELAY,FILEINFO,FILE_GETINFO_)>
# define INET_ERROR 4294967295                                
#pragma list                                                  
#elif defined(__TANDEM) /* Support for OSS */
# define int32_t int
# define SOAP_BUFLEN (32767)
#endif                                                        

#ifdef WITH_NTLM
# include <ntlm.h>
#endif

#ifdef HAVE_POLL
# include <poll.h>
#endif

#ifdef __cplusplus
# include <new>
# include <iterator>
# include <memory>
# ifndef WITH_COMPAT
#  include <string>
#  include <iostream>
# endif
#endif

#ifdef WITH_NOHTTP
# ifndef WITH_NOIO
#  define WITH_NOIO
#  undef WITH_COOKIES
# endif
#endif

/* Suggestion when SOAP_FD_EXCEEDED error occurs:
   Some systems allow increasing FD_SETSIZE before including sys/types.h:
#define FD_SETSIZE (2048)
*/

#ifndef UNDER_CE
# ifndef WITH_NOIO
#  include <errno.h>
#  include <sys/types.h>
# endif
# ifndef WITH_LEAN
#  ifdef HAVE_SYS_TIMEB_H
#   ifdef HAVE_FTIME
#    include <sys/timeb.h>              /* for ftime() */
#   endif
#  endif
#  include <time.h>
# endif
#endif

#ifdef OPENSERVER
# include <sys/socket.h>
# include <sys/stream.h>
# include <sys/protosw.h>
  extern int h_errno;
#endif

#ifdef HAVE_GETTIMEOFDAY
# ifndef WIN32
#  ifdef VXWORKS
#   ifdef _WRS_KERNEL
#    include <sys/times.h>
#   endif
# else
#   include <sys/time.h>                /* for timeval and gettimeofday() */
#  endif
# endif
#endif

#ifndef WITH_NOIO
# ifndef WIN32
#  include <sys/socket.h>
#  ifdef VXWORKS
#   include <sockLib.h>
#   include <selectLib.h>
#   ifndef _WRS_KERNEL
#    include <strings.h>
#   endif
#  else
#   ifndef SYMBIAN
#    include <strings.h>
#   endif
#  endif
#  ifdef SUN_OS
#   include <sys/stream.h>             /* SUN */
#   include <sys/socketvar.h>          /* only needed with SUN < 2.8 ? */
#  endif
#  ifdef VXWORKS
#   ifdef _WRS_KERNEL
#    include <sys/times.h>
#   endif
#  endif
#  include <netinet/in.h>
#  ifdef OS390
#   include <netinet/tcp_var.h>
#  else
#   include <netinet/tcp.h>          /* TCP_NODELAY, TCP_FASTOPEN */
#  endif
#  include <arpa/inet.h>
# endif
#endif

#ifdef WIN32
# define SOAP_WINSOCKINT int
#else
# define SOAP_WINSOCKINT size_t
#endif

#ifdef WIN32
# undef WITH_SELF_PIPE
#endif

#if defined(WITH_IPV6_V6ONLY)
# ifndef WITH_IPV6
#  define WITH_IPV6
# endif
#endif

#ifdef WIN32
# ifndef UNDER_CE
#  include <io.h>
#  include <fcntl.h>
# endif
// When you get macro redefinition errors when compiling the code below, then:
// a) try arrange your includes so <windows.h> is included after "stdsoap2.h"
// b) or define _WINSOCKAPI_ first:
//    #define _WINSOCKAPI_    // stops windows.h including winsock.h
//    #include <windows.h>
//    #include "stdsoap2.h"
// c) or compile with the -DWIN32_LEAN_AND_MEAN switch
# include <winsock2.h> // Visual Studio 2005 users: install Platform SDK (R2)
# include <ws2tcpip.h>
// # define _WSPIAPI_COUNTOF // DEV NOTE: enble to fix problems with VC6
// # include <wspiapi.h>
# include <ws2spi.h> // DEV NOTE: replaces older wspiapi.h above
# ifdef WITH_IPV6
#  define SOAP_GAI_STRERROR gai_strerrorA
# endif
#else
# ifdef VXWORKS
#  include <hostLib.h>
#  include <ioctl.h>
#  include <ioLib.h>
# endif
# ifndef WITH_NOIO
#  include <netdb.h>
#  include <netinet/in.h>
#  include <unistd.h>
#  include <fcntl.h>
#  ifdef _AIX41
#   include <sys/select.h>
#  endif
# endif
#endif

#ifdef WITH_FASTCGI
# include <fcgi_stdio.h>
#endif

#ifdef WITH_OPENSSL
# ifdef __VMS
#  pragma names save
#  pragma names uppercase
# endif
# undef WITH_GNUTLS
# define OPENSSL_NO_KRB5
# include <openssl/bio.h>
# include <openssl/err.h>
# include <openssl/rand.h>
# include <openssl/ssl.h>
# include <openssl/rsa.h>
# include <openssl/dh.h>
# include <openssl/asn1.h>
# include <openssl/x509v3.h>
# ifndef ALLOW_OLD_VERSIONS
#  if (OPENSSL_VERSION_NUMBER < 0x00905100L)
#   error "Must use OpenSSL 0.9.6 or later"
#  endif
# endif
# ifdef __VMS
#  pragma names restore
# endif
#endif

#ifdef WITH_GNUTLS
# include <gnutls/gnutls.h>
# include <gnutls/x509.h>
# if GNUTLS_VERSION_NUMBER < 0x020b00
/* deprecated since GNUTLS 2.11.0 */
#  include <gcrypt.h>
# endif
# ifndef HAVE_PTHREAD_H
#  ifdef _POSIX_THREADS
#   define HAVE_PTHREAD_H /* make GNUTLS thread safe */
#  endif
# endif
#endif

#ifdef WITH_SYSTEMSSL
# include <gskssl.h>
#endif

#ifdef WITH_GZIP
# ifndef WITH_ZLIB
#  define WITH_ZLIB
# endif
#endif

#ifdef WITH_CASEINSENSITIVETAGS
# define SOAP_STRCMP soap_tag_cmp       /* case insensitive XML element/attribute names */
#else
# define SOAP_STRCMP strcmp             /* case sensitive XML element/attribute names */
#endif

#ifdef WITH_ZLIB
# include <zlib.h>
#endif

/* #define DEBUG */ /* Uncomment to debug sending (in file SENT.log) receiving (in file RECV.log) and internal operations (in file TEST.log) */

/* #define DEBUG_STAMP */ /* Uncomment to debug sending (in file SENT.log) receiving (in file RECV.log) and time-stamped operations (in file TEST.log) */

#ifdef __cplusplus
extern "C" {
#endif

/* Portability (X/Open, BSD sockets etc): define SOAP_SOCKLEN_T as socklen_t or int or ... */
#if defined(HAVE_SOCKLEN_T)
# define SOAP_SOCKLEN_T socklen_t
#elif defined(_AIX) || defined(AIX) || defined(HP_UX)
# if defined(_AIX43)
#  define SOAP_SOCKLEN_T socklen_t
# else
#  define SOAP_SOCKLEN_T int
# endif
#elif defined(SOCKLEN_T)
# define SOAP_SOCKLEN_T SOCKLEN_T
#elif defined(__socklen_t_defined) || defined(_SOCKLEN_T) || defined(__ANDROID__) || !defined(_GNU_SOURCE) || (!(~_GNU_SOURCE+1) && !defined(_POSIX_C_SOURCE) && !defined(_XOPEN_SOURCE)) || _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600
# define SOAP_SOCKLEN_T socklen_t
#elif defined(IRIX) || defined(WIN32) || defined(SUN_OS) || defined(OPENSERVER) || defined(TRU64) || defined(VXWORKS)
# define SOAP_SOCKLEN_T int
#elif !defined(SOAP_SOCKLEN_T)
# define SOAP_SOCKLEN_T socklen_t
#endif

/* AIX DCE threads portability: define SOAP_FUNC_R_ERR gmtime_r and localtime_r err ret val as -1 */
#ifdef _AIX32_THREADS
# define SOAP_FUNC_R_ERR (-1)
#elif !defined(SOAP_FUNC_R_ERR)
# define SOAP_FUNC_R_ERR (NULL)
#endif

#ifndef SOAP_SOCKET
# ifdef WIN32
#  define SOAP_SOCKET SOCKET
#  define soap_closesocket(n) closesocket(n)
# else
#  define SOAP_SOCKET int
#  define soap_closesocket(n) close(n)
# endif
#endif

#define SOAP_INVALID_SOCKET ((SOAP_SOCKET)-1)
#define soap_valid_socket(sk) ((sk) != SOAP_INVALID_SOCKET)

#define SOAP_SHUT_WR (1)
#define SOAP_SHUT_RDWR (2)

#ifndef SOAP_GAI_STRERROR
# define SOAP_GAI_STRERROR gai_strerror
#endif

#ifndef FD_SETSIZE
# define FD_SETSIZE (1024)
#endif

#ifdef WITH_LEAN
# define SOAP_CHK_EOF SOAP_EOF
#else
# define SOAP_CHK_EOF (soap->error ? soap->error : SOAP_EOF)
#endif

#ifdef __cplusplus
# ifndef __STDC_FORMAT_MACROS
#  define __STDC_FORMAT_MACROS
# endif
#endif

#if defined(SYMBIAN)
# warning "Symbian build: removing 64 bit integer support"
# define LONG64 long
# define ULONG64 unsigned LONG64
# ifndef SOAP_LONG_FORMAT
#  define SOAP_LONG_FORMAT "%ld"
# endif
# ifndef SOAP_ULONG_FORMAT
#  define SOAP_ULONG_FORMAT "%lu"
# endif
#elif !defined(WIN32) || defined(CYGWIN) || defined(__GLIBC__) || defined(__GNU__) || defined(__GNUC__)
# ifndef LONG64
#  if defined(HAVE_INTTYPES_H)
#   include <inttypes.h>
#   define LONG64 int64_t
#   define ULONG64 uint64_t
#   if defined(PRId64) && defined(PRIu64)
#    ifndef SOAP_LONG_FORMAT
#     define SOAP_LONG_FORMAT "%" PRId64
#    endif
#    ifndef SOAP_ULONG_FORMAT
#     define SOAP_ULONG_FORMAT "%" PRIu64
#    endif
#   endif
#  elif defined(HAVE_SYS_INTTYPES_H)
#   include <sys/inttypes.h>
#   define LONG64 int64_t
#   define ULONG64 uint64_t
#   if defined(PRId64) && defined(PRIu64)
#    ifndef SOAP_LONG_FORMAT
#     define SOAP_LONG_FORMAT "%" PRId64
#    endif
#    ifndef SOAP_ULONG_FORMAT
#     define SOAP_ULONG_FORMAT "%" PRIu64
#    endif
#   endif
#  elif defined(HAVE_STDINT_H)
#   include <stdint.h>
#   define LONG64 int64_t
#   define ULONG64 uint64_t
#   if defined(PRId64) && defined(PRIu64)
#    ifndef SOAP_LONG_FORMAT
#     define SOAP_LONG_FORMAT "%" PRId64
#    endif
#    ifndef SOAP_ULONG_FORMAT
#     define SOAP_ULONG_FORMAT "%" PRIu64
#    endif
#   endif
#  elif defined(CYGWIN) || defined(__GLIBC__)
#   include <bits/wordsize.h>
#   if (__WORDSIZE == 64)
#    define LONG64 int64_t
#    define ULONG64 uint64_t
#    ifndef SOAP_LONG_FORMAT
#     define SOAP_LONG_FORMAT "%ld"
#    endif
#    ifndef SOAP_ULONG_FORMAT
#     define SOAP_ULONG_FORMAT "%lu"
#    endif
#   else
#    define LONG64 long long
#    define ULONG64 unsigned LONG64
#   endif
#  else
#   define LONG64 long long
#   define ULONG64 unsigned LONG64
#  endif
# endif
#elif defined(UNDER_CE)
# define LONG64 __int64
# define ULONG64 unsigned LONG64
#elif defined(__BORLANDC__)
# define LONG64 __int64
# define ULONG64 unsigned LONG64
#endif

#ifndef SOAP_LONG_FORMAT
# define SOAP_LONG_FORMAT "%lld"        /* printf format for 64 bit long long ints */
#endif

#ifndef SOAP_ULONG_FORMAT
# define SOAP_ULONG_FORMAT "%llu"       /* printf format for unsigned 64 bit long long ints */
#endif

#if defined(WIN32) && !defined(CYGWIN)
# define soap_int32 __int32
#elif defined(SYMBIAN)
# define soap_int32 long
#elif defined(_AIX) || defined(AIX)
# if defined(_AIX43)
#  define soap_int32 int32_t
# else
#  define soap_int32 signed int
# endif
#else
# define soap_int32 int32_t
#endif

#ifdef WIN32
# define SOAP_ERANGE ERANGE
# define SOAP_EINTR WSAEINTR
# define SOAP_EAGAIN WSAEWOULDBLOCK
# define SOAP_EWOULDBLOCK WSAEWOULDBLOCK
# define SOAP_EINPROGRESS WSAEINPROGRESS
# define SOAP_EADDRINUSE WSAEADDRINUSE
# define SOAP_ECONNREFUSED WSAECONNREFUSED
#else
# ifdef ERANGE
#  define SOAP_ERANGE ERANGE
# else
#  define SOAP_ERANGE (34)
# endif
# define SOAP_EINTR EINTR
# define SOAP_EAGAIN EAGAIN
# define SOAP_EADDRINUSE EADDRINUSE
# define SOAP_ECONNREFUSED ECONNREFUSED
# ifdef SYMBIAN
#  define SOAP_EWOULDBLOCK 9898
#  define SOAP_EINPROGRESS 9899
# else
#  define SOAP_EWOULDBLOCK EWOULDBLOCK
#  define SOAP_EINPROGRESS EINPROGRESS
# endif
#endif

#ifdef WIN32
# ifdef UNDER_CE
#  define soap_errno GetLastError()
#  define soap_socket_errno GetLastError()
#  define soap_reset_errno SetLastError(0)
# else
#  define soap_errno errno
#  define soap_socket_errno WSAGetLastError()
#  define soap_reset_errno _set_errno(0)
# endif
#else
# ifndef WITH_NOIO
#  define soap_errno errno
#  define soap_socket_errno errno
#  define soap_reset_errno (errno = ENOENT)
# else
#  define soap_errno 0
#  define soap_socket_errno 0
#  define soap_reset_errno
# endif
#endif

#ifndef SOAP_BUFLEN
# if !defined(WITH_LEAN)
#  define SOAP_BUFLEN (65536) /* buffer length for socket packets, so don't make this too small */
# else
#  define SOAP_BUFLEN  (2048) /* lean size */
# endif
#endif
#ifndef SOAP_LABLEN
# define SOAP_LABLEN     (256) /* initial look-aside buffer length */
#endif
#ifndef SOAP_PTRBLK
# define SOAP_PTRBLK     (32) /* block allocation for pointer hash table chains */
#endif
#ifndef SOAP_PTRHASH
# ifndef WITH_LEAN
#  define SOAP_PTRHASH (4096) /* size of pointer analysis hash table (must be power of 2) */
# else
#  define SOAP_PTRHASH   (32)
# endif
#endif
#ifndef SOAP_IDHASH
# ifndef WITH_LEAN
#  define SOAP_IDHASH  (1999) /* 1999, 8191: prime size of hash table for parsed id/ref */
# else
#  define SOAP_IDHASH    (19) /* 19, 199: small prime */
# endif
#endif
#ifndef SOAP_BLKLEN
# ifndef WITH_LEAN
#  define SOAP_BLKLEN   (256) /* to collect long strings and XML attributes */
# else
#  define SOAP_BLKLEN    (32)
# endif
#endif
#ifndef SOAP_TAGLEN
# ifndef WITH_LEAN
#  define SOAP_TAGLEN  (1024) /* maximum length of XML element tag/attribute name or host/path name + 1 */
# else
#  define SOAP_TAGLEN   (256)
# endif
#endif
#ifndef SOAP_HDRLEN
# ifndef WITH_LEAN
#  define SOAP_HDRLEN  (8192) /* maximum length of HTTP header line (must be >=4096 to read cookies) */
# else
#  define SOAP_HDRLEN  (1024)
# endif
#endif
#ifndef SOAP_TMPLEN
# ifndef WITH_LEAN
#  define SOAP_TMPLEN  (2048) /* maximum length of msgbuf and tmpbuf short message buffers, must be >=1024 */
# else
#  define SOAP_TMPLEN  (1024)
# endif
#endif
#ifndef SOAP_MAXDIMS
# ifndef WITH_LEAN
#  define SOAP_MAXDIMS   (16) /* maximum array dimensions (array nestings) must be less than 64 to protect soap->tmpbuf */
# else
#  define SOAP_MAXDIMS    (4)
# endif
#endif
#ifndef SOAP_MAXPTRS
# ifndef WITH_LEAN
#  define SOAP_MAXPTRS    (4) /* maximum depth + 1 of id-ref deserialized pointer types (int* has depth 0, int*** has depth 2) */
# else
#  define SOAP_MAXPTRS    (2)
# endif
#endif

#ifndef SOAP_MAXALLOCSIZE
# define SOAP_MAXALLOCSIZE (0) /* max size that malloc() can handle, zero for no limit */
#endif

#ifndef SOAP_MAXLOGS
# define SOAP_MAXLOGS     (3) /* max number of debug logs per struct soap environment */
# define SOAP_INDEX_RECV  (0)
# define SOAP_INDEX_SENT  (1)
# define SOAP_INDEX_TEST  (2)
#endif

/* Tag name of multiref elements in SOAP 1.1 encoding */
#ifndef SOAP_MULTIREFTAG
# define SOAP_MULTIREFTAG "id"
#endif

/* href-id value base name in multiref SOAP encoding */
#ifndef SOAP_BASEREFNAME
# define SOAP_BASEREFNAME "_"
#endif

/* Max number of EINTR interrupts to ignore while poll/select for pending activity on a socket */
/* Each EINTR ignored may increase the I/O blocking time by at most one second */
#ifndef SOAP_MAXEINTR
# define SOAP_MAXEINTR (10)
#endif

/* SOAP_MAXKEEPALIVE: Max iterations in soap_serve() to keep server connection alive */
#ifndef SOAP_MAXKEEPALIVE
# define SOAP_MAXKEEPALIVE (100)
#endif

/* SOAP_MAXARRAYSIZE: Trusted total max size of an inbound SOAP Array.
   Arrays of larger size are not pre-allocated, but deserialized
   on an element-by-element basis until XML validation contrains kick in.
   This macro only affects the efficiency of parsing SOAP arrays.
*/
#ifndef SOAP_MAXARRAYSIZE
# define SOAP_MAXARRAYSIZE (100000)
#endif

/* SOAP_MAXDIMESIZE: Trusted max size of inbound DIME data.
   Increase if necessary to allow larger attachments, or decrease when server
   resources are limited.  This is to deny senders to allocate more than 8 MB
   at the receiver without actually sending the whole message.
*/
#ifndef SOAP_MAXDIMESIZE
# define SOAP_MAXDIMESIZE (8*1048576) /* 8 MB */
#endif

/* SOAP_MAXINFLATESIZE: Trusted inflated content size.
   Larger content is subject to the SOAP_MINDEFLATERATIO constraint.
   If SOAP_MINDEFLATERATIO is 1.0, SOAP_MAXINFLATESIZE is always the max
   size of uncompressed content.
*/
#ifndef SOAP_MAXINFLATESIZE
# define SOAP_MAXINFLATESIZE (1*1048576) /* 1 MB */
#endif

/* SOAP_MINDEFLATERATIO: Trusted deflation ratio after SOAP_MAXINFLATESIZE is reached.
   Trust when compressed / deflated > SOAP_MINDEFLATERATIO
   Sets a ratio > 0.00096899224806 (1032:1)
   According to the zlib site: the limit (1032:1) comes from the fact that one
   length/distance pair can represent at most 258 output bytes. A length
   requires at least one bit and a distance requires at least one bit, so two
   bits in can give 258 bytes out, or eight bits in give 1032 bytes out. A
   dynamic block has no length restriction, so you could get arbitrarily close
   to the limit of 1032:1.
*/
#ifndef SOAP_MINDEFLATERATIO
# define SOAP_MINDEFLATERATIO (1.0/1032.0) /* ratio of deflated/inflated */
#endif

/* maximum XML nesting depth level allowed for inbound XML parsing, must be greater than zero (0) */
#ifndef SOAP_MAXLEVEL
# define SOAP_MAXLEVEL (10000)
#endif

/* maximum string content length if not already constrained by XML schema validation maxLength constraints, zero or negative means unlimited string lengths are allowed unless restricted by XML schema maxLength */ 
#ifndef SOAP_MAXLENGTH
# define SOAP_MAXLENGTH (0)
#endif

/* maximum number of array or container elements, must be greater than zero (0) */
#ifndef SOAP_MAXOCCURS
# define SOAP_MAXOCCURS (100000)
#endif

#ifdef VXWORKS
# ifdef WMW_RPM_IO
#  include "httpLib.h"
# endif
# ifdef __INCmathh 
#  include <private/mathP.h>
#  ifndef HAVE_ISNAN
#   define HAVE_ISNAN
#  endif
#  ifndef soap_isnan
#   define soap_isnan(num) isNan(num)
#  endif
# endif
# ifdef WM_SECURE_KEY_STORAGE
#  include <ipcom_key_db.h>
# endif
#endif

#ifdef WIN32 
# ifndef HAVE_ISNAN
#  define HAVE_ISNAN
# endif
# define soap_isnan(n) _isnan(n)
# ifndef HAVE_ISINF
#  define HAVE_ISINF
# endif
# define soap_isinf(n) (!_finite(n))
#endif

#ifdef SUN_OS
# ifndef HAVE_ISNAN
#  define HAVE_ISNAN
# endif
#endif

#ifdef __APPLE__
# ifndef HAVE_ISNAN
#  define HAVE_ISNAN
# endif
# ifndef HAVE_ISINF
#  define HAVE_ISINF
# endif
#endif

#if !defined(HAVE_ISNAN) && (defined(_MATH_H) || defined(_MATH_INCLUDED))
# define HAVE_ISNAN
#endif

union soap_double_nan {struct {unsigned int n1, n2;} iv; double dv; float fv;};
extern const union soap_double_nan soap_double_nan;
extern const char soap_base64o[], soap_base64i[];

#ifdef VXWORKS
# ifndef FLT_MAX
#  define FLT_MAX _ARCH_FLT_MAX
# endif
# ifndef DBL_MAX
#  define DBL_MAX _ARCH_DBL_MAX
# endif
#endif

#ifndef FLT_NAN
# define FLT_NAN (soap_double_nan.fv)
#endif

#ifndef FLT_PINFTY
# if defined(INFINITY)
#  define FLT_PINFTY INFINITY
# elif defined(FLT_MAX)
#  define FLT_PINFTY FLT_MAX
# elif defined(HUGE_VALF)
#  define FLT_PINFTY (float)HUGE_VALF
# elif defined(HUGE_VAL)
#  define FLT_PINFTY (float)HUGE_VAL
# elif defined(FLOAT_MAX)
#  define FLT_PINFTY FLOAT_MAX
# else
#  define FLT_PINFTY (3.40282347e+38F)
# endif
#endif

#ifndef FLT_NINFTY
# define FLT_NINFTY (-FLT_PINFTY)
#endif

#ifndef DBL_NAN
# define DBL_NAN (soap_double_nan.dv)
#endif

#ifndef DBL_PINFTY
# if defined(INFINITY)
#  define DBL_PINFTY INFINITY
# elif defined(DBL_MAX)
#  define DBL_PINFTY DBL_MAX
# elif defined(HUGE_VALF)
#  define DBL_PINFTY (double)HUGE_VALF
# elif defined(HUGE_VAL)
#  define DBL_PINFTY (double)HUGE_VAL
# elif defined(DOUBLE_MAX)
#  define DBL_PINFTY DOUBLE_MAX
# else
#  define DBL_PINFTY (1.7976931348623157e+308)
# endif
#endif

#ifndef DBL_NINFTY
# define DBL_NINFTY (-DBL_PINFTY)
#endif

#ifndef soap_isnan
# ifdef HAVE_ISNAN
#  define soap_isnan(n) isnan(n)
# else
#  define soap_isnan(n) ((n) != (n))
# endif
#endif

#ifndef soap_isinf
# ifdef HAVE_ISINF
#  define soap_isinf(n) isinf(n)
# else
#  define soap_isinf(n) (!soap_isnan(n) && soap_isnan((n) - (n)))
# endif
#endif

#define soap_ispinfd(n) ((n) > 0 && soap_isinf(n))
#define soap_ispinff(n) ((n) > 0 && soap_isinf(n))
#define soap_isninfd(n) ((n) < 0 && soap_isinf(n))
#define soap_isninff(n) ((n) < 0 && soap_isinf(n))

/* Safer str & mem functions */

/* The gSOAP code uses guards to ensure that these functions are well behaved
   and do not raise errors. Therefore, the WIN _s functions should never
   execute the "invalid parameter handler".
*/

/* use safer snprintf if possible or guard sprintf against overrun (assumes no variadic macros) */
# ifdef HAVE_SNPRINTF
#  if defined(_MSC_VER) && _MSC_VER >= 1400
#   define SOAP_SNPRINTF(buf, len, num) void)_snprintf_s((buf), (len), _TRUNCATE
#   define SOAP_SNPRINTF_SAFE(buf, len) void)_snprintf_s((buf), (len), _TRUNCATE
#  else
#   define SOAP_SNPRINTF(buf, len, num) void)snprintf((buf), (len)
#   define SOAP_SNPRINTF_SAFE(buf, len) void)snprintf((buf), (len)
#  endif
# else
#  define SOAP_SNPRINTF(buf, len, num) (len) <= (num)) ? (void)((buf)[0] = '\0') : (void)sprintf((buf)
#  define SOAP_SNPRINTF_SAFE(buf, len) void)sprintf((buf)
# endif

/* copy string (truncating the result, strings must not be NULL) */
#if defined(_MSC_VER) && _MSC_VER >= 1400
# define soap_strcpy(buf, len, src) (void)strncpy_s((buf), (len), (src), _TRUNCATE)
#elif defined(HAVE_STRLCPY)
# define soap_strcpy(buf, len, src) (void)strlcpy((buf), (src), (len))
#else
# define soap_strcpy(buf, len, src) (void)(strncpy((buf), (src), (len)), (buf)[(len) - 1] = '\0')
#endif

/* concat string (truncating the result, strings must not be NULL) */
#if defined(_MSC_VER) && _MSC_VER >= 1400
# define soap_strcat(buf, len, src) (void)strncat_s((buf), (len), (src), _TRUNCATE)
#elif defined(HAVE_STRLCAT)
# define soap_strcat(buf, len, src) (void)strlcat((buf), (src), (len))
#else
SOAP_FMAC1 void SOAP_FMAC2 soap_strcat(char *buf, size_t len, const char *src);
#endif

/* copy string up to num chars (sets string to empty on overrun and returns nonzero, zero if OK) */
#if defined(_MSC_VER) && _MSC_VER >= 1400
# define soap_strncpy(buf, len, src, num) ((buf) == NULL || ((size_t)(len) > (size_t)(num) ? strncpy_s((buf), (len), (src), (num)) : ((buf)[0] = '\0', 1)))
#else
# define soap_strncpy(buf, len, src, num) ((buf) == NULL || ((size_t)(len) > (size_t)(num) ? (strncpy((buf), (src), (num)), (buf)[(size_t)(num)] = '\0') : ((buf)[0] = '\0', 1)))
#endif

/* concat string up to n chars (leaves destination intact on overrun and returns nonzero, zero if OK) */
#if defined(_MSC_VER) && _MSC_VER >= 1400
# define soap_strncat(buf, len, src, num) ((buf) == NULL || ((size_t)(len) > strlen((buf)) + (size_t)(num) ? strncat_s((buf), (len), (src), (num)) : 1))
#else
SOAP_FMAC1 int SOAP_FMAC2 soap_strncat(char *buf, size_t len, const char *src, size_t num);
#endif

/* copy memory (returns SOAP_ERANGE on overrun, zero if OK, pointers must not be NULL) */
#if defined(_MSC_VER) && _MSC_VER >= 1400
# define soap_memcpy(buf, len, src, num) ((size_t)(len) >= (size_t)(num) ? memcpy_s((buf), (len), (src), (num)) : SOAP_ERANGE)
#else
# define soap_memcpy(buf, len, src, num) ((size_t)(len) >= (size_t)(num) ? !memcpy((buf), (src), (num)) : SOAP_ERANGE)
#endif

/* move memory (returns SOAP_ERANGE on overrun, zero if OK, pointers must not be NULL) */
#if defined(_MSC_VER) && _MSC_VER >= 1400
# define soap_memmove(buf, len, src, num) ((size_t)(len) >= (size_t)(num) ? memmove_s((buf), (len), (src), (num)) : SOAP_ERANGE)
#else
# define soap_memmove(buf, len, src, num) ((size_t)(len) >= (size_t)(num) ? !memmove((buf), (src), (num)) : SOAP_ERANGE)
#endif

/* gSOAP status and error codes */

typedef soap_int32 soap_status;

#define SOAP_EOF                        EOF
#define SOAP_OK                         0
#define SOAP_CLI_FAULT                  1
#define SOAP_SVR_FAULT                  2
#define SOAP_TAG_MISMATCH               3
#define SOAP_TYPE                       4
#define SOAP_SYNTAX_ERROR               5
#define SOAP_NO_TAG                     6
#define SOAP_IOB                        7
#define SOAP_MUSTUNDERSTAND             8
#define SOAP_NAMESPACE                  9
#define SOAP_USER_ERROR                 10
#define SOAP_FATAL_ERROR                11      /* deprecated */
#define SOAP_FAULT                      12
#define SOAP_NO_METHOD                  13
#define SOAP_NO_DATA                    14
#define SOAP_GET_METHOD                 15
#define SOAP_PUT_METHOD                 16
#define SOAP_PATCH_METHOD               17
#define SOAP_DEL_METHOD                 18
#define SOAP_HTTP_METHOD                19
#define SOAP_EOM                        20
#define SOAP_MOE                        21
#define SOAP_HDR                        22
#define SOAP_NULL                       23
#define SOAP_DUPLICATE_ID               24
#define SOAP_MISSING_ID                 25
#define SOAP_HREF                       26
#define SOAP_UDP_ERROR                  27
#define SOAP_TCP_ERROR                  28
#define SOAP_HTTP_ERROR                 29
#define SOAP_SSL_ERROR                  30
#define SOAP_ZLIB_ERROR                 31
#define SOAP_DIME_ERROR                 32
#define SOAP_DIME_HREF                  33
#define SOAP_DIME_MISMATCH              34
#define SOAP_DIME_END                   35
#define SOAP_MIME_ERROR                 36
#define SOAP_MIME_HREF                  37
#define SOAP_MIME_END                   38
#define SOAP_VERSIONMISMATCH            39
#define SOAP_PLUGIN_ERROR               40
#define SOAP_DATAENCODINGUNKNOWN        41      /* unused */
#define SOAP_REQUIRED                   42
#define SOAP_PROHIBITED                 43
#define SOAP_OCCURS                     44
#define SOAP_LENGTH                     45
#define SOAP_PATTERN                    46
#define SOAP_FD_EXCEEDED                47
#define SOAP_UTF_ERROR                  48
#define SOAP_NTLM_ERROR                 49
#define SOAP_LEVEL                      50
#define SOAP_FIXED                      51
#define SOAP_EMPTY                      52
#define SOAP_END_TAG                    53
#define SOAP_ERR                        99

#define soap_xml_error_check(e) \
  ((e) == SOAP_TAG_MISMATCH || \
   (e) == SOAP_NO_TAG || \
   (e) == SOAP_IOB || \
   (e) == SOAP_SYNTAX_ERROR || \
   (e) == SOAP_NAMESPACE || \
   (e) == SOAP_TYPE || \
   (e) == SOAP_DUPLICATE_ID || \
   (e) == SOAP_MISSING_ID || \
   (e) == SOAP_REQUIRED || \
   (e) == SOAP_PROHIBITED || \
   (e) == SOAP_OCCURS || \
   (e) == SOAP_LENGTH || \
   (e) == SOAP_LEVEL || \
   (e) == SOAP_PATTERN || \
   (e) == SOAP_NULL || \
   (e) == SOAP_HREF || \
   (e) == SOAP_FIXED || \
   (e) == SOAP_EMPTY || \
   (e) == SOAP_END_TAG || \
   (e) == SOAP_UTF_ERROR)

#define soap_soap_error_check(e) \
  ((e) == SOAP_CLI_FAULT || \
   (e) == SOAP_SVR_FAULT || \
   (e) == SOAP_VERSIONMISMATCH || \
   (e) == SOAP_MUSTUNDERSTAND || \
   (e) == SOAP_FAULT || \
   (e) == SOAP_NO_METHOD || \
   (e) == SOAP_DATAENCODINGUNKNOWN)

#define soap_http_error_check(e) \
  ((e) == SOAP_HTTP_ERROR || \
   (e) == SOAP_NO_DATA || \
   ((e) >= SOAP_GET_METHOD && (e) <= SOAP_HTTP_METHOD) || \
   ((e) >= 100 && (e) < 600))

#define soap_dime_error_check(e) \
  ((e) == SOAP_DIME_ERROR || \
   (e) == SOAP_DIME_HREF || \
   (e) == SOAP_DIME_MISMATCH || \
   (e) == SOAP_DIME_END)

#define soap_mime_error_check(e) \
  ((e) == SOAP_MIME_ERROR || \
   (e) == SOAP_MIME_HREF || \
   (e) == SOAP_MIME_END)

#define soap_tcp_error_check(e) \
  ((e) == SOAP_EOF || \
   (e) == SOAP_TCP_ERROR)

#define soap_udp_error_check(e) \
  ((e) == SOAP_EOF || \
   (e) == SOAP_UDP_ERROR)

#define soap_ssl_error_check(e) ((e) == SOAP_SSL_ERROR)

#define soap_zlib_error_check(e) ((e) == SOAP_ZLIB_ERROR)

/* gSOAP HTTP response status codes 100 to 599 are reserved */

/* Codes 600 to 999 are user definable */

/* Exceptional gSOAP HTTP server response status codes >= 1000 */

#define SOAP_STOP               1000    /* No HTTP response should follow after SOAP_STOP is issued */
#define SOAP_FORM               1001    /* Request (form) data is present, no HTTP response should follow */
#define SOAP_HTML               1002    /* Custom HTML response */
#define SOAP_FILE               1200    /* Custom file-based response with soap::http_content and optional http status */

/* gSOAP HTTP method codes (client) */

typedef int soap_http_command;

#define SOAP_POST               2000    /* POST request */
#define SOAP_POST_FILE          2001    /* Custom file-based POST request */
#define SOAP_GET                2002    /* GET request */
#define SOAP_PUT                2003    /* PUT request */
#define SOAP_PATCH              2004    /* PATCH request */
#define SOAP_DEL                2005    /* DELETE request */
#define SOAP_HEAD               2006    /* HEAD request */
#define SOAP_OPTIONS            2007    /* OPTIONS request */
#define SOAP_CONNECT            2008    /* CONNECT request */

/* gSOAP DIME */

#define SOAP_DIME_CF            0x01
#define SOAP_DIME_ME            0x02
#define SOAP_DIME_MB            0x04
#define SOAP_DIME_VERSION       0x08 /* DIME version 1 */
#define SOAP_DIME_MEDIA         0x10
#define SOAP_DIME_ABSURI        0x20

/* gSOAP ZLIB */

#define SOAP_ZLIB_NONE          0x00
#define SOAP_ZLIB_DEFLATE       0x01
#define SOAP_ZLIB_INFLATE       0x02
#define SOAP_ZLIB_GZIP          0x02

/* gSOAP transport, connection, and content encoding modes */

typedef soap_int32 soap_mode;

#define SOAP_IO                 0x00000003      /* IO mask */
#define SOAP_IO_FLUSH           0x00000000      /* out: flush output immediately, no buffering */
#define SOAP_IO_BUFFER          0x00000001      /* out: buffer output in packets of size SOAP_BUFLEN */
#define SOAP_IO_STORE           0x00000002      /* out: store entire output to determine length for transport */
#define SOAP_IO_CHUNK           0x00000003      /* out: use HTTP chunked transfer AND buffer packets */

#define SOAP_IO_UDP             0x00000004      /* in/out: enable UDP instead of TCP */
#define SOAP_IO_LENGTH          0x00000008      /* out: calc message length (internal) */
#define SOAP_IO_KEEPALIVE       0x00000010      /* out: keep connection alive */

#define SOAP_ENC                0x00000FFF      /* IO and ENC mask */
#define SOAP_ENC_LATIN          0x00000020      /* in: accept iso-8859-1 */
#define SOAP_ENC_PLAIN          0x00000040      /* out: plain (XML or other) body, no HTTP header */
#define SOAP_ENC_XML            0x00000040      /* deprecated, alias for SOAP_ENC_PLAIN */
#define SOAP_ENC_DIME           0x00000080
#define SOAP_ENC_MIME           0x00000100
#define SOAP_ENC_MTOM           0x00000200
#define SOAP_ENC_ZLIB           0x00000400
#define SOAP_ENC_SSL            0x00000800

#define SOAP_XML_STRICT         0x00001000      /* in:  strict validation */
#define SOAP_XML_INDENT         0x00002000      /* out: emit indented XML */
#define SOAP_XML_IGNORENS       0x00004000      /* in:  ignore namespaces */
#define SOAP_XML_DEFAULTNS      0x00008000      /* out: emit xmlns="..." */
#define SOAP_XML_CANONICAL      0x00010000      /* out: exc-C14N exclusive canonical XML */
#define SOAP_XML_TREE           0x00020000      /* in/out: XML tree (no id/ref) */
#define SOAP_XML_NIL            0x00040000      /* out: all NULLs as xsi:nil */
#define SOAP_XML_NOTYPE         0x00080000      /* out: do not add xsi:type */

#define SOAP_DOM_TREE           0x00100000      /* see DOM manual */
#define SOAP_DOM_NODE           0x00200000
#define SOAP_DOM_ASIS           0x00400000

#define SOAP_XML_CANONICAL_NA   0x00800000      /* out: (exc) C14N not QName aware */

#define SOAP_C_NOIOB            0x01000000      /* don't fault on array index out of bounds (just ignore) */
#define SOAP_C_UTFSTRING        0x02000000      /* (de)serialize strings with UTF8 content */
#define SOAP_C_MBSTRING         0x04000000      /* (de)serialize strings with multi-byte content */
#define SOAP_C_NILSTRING        0x08000000      /* serialize empty strings as nil (omitted) */

#define SOAP_XML_DOM            0x10000000      /* enable internal DOM */
#define SOAP_XML_GRAPH          0x20000000      /* force id-ref XML graph */

#define SOAP_MIME_POSTCHECK     0x40000000      /* MIME flag (internal) */

#define SOAP_SEC_WSUID          0x80000000      /* Add Body wsu:Id flag (internal) */

/* WITH_XMLNS backward compatibility: always use XML default namespaces */
#ifdef WITH_XMLNS
# define SOAP_IO_DEFAULT                (SOAP_IO_FLUSH | SOAP_XML_DEFAULTNS)
#else
# define SOAP_IO_DEFAULT                SOAP_IO_FLUSH
#endif

/* SSL client/server authentication settings */

#define SOAP_SSL_NO_AUTHENTICATION              (0x0000)  /* no authentication */
#define SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION  (0x0001)  /* client requires server to authenticate */
#define SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION  (0x0002)  /* server requires client to authenticate */
#define SOAP_SSL_SKIP_HOST_CHECK                (0x0004)  /* client skips common name check against host name */
#define SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE      (0x0008)  /* allow self-signed and expired certificates and those w/o CRL */
#define SOAP_SSL_NO_DEFAULT_CA_PATH             (0x0010)  /* don't use SSL_CTX_set_default_verify_paths */
#define SOAP_SSL_RSA                            (0x0020)  /* use RSA */
#define SOAP_SSLv3                              (0x0080)  /* enable SSL v3 */
#define SOAP_TLSv1_0                            (0x0100)  /* enable TLS v1.0 */
#define SOAP_TLSv1_1                            (0x0200)  /* enable TLS v1.1 */
#define SOAP_TLSv1_2                            (0x0400)  /* enable TLS v1.2 */
#define SOAP_TLSv1_3                            (0x0800)  /* enable TLS v1.3 */
#define SOAP_TLSv1                              (SOAP_TLSv1_0 | SOAP_TLSv1_1 | SOAP_TLSv1_2 | SOAP_TLSv1_3)
#define SOAP_SSLv3_TLSv1                        (SOAP_SSLv3 | SOAP_TLSv1)

#define SOAP_SSL_CLIENT                         (0x8000)  /* client context flag for internal use */

#define SOAP_SSL_DEFAULT                        SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION
 
typedef unsigned short soap_ssl_flags;

/* state */

#define SOAP_NONE               0
#define SOAP_INIT               1
#define SOAP_COPY               2

#define soap_check_state(soap) (!(soap) || ((soap)->state != SOAP_INIT && (soap)->state != SOAP_COPY))

/* parts */

#define SOAP_END                0
#define SOAP_BEGIN_SEND         1
#define SOAP_BEGIN_RECV         2
#define SOAP_IN_ENVELOPE        3
#define SOAP_IN_HEADER          4
#define SOAP_END_HEADER         5
#define SOAP_NO_BODY            6
#define SOAP_IN_BODY            7
#define SOAP_END_BODY           8
#define SOAP_END_ENVELOPE       9

/* events */

#define SOAP_SEC_BEGIN          1
#define SOAP_SEC_SIGN           2
#define SOAP_SEC_DECRYPT        3

/* DEBUG macros */

#ifdef DEBUG_STAMP
# ifndef DEBUG
#  define DEBUG
# endif
#endif

#ifdef DEBUG
# ifndef SOAP_DEBUG
#  define SOAP_DEBUG
# endif
# ifndef SOAP_MEM_DEBUG
#  define SOAP_MEM_DEBUG
# endif
#endif

#ifdef SOAP_MEM_DEBUG
# ifndef SOAP_MALLOC
#  define SOAP_MALLOC(soap, size) soap_track_malloc((soap), __FILE__, __LINE__, (size))
# endif
# ifndef SOAP_FREE
#  define SOAP_FREE(soap, ptr) soap_track_free((soap), __FILE__, __LINE__, (void*)(ptr))
# endif
#endif

#ifndef SOAP_MALLOC                     /* use libc malloc */
# define SOAP_MALLOC(soap, size) malloc((size))
#endif

#ifndef SOAP_FREE                       /* use libc free */
# define SOAP_FREE(soap, ptr) free((void*)(ptr))
#endif

#ifndef SOAP_MALLOC_UNMANAGED           /* use libc malloc to alloc soap context with soap_new() */
# define SOAP_MALLOC_UNMANAGED(size) malloc((size))
#endif

#ifndef SOAP_FREE_UNMANAGED             /* use libc free to free soap context with soap_free() */
# define SOAP_FREE_UNMANAGED(ptr) free((void*)(ptr))
#endif

#ifndef SOAP_NOTHROW
# if defined(__GNUC__) && (__GNUC__ <= 2)
#  define SOAP_NOTHROW
# elif defined(WITH_LEAN) || defined(WITH_COMPAT)
#  define SOAP_NOTHROW
# else
#  define SOAP_NOTHROW (std::nothrow)
# endif
#endif

#if defined(__BORLANDC__) && !defined(__clang__)
/* Embarcadero Classic compiler special case */
# ifndef SOAP_NEW
#  define SOAP_NEW(soap, type) new SOAP_NOTHROW (type)
# endif
# ifndef SOAP_NEW_ARRAY
#  define SOAP_NEW_ARRAY(soap, type, n) new SOAP_NOTHROW (type[n])
# endif
# ifndef SOAP_PLACEMENT_NEW
#  define SOAP_PLACEMENT_NEW(soap, buf, type) new (buf) (type)
# endif
#elif (defined(__GNUC__) && (__GNUC__ <= 2)) || defined(__clang__) || defined(_AIX) || defined(AIX)
/* old form w/o parenthesis, soap context may be NULL */
# ifndef SOAP_NEW
#  define SOAP_NEW(soap, type) new SOAP_NOTHROW type
# endif
# ifndef SOAP_NEW_ARRAY
#  define SOAP_NEW_ARRAY(soap, type, n) new SOAP_NOTHROW type[n]
# endif
# ifndef SOAP_PLACEMENT_NEW
#  define SOAP_PLACEMENT_NEW(soap, buf, type) new (buf) type
# endif
#else
/* new form with parenthesis for (type) but not type[n], soap context may be NULL */
# ifndef SOAP_NEW
#  define SOAP_NEW(soap, type) new SOAP_NOTHROW (type)
# endif
# ifndef SOAP_NEW_ARRAY
#  define SOAP_NEW_ARRAY(soap, type, n) new SOAP_NOTHROW type[n]
# endif
# ifndef SOAP_PLACEMENT_NEW
#  define SOAP_PLACEMENT_NEW(soap, buf, type) new (buf) (type)
# endif
#endif

#ifndef SOAP_DELETE                     /* use C++ delete operator, soap context may be NULL */
# define SOAP_DELETE(soap, obj, type) delete obj
#endif

#ifndef SOAP_DELETE_ARRAY               /* use C++ delete[] operator, soap context may be NULL */
# define SOAP_DELETE_ARRAY(soap, obj, type) delete[] obj
#endif

#ifndef SOAP_NEW_UNMANAGED              /* use C++ unmanaged new operator for soap_new() and soap::copy() */
# define SOAP_NEW_UNMANAGED(soap) new SOAP_NOTHROW soap
#endif

#ifndef SOAP_DELETE_UNMANAGED           /* use C++ unmanaged delete operator for soap_free() */
# define SOAP_DELETE_UNMANAGED(soap) delete soap
#endif

#ifdef SOAP_DEBUG
# ifndef SOAP_MESSAGE
#  define SOAP_MESSAGE fprintf
# endif
# ifndef DBGLOG
#  ifdef DEBUG_STAMP
#   ifdef WIN32
#    define DBGLOG(DBGFILE, CMD) \
{ if (soap)\
  { if (!soap->fdebug[SOAP_INDEX_##DBGFILE])\
      soap_open_logfile((struct soap*)soap, SOAP_INDEX_##DBGFILE);\
    if (soap->fdebug[SOAP_INDEX_##DBGFILE])\
    { FILE *fdebug = soap->fdebug[SOAP_INDEX_##DBGFILE];\
      SYSTEMTIME _localTime;\
      ::GetLocalTime(&_localTime); \
      fprintf(fdebug, "%02d%02d%02d %02d:%02d:%02d.%03d|", (int)_localTime.wYear%100, (int)_localTime.wMonth, (int)_localTime.wDay, (int)_localTime.wHour, (int)_localTime.wMinute, (int)_localTime.wSecond, (int)_localTime.wMilliseconds);\
      CMD;\
      fflush(fdebug);\
    }\
  }\
}
#   else
#    define DBGLOG(DBGFILE, CMD) \
{ if (soap)\
  { if (!soap->fdebug[SOAP_INDEX_##DBGFILE])\
      soap_open_logfile((struct soap*)soap, SOAP_INDEX_##DBGFILE);\
    if (soap->fdebug[SOAP_INDEX_##DBGFILE])\
    { FILE *fdebug = soap->fdebug[SOAP_INDEX_##DBGFILE];\
      struct timeval _tv;\
      struct tm _tm;\
      gettimeofday(&_tv, NULL);\
      localtime_r(&_tv.tv_sec, &_tm);\
      fprintf(fdebug, "%02d%02d%02d %02d:%02d:%02d.%06ld|", (int)_tm.tm_year%100, (int)_tm.tm_mon+1, (int)_tm.tm_mday, (int)_tm.tm_hour, (int)_tm.tm_min, (int)_tm.tm_sec, (long)_tv.tv_usec);\
      CMD;\
      fflush(fdebug);\
    }\
  }\
}
#   endif
#  else
#   define DBGLOG(DBGFILE, CMD) \
{ if (soap)\
  { if (!soap->fdebug[SOAP_INDEX_##DBGFILE])\
      soap_open_logfile((struct soap*)soap, SOAP_INDEX_##DBGFILE);\
    if (soap->fdebug[SOAP_INDEX_##DBGFILE])\
    { FILE *fdebug = soap->fdebug[SOAP_INDEX_##DBGFILE];\
      CMD;\
      fflush(fdebug);\
    }\
  }\
}
#  endif
# endif
# ifndef DBGMSG
#  define DBGMSG(DBGFILE, MSG, LEN) \
{ if (soap)\
  { if (!soap->fdebug[SOAP_INDEX_##DBGFILE])\
      soap_open_logfile((struct soap*)soap, SOAP_INDEX_##DBGFILE);\
    if (soap->fdebug[SOAP_INDEX_##DBGFILE])\
    { fwrite((void*)(MSG), 1, (size_t)(LEN), soap->fdebug[SOAP_INDEX_##DBGFILE]);\
      fflush(soap->fdebug[SOAP_INDEX_##DBGFILE]);\
    }\
  }\
}
# endif
# ifndef DBGFUN
#  define DBGFUN(FNAME) DBGLOG(TEST, SOAP_MESSAGE(fdebug, "%s(%d): %s()\n", __FILE__, __LINE__, FNAME))
#  define DBGFUN1(FNAME, FMT, ARG) DBGLOG(TEST, SOAP_MESSAGE(fdebug, "%s(%d): %s(" FMT ")\n", __FILE__, __LINE__, FNAME, (ARG)))
#  define DBGFUN2(FNAME, FMT1, ARG1, FMT2, ARG2) DBGLOG(TEST, SOAP_MESSAGE(fdebug, "%s(%d): %s(" FMT1 ", " FMT2 ")\n", __FILE__, __LINE__, FNAME, (ARG1), (ARG2)))
#  define DBGFUN3(FNAME, FMT1, ARG1, FMT2, ARG2, FMT3, ARG3) DBGLOG(TEST, SOAP_MESSAGE(fdebug, "%s(%d): %s(" FMT1 ", " FMT2 ", " FMT3 ")\n", __FILE__, __LINE__, FNAME, (ARG1), (ARG2), (ARG3)))
#  define DBGFUN4(FNAME, FMT1, ARG1, FMT2, ARG2, FMT3, ARG3, FMT4, ARG4) DBGLOG(TEST, SOAP_MESSAGE(fdebug, "%s(%d): %s(" FMT1 ", " FMT2 ", " FMT3 ", " FMT4 ")\n", __FILE__, __LINE__, FNAME, (ARG1), (ARG2), (ARG3), (ARG4)))
# endif
# ifndef DBGHEX
#  define DBGHEX(DBGFILE, MSG, LEN) \
{ if (soap)\
  { if (!soap->fdebug[SOAP_INDEX_##DBGFILE])\
      soap_open_logfile(soap, SOAP_INDEX_##DBGFILE);\
    if (soap->fdebug[SOAP_INDEX_##DBGFILE])\
    { int i; char *s;\
      for (s = (char*)(MSG), i = (LEN); i; i--)\
        fprintf(soap->fdebug[SOAP_INDEX_##DBGFILE], "%2.2X  ", (int)*s++&0xFF);\
      fflush(soap->fdebug[SOAP_INDEX_##DBGFILE]);\
    }\
  }\
}
# endif
#else
# define DBGLOG(DBGFILE, CMD)
# define DBGMSG(DBGFILE, MSG, LEN)
# define DBGFUN(FNAME)
# define DBGFUN1(FNAME, FMT, ARG)
# define DBGFUN2(FNAME, FMT1, ARG1, FMT2, ARG2)
# define DBGFUN3(FNAME, FMT1, ARG1, FMT2, ARG2, FMT3, ARG3)
# define DBGFUN4(FNAME, FMT1, ARG1, FMT2, ARG2, FMT3, ARG3, FMT4, ARG4)
# define DBGHEX(DBGFILE, MSG, LEN)
#endif

/* UCS-4 requires 32 bits (0-7FFFFFFF, the sign bit is used by gSOAP to distinguish XML entities) */
typedef soap_int32 soap_wchar;

/* forward declaration */
struct SOAP_CMAC soap;

/* namespace table row */
struct SOAP_CMAC Namespace
{
  const char *id;
  const char *ns;
  const char *in;
  char *out;
};

/* namespace stack */
struct soap_nlist
{
  struct soap_nlist *next;
  unsigned int level; /* nesting depth level */
  short index; /* corresponding entry in ns mapping table */
  const char *ns; /* only set when parsed ns URI is not in the ns mapping table */
  char id[1]; /* the actual string value flows into the allocated region id[0...] below this struct */
};

/* block header, the data flows into the allocated region below this struct */
struct soap_bhead
{
  struct soap_bhead *next;
  size_t size;
};

/* block stack for nested block allocations */
struct soap_blist
{
  struct soap_blist *next;
  struct soap_bhead *head;
  size_t size;
  size_t item;
};

/* pointer serialization management */
struct soap_plist
{
  struct soap_plist *next;
  const void *ptr;
  void *dup;
  const void *array; /* array pointer */
  int size; /* array size */
  int type;
  int id;
  char mark1; /* 0=single-ref, 1=embedded-multi-ref (SOAP1.1), 2=multi-ref, 3=attachment */
  char mark2;
};

/* block allocation for pointer serialization management */
struct soap_pblk
{
  struct soap_pblk *next;
  struct soap_plist plist[SOAP_PTRBLK];
};

#ifdef SOAP_MEM_DEBUG
/* malloc/free tracking for debugging */
struct soap_mlist
{
  struct soap_mlist *next;
  const void *ptr;
  const char *file;
  int line;
  short live;
};
#endif

/* class allocation list */
struct soap_clist
{
  struct soap_clist *next;
  void *ptr;
  int type;
  int size; /* array size */
  int (*fdelete)(struct soap*, struct soap_clist*);
};

/* attributes */
struct soap_attribute
{
  struct soap_attribute *next;
  short flag;   /* soap_set_attr: 1 = normal, 2 = utf content */
  char *value;
  size_t size;
  const char *ns;
  short visible;
  char name[1]; /* the actual name string flows into the allocated region below this struct */
};

#if !defined(WITH_LEAN) || defined(WITH_COOKIES)
struct soap_cookie
{
  struct soap_cookie *next;
  char *name;
  char *value;
  char *domain;
  char *path;
  ULONG64 expire;       /* client-side: local time to expire (value cast to time_t) */
  long maxage;          /* server-side: seconds to expire */
  unsigned int version;
  short secure;
  short session;        /* server-side */
  short env;            /* server-side: got cookie from client and should not be (re)send */
  short modified;       /* server-side: client cookie was modified and should be send */
};
#endif

SOAP_FMAC1 struct soap_multipart* SOAP_FMAC2 soap_next_multipart(struct soap_multipart*);

#ifdef __cplusplus
class soap_multipart_iterator
{
 public:
  typedef ptrdiff_t difference_type;
  typedef soap_multipart value_type;
  typedef soap_multipart& reference;
  typedef soap_multipart* pointer;
  typedef std::forward_iterator_tag iterator_category;
  struct soap_multipart *content;
  soap_multipart_iterator() : content(NULL)                      { }
  soap_multipart_iterator(struct soap_multipart *p) : content(p) { }
  bool operator==(const soap_multipart_iterator& iter) const     { return (bool)(content == iter.content); }
  bool operator!=(const soap_multipart_iterator& iter) const     { return (bool)(content != iter.content); }
  struct soap_multipart& operator*() const                       { return *content; }
  struct soap_multipart *operator->() const                      { return content; }
  soap_multipart_iterator& operator++()                          { content = soap_next_multipart(content); return *this; }
  soap_multipart_iterator operator++(int)                        { soap_multipart_iterator iter(*this); content = soap_next_multipart(content); return iter; }
};
#endif

#ifndef WITH_LEANER
struct soap_dime
{
  size_t count;
  size_t size;
  size_t chunksize;
  size_t buflen;
  char flags;
  char *ptr;
  const char *id;
  const char *type;
  const char *options;
  struct soap_multipart *list;          /* list of DIME attachments received */
  struct soap_multipart *first, *last;  /* temporary in/out queue */
#ifdef __cplusplus
  typedef soap_multipart_iterator iterator;
  typedef soap_multipart_iterator const_iterator;
  const_iterator cbegin() const { soap_multipart_iterator iter(list); return iter; }
  const_iterator cend() const   { soap_multipart_iterator iter(NULL); return iter; }
  iterator begin()              { soap_multipart_iterator iter(list); return iter; }
  iterator end()                { soap_multipart_iterator iter(NULL); return iter; }
#endif
};
#endif

#ifndef WITH_LEANER
struct soap_mime
{
  char *boundary;                       /* MIME boundary */
  const char *start;                    /* MIME start ID */
  struct soap_multipart *list;          /* list of MIME attachments received */
  struct soap_multipart *first, *last;  /* temporary in/out queue */
#ifdef __cplusplus
  typedef soap_multipart_iterator iterator;
  typedef soap_multipart_iterator const_iterator;
  const_iterator cbegin() const { soap_multipart_iterator iter(list); return iter; }
  const_iterator cend() const   { soap_multipart_iterator iter(NULL); return iter; }
  iterator begin()              { soap_multipart_iterator iter(list); return iter; }
  iterator end()                { soap_multipart_iterator iter(NULL); return iter; }
#endif
};
#endif

#ifndef WITH_LEANER
/* RFC2045 MIME content transfer encodings */
enum soap_mime_encoding
{
  SOAP_MIME_NONE,
  SOAP_MIME_7BIT,
  SOAP_MIME_8BIT,
  SOAP_MIME_BINARY,
  SOAP_MIME_QUOTED_PRINTABLE,
  SOAP_MIME_BASE64,
  SOAP_MIME_IETF_TOKEN,
  SOAP_MIME_X_TOKEN
};
#endif

#ifndef WITH_LEANER
/* DIME/MIME multipart list */
struct soap_multipart
{
  struct soap_multipart *next;
  const char *ptr;                      /* points to raw data content */
  size_t size;                          /* size of data content */
  const char *id;                       /* DIME/MIME content ID or form data name */
  const char *type;                     /* DIME/MIME type (MIME type format) */
  const char *options;                  /* DIME options */
  enum soap_mime_encoding encoding;     /* MIME Content-Transfer-Encoding */
  const char *location;                 /* MIME Content-Location (optional) */
  const char *description;              /* MIME Content-Description (optional) */
#ifdef __cplusplus
  typedef soap_multipart_iterator iterator;
  typedef soap_multipart_iterator const_iterator;
#endif
};
#endif

#ifndef WITH_LEANER
/* attachment DIME and MTOM XOP forwarding */
struct soap_xlist
{
  struct soap_xlist *next;
  unsigned char **ptr;
  int *size;
  char *id;
  char **type;
  char **options;
};
#endif

/******************************************************************************/

#ifndef WITH_LEANER

/* dom.c[pp] functions (optional, compile and link dom.c[pp] */
struct SOAP_CMAC soap_dom_element;
struct SOAP_CMAC soap_dom_attribute;

/* soap_dom_element construction */

SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_new(struct soap *soap, const char *ns, const char *tag);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_new_w(struct soap *soap, const char *ns, const wchar_t *tag);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_set(struct soap_dom_element *elt, const char *ns, const char *tag);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_set_w(struct soap_dom_element *elt, const char *ns, const wchar_t *tag);

SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att(struct soap_dom_element *elt, const char *ns, const char *tag);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_w(struct soap_dom_element *elt, const char *ns, const wchar_t *tag);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt(struct soap_dom_element *elt, const char *ns, const char *tag);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_w(struct soap_dom_element *elt, const char *ns, const wchar_t *tag);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_nth_elt(struct soap_dom_element *elt, const char *ns, const char *tag, size_t n);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_nth_elt_w(struct soap_dom_element *elt, const char *ns, const wchar_t *tag, size_t n);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_nth(struct soap_dom_element *elt, size_t n);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_add_att(struct soap_dom_element *elt, const struct soap_dom_attribute *node);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_add_elt(struct soap_dom_element *elt, const struct soap_dom_element *node);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_add_atts(struct soap_dom_element *elt, const struct soap_dom_attribute *atts);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_add_elts(struct soap_dom_element *elt, const struct soap_dom_element *elts);

/* soap_dom_element assignment */

SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_bool(struct soap_dom_element *elt, LONG64 b);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_int(struct soap_dom_element *elt, LONG64 n);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_double(struct soap_dom_element *elt, double x);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_text(struct soap_dom_element *elt, const char *text);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_text_w(struct soap_dom_element *elt, const wchar_t *text);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_node(struct soap_dom_element *elt, const void *node, int type);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_copy(struct soap_dom_element *elt, const struct soap_dom_element *node);

/* soap_dom_element properties */

SOAP_FMAC1 int SOAP_FMAC2 soap_elt_match(const struct soap_dom_element *elt, const char *ns, const char *patt);
SOAP_FMAC1 int SOAP_FMAC2 soap_elt_match_w(const struct soap_dom_element *elt, const char *ns, const wchar_t *patt);
SOAP_FMAC1 const char * SOAP_FMAC2 soap_elt_get_ns(const struct soap_dom_element *elt);
SOAP_FMAC1 const char * SOAP_FMAC2 soap_elt_get_tag(const struct soap_dom_element *elt);
SOAP_FMAC1 int SOAP_FMAC2 soap_elt_is_true(const struct soap_dom_element *elt);
SOAP_FMAC1 int SOAP_FMAC2 soap_elt_is_false(const struct soap_dom_element *elt);
SOAP_FMAC1 int SOAP_FMAC2 soap_elt_get_int(const struct soap_dom_element *elt);
SOAP_FMAC1 long SOAP_FMAC2 soap_elt_get_long(const struct soap_dom_element *elt);
SOAP_FMAC1 LONG64 SOAP_FMAC2 soap_elt_get_LONG64(const struct soap_dom_element *elt);
SOAP_FMAC1 double SOAP_FMAC2 soap_elt_get_double(const struct soap_dom_element *elt);
SOAP_FMAC1 const char * SOAP_FMAC2 soap_elt_get_text(const struct soap_dom_element *elt);
SOAP_FMAC1 const void * SOAP_FMAC2 soap_elt_get_node(const struct soap_dom_element *elt, int type);
SOAP_FMAC1 int SOAP_FMAC2 soap_elt_get_type(const struct soap_dom_element *elt, const void **node);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_parent(const struct soap_dom_element *elt);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_elt_depth(const struct soap_dom_element *elt);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_elt_index(const struct soap_dom_element *elt);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_elt_len(const struct soap_dom_element *elt);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_elt_nth(const struct soap_dom_element *elt);

/* soap_dom_attribute construction */

SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_new(struct soap *soap, const char *ns, const char *tag);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_new_w(struct soap *soap, const char *ns, const wchar_t *tag);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_set(struct soap_dom_attribute *att, const char *ns, const char *tag);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_set_w(struct soap_dom_attribute *att, const char *ns, const wchar_t *tag);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_add(struct soap_dom_attribute *att, const char *ns, const char *tag);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_add_w(struct soap_dom_attribute *att, const char *ns, const wchar_t *tag);

/* soap_dom_attribute assignment */

SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_bool(struct soap_dom_attribute *att, LONG64 b);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_int(struct soap_dom_attribute *att, LONG64 n);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_double(struct soap_dom_attribute *att, double x);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_text(struct soap_dom_attribute *att, const char *text);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_text_w(struct soap_dom_attribute *att, const wchar_t *text);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_copy(struct soap_dom_attribute *att, const struct soap_dom_attribute *node);

/* soap_dom_attribute properties */

SOAP_FMAC1 int SOAP_FMAC2 soap_att_match(const struct soap_dom_attribute *att, const char *ns, const char *patt);
SOAP_FMAC1 int SOAP_FMAC2 soap_att_match_w(const struct soap_dom_attribute *att, const char *ns, const wchar_t *patt);
SOAP_FMAC1 const char * SOAP_FMAC2 soap_att_get_ns(const struct soap_dom_attribute *att);
SOAP_FMAC1 const char * SOAP_FMAC2 soap_att_get_tag(const struct soap_dom_attribute *att);
SOAP_FMAC1 int SOAP_FMAC2 soap_att_is_true(const struct soap_dom_attribute *att);
SOAP_FMAC1 int SOAP_FMAC2 soap_att_is_false(const struct soap_dom_attribute *att);
SOAP_FMAC1 int SOAP_FMAC2 soap_att_get_int(const struct soap_dom_attribute *att);
SOAP_FMAC1 long SOAP_FMAC2 soap_att_get_long(const struct soap_dom_attribute *att);
SOAP_FMAC1 LONG64 SOAP_FMAC2 soap_att_get_LONG64(const struct soap_dom_attribute *att);
SOAP_FMAC1 double SOAP_FMAC2 soap_att_get_double(const struct soap_dom_attribute *att);
SOAP_FMAC1 const char * SOAP_FMAC2 soap_att_get_text(const struct soap_dom_attribute *att);

/* DOM local traversal */

SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_first(struct soap_dom_element *elt);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_next(const struct soap_dom_attribute *att);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_first(struct soap_dom_element *elt);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_next(const struct soap_dom_element *elt);

/* DOM local retrieval */

SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_get(const struct soap_dom_element *elt, const char *ns, const char *tag);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_get_w(const struct soap_dom_element *elt, const char *ns, const wchar_t *tag);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_get(const struct soap_dom_element *elt, const char *ns, const char *tag);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_get_w(const struct soap_dom_element *elt, const char *ns, const wchar_t *tag);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_get_next(const struct soap_dom_element *elt);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_get_nth(struct soap_dom_element *elt, size_t n);

/* DOM local search */

SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_find(struct soap_dom_element *elt, const char *ns, const char *patt);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_att_find_next(const struct soap_dom_attribute *att, const char *ns, const char *patt);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_find(struct soap_dom_element *elt, const char *ns, const char *patt);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_find_next(const struct soap_dom_element *elt, const char *ns, const char *patt);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_find_type(struct soap_dom_element *elt, const char *ns, const char *patt, int type);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_elt_find_next_type(const struct soap_dom_element *elt, const char *ns, const char *patt, int type);

/* DOM size of local search results */

SOAP_FMAC1 size_t SOAP_FMAC2 soap_att_size(struct soap_dom_element *elt, const char *ns, const char *patt);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_elt_size(struct soap_dom_element *elt, const char *ns, const char *patt);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_elt_size_type(struct soap_dom_element *elt, const char *ns, const char *patt, int type);

/* DOM deep traversal */

SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_dom_next_attribute(const struct soap_dom_attribute *att);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_dom_next_element(const struct soap_dom_element *elt, const struct soap_dom_element *end);

/* DOM deep search */

SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_dom_find(struct soap_dom_element *begin, const struct soap_dom_element *end, const char *ns, const char *patt, int type);
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_dom_find_next(const struct soap_dom_element *elt, const struct soap_dom_element *end, const char *ns, const char *patt, int type);

#endif

#if defined(__cplusplus)
} /* extern "C" */
#endif

#ifndef WITH_LEANER
#ifdef __cplusplus
class SOAP_CMAC soap_dom_attribute_iterator
{
 public:
  typedef ptrdiff_t difference_type;
  typedef soap_dom_attribute value_type;
  typedef soap_dom_attribute& reference;
  typedef soap_dom_attribute* pointer;
  typedef std::forward_iterator_tag iterator_category;
  struct soap_dom_attribute *iter;
  const char *nstr;
  const char *name;
  bool operator==(const soap_dom_attribute_iterator&) const;
  bool operator!=(const soap_dom_attribute_iterator&) const;
  struct soap_dom_attribute& operator*() const;
  struct soap_dom_attribute *operator->() const;
  soap_dom_attribute_iterator& operator++();
  soap_dom_attribute_iterator operator++(int);
  soap_dom_attribute_iterator();
  soap_dom_attribute_iterator(struct soap_dom_attribute*);
  ~soap_dom_attribute_iterator();
};
#endif
#endif

#ifndef WITH_LEANER
struct SOAP_CMAC soap_dom_attribute
{
  struct soap_dom_attribute *next;
  const char *nstr;
  const char *name;
  const char *text;
  struct soap *soap;
#ifdef __cplusplus
  typedef soap_dom_attribute_iterator iterator;
  typedef soap_dom_attribute_iterator const_iterator;
  soap_dom_attribute(struct soap *soap = NULL);
  soap_dom_attribute(const soap_dom_attribute& att);
  soap_dom_attribute(struct soap *soap, const char *tag);
  soap_dom_attribute(struct soap *soap, const wchar_t *tag);
  soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const char *str);
  soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const wchar_t *str);
  soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const char *str);
  soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const wchar_t *str);
  soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const std::string& str);
  soap_dom_attribute(struct soap *soap, const char *ns, const char *tag, const std::wstring& str);
  soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const std::string& str);
  soap_dom_attribute(struct soap *soap, const char *ns, const wchar_t *tag, const std::wstring& str);
  ~soap_dom_attribute();
  soap_dom_attribute& set(const char *ns, const char *tag)      { return *soap_att_set(this, ns, tag); }
  soap_dom_attribute& set(const char *ns, const wchar_t *tag)   { return *soap_att_set_w(this, ns, tag); }
  soap_dom_attribute& set(bool b)                               { return *soap_att_bool(this, b); }
  soap_dom_attribute& set(int n)                                { return *soap_att_int(this, n); }
  soap_dom_attribute& set(LONG64 n)                             { return *soap_att_int(this, n); }
  soap_dom_attribute& set(float x)                              { return *soap_att_double(this, x); }
  soap_dom_attribute& set(double x)                             { return *soap_att_double(this, x); }
  soap_dom_attribute& set(const char *str)                      { return *soap_att_text(this, str); }
  soap_dom_attribute& set(const wchar_t *str)                   { return *soap_att_text_w(this, str); }
#ifndef WITH_COMPAT
  soap_dom_attribute& set(const std::string& str)               { return *soap_att_text(this, str.c_str()); }
  soap_dom_attribute& set(const std::wstring& str)              { return *soap_att_text_w(this, str.c_str()); }
#endif
  soap_dom_attribute& operator=(bool b)                         { return *soap_att_bool(this, b); }
  soap_dom_attribute& operator=(int n)                          { return *soap_att_int(this, n); }
  soap_dom_attribute& operator=(LONG64 n)                       { return *soap_att_int(this, n); }
  soap_dom_attribute& operator=(float x)                        { return *soap_att_double(this, x); }
  soap_dom_attribute& operator=(double x)                       { return *soap_att_double(this, x); }
  soap_dom_attribute& operator=(const char *str)                { return *soap_att_text(this, str); }
  soap_dom_attribute& operator=(const wchar_t *str)             { return *soap_att_text_w(this, str); }
#ifndef WITH_COMPAT
  soap_dom_attribute& operator=(const std::string& str)         { return *soap_att_text(this, str.c_str()); }
  soap_dom_attribute& operator=(const std::wstring& str)        { return *soap_att_text_w(this, str.c_str()); }
#endif
  soap_dom_attribute& operator=(const soap_dom_attribute& att)  { return *soap_att_copy(this, &att); }
  soap_dom_attribute& att(const char *tag)                      { return *soap_att_add(this, NULL, tag); }
  soap_dom_attribute& att(const wchar_t *tag)                   { return *soap_att_add_w(this, NULL, tag); }
  soap_dom_attribute& att(const char *ns, const char *tag)      { return *soap_att_add(this, ns, tag); }
  soap_dom_attribute& att(const char *ns, const wchar_t *tag)   { return *soap_att_add_w(this, ns, tag); }
  bool match(const char *patt) const                            { return soap_att_match(this, NULL, patt) != 0; }
  bool match(const wchar_t *patt) const                         { return soap_att_match_w(this, NULL, patt) != 0; }
  bool match(const char *ns, const char *patt) const            { return soap_att_match(this, ns, patt) != 0; }
  bool match(const char *ns, const wchar_t *patt) const         { return soap_att_match_w(this, ns, patt) != 0; }
  const char *ns() const                                        { return this->nstr; }
  const char *tag() const                                       { return this->name; }
  bool is_true() const                                          { return soap_att_is_true(this) != 0; }
  bool is_false() const                                         { return soap_att_is_false(this) != 0; }
  LONG64 get_int() const                                        { return soap_att_get_LONG64(this); }
  double get_double() const                                     { return soap_att_get_double(this); }
  const char *get_text() const                                  { return this->text; }
  operator bool() const                                         { return soap_att_is_true(this) != 0; }
  operator int() const                                          { return soap_att_get_int(this); }
  operator LONG64() const                                       { return soap_att_get_LONG64(this); }
  operator double() const                                       { return soap_att_get_double(this); }
  operator const char*() const                                  { return this->text; }
  const_iterator cbegin()                                       { return this->att_begin(); }
  const_iterator cend()                                         { return this->att_end(); }
  iterator begin()                                              { return this->att_begin(); }
  iterator end()                                                { return this->att_end(); }
  iterator att_begin()                                          { return soap_dom_attribute_iterator(this); }
  iterator att_end()                                            { return soap_dom_attribute_iterator(NULL); }
  iterator att_find(const char *patt)                           { return att_find(NULL, patt); }
  iterator att_find(const wchar_t *patt)                        { return att_find(NULL, patt); }
  iterator att_find(const char *ns, const char *patt);
  iterator att_find(const char *ns, const wchar_t *patt);
  void unlink();
#endif
};
#endif

#ifndef WITH_LEANER
#ifdef __cplusplus
class SOAP_CMAC soap_dom_element_iterator
{
 public:
  typedef ptrdiff_t difference_type;
  typedef soap_dom_element value_type;
  typedef soap_dom_element& reference;
  typedef soap_dom_element* pointer;
  typedef std::forward_iterator_tag iterator_category;
  struct soap_dom_element *iter;
  struct soap_dom_element *stop;
  const char *nstr;
  const char *name;
  int type;
  bool deep;
  bool operator==(const soap_dom_element_iterator&) const;
  bool operator!=(const soap_dom_element_iterator&) const;
  struct soap_dom_element& operator*() const;
  struct soap_dom_element *operator->() const;
  soap_dom_element_iterator& operator++();
  soap_dom_element_iterator operator++(int);
  soap_dom_element_iterator();
  soap_dom_element_iterator(struct soap_dom_element*);
  ~soap_dom_element_iterator();
};
#endif
#endif

#ifndef WITH_LEANER
struct SOAP_CMAC soap_dom_element
{
  struct soap_dom_element *next;
  struct soap_dom_element *prnt;
  struct soap_dom_element *elts;
  struct soap_dom_attribute *atts;
  const char *nstr;
  const char *name;
  const char *lead;
  const char *text;
  const char *code;
  const char *tail;
  const void *node;
  int type;
  struct soap *soap;
#ifdef __cplusplus
  typedef soap_dom_element_iterator iterator;
  typedef soap_dom_element_iterator const_iterator;
  soap_dom_element(struct soap *soap = NULL);
  soap_dom_element(const soap_dom_element& elt);
  soap_dom_element(struct soap *soap, const char *tag);
  soap_dom_element(struct soap *soap, const wchar_t *tag);
  soap_dom_element(struct soap *soap, const char *ns, const char *tag);
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag);
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const char *str);
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const wchar_t *str);
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const char *str);
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const wchar_t *str);
#ifndef WITH_COMPAT
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const std::string& str);
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const std::wstring& str);
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const std::string& str);
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const std::wstring& str);
#endif
  soap_dom_element(struct soap *soap, const char *ns, const char *tag, const void *nod, int typ);
  soap_dom_element(struct soap *soap, const char *ns, const wchar_t *tag, const void *nod, int typ);
  ~soap_dom_element();
  soap_dom_element& set(const char *ns, const char *tag)                { return *soap_elt_set(this, ns, tag); }
  soap_dom_element& set(const char *ns, const wchar_t *tag)             { return *soap_elt_set_w(this, ns, tag); }
  soap_dom_element& set(bool b)                                         { return *soap_elt_bool(this, b); }
  soap_dom_element& set(int n)                                          { return *soap_elt_int(this, n); }
  soap_dom_element& set(LONG64 n)                                       { return *soap_elt_int(this, n); }
  soap_dom_element& set(float x)                                        { return *soap_elt_double(this, x); }
  soap_dom_element& set(double x)                                       { return *soap_elt_double(this, x); }
  soap_dom_element& set(const char *str)                                { return *soap_elt_text(this, str); }
  soap_dom_element& set(const wchar_t *str)                             { return *soap_elt_text_w(this, str); }
#ifndef WITH_COMPAT
  soap_dom_element& set(const std::string& str)                         { return *soap_elt_text(this, str.c_str()); }
  soap_dom_element& set(const std::wstring& str)                        { return *soap_elt_text_w(this, str.c_str()); }
#endif
  soap_dom_element& set(const void *nod, int typ)                       { return *soap_elt_node(this, nod, typ); }
  soap_dom_element& add(soap_dom_element& elt)                          { return *soap_add_elt(this, &elt); }
  soap_dom_element& add(soap_dom_element *elt)                          { return *soap_add_elt(this, elt); }
  soap_dom_element& add(soap_dom_attribute& att)                        { return *soap_add_att(this, &att); }
  soap_dom_element& add(soap_dom_attribute *att)                        { return *soap_add_att(this, att); }
  soap_dom_element& adds(soap_dom_element& elt)                         { return *soap_add_elts(this, &elt); }
  soap_dom_element& adds(soap_dom_element *elt)                         { return *soap_add_elts(this, elt); }
  soap_dom_element& adds(soap_dom_attribute& att)                       { return *soap_add_atts(this, &att); }
  soap_dom_element& adds(soap_dom_attribute *att)                       { return *soap_add_atts(this, att); }
  soap_dom_element& operator=(bool b)                                   { return *soap_elt_bool(this, b); }
  soap_dom_element& operator=(int n)                                    { return *soap_elt_int(this, n); }
  soap_dom_element& operator=(LONG64 n)                                 { return *soap_elt_int(this, n); }
  soap_dom_element& operator=(float x)                                  { return *soap_elt_double(this, x); }
  soap_dom_element& operator=(double x)                                 { return *soap_elt_double(this, x); }
  soap_dom_element& operator=(const char *str)                          { return *soap_elt_text(this, str); }
  soap_dom_element& operator=(const wchar_t *str)                       { return *soap_elt_text_w(this, str); }
#ifndef WITH_COMPAT
  soap_dom_element& operator=(const std::string& str)                   { return *soap_elt_text(this, str.c_str()); }
  soap_dom_element& operator=(const std::wstring& str)                  { return *soap_elt_text_w(this, str.c_str()); }
#endif
  soap_dom_element& operator=(const soap_dom_element& elt)              { return *soap_elt_copy(this, &elt); }
  template<class T> soap_dom_element& operator=(const T& nod)           { return this->set(&nod, nod.soap_type()); }
  template<class T> soap_dom_element& operator=(const T *nod)           { return this->set(nod, nod->soap_type()); }
  template<class T> soap_dom_element& operator=(T *nod)                 { return this->set(nod, nod->soap_type()); }
  soap_dom_attribute& att(const char *tag)                              { return *soap_att(this, NULL, tag); }
  soap_dom_attribute& att(const wchar_t *tag)                           { return *soap_att_w(this, NULL, tag); }
  soap_dom_attribute& att(const char *ns, const char *tag)              { return *soap_att(this, ns, tag); }
  soap_dom_attribute& att(const char *ns, const wchar_t *tag)           { return *soap_att_w(this, ns, tag); }
  soap_dom_element& elt()                                               { return *soap_elt(this, NULL, NULL); }
  soap_dom_element& elt(const char *tag)                                { return *soap_elt(this, NULL, tag); }
  soap_dom_element& elt(const wchar_t *tag)                             { return *soap_elt_w(this, NULL, tag); }
  soap_dom_element& elt(const char *ns, const char *tag)                { return *soap_elt(this, ns, tag); }
  soap_dom_element& elt(const char *ns, const wchar_t *tag)             { return *soap_elt_w(this, ns, tag); }
  soap_dom_element& operator[](const char *tag)                         { return *soap_elt(this, NULL, tag); }
  soap_dom_element& operator[](const wchar_t *tag)                      { return *soap_elt_w(this, NULL, tag); }
  soap_dom_element& operator[](size_t n)                                { return *soap_nth(this, n); }
  soap_dom_attribute *att_get(const char *tag) const                    { return soap_att_get(this, NULL, tag); }
  soap_dom_attribute *att_get(const wchar_t *tag) const                 { return soap_att_get_w(this, NULL, tag); }
  soap_dom_attribute *att_get(const char *ns, const char *tag) const    { return soap_att_get(this, ns, tag); }
  soap_dom_attribute *att_get(const char *ns, const wchar_t *tag) const { return soap_att_get_w(this, ns, tag); }
  soap_dom_element *elt_get() const                                     { return soap_elt_get(this, NULL, NULL); }
  soap_dom_element *elt_get(const char *tag) const                      { return soap_elt_get(this, NULL, tag); }
  soap_dom_element *elt_get(const wchar_t *tag) const                   { return soap_elt_get_w(this, NULL, tag); }
  soap_dom_element *elt_get(const char *ns, const char *tag) const      { return soap_elt_get(this, ns, tag); }
  soap_dom_element *elt_get(const char *ns, const wchar_t *tag) const   { return soap_elt_get_w(this, ns, tag); }
  soap_dom_element *get_next() const                                    { return soap_elt_get_next(this); }
  soap_dom_element *get_nth(size_t n)                                   { return soap_elt_get_nth(this, n); }
  bool match(const char *pat) const                                     { return soap_elt_match(this, NULL, pat) != 0; }
  bool match(const wchar_t *pat) const                                  { return soap_elt_match_w(this, NULL, pat) != 0; }
  bool match(const char *ns, const char *pat) const                     { return soap_elt_match(this, ns, pat) != 0; }
  bool match(const char *ns, const wchar_t *pat) const                  { return soap_elt_match_w(this, ns, pat) != 0; }
  const char *ns() const                                                { return this->nstr; }
  const char *tag() const                                               { return this->name; }
  soap_dom_element *parent()                                            { return this->prnt; }
  size_t depth() const                                                  { return soap_elt_depth(this); }
  size_t index() const                                                  { return soap_elt_index(this); }
  size_t len() const                                                    { return soap_elt_len(this); }
  size_t nth() const                                                    { return soap_elt_nth(this); }
  size_t elt_size()                                                     { return soap_elt_size(this, NULL, NULL); }
  size_t elt_size(const char *pat, int typ = 0)                         { return elt_size(NULL, pat, typ); }
  size_t elt_size(const char *ns, const char *pat, int typ = 0)         { return soap_elt_size_type(this, ns, pat, typ); }
  size_t att_size()                                                     { return soap_att_size(this, NULL, NULL); }
  size_t att_size(const char *pat)                                      { return att_size(NULL, pat); }
#ifndef WITH_COMPAT
  size_t att_size(const std::string& pat)                               { return att_size(NULL, pat); }
#endif
  size_t att_size(const char *ns, const char *pat)                      { return soap_att_size(this, ns, pat); }
#ifndef WITH_COMPAT
  size_t att_size(const char *ns, const std::string& pat)               { return soap_att_size(this, ns, pat.c_str()); }
#endif
  bool is_true() const                                                  { return soap_elt_is_true(this) != 0; }
  bool is_false() const                                                 { return soap_elt_is_false(this) != 0; }
  LONG64 get_int() const                                                { return soap_elt_get_LONG64(this); }
  double get_double() const                                             { return soap_elt_get_double(this); }
  const char *get_text() const                                          { return this->text; }
  const void *get_node(int typ) const                                   { return soap_elt_get_node(this, typ); }
  int get_type(const void **nod) const                                  { return soap_elt_get_type(this, nod); }
  operator bool() const                                                 { return soap_elt_is_true(this) != 0; }
  operator int() const                                                  { return soap_elt_get_int(this); }
  operator LONG64() const                                               { return soap_elt_get_LONG64(this); }
  operator double() const                                               { return soap_elt_get_double(this); }
  operator const char*() const                                          { return this->text; }
  const_iterator cbegin()                                               { return this->begin(); }
  const_iterator cend()                                                 { return this->end(); }
  iterator begin();
  iterator end()                                                        { return soap_dom_element_iterator(NULL); }
  iterator elt_begin()                                                  { return soap_dom_element_iterator(this->elts); }
  iterator elt_end()                                                    { return soap_dom_element_iterator(NULL); }
  soap_dom_attribute::iterator att_begin()                              { return soap_dom_attribute_iterator(this->atts); }
  soap_dom_attribute::iterator att_end()                                { return soap_dom_attribute_iterator(NULL); }
  iterator find(const char *pat, int typ = 0)                           { return find(NULL, pat, typ); }
  iterator find(const wchar_t *pat, int typ = 0)                        { return find(NULL, pat, typ); }
  iterator find(const char *ns, const char *pat, int typ = 0);
  iterator find(const char *ns, const wchar_t *pat, int typ = 0);
  iterator find(int typ);
  iterator elt_find(const char *pat, int typ = 0)                       { return elt_find(NULL, pat, typ); }
  iterator elt_find(const wchar_t *pat, int typ = 0)                    { return elt_find(NULL, pat, typ); }
  iterator elt_find(const char *ns, const char *pat, int typ = 0);
  iterator elt_find(const char *ns, const wchar_t *pat, int typ = 0);
  iterator elt_find(int typ);
  soap_dom_attribute::iterator att_find(const char *pat)                { return att_find(NULL, pat); }
  soap_dom_attribute::iterator att_find(const wchar_t *pat)             { return att_find(NULL, pat); }
  soap_dom_attribute::iterator att_find(const char *ns, const char *pat);
  soap_dom_attribute::iterator att_find(const char *ns, const wchar_t *pat);
  void unlink();
#endif
};
#endif

#ifndef WITH_LEANER
SOAP_FMAC1 struct soap_dom_element * SOAP_FMAC2 soap_dup_xsd__anyType(struct soap *soap, struct soap_dom_element *d, const struct soap_dom_element *a);
SOAP_FMAC1 void SOAP_FMAC2 soap_del_xsd__anyType(const struct soap_dom_element *a);
SOAP_FMAC1 struct soap_dom_attribute * SOAP_FMAC2 soap_dup_xsd__anyAttribute(struct soap *soap, struct soap_dom_attribute *d, const struct soap_dom_attribute *a);
SOAP_FMAC1 void SOAP_FMAC2 soap_del_xsd__anyAttribute(const struct soap_dom_attribute *a);
SOAP_FMAC1 int SOAP_FMAC2 soap_dom_call(struct soap *soap, const char *endpoint, const char *action, const struct soap_dom_element *in, struct soap_dom_element *out);
#endif

#ifndef WITH_LEANER
#if defined(__cplusplus) && !defined(WITH_COMPAT)
extern std::ostream &operator<<(std::ostream&, const struct soap_dom_element&);
extern std::istream &operator>>(std::istream&, struct soap_dom_element&);

SOAP_FMAC1 int SOAP_FMAC2 soap_dom_call(struct soap *soap, const char *endpoint, const char *action, const struct soap_dom_element& in, struct soap_dom_element& out);
SOAP_FMAC1 int SOAP_FMAC2 soap_dom_call(struct soap *soap, const char *endpoint, const char *action, const struct soap_dom_element *in, struct soap_dom_element& out);
SOAP_FMAC1 int SOAP_FMAC2 soap_dom_call(struct soap *soap, const char *endpoint, const char *action, const struct soap_dom_element& in, struct soap_dom_element *out);
#endif
#endif

#if defined(__cplusplus)
extern "C" {
#endif

/******************************************************************************/

struct SOAP_CMAC soap
{
  short state;                  /* 0 = uninitialized, 1 = initialized, 2 = copy of another soap struct */
  short version;                /* 1 = SOAP1.1 and 2 = SOAP1.2 (set automatically from namespace URI in nsmap table), 0 indicates non-SOAP content */
  soap_mode mode;               /* internal mode flag, combines imode/omode */
  soap_mode imode;              /* input mode flag set with soap_init1(), soap_new1(), or soap_set_imode() */
  soap_mode omode;              /* ouput mode flag set with soap_init1(), soap_new1(), or soap_set_omode() */
  const char *float_format;     /* user-definable format string for floats (<1024 chars) */
  const char *double_format;    /* user-definable format string for doubles (<1024 chars) */
  const char *long_double_format;/* user-definable format string for long doubles (<1024 chars) */
  const char *dime_id_format;   /* user-definable format string for integer DIME id (<SOAP_TAGLEN chars) */
  ULONG64 recv_maxlength;       /* user-definable, when > 0, sets max message size that can be received */
  int recv_timeout;             /* user-definable, when > 0, sets socket recv stall timeout in seconds, < 0 in usec */
  int send_timeout;             /* user-definable, when > 0, sets socket send stall timeout in seconds, < 0 in usec */
  int transfer_timeout;         /* user-definable, when > 0, sets socket total transfer timeout in seconds, < 0 in usec */
  int connect_timeout;          /* user-definable, when > 0, sets socket connect() timeout in seconds, < 0 in usec */
  int accept_timeout;           /* user-definable, when > 0, sets socket accept() timeout in seconds, < 0 in usec */
  int socket_flags;             /* user-definable socket recv() and send() flags, e.g. set to MSG_NOSIGNAL to disable sigpipe */
  int connect_flags;            /* user-definable connect() SOL_SOCKET sockopt flags, e.g. set to SO_DEBUG to debug socket */
  int connect_retry;            /* number of times to retry connecting (exponential backoff), zero by default */
  int bind_flags;               /* user-definable bind() SOL_SOCKET sockopt flags, e.g. set to SO_REUSEADDR to enable reuse */
  short bind_inet6;             /* user-definable, when > 0 use AF_INET6 instead of PF_UNSPEC (only with -DWITH_IPV6) */
  short bind_v6only;            /* user-definable, when > 0 use IPPROTO_IPV6 sockopt IPV6_V6ONLY (only with -DWITH_IPV6) */
  int accept_flags;             /* user-definable accept() SOL_SOCKET sockopt flags */
#ifdef WITH_SELF_PIPE
  int pipe_fd[2];               /* self pipe trick file descriptors used to close the select call from another thread */
#endif
  int sndbuf;                   /* user-definable SO_SNDBUF setsockopt value */
  int rcvbuf;                   /* user-definable SO_RCVBUF setsockopt value */
  unsigned short linger_time;   /* user-definable linger time for SO_LINGER option */
  unsigned int maxlevel;        /* user-definable max XML nesting depth levels, initialized to SOAP_MAXLEVEL */
  long maxlength;               /* user-definable max string length, initialized to SOAP_MAXLENGTH, maxlength<=0 is unbounded */
  size_t maxoccurs;             /* user-definable max array/container size, initialized to SOAP_MAXOCCURS */
  const char *http_version;     /* HTTP version used "1.0" or "1.1" */
  const char *http_content;     /* optional custom HTTP content type (with SOAP_PUT, SOAP_POST_FILE, SOAP_FILE) */
  const char *http_extra_header;/* optional custom HTTP header of the form 'key: val' (multiple headers should be separated in the string by \r\n - crlf) */
  const char *encodingStyle;    /* default = "" which means that SOAP encoding is used */
  const char *actor;            /* SOAP-ENV:actor or role attribute value */
  const char *lang;             /* user-definable xml:lang attribute value of SOAP-ENV:Text */
  const struct Namespace *namespaces;   /* Pointer to global namespace mapping table */
  struct Namespace *local_namespaces;   /* Local namespace mapping table */
  struct soap_nlist *nlist;     /* namespace stack */
  struct soap_blist *blist;     /* block allocation stack */
  struct soap_clist *clist;     /* class instance allocation list */
  void *alist;                  /* memory allocation (malloc) list */
#if !defined(WITH_LEANER) || !defined(WITH_NOIDREF)
  struct soap_ilist *iht[SOAP_IDHASH];
#endif
  struct soap_plist *pht[SOAP_PTRHASH];
  struct soap_pblk *pblk;       /* plist block allocation */
  short pidx;                   /* plist block allocation */
  short shaky;                  /* objects in reallocatable containers are on shaky grounds */
  struct SOAP_ENV__Header *header;
  struct SOAP_ENV__Fault *fault;
  int idnum;
  void *user;                   /* for user to pass user-defined data to callbacks */
  void *data[4];                /* extension data = {smdevp, mecevp, ...} */
  struct soap_plugin *plugins;  /* linked list of plug-in data */
  const char *bearer;           /* HTTP authorization bearer token value */
  const char *userid;           /* HTTP Basic authorization userid */
  const char *passwd;           /* HTTP Basic authorization passwd */
  const char *authrealm;        /* HTTP authentication realm (and NTLM domain) */
#if !defined(WITH_LEAN) || defined(WITH_NTLM)
  const char *ntlm_challenge;   /* HTTP NTLM challenge key string */
  short ntlm_auth;              /* HTTP NTLM authentication type */
#endif
  int (*fpost)(struct soap*, const char*, const char*, int, const char*, const char*, ULONG64);
  int (*fget)(struct soap*);    /* HTTP GET hook (not set by default) */
  int (*fput)(struct soap*);    /* HTTP PUT hook (handled as POST by default) */
  int (*fpatch)(struct soap*);  /* HTTP PATCH hook (handled as POST by default) */
  int (*fdel)(struct soap*);    /* HTTP DELETE hook (not set by default) */
  int (*fopt)(struct soap*);    /* HTTP OPTIONS hook (not set by default) */
  int (*fhead)(struct soap*);   /* HTTP HEAD hook (not set by default) */
  int (*fform)(struct soap*);   /* HTTP/HTML form handler for plugins */
  int (*fposthdr)(struct soap*, const char*, const char*);
  int (*fresponse)(struct soap*, int, ULONG64);
  int (*fparse)(struct soap*);
  int (*fparsehdr)(struct soap*, const char*, const char*);
  int (*fheader)(struct soap*);
  int (*fresolve)(struct soap*, const char*, struct in_addr* inaddr);
  int (*fconnect)(struct soap*, const char*, const char*, int);
  int (*fdisconnect)(struct soap*);
  int (*fclosesocket)(struct soap*, SOAP_SOCKET);
  int (*fshutdownsocket)(struct soap*, SOAP_SOCKET, int);
  SOAP_SOCKET (*fopen)(struct soap*, const char*, const char*, int);
  SOAP_SOCKET (*faccept)(struct soap*, SOAP_SOCKET, struct sockaddr*, int *n);
  int (*fclose)(struct soap*);
  int (*fsend)(struct soap*, const char*, size_t);
  size_t (*frecv)(struct soap*, char*, size_t);
  int (*fpoll)(struct soap*);
  void (*fseterror)(struct soap*, const char **c, const char **s);
  int (*fignore)(struct soap*, const char*);
  int (*fserveloop)(struct soap*);
  void *(*fplugin)(struct soap*, const char*);
#ifndef WITH_LEANER
  int (*fsvalidate)(struct soap*, const char*, const char*);
  int (*fwvalidate)(struct soap*, const char*, const wchar_t*);
  int (*feltbegin)(struct soap*, const char*);
  int (*feltendin)(struct soap*, const char*, const char*);
  int (*feltbegout)(struct soap*, const char*, int, const char*);
  int (*feltendout)(struct soap*, const char*);
  int (*fprepareinitsend)(struct soap*);
  int (*fprepareinitrecv)(struct soap*);
  int (*fpreparesend)(struct soap*, const char*, size_t);
  int (*fpreparerecv)(struct soap*, const char*, size_t);
  int (*fpreparefinalsend)(struct soap*);
  int (*fpreparefinalrecv)(struct soap*);
  int recverror; /* last soap_recv_raw error code for filterrecv */
  int (*ffiltersend)(struct soap*, const char**, size_t*);
  int (*ffilterrecv)(struct soap*, char*, size_t*, size_t);
  void *(*fdimereadopen)(struct soap*, void*, const char*, const char*, const char*);
  void *(*fdimewriteopen)(struct soap*, const char*, const char*, const char*);
  void (*fdimereadclose)(struct soap*, void*);
  void (*fdimewriteclose)(struct soap*, void*);
  size_t (*fdimeread)(struct soap*, void*, char*, size_t);
  int (*fdimewrite)(struct soap*, void*, const char*, size_t);
  void *(*fmimereadopen)(struct soap*, void*, const char*, const char*, const char*);
  void *(*fmimewriteopen)(struct soap*, void*, const char*, const char*, const char*, enum soap_mime_encoding);
  void (*fmimereadclose)(struct soap*, void*);
  void (*fmimewriteclose)(struct soap*, void*);
  size_t (*fmimeread)(struct soap*, void*, char*, size_t);
  int (*fmimewrite)(struct soap*, void*, const char*, size_t);
#endif
  SOAP_SOCKET master;   /* socket bound to TCP/IP port */
  SOAP_SOCKET socket;   /* socket to send and receive */
  SOAP_SOCKET sendsk;   /* socket to send (overrides ::socket) */
  SOAP_SOCKET recvsk;   /* socket to receive (overrides ::socket) */
#if defined(__cplusplus) && !defined(WITH_COMPAT)
  std::ostream *os;     /* C++ only: ostream to send */
  std::istream *is;     /* C++ only: istream to receive */
#else
  const char **os;      /* C only: pointer to a const char*, will be set to point to the string output */
  const char *is;       /* C only: a const char* to read from (soap->is will advance) */
#endif
#ifndef UNDER_CE
  int sendfd;           /* int file descriptor for sending */
  int recvfd;           /* int file descriptor for receiving */
#else
  FILE *sendfd;         /* WinCE FILE* to send */
  FILE *recvfd;         /* WinCE FILE* to receive */
#endif
#if defined(WITH_OPENSSL)       /* OpenSSL */
  int (*fsslauth)(struct soap*);
  int (*fsslverify)(int, X509_STORE_CTX*);
  BIO *bio;
  SSL *ssl;
  SSL_CTX *ctx;
  SSL_SESSION *session;
  const char *dhfile;
  const char *randfile;
#elif defined(WITH_GNUTLS)      /* GNUTLS */
  int (*fsslauth)(struct soap*);
  void *fsslverify;
  gnutls_certificate_credentials_t xcred;       /* cert pointer */
  gnutls_anon_client_credentials_t acred;       /* anon pointer */
  gnutls_priority_t cache;                      /* priority cache pointer */
  gnutls_session_t session;                     /* session pointer */
  gnutls_dh_params_t dh_params;
  gnutls_rsa_params_t rsa_params;
#elif defined(WITH_SYSTEMSSL)   /* SYSTEM SSL */
  int (*fsslauth)(struct soap*);
  void *fsslverify;             /* N/A */
  void *bio;                    /* N/A */
  gsk_handle ctx;               /* environment */
  gsk_handle ssl;               /* ssl socket */
  void *session;                /* N/A */
  const char *dhfile;           /* N/A */
  const char *randfile;         /* N/A */
#else                           /* No SSL/TLS */
  void *fsslauth;               /* dummy members, to preserve struct size */
  void *fsslverify;
  void *bio;
  void *ssl;
  void *ctx;
  void *session;
  void *dh_params;
  void *rsa_params;
#endif
  unsigned short ssl_flags;
  const char *keyfile;
  const char *keyid;
  const char *password;
  const char *cafile;
  const char *capath;
  const char *crlfile;
  char session_host[SOAP_TAGLEN];
  int session_port;
  size_t bufidx;        /* index in soap.buf[] */
  size_t buflen;        /* length of soap.buf[] content */
  soap_wchar ahead;     /* parser lookahead */
  short cdata;          /* CDATA parser state */
  short body;           /* HTTP or XML element has a body (1) or not (0) */
  unsigned int level;   /* XML nesting level */
#ifndef WITH_LEAN
  ULONG64 start;        /* start time of send/recv (value cast to time_t) */
#endif
  ULONG64 count;        /* message length counter */
  ULONG64 length;       /* message length as was set by HTTP header received */
  char *labbuf;         /* look-aside buffer */
  size_t lablen;        /* look-aside buffer allocated length */
  size_t labidx;        /* look-aside buffer index to available part */
  char buf[SOAP_BUFLEN];/* send and receive buffer */
  char msgbuf[SOAP_TMPLEN]; /* in/out buffer for HTTP/MIME headers and short messages, must be >=1024 bytes */
  char tmpbuf[SOAP_TMPLEN]; /* in/out buffer for HTTP/MIME headers, simpleType values, element and attribute tag names, and DIME must be >=1024 bytes */
  char tag[SOAP_TAGLEN];
  char id[SOAP_TAGLEN];
  char href[SOAP_TAGLEN];
  char type[SOAP_TAGLEN];
  char arrayType[SOAP_TAGLEN];
  char arraySize[SOAP_TAGLEN];
  char arrayOffset[SOAP_TAGLEN];
  int position;
  int positions[SOAP_MAXDIMS];
  struct soap_attribute *attributes;    /* attribute list */
  short other;
  short root;
  short encoding;       /* when set, output encodingStyle */
  short mustUnderstand; /* a mustUnderstand element was parsed or is output */
  short null;           /* parsed XML is xsi:nil */
  short ns;             /* zero to output all xmlns */
  short part;           /* SOAP part state (header or body) */
  short event;          /* engine events and states for use by plugins */
  short peeked;
  unsigned int evlev;   /* event level */
  int alloced;
  size_t chunksize;
  size_t chunkbuflen;
  char endpoint[SOAP_TAGLEN];
  char path[SOAP_TAGLEN];
  char host[SOAP_TAGLEN];
  char *action;                 /* SOAPAction string */
  const char *prolog;           /* XML declaration prolog */
  unsigned int ip;              /* IP number retrieved from request */
  unsigned int ip6[4];          /* same for IPv6: upper in ip6[0] to lower in ip6[3] requires WITH_IPV6 */
  int port;                     /* port number */
  const char *override_host;    /* to override the client-side host name/IP when connecting */
  int override_port;            /* to override client-side port number when connecting */
  int keep_alive;               /* connection should be kept open (-1, 0, or counts down) */
  int tcp_keep_alive;           /* enable SO_KEEPALIVE */
  unsigned int tcp_keep_idle;   /* set TCP_KEEPIDLE */
  unsigned int tcp_keep_intvl;  /* set TCP_KEEPINTVL */
  unsigned int tcp_keep_cnt;    /* set TCP_KEEPCNT */
  int max_keep_alive;           /* maximum keep-alive session (default=100) 0 to always keep open */
  const char *proxy_http_version;/* HTTP version of proxy "1.0" or "1.1" */
  const char *proxy_host;       /* Proxy Server host name */
  int proxy_port;               /* Proxy Server port (default = 8080) */
  const char *proxy_userid;     /* Proxy Authorization user name */
  const char *proxy_passwd;     /* Proxy Authorization password */
  const char *proxy_from;       /* X-Forwarding-For header returned by proxy */
  const char *origin;           /* Origin header received */
  const char *cors_origin;      /* CORS Allow-Origin header returned by server */
  const char *cors_allow;       /* CORS Allow-Origin header default value of "*" */
  const char *cors_method;      /* CORS Request-Method header received */
  const char *cors_header;      /* CORS Request-Headers header received */
  const char *cors_methods;     /* CORS Allow-Methods header returned by server */
  const char *cors_headers;     /* CORS Allow-Headers header returned by server */
  const char *x_frame_options;  /* "DENY", "SAMEORIGIN" (default), or "ALLOW-FROM uri" */
  int status;                   /* HTTP status code, HTTP method, or other error code */
  int error;
  int errmode;
  int errnum;
#ifndef WITH_LEANER
  struct soap_dom_element *dom;
  struct soap_dime dime;
  struct soap_mime mime;
  struct soap_xlist *xlist;
#endif
  const char *logfile[SOAP_MAXLOGS];
  FILE *fdebug[SOAP_MAXLOGS];
  struct soap_mlist *mht[SOAP_PTRHASH];
#ifndef WITH_LEAN
  const char *wsuid;            /* space-separated string of element tags */
  const char *c14nexclude;      /* space-separated string of prefixes for c14n exclusion */
  const char *c14ninclude;      /* space-separated string of prefixes for c14n inclusion */
#endif
  struct soap_cookie *cookies;
  const char *cookie_domain;
  const char *cookie_path;
  int cookie_max;
#ifndef WITH_NOIO
  unsigned int ipv6_multicast_if; /* in_addr_t in6addr->sin6_scope_id IPv6 value */
  char* ipv4_multicast_if; /* IP_MULTICAST_IF IPv4 setsockopt interface_addr */
  unsigned char ipv4_multicast_ttl; /* IP_MULTICAST_TTL value 0..255 */
  const char *client_addr; /* when non-NULL, client binds to this address before connect */
  const char *client_addr_ipv6; /* WITH_IPV6: when non-NULL and client_addr is non-NULL and when connecting to a IPv6 server, client binds to this IPv6 address instead of client_addr */
  int client_port; /* when nonnegative, client binds to this port before connect */
  const char *client_interface; /* when non-NULL, override client-side interface address using this address */
  union {
    struct sockaddr addr;
    struct sockaddr_in in;
    struct sockaddr_storage storage;
  } peer; /* set by soap_connect/soap_accept and by UDP recv */
  size_t peerlen;
#endif
#ifdef SOAP_LOCALE_T
  SOAP_LOCALE_T c_locale;       /* if this does not compile, use ./configure --enable-xlocale or compile with -DWITH_INCLUDE_XLOCALE_H, or use -DWITH_NO_C_LOCALE to disable locale support */
#else
  void *c_locale;
#endif
#ifdef WITH_ZLIB
  z_stream *d_stream;           /* decompression stream */
  uLong z_crc;                  /* internal gzip crc */
#else
  void *d_stream;               /* dummy members, to preserve struct size */
  soap_int32 z_crc;
#endif
  const char *z_dict;           /* support for zlib static dictionaries */
  unsigned int z_dict_len;
  short zlib_state;             /* SOAP_ZLIB_NONE, SOAP_ZLIB_DEFLATE, or SOAP_ZLIB_INFLATE */
  short zlib_in;                /* SOAP_ZLIB_NONE, SOAP_ZLIB_DEFLATE, or SOAP_ZLIB_GZIP */
  short zlib_out;               /* SOAP_ZLIB_NONE, SOAP_ZLIB_DEFLATE, or SOAP_ZLIB_GZIP */
  char *z_buf;                  /* buffer */
  size_t z_buflen;
  unsigned short z_level;       /* compression level to be used (0=none, 1=fast to 9=best) */
  float z_ratio_in;             /* detected compression ratio compressed_length/length of inbound message */
  float z_ratio_out;            /* detected compression ratio compressed_length/length of outbound message */
#ifdef WMW_RPM_IO               /* vxWorks compatibility */
  void *rpmreqid;
#endif
#ifdef __cplusplus
  soap();
  soap(soap_mode);
  soap(soap_mode, soap_mode);
  soap(const struct soap&);
  struct soap& operator=(const struct soap&);
  void destroy();
  ~soap();                      /* no virtual methods, so sizeof(soap) should be the same in C and C++ */
#endif
};

struct soap_code_map
{
  LONG64 code;
  const char *string;
};

/* forwarding list */
struct soap_flist
{
  struct soap_flist *next;
  int type;
  void *ptr;
  unsigned int level;
  size_t index;
  void (*finsert)(struct soap*, int, int, void*, size_t, const void*, void**);
};

/* id-ref forwarding list */
struct soap_ilist
{
  struct soap_ilist *next;
  int type;
  size_t size;
  void *ptr;
  void **spine;
  void *link;
  void *copy;
  struct soap_flist *flist;
  void *smart;
  short shaky;
  char id[1]; /* the actual id string value flows into the allocated region below this struct */
};

struct soap_plugin
{
  struct soap_plugin *next;
  const char *id;
  void *data;
  int (*fcopy)(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src);
  void (*fdelete)(struct soap *soap, struct soap_plugin *p); /* should delete fields of plugin only and not free(p) */
};

extern SOAP_NMAC struct Namespace namespaces[];

#ifndef WITH_LEAN
# define soap_get0(soap) (((soap)->bufidx>=(soap)->buflen && soap_recv((soap))) ? EOF : (unsigned char)(soap)->buf[(soap)->bufidx])
# define soap_get1(soap) (((soap)->bufidx>=(soap)->buflen && soap_recv((soap))) ? EOF : (unsigned char)(soap)->buf[(soap)->bufidx++])
# define soap_getchar(soap) ((soap)->ahead ? soap_getahead((soap)) : soap_get1((soap)))
SOAP_FMAC1 soap_wchar SOAP_FMAC2 soap_getahead(struct soap*);
#else
SOAP_FMAC1 soap_wchar SOAP_FMAC2 soap_get0(struct soap*);
SOAP_FMAC1 soap_wchar SOAP_FMAC2 soap_get1(struct soap*);
SOAP_FMAC1 soap_wchar SOAP_FMAC2 soap_getchar(struct soap*);
#endif

#define soap_versioning_paste(name, ext) name##_REQUIRE_lib_v##ext
#define soap_versioning_ext(name, ext) soap_versioning_paste(name, ext)
#define soap_versioning(name) soap_versioning_ext(name, GSOAP_VERSION)

#define soap_init(soap) soap_init1(soap, SOAP_IO_DEFAULT)
#define soap_init1(soap, mode) soap_init2(soap, mode, mode)
#define soap_init2(soap, imode, omode) soap_versioning(soap_init)(soap, imode, omode)

#define soap_new() soap_new1(SOAP_IO_DEFAULT)
#define soap_new1(mode) soap_new2(mode, mode)
#define soap_new2(imode, omode) soap_versioning(soap_new)(imode, omode)

#define soap_revget1(soap) ((soap)->bufidx--)
#define soap_unget(soap, c) ((soap)->ahead = c)
#define soap_peek(soap) ((soap)->ahead = soap_get(soap))
#define soap_register_plugin(soap, plugin) soap_register_plugin_arg(soap, plugin, NULL)
#define soap_mode(soap, n) ((soap)->mode = (soap)->imode = (soap)->omode = (n))
#define soap_imode(soap, n) ((soap)->imode = (n))
#define soap_omode(soap, n) ((soap)->omode = (n))
#define soap_set_imode(soap, n) ((soap)->imode |= (n))
#define soap_clr_imode(soap, n) ((soap)->imode &= ~(n))
#define soap_set_omode(soap, n) ((soap)->omode |= (n))
#define soap_clr_omode(soap, n) ((soap)->omode &= ~(n))
#define soap_set_mode(soap, n) ((soap)->mode |= (n), (soap)->imode |= (n), (soap)->omode |= (n))
#define soap_clr_mode(soap, n) ((soap)->mode &= ~(n), (soap)->imode &= ~(n), (soap)->omode &= ~(n))
#define soap_destroy(soap) soap_delete((soap), NULL)

#define SOAP_NO_LINK_TO_DELETE (-2) /* pass to soap_link() as size n: do not manage, smart pointers are self-managing */

#ifdef HAVE_STRRCHR
# define soap_strrchr(s, t) strrchr(s, t)
#else
 SOAP_FMAC1 char* SOAP_FMAC2 soap_strrchr(const char *s, int t);
#endif

#ifdef HAVE_STRTOL
# define soap_strtol(s, t, b) strtol(s, t, b)
#else
 SOAP_FMAC1 long SOAP_FMAC2 soap_strtol(const char*, char**, int);
#endif

#ifdef HAVE_STRTOUL
# define soap_strtoul(s, t, b) strtoul(s, t, b)
#else
 SOAP_FMAC1 unsigned long SOAP_FMAC2 soap_strtoul(const char*, char**, int);
#endif

#if defined(WIN32) && !defined(__MINGW32__) && !defined(__MINGW64__) && !defined(__BORLANDC__)
# define soap_strtoll _strtoi64
#elif defined(HAVE_STRTOLL) && !defined(soap_strtoll)
# define soap_strtoll strtoll
#elif !defined(soap_strtoll)
 SOAP_FMAC1 LONG64 SOAP_FMAC2 soap_strtoll(const char*, char**, int);
#endif

#if defined(WIN32) && !defined(__MINGW32__) && !defined(__MINGW64__) && !defined(__BORLANDC__)
# define soap_strtoull _strtoui64
#elif defined(HAVE_STRTOULL) && !defined(soap_strtoull)
# define soap_strtoull strtoull
#elif !defined(soap_strtoull)
 SOAP_FMAC1 ULONG64 SOAP_FMAC2 soap_strtoull(const char*, char**, int);
#endif

#if defined(WITH_OPENSSL)
# define soap_random soap_rand()
 SOAP_FMAC1 int SOAP_FMAC2 soap_rand(void);
#elif defined(UNDER_CE)
# define soap_random (int)Random()
#elif defined(HAVE_RANDOM)
# define soap_random (int)random()
#else
# define soap_random rand()
#endif

#ifdef WITH_NOIDREF
# define soap_embedded(s, p, t)                           ((void)(s), 0)
# define soap_id_lookup(s, i, p, t, n, k, fb)             ((void)(s), (p))
# define soap_id_forward(s, h, p, i, t, tt, n, k, fi, fb) ((void)(s), (p))
# define soap_id_nullify(s, i)                            ((void)(s), (i))
# define soap_reference(s, a, t)                          ((void)(s), 1)
# define soap_array_reference(s, p, a, n, t)              ((void)(s), 1)
# define soap_attachment_reference(s, p, a, n, t, i, y)   ((void)(s), 1)
# define soap_embed(s, p, a, n, t)                        ((void)(s), 0)
# define soap_embedded_id(s, i, p, t)                     ((void)(s), (void)(t), i)
# define soap_is_embedded(s, p)                           ((void)(s), 0)
# define soap_is_single(s, p)                             ((void)(s), 1)
# define soap_lookup_type(s, i)                           ((void)(s), 0)
# define soap_getindependent(s)                           ((void)(s), 0)
# define soap_putindependent(s)                           ((void)(s), 0)
# define soap_markelement(s, p, t)                        ((void)(s), 0)
# define soap_begin_shaky(s)                              ((void)(s), 0)
# define soap_end_shaky(s, f)                             ((void)(s), (void)(f), 0)
#endif

/* soap_traverse() traversal/walker routines take walker function arguments */
typedef void soap_walker(struct soap*, void*, int, const char*, const char*);

SOAP_FMAC5 int SOAP_FMAC6 soap_serve(struct soap *soap);
SOAP_FMAC5 int SOAP_FMAC6 soap_serve_request(struct soap *soap);

SOAP_FMAC1 void SOAP_FMAC2 soap_ssl_init(void);
SOAP_FMAC1 void SOAP_FMAC2 soap_ssl_noinit(void);
SOAP_FMAC1 int SOAP_FMAC2 soap_GET(struct soap*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_PUT(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_PATCH(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_POST(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_DELETE(struct soap*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_connect_command(struct soap*, int, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_connect(struct soap*, const char*, const char*);
SOAP_FMAC1 SOAP_SOCKET SOAP_FMAC2 soap_bind(struct soap*, const char*, int, int);
SOAP_FMAC1 SOAP_SOCKET SOAP_FMAC2 soap_accept(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_ssl_accept(struct soap*);
SOAP_FMAC1 const char * SOAP_FMAC2 soap_ssl_error(struct soap*, int ret, int err);
SOAP_FMAC1 int SOAP_FMAC2 soap_ssl_crl(struct soap*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_poll(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_ready(struct soap*);

#if defined(VXWORKS) && defined(WM_SECURE_KEY_STORAGE)
SOAP_FMAC1 int SOAP_FMAC2 soap_ssl_server_context(struct soap *soap, unsigned short flags, const char *keyfile, const char *keyid, const char *password, const char *cafile, const char *capath, const char *dhfile, const char *randfile, const char *sid);
#else
SOAP_FMAC1 int SOAP_FMAC2 soap_ssl_server_context(struct soap *soap, unsigned short flags, const char *keyfile, const char *password, const char *cafile, const char *capath, const char *dhfile, const char *randfile, const char *sid);
#endif
#if defined(VXWORKS) && defined(WM_SECURE_KEY_STORAGE)
SOAP_FMAC1 int SOAP_FMAC2 soap_ssl_client_context(struct soap *soap, unsigned short flags, const char *keyfile, const char *keyid, const char *password, const char *cafile, const char *capath, const char *randfile);
#else
SOAP_FMAC1 int SOAP_FMAC2 soap_ssl_client_context(struct soap *soap, unsigned short flags, const char *keyfile, const char *password, const char *cafile, const char *capath, const char *randfile);
#endif

SOAP_FMAC1 const char * SOAP_FMAC2 soap_http_content_type(struct soap *soap, int status);
SOAP_FMAC1 int SOAP_FMAC2 soap_puthttphdr(struct soap*, int status, ULONG64 count);

SOAP_FMAC1 const char* SOAP_FMAC2 soap_http_header_attribute(struct soap*, const char*, const char*);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_decode_key(char*, size_t, const char*);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_decode_val(char*, size_t, const char*);

SOAP_FMAC1 size_t SOAP_FMAC2 soap_hash(const char*);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_endpoint(struct soap*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_flush_raw(struct soap*, const char*, size_t);
SOAP_FMAC1 int SOAP_FMAC2 soap_flush(struct soap*);
SOAP_FMAC1 soap_wchar SOAP_FMAC2 soap_get(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_tag_cmp(const char*, const char*);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_fault(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_sender_fault(struct soap*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_sender_fault_subcode(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_receiver_fault(struct soap*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_receiver_fault_subcode(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_sender_error(struct soap*, const char*, const char*, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_receiver_error(struct soap*, const char*, const char*, int);

SOAP_FMAC1 int SOAP_FMAC2 soap_send_raw(struct soap*, const char*, size_t);
SOAP_FMAC1 int SOAP_FMAC2 soap_recv_raw(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_recv(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_send(struct soap*, const char*);

#ifndef WITH_LEANER
SOAP_FMAC1 int SOAP_FMAC2 soap_send2(struct soap*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_send3(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_query_send_key(struct soap*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_query_send_val(struct soap*, const char*);
SOAP_FMAC1 char * SOAP_FMAC2 soap_query(struct soap*);
SOAP_FMAC1 char * SOAP_FMAC2 soap_query_key(struct soap*, char**);
SOAP_FMAC1 char * SOAP_FMAC2 soap_query_val(struct soap*, char**);
SOAP_FMAC1 const char * SOAP_FMAC2 soap_query_decode(char*, size_t, const char*);
#endif

SOAP_FMAC1 int SOAP_FMAC2 soap_pututf8(struct soap*, unsigned long);
SOAP_FMAC1 soap_wchar SOAP_FMAC2 soap_getutf8(struct soap*);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_utf8len(const char*);

SOAP_FMAC1 int SOAP_FMAC2 soap_putbase64(struct soap*, const unsigned char*, int);
SOAP_FMAC1 unsigned char* SOAP_FMAC2 soap_getbase64(struct soap*, int*, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_puthex(struct soap*, const unsigned char*, int);
SOAP_FMAC1 unsigned char* SOAP_FMAC2 soap_gethex(struct soap*, int*);

#ifndef WITH_LEANER
SOAP_FMAC1 int SOAP_FMAC2 soap_xop_forward(struct soap*, unsigned char**, int*, char**, char**, char**);
SOAP_FMAC1 int SOAP_FMAC2 soap_attachment_forward(struct soap*, unsigned char**, int*, char**, char**, char**);
#endif

SOAP_FMAC1 int SOAP_FMAC2 soap_pointer_lookup(struct soap*, const void *p, int t, struct soap_plist**);
SOAP_FMAC1 int SOAP_FMAC2 soap_pointer_enter(struct soap*, const void *p, const void *a, int n, int t, struct soap_plist**);
SOAP_FMAC1 int SOAP_FMAC2 soap_array_pointer_lookup(struct soap*, const void *p, const void *a, int n, int t, struct soap_plist**);
#ifndef WITH_NOIDREF
SOAP_FMAC1 int SOAP_FMAC2 soap_pointer_lookup_id(struct soap*, void *p, int t, struct soap_plist**);
SOAP_FMAC1 int SOAP_FMAC2 soap_embed(struct soap *soap, const void *p, const void *a, int n, int t);
SOAP_FMAC1 struct soap_ilist* SOAP_FMAC2 soap_lookup(struct soap*, const char*);
SOAP_FMAC1 struct soap_ilist* SOAP_FMAC2 soap_enter(struct soap*, const char*, int t, size_t n);
SOAP_FMAC1 int SOAP_FMAC2 soap_resolve(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_embedded(struct soap*, const void *p, int t);
SOAP_FMAC1 int SOAP_FMAC2 soap_reference(struct soap*, const void *p, int t);
SOAP_FMAC1 int SOAP_FMAC2 soap_array_reference(struct soap*, const void *p, const void *a, int n, int t);
SOAP_FMAC1 int SOAP_FMAC2 soap_attachment_reference(struct soap *soap, const void *p, const void *a, int n, int t, const char *id, const char *type);
SOAP_FMAC1 int SOAP_FMAC2 soap_embedded_id(struct soap*, int id, const void *p, int t);
SOAP_FMAC1 int SOAP_FMAC2 soap_is_embedded(struct soap*, struct soap_plist*);
SOAP_FMAC1 int SOAP_FMAC2 soap_is_single(struct soap*, struct soap_plist*);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_embedded(struct soap*, struct soap_plist*);
#endif
SOAP_FMAC1 int SOAP_FMAC2 soap_check_and_mark(struct soap *soap, const void *p, int t, char **mark);
SOAP_FMAC1 void * SOAP_FMAC2 soap_mark_lookup(struct soap *soap, const void *p, int t, struct soap_plist **pp, char **mark);
SOAP_FMAC1 int SOAP_FMAC2 soap_mark_cycle(struct soap *soap, struct soap_plist *pp);
SOAP_FMAC1 void SOAP_FMAC2 soap_mark_dup(struct soap *soap, void *a, struct soap_plist *pp);
SOAP_FMAC1 void SOAP_FMAC2 soap_unmark(struct soap *soap, char *mark);

SOAP_FMAC1 int SOAP_FMAC2 soap_begin_count(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_end_count(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_begin_send(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_end_send(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_end_send_flush(struct soap*);

SOAP_FMAC1 const struct soap_code_map* SOAP_FMAC2 soap_code(const struct soap_code_map*, const char*);
SOAP_FMAC1 LONG64 SOAP_FMAC2 soap_code_int(const struct soap_code_map*, const char*, LONG64);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_code_str(const struct soap_code_map*, long);
SOAP_FMAC1 LONG64 SOAP_FMAC2 soap_code_bits(const struct soap_code_map*, const char*);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_code_list(struct soap*, const struct soap_code_map*, long);

SOAP_FMAC1 int SOAP_FMAC2 soap_binary_search_string(const char**, int, const char*);

SOAP_FMAC1 int SOAP_FMAC2 soap_getline(struct soap*, char*, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_begin_serve(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_begin_recv(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_end_recv(struct soap*);

SOAP_FMAC1 void* SOAP_FMAC2 soap_malloc(struct soap*, size_t);
SOAP_FMAC1 void SOAP_FMAC2 soap_dealloc(struct soap*, void*);
SOAP_FMAC1 struct soap_clist * SOAP_FMAC2 soap_link(struct soap*, int, int, int (*fdelete)(struct soap*, struct soap_clist*));
SOAP_FMAC1 int SOAP_FMAC2 soap_unlink(struct soap*, const void*);
SOAP_FMAC1 void SOAP_FMAC2 soap_free_temp(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_del(struct soap*);

SOAP_FMAC1 void* SOAP_FMAC2 soap_track_malloc(struct soap*, const char*, int, size_t);
SOAP_FMAC1 void SOAP_FMAC2 soap_track_free(struct soap*, const char*, int, void*);

#ifndef WITH_NOIDREF
SOAP_FMAC1 int SOAP_FMAC2 soap_lookup_type(struct soap*, const char *id);
SOAP_FMAC1 short SOAP_FMAC2 soap_begin_shaky(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_end_shaky(struct soap*, short);
SOAP_FMAC1 void* SOAP_FMAC2 soap_id_lookup(struct soap*, const char *id, void **p, int t, size_t n, unsigned int k, int (*fbase)(int, int));
SOAP_FMAC1 void* SOAP_FMAC2 soap_id_forward(struct soap*, const char *id, void *p, size_t i, int t, int tt, size_t n, unsigned int k, void(*finsert)(struct soap*, int, int, void*, size_t, const void*, void**), int (*fbase)(int, int));
SOAP_FMAC1 int SOAP_FMAC2 soap_id_nullify(struct soap*, const char*);
#endif
SOAP_FMAC1 void* SOAP_FMAC2 soap_id_enter(struct soap*, const char *id, void *p, int t, size_t n, const char *type, const char *arrayType, void *(*finstantiate)(struct soap*, int, const char*, const char*, size_t*), int (*fbase)(int, int));
SOAP_FMAC1 void** SOAP_FMAC2 soap_id_smart(struct soap *soap, const char*, int t, size_t n);

SOAP_FMAC1 size_t SOAP_FMAC2 soap_size(const int *, int);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_getsizes(const char *, int *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_getoffsets(const char *, const int *, int *, int);

SOAP_FMAC1 int SOAP_FMAC2 soap_getposition(const char *, int *);

SOAP_FMAC1 char* SOAP_FMAC2 soap_putsizesoffsets(struct soap*, const char *, const int *, const int *, int);
SOAP_FMAC1 char* SOAP_FMAC2 soap_putoffsets(struct soap*, const int *, int);
 
SOAP_FMAC1 int SOAP_FMAC2 soap_closesock(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_force_closesock(struct soap*);

SOAP_FMAC1 struct soap *SOAP_FMAC2 soap_versioning(soap_new)(soap_mode, soap_mode);
SOAP_FMAC1 void SOAP_FMAC2 soap_free(struct soap*);
SOAP_FMAC1 struct soap *SOAP_FMAC2 soap_copy(const struct soap*);
SOAP_FMAC1 struct soap *SOAP_FMAC2 soap_copy_context(struct soap*, const struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_copy_stream(struct soap*, struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_free_stream(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_versioning(soap_init)(struct soap*, soap_mode, soap_mode);
SOAP_FMAC1 void SOAP_FMAC2 soap_initialize(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_done(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_cleanup(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_begin(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_end(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_delete(struct soap*, void*);
SOAP_FMAC1 void SOAP_FMAC2 soap_delegate_deletion(struct soap*, struct soap*);
#ifdef WITH_SELF_PIPE
SOAP_FMAC1 void SOAP_FMAC2 soap_close_connection(struct soap*);
#endif

/* API functions available with DEBUG or SOAP_DEBUG defined: */
SOAP_FMAC1 void SOAP_FMAC2 soap_set_recv_logfile(struct soap*, const char*);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_sent_logfile(struct soap*, const char*);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_test_logfile(struct soap*, const char*);
SOAP_FMAC1 void SOAP_FMAC2 soap_close_logfiles(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_open_logfile(struct soap*, int);
/* */

SOAP_FMAC1 const char* SOAP_FMAC2 soap_value(struct soap*);

SOAP_FMAC1 int SOAP_FMAC2 soap_match_tag(struct soap*, const char*, const char *);
SOAP_FMAC1 int SOAP_FMAC2 soap_match_att(struct soap*, const char*, const char *);
SOAP_FMAC1 int SOAP_FMAC2 soap_match_array(struct soap*, const char*);

SOAP_FMAC1 int SOAP_FMAC2 soap_element(struct soap*, const char*, int, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_start_end_out(struct soap*, const char *tag);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_begin_out(struct soap*, const char *tag, int id, const char *type);
SOAP_FMAC1 int SOAP_FMAC2 soap_array_begin_out(struct soap*, const char *tag, int id, const char *type, const char *offset);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_ref(struct soap*, const char *tag, int id, int href);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_href(struct soap*, const char *tag, int id, const char *ref, const char *val);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_null(struct soap*, const char *tag, int id, const char *type);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_empty(struct soap*, const char *tag, int id, const char *type);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_nil(struct soap*, const char *tag);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_id(struct soap*, const char *tag, int id, const void *p, const void *a, int n, const char *type, int t, char **mark);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_result(struct soap*, const char *tag);
SOAP_FMAC1 void SOAP_FMAC2 soap_check_result(struct soap*, const char *tag);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_end_out(struct soap*, const char *tag);
SOAP_FMAC1 int SOAP_FMAC2 soap_element_end(struct soap*, const char *tag);

SOAP_FMAC1 int SOAP_FMAC2 soap_attribute(struct soap*, const char*, const char*);

SOAP_FMAC1 int SOAP_FMAC2 soap_element_begin_in(struct soap*, const char *tag, int nillable, const char *type);

SOAP_FMAC1 int SOAP_FMAC2 soap_element_end_in(struct soap*, const char *tag);

SOAP_FMAC1 int SOAP_FMAC2 soap_peek_element(struct soap*);

SOAP_FMAC1 void SOAP_FMAC2 soap_retry(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_revert(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_ignore(struct soap*);

SOAP_FMAC1 void* SOAP_FMAC2 soap_memdup(struct soap*, const void*, size_t);
SOAP_FMAC1 char* SOAP_FMAC2 soap_strdup(struct soap*, const char*);
SOAP_FMAC1 wchar_t* SOAP_FMAC2 soap_wstrdup(struct soap*, const wchar_t*);
SOAP_FMAC1 char* SOAP_FMAC2 soap_strtrim(struct soap*, char*);
SOAP_FMAC1 wchar_t* SOAP_FMAC2 soap_wstrtrim(struct soap*, wchar_t*);
SOAP_FMAC1 const char * SOAP_FMAC2 soap_tagsearch(const char *big, const char *little);

SOAP_FMAC1 int SOAP_FMAC2 soap_string_out(struct soap*, const char *s, int flag);
SOAP_FMAC1 char* SOAP_FMAC2 soap_string_in(struct soap*, int, long, long, const char*);

#ifndef WITH_LEANER
SOAP_FMAC1 int SOAP_FMAC2 soap_wstring_out(struct soap*, const wchar_t *s, int flag);
SOAP_FMAC1 wchar_t* SOAP_FMAC2 soap_wstring_in(struct soap*, int, long, long, const char*);
#endif

SOAP_FMAC1 int SOAP_FMAC2 soap_match_namespace(struct soap*, const char *, const char*, size_t n1, size_t n2);

SOAP_FMAC1 void SOAP_FMAC2 soap_set_version(struct soap*, short);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_namespaces(struct soap*, const struct Namespace*);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_local_namespaces(struct soap*);

SOAP_FMAC1 void SOAP_FMAC2 soap_pop_namespace(struct soap*);
SOAP_FMAC1 struct soap_nlist* SOAP_FMAC2 soap_push_namespace(struct soap*, const char *,const char *);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_current_namespace_tag(struct soap *soap, const char *tag);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_current_namespace_att(struct soap *soap, const char *tag);

SOAP_FMAC1 struct soap_nlist* SOAP_FMAC2 soap_lookup_ns(struct soap *soap, const char *tag, size_t n);

SOAP_FMAC1 int SOAP_FMAC2 soap_store_lab(struct soap*, const char*, size_t);
SOAP_FMAC1 int SOAP_FMAC2 soap_append_lab(struct soap*, const char*, size_t);

SOAP_FMAC1 struct soap_blist* SOAP_FMAC2 soap_alloc_block(struct soap*);
SOAP_FMAC1 void* SOAP_FMAC2 soap_push_block(struct soap*, struct soap_blist*, size_t);
SOAP_FMAC1 void* SOAP_FMAC2 soap_push_block_max(struct soap*, struct soap_blist*, size_t);
SOAP_FMAC1 void SOAP_FMAC2 soap_pop_block(struct soap*, struct soap_blist*);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_size_block(struct soap*, struct soap_blist*, size_t);
SOAP_FMAC1 char* SOAP_FMAC2 soap_first_block(struct soap*, struct soap_blist*);
SOAP_FMAC1 char* SOAP_FMAC2 soap_next_block(struct soap*, struct soap_blist*);
SOAP_FMAC1 size_t SOAP_FMAC2 soap_block_size(struct soap*, struct soap_blist*);
SOAP_FMAC1 void* SOAP_FMAC2 soap_save_block(struct soap*, struct soap_blist*, char*, int);
SOAP_FMAC1 void SOAP_FMAC2 soap_end_block(struct soap*, struct soap_blist*);
SOAP_FMAC1 void SOAP_FMAC2 soap_update_pointers(struct soap *soap, const char *dst, const char *src, size_t len);

SOAP_FMAC1 int SOAP_FMAC2 soap_envelope_begin_out(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_envelope_end_out(struct soap*);

SOAP_FMAC1 int SOAP_FMAC2 soap_http_has_body(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_http_skip_body(struct soap*);
SOAP_FMAC1 char * SOAP_FMAC2 soap_http_get_body(struct soap*, size_t *len);
SOAP_FMAC1 char * SOAP_FMAC2 soap_http_get_form(struct soap*);
SOAP_FMAC1 char * SOAP_FMAC2 soap_http_get_body_prefix(struct soap*, size_t *len, const char *prefix);

SOAP_FMAC1 int SOAP_FMAC2 soap_envelope_begin_in(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_envelope_end_in(struct soap*);

SOAP_FMAC1 int SOAP_FMAC2 soap_body_begin_out(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_body_end_out(struct soap*);

SOAP_FMAC1 int SOAP_FMAC2 soap_body_begin_in(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_body_end_in(struct soap*);

SOAP_FMAC1 int SOAP_FMAC2 soap_recv_header(struct soap*);

SOAP_FMAC1 int SOAP_FMAC2 soap_response(struct soap*, int);

SOAP_FMAC1 int SOAP_FMAC2 soap_send_empty_response(struct soap*, int status);
SOAP_FMAC1 int SOAP_FMAC2 soap_recv_empty_response(struct soap*);

SOAP_FMAC1 int SOAP_FMAC2 soap_send_fault(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_recv_fault(struct soap*, int check);

#ifndef WITH_NOSTDLIB
SOAP_FMAC1 void SOAP_FMAC2 soap_print_fault(struct soap*, FILE*);
SOAP_FMAC1 void SOAP_FMAC2 soap_print_fault_location(struct soap*, FILE*);
# ifndef WITH_LEAN
#  ifndef WITH_COMPAT
#   ifdef __cplusplus
SOAP_FMAC1 void SOAP_FMAC2 soap_stream_fault(struct soap*, std::ostream&);
SOAP_FMAC1 void SOAP_FMAC2 soap_stream_fault_location(struct soap*, std::ostream&);
#   endif
#  endif
SOAP_FMAC1 char* SOAP_FMAC2 soap_sprint_fault(struct soap*, char*, size_t);
# endif
#endif

SOAP_FMAC1 int SOAP_FMAC2 soap_s2byte(struct soap*, const char*, char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2short(struct soap*, const char*, short*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2int(struct soap*, const char*, int*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2long(struct soap*, const char*, long*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2LONG64(struct soap*, const char*, LONG64*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2float(struct soap*, const char*, float*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2double(struct soap*, const char*, double*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2unsignedByte(struct soap*, const char*, unsigned char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2unsignedShort(struct soap*, const char*, unsigned short*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2unsignedInt(struct soap*, const char*, unsigned int*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2unsignedLong(struct soap*, const char*, unsigned long*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2ULONG64(struct soap*, const char*, ULONG64*);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2char(struct soap*, const char*, char**, int, long minlen, long maxlen, const char *pattern);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2QName(struct soap*, const char*, char**, long minlen, long maxlen, const char *pattern);

#ifndef WITH_COMPAT
#ifdef __cplusplus
SOAP_FMAC1 int SOAP_FMAC2 soap_s2stdQName(struct soap*, const char*, std::string*, long minlen, long maxlen, const char *pattern);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2stdchar(struct soap*, const char*, std::string*, int, long minlen, long maxlen, const char *pattern);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2stdwchar(struct soap*, const char*, std::wstring*, int, long minlen, long maxlen, const char *pattern);
#endif
#endif

#ifndef WITH_LEAN
SOAP_FMAC1 int SOAP_FMAC2 soap_s2wchar(struct soap*, const char*, wchar_t**, int, long minlen, long maxlen, const char *pattern);
SOAP_FMAC1 int SOAP_FMAC2 soap_s2dateTime(struct soap*, const char*, time_t*);
#endif

SOAP_FMAC1 char* SOAP_FMAC2 soap_s2base64(struct soap*, const unsigned char*, char*, int);
SOAP_FMAC1 char* SOAP_FMAC2 soap_s2hex(struct soap*, const unsigned char*, char*, int);

SOAP_FMAC1 const char* SOAP_FMAC2 soap_byte2s(struct soap*, char);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_short2s(struct soap*, short);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_int2s(struct soap*, int);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_long2s(struct soap*, long);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_LONG642s(struct soap*, LONG64);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_float2s(struct soap*, float);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_double2s(struct soap*, double);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_unsignedByte2s(struct soap*, unsigned char);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_unsignedShort2s(struct soap*, unsigned short);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_unsignedInt2s(struct soap*, unsigned int);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_unsignedLong2s(struct soap*, unsigned long);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_ULONG642s(struct soap*, ULONG64);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_QName2s(struct soap*, const char*);

#ifndef WITH_LEAN
SOAP_FMAC1 const char* SOAP_FMAC2 soap_wchar2s(struct soap*, const wchar_t*);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_dateTime2s(struct soap*, time_t);
#endif

SOAP_FMAC1 const char* SOAP_FMAC2 soap_base642s(struct soap*, const char*, char*, size_t, int*);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_hex2s(struct soap*, const char*, char*, size_t, int*);

SOAP_FMAC1 int* SOAP_FMAC2 soap_inint(struct soap*, const char *tag, int *p, const char *, int);
SOAP_FMAC1 char* SOAP_FMAC2 soap_inbyte(struct soap*, const char *tag, char *p, const char *, int);
SOAP_FMAC1 long* SOAP_FMAC2 soap_inlong(struct soap*, const char *tag, long *p, const char *, int);
SOAP_FMAC1 LONG64* SOAP_FMAC2 soap_inLONG64(struct soap*, const char *tag, LONG64 *p, const char *, int);
SOAP_FMAC1 short* SOAP_FMAC2 soap_inshort(struct soap*, const char *tag, short *p, const char *, int);
SOAP_FMAC1 float* SOAP_FMAC2 soap_infloat(struct soap*, const char *tag, float *p, const char *, int);
SOAP_FMAC1 double* SOAP_FMAC2 soap_indouble(struct soap*, const char *tag, double *p, const char *, int);
SOAP_FMAC1 unsigned char* SOAP_FMAC2 soap_inunsignedByte(struct soap*, const char *tag, unsigned char *p, const char *, int);
SOAP_FMAC1 unsigned short* SOAP_FMAC2 soap_inunsignedShort(struct soap*, const char *tag, unsigned short *p, const char *, int);
SOAP_FMAC1 unsigned int* SOAP_FMAC2 soap_inunsignedInt(struct soap*, const char *tag, unsigned int *p, const char *, int);
SOAP_FMAC1 unsigned long* SOAP_FMAC2 soap_inunsignedLong(struct soap*, const char *tag, unsigned long *p, const char *, int);
SOAP_FMAC1 ULONG64* SOAP_FMAC2 soap_inULONG64(struct soap*, const char *tag, ULONG64 *p, const char *, int);
SOAP_FMAC1 char** SOAP_FMAC2 soap_instring(struct soap*, const char *tag, char **p, const char *, int, int, long, long, const char*);
SOAP_FMAC1 char** SOAP_FMAC2 soap_inliteral(struct soap*, const char *tag, char **p);

#ifndef WITH_LEAN
SOAP_FMAC1 time_t* SOAP_FMAC2 soap_indateTime(struct soap*, const char *tag, time_t *p, const char *, int);
#endif

#if !defined(WITH_LEAN) || defined(WITH_COOKIES)
SOAP_FMAC1 time_t SOAP_FMAC2 soap_timegm(struct tm*);
#endif

#ifndef WITH_LEANER
SOAP_FMAC1 wchar_t** SOAP_FMAC2 soap_inwstring(struct soap*, const char *tag, wchar_t **p, const char *, int, int, long, long, const char*);
SOAP_FMAC1 wchar_t** SOAP_FMAC2 soap_inwliteral(struct soap*, const char *tag, wchar_t **p);
#endif

SOAP_FMAC1 int SOAP_FMAC2 soap_outbyte(struct soap*, const char *tag, int id, const char *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outshort(struct soap*, const char *tag, int id, const short *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outint(struct soap*, const char *tag, int id, const int *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outlong(struct soap*, const char *tag, int id, const long *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outLONG64(struct soap*, const char *tag, int id, const LONG64 *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outfloat(struct soap*, const char *tag, int id, const float *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outdouble(struct soap*, const char *tag, int id, const double *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outunsignedByte(struct soap*, const char *tag, int id, const unsigned char *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outunsignedShort(struct soap*, const char *tag, int id, const unsigned short *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outunsignedInt(struct soap*, const char *tag, int id, const unsigned int *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outunsignedLong(struct soap*, const char *tag, int id, const unsigned long *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outULONG64(struct soap*, const char *tag, int id, const ULONG64 *p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outstring(struct soap*, const char *tag, int id, char *const*p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outliteral(struct soap*, const char *tag, char *const*p, const char *type);

#ifndef WITH_LEAN
SOAP_FMAC1 int SOAP_FMAC2 soap_outdateTime(struct soap*, const char *tag, int id, const time_t *p, const char *, int);
#endif

#ifndef WITH_LEANER
SOAP_FMAC1 int SOAP_FMAC2 soap_outwstring(struct soap*, const char *tag, int id, wchar_t *const*p, const char *, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_outwliteral(struct soap*, const char *tag, wchar_t *const*p, const char *type);
#endif

#ifndef WITH_LEANER
SOAP_FMAC1 int SOAP_FMAC2 soap_attachment(struct soap *, const char*, int, const void*, const void*, int, const char*, const char*, const char*, const char*, int);
SOAP_FMAC1 int SOAP_FMAC2 soap_move(struct soap*, ULONG64);
SOAP_FMAC1 ULONG64 SOAP_FMAC2 soap_tell(struct soap*);
SOAP_FMAC1 char* SOAP_FMAC2 soap_dime_option(struct soap*, unsigned short, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_getdimehdr(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_getdime(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_putdimehdr(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_putdime(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_getmimehdr(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_getmime(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_putmimehdr(struct soap*, struct soap_multipart*);
SOAP_FMAC1 int SOAP_FMAC2 soap_putmime(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_dime(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_mime(struct soap*, const char *boundary, const char *start);
SOAP_FMAC1 void SOAP_FMAC2 soap_clr_dime(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_clr_mime(struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_dime_attachment(struct soap*, const char *ptr, size_t size, const char *type, const char *id, unsigned short optype, const char *option);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_mime_attachment(struct soap*, const char *ptr, size_t size, enum soap_mime_encoding encoding, const char *type, const char *id, const char *location, const char *description);
SOAP_FMAC1 void SOAP_FMAC2 soap_post_check_mime_attachments(struct soap *soap);
SOAP_FMAC1 int SOAP_FMAC2 soap_check_mime_attachments(struct soap *soap);
SOAP_FMAC1 struct soap_multipart* SOAP_FMAC2 soap_recv_mime_attachment(struct soap *soap, void *handle);
SOAP_FMAC1 int SOAP_FMAC2 soap_match_cid(struct soap*, const char*, const char*);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_rand_uuid(struct soap*, const char*);
#endif

SOAP_FMAC1 int SOAP_FMAC2 soap_register_plugin_arg(struct soap*, int (*fcreate)(struct soap*, struct soap_plugin*, void*), void*);
SOAP_FMAC1 void* SOAP_FMAC2 soap_lookup_plugin(struct soap*, const char*);

SOAP_FMAC1 const char* SOAP_FMAC2 soap_attr_value(struct soap *soap, const char *name, int flag, int occurs);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_attr(struct soap *soap, const char *name, const char *value, int flag);
SOAP_FMAC1 void SOAP_FMAC2 soap_clr_attr(struct soap *soap);

SOAP_FMAC1 const char* SOAP_FMAC2 soap_extend_url(struct soap *soap, const char*, const char*);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_extend_url_query(struct soap *soap, const char*, const char*);
SOAP_FMAC1 void SOAP_FMAC2 soap_url_query(struct soap *soap, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_encode_url(const char*, char*, int);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_encode_url_string(struct soap*, const char*);
#ifdef WITH_COOKIES
SOAP_FMAC1 void SOAP_FMAC2 soap_getcookies(struct soap *soap, const char *val);
SOAP_FMAC1 struct soap_cookie* SOAP_FMAC2 soap_set_cookie(struct soap*, const char*, const char*, const char*, const char*);
SOAP_FMAC1 struct soap_cookie* SOAP_FMAC2 soap_cookie(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 struct soap_cookie* SOAP_FMAC2 soap_cookie_env(struct soap*, const char*, const char*, const char*, short);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_cookie_value(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_env_cookie_value(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 time_t SOAP_FMAC2 soap_cookie_expire(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_cookie_expire(struct soap*, const char*, long, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_cookie_secure(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_cookie_session(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_clr_cookie_session(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 void SOAP_FMAC2 soap_clr_cookie(struct soap*, const char*, const char*, const char*);
SOAP_FMAC1 int SOAP_FMAC2 soap_getenv_cookies(struct soap*);
SOAP_FMAC1 void SOAP_FMAC2 soap_free_cookies(struct soap*);
SOAP_FMAC1 struct soap_cookie* SOAP_FMAC2 soap_copy_cookies(struct soap*, const struct soap*);
SOAP_FMAC1 int SOAP_FMAC2 soap_putsetcookies(struct soap *soap);
SOAP_FMAC1 int SOAP_FMAC2 soap_putcookies(struct soap *soap, const char *domain, const char *path, int secure);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

/* declare global SOAP header and fault processing functions, unless WITH_STATIC is defined (generated in soapXYXLib code) */
#ifndef WITH_STATIC
SOAP_FMAC3 void SOAP_FMAC4 soap_header(struct soap*);
SOAP_FMAC3 void SOAP_FMAC4 soap_fault(struct soap*);
SOAP_FMAC3 const char ** SOAP_FMAC4 soap_faultcode(struct soap*);
SOAP_FMAC3 const char ** SOAP_FMAC4 soap_faultsubcode(struct soap*);
SOAP_FMAC3 const char ** SOAP_FMAC4 soap_faultstring(struct soap*);
SOAP_FMAC3 const char ** SOAP_FMAC4 soap_faultdetail(struct soap*);
SOAP_FMAC3 const char * SOAP_FMAC4 soap_fault_subcode(struct soap*);
SOAP_FMAC3 const char * SOAP_FMAC4 soap_fault_string(struct soap*);
SOAP_FMAC3 const char * SOAP_FMAC4 soap_fault_detail(struct soap*);
SOAP_FMAC3 void SOAP_FMAC4 soap_serializefault(struct soap*);
SOAP_FMAC3 void SOAP_FMAC4 soap_serializeheader(struct soap*);
SOAP_FMAC3 int SOAP_FMAC4 soap_getheader(struct soap*);
SOAP_FMAC3 int SOAP_FMAC4 soap_putheader(struct soap*);
SOAP_FMAC3 int SOAP_FMAC4 soap_getfault(struct soap*);
SOAP_FMAC3 int SOAP_FMAC4 soap_putfault(struct soap*);
#endif

#ifdef __cplusplus

/* C++ templates (for generated C++ only, not needed nor used in generated C code) */

template<class T>
struct soap_block
{
  static T *push(struct soap *soap, struct soap_blist *b)
  {
    if (!b)
      b = soap->blist;
    if (!b)
      return NULL;
    T *p = (T*)soap_push_block_max(soap, b, sizeof(T));
    if (p)
      SOAP_PLACEMENT_NEW(soap, p, T);
    return p;
  }
  static void pop(struct soap *soap, struct soap_blist *b)
  {
    if (!b)
      b = soap->blist;
    if (!b || !b->head)
      return;
    ((T*)(b->head + 1))->~T();
    soap_pop_block(soap, b);
  }
  static void save(struct soap *soap, struct soap_blist *b, T *p)
  {
    if (!b)
      b = soap->blist;
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wcast-align"
    for (T *q = (T*)soap_first_block(soap, b); q; q = (T*)soap_next_block(soap, b))
    {
    #pragma GCC diagnostic pop
      soap_update_pointers(soap, (const char*)p, (const char*)q, sizeof(T));
      *p++ = *q;
      q->~T();
    }
    soap_end_block(soap, b);
  }
  static void end(struct soap *soap, struct soap_blist *b)
  {
    if (!b)
      b = soap->blist;
    for (T *p = (T*)soap_first_block(soap, b); p; p = (T*)soap_next_block(soap, b))
      p->~T();
    soap_end_block(soap, b);
  }
};

#endif

#endif /* STDSOAP_H */
