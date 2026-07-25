/*
	threads.h

	Portable threads and locks API

gSOAP XML Web services tools
Copyright (C) 2000-2010, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
GPL or the gSOAP public license.
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
Copyright (C) 2000-2010, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

/**

@page threads Portable threads and locking support

The threads.h and threads.c code define the following portable API:

- THREAD_TYPE            portable thread type
- THREAD_ID              returns current thread ID of type THREAD_TYPE*
- THREAD_CREATE(t,f,a)   start thread (THREAD_TYPE*)t for f(a), return 0 if OK
- THREAD_CREATEX(t,f,a)  Windows only: start joinable thread (THREAD_TYPE*)t for f(a), return 0 if OK
- THREAD_CLOSE(t)        Windows only: close and destroy thread ID (a handle) when done
- THREAD_DETACH(t)       detach thread (THREAD_TYPE*)t
- THREAD_JOIN(t)         wait to join (THREAD_TYPE*)t
- THREAD_EXIT            exit the current thread
- THREAD_CANCEL(t)       kill a thread, dangerous, use only in extreme cases!

- MUTEX_TYPE             portable mutex type
- MUTEX_INITIALIZER      global initializer value for static locks
- MUTEX_SETUP(m)         setup lock (MUTEX_TYPE*)m
- MUTEX_CLEANUP(m)       cleanup lock (MUTEX_TYPE*)m
- MUTEX_LOCK(m)          acquire lock (MUTEX_TYPE*)m
- MUTEX_UNLOCK(m)        release lock (MUTEX_TYPE*)m

- COND_TYPE		 portable condition variable type
- COND_SETUP(c)          setup condition variable (COND_TYPE*)c
- COND_CLEANUP(c)        cleanup condition variable (COND_TYPE*)c
- COND_SIGNAL(c)         signal condition variable (COND_TYPE*)c
- COND_WAIT(c,m)         wait on variable (COND_TYPE*)c in mutex (MUTEX_TYPE*)m

*/

#ifndef THREADS_H
#define THREADS_H

#include "stdsoap2.h"

#ifndef WIN32
# include <unistd.h>	/* defines _POSIX_THREADS if pthreads are available */
#else
# define ssize_t int
# include <io.h>
# include <sys/types.h>
# include <process.h>
# include <windows.h>
#endif

#if defined(_POSIX_THREADS) || defined(_SC_THREADS)
# include <pthread.h>
#endif

/******************************************************************************\
 *
 *	Threads
 *
\******************************************************************************/

#if defined(WIN32)
# define THREAD_TYPE		HANDLE
# define THREAD_ID		GetCurrentThreadId()
# define THREAD_CREATE(x,y,z)	((*(x) = (HANDLE)_beginthread((void(__cdecl*)(void*))(y), 0, (z))) == (HANDLE)-1L)
# define THREAD_CREATEX(x,y,z)  ((*(x) = (HANDLE)_beginthreadex(NULL, 0, (void(__cdecl*)(void*))(y), (z), 0, NULL)) == (HANDLE)0L)
# define THREAD_CLOSE(x)	CloseHandle(x)
# define THREAD_DETACH(x)	
# define THREAD_JOIN(x)		(WaitForSingleObject((x), INFINITE) == (DWORD)0xFFFFFFFF)
# define THREAD_EXIT		_endthread()
# define THREAD_CANCEL(x)       (TerminateThread(x, 0) == 0)
# define MUTEX_TYPE		HANDLE
# define MUTEX_INITIALIZER	NULL
# define MUTEX_SETUP(x)		(x) = CreateMutex(NULL, FALSE, NULL)
# define MUTEX_CLEANUP(x)	(CloseHandle(x) == 0)
# define MUTEX_LOCK(x)		emulate_pthread_mutex_lock(&(x))
# define MUTEX_UNLOCK(x)	(ReleaseMutex(x) == 0)
# define COND_SETUP(x)		emulate_pthread_cond_init(&(x))
# define COND_CLEANUP(x)	emulate_pthread_cond_destroy(&(x))
# define COND_SIGNAL(x)		emulate_pthread_cond_signal(&(x))
# define COND_WAIT(x,y)		emulate_pthread_cond_wait(&(x), &(y))
typedef struct {
  UINT waiters_count_;
  CRITICAL_SECTION waiters_count_lock_;
  HANDLE signal_event_;
} COND_TYPE;
#ifdef __cplusplus
extern "C" {
#endif
SOAP_FMAC1 int SOAP_FMAC2 emulate_pthread_mutex_lock(volatile MUTEX_TYPE*);
SOAP_FMAC1 int SOAP_FMAC2 emulate_pthread_cond_init(COND_TYPE*);
SOAP_FMAC1 int SOAP_FMAC2 emulate_pthread_cond_destroy(COND_TYPE*);
SOAP_FMAC1 int SOAP_FMAC2 emulate_pthread_cond_signal(COND_TYPE*);
SOAP_FMAC1 int SOAP_FMAC2 emulate_pthread_cond_wait(COND_TYPE*, MUTEX_TYPE*);
#ifdef __cplusplus
}
#endif
#elif defined(_POSIX_THREADS) || defined(_SC_THREADS)
# define THREAD_TYPE		pthread_t
# define THREAD_ID		pthread_self()
# define THREAD_CREATE(x,y,z)	pthread_create((x), NULL, (y), (z))
# define THREAD_CREATEX(x,y,z)	pthread_create((x), NULL, (y), (z))
# define THREAD_CLOSE(x)
# define THREAD_DETACH(x)	pthread_detach((x))
# define THREAD_JOIN(x)		pthread_join((x), NULL)
# define THREAD_EXIT		pthread_exit(NULL)
# define THREAD_CANCEL(x)	pthread_cancel(x)
# define MUTEX_TYPE		pthread_mutex_t
# define MUTEX_INITIALIZER	PTHREAD_MUTEX_INITIALIZER
# define MUTEX_SETUP(x)		pthread_mutex_init(&(x), NULL)
# define MUTEX_CLEANUP(x)	pthread_mutex_destroy(&(x))
#if 0 /* 1: DEBUG MUTEX */
# define MUTEX_LOCK(x)		(fprintf(stderr, "! LOCK   %p %s:%d\n", &x, __FILE__, __LINE__), pthread_mutex_lock(&(x)))
# define MUTEX_UNLOCK(x)	(fprintf(stderr, "! UNLOCK %p %s:%d\n", &x, __FILE__, __LINE__), pthread_mutex_unlock(&(x)))
#else
# define MUTEX_LOCK(x)		pthread_mutex_lock(&(x))
# define MUTEX_UNLOCK(x)	pthread_mutex_unlock(&(x))
#endif
# define COND_TYPE		pthread_cond_t
# define COND_SETUP(x)		pthread_cond_init(&(x), NULL)
# define COND_CLEANUP(x)	pthread_cond_destroy(&(x))
# define COND_SIGNAL(x)		pthread_cond_signal(&(x))
# define COND_WAIT(x,y)		pthread_cond_wait(&(x), &(y))
#else
# error "No POSIX threads detected: we need thread and mutex operations. See for example OpenSSL /threads/th-lock.c on how to implement mutex on your platform"
#endif

#endif
