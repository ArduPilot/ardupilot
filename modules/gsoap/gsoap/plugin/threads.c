/*
	threads.c

	Portable threads and locks API implementation

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

#include "threads.h"

#ifdef WIN32

/******************************************************************************\
 *
 *	Emulation of POSIX condition variables for WIN32
 *
\******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

SOAP_FMAC1
int
SOAP_FMAC2
emulate_pthread_mutex_lock(volatile MUTEX_TYPE *mx)
{
  if (*mx == NULL) /* static initializer? */
  {
    HANDLE p = CreateMutex(NULL, FALSE, NULL);

    if (InterlockedCompareExchangePointer((PVOID*)mx, (PVOID)p, NULL) != NULL)
      CloseHandle(p);
  }

  return WaitForSingleObject(*mx, INFINITE) == WAIT_FAILED;
}

SOAP_FMAC1
int
SOAP_FMAC2
emulate_pthread_cond_init(COND_TYPE *cv)
{
  cv->waiters_count_ = 0;
  cv->signal_event_ = CreateEvent(NULL, FALSE, FALSE, NULL);
  InitializeCriticalSection(&cv->waiters_count_lock_);

  return 0;
}

SOAP_FMAC1
int
SOAP_FMAC2
emulate_pthread_cond_destroy(COND_TYPE *cv)
{
  CloseHandle(cv->signal_event_);
  DeleteCriticalSection(&cv->waiters_count_lock_);

  return 0;
}

SOAP_FMAC1
int
SOAP_FMAC2
emulate_pthread_cond_signal(COND_TYPE *cv)
{
  int have_waiters;

  EnterCriticalSection(&cv->waiters_count_lock_);
  have_waiters = cv->waiters_count_ > 0;
  LeaveCriticalSection(&cv->waiters_count_lock_);

  if (have_waiters)
    return SetEvent(cv->signal_event_) == 0;

  return 0;
}

SOAP_FMAC1
int
SOAP_FMAC2
emulate_pthread_cond_wait(COND_TYPE *cv, MUTEX_TYPE *cs)
{
  int result;

  EnterCriticalSection(&cv->waiters_count_lock_);
  cv->waiters_count_++;
  LeaveCriticalSection(&cv->waiters_count_lock_);

  ReleaseMutex(*cs);

  result = (WaitForSingleObject(cv->signal_event_, INFINITE) == WAIT_FAILED);

  if (!result)
  {
    EnterCriticalSection(&cv->waiters_count_lock_);
    cv->waiters_count_--;
    LeaveCriticalSection(&cv->waiters_count_lock_);

    result = (WaitForSingleObject(*cs, INFINITE) == WAIT_FAILED);
  }

  return result;
}

#ifdef __cplusplus
}
#endif

#endif 
