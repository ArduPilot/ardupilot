/*
        sessions.h

        Sessions plugin for thread-safe server-side HTTP session management.
        by Chris Moutsos & Robert van Engelen
  
gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

#ifndef SESSIONS_H
#define SESSIONS_H

#include "stdsoap2.h"
#include "threads.h"

#ifdef __cplusplus
extern "C" {
#endif

/** plugin identification for plugin registry */
#define SESSIONS_ID "SOAP-SESSIONS/1.0"

/** Max number of sessions allowed */
#define SESSION_MAX (8192)

/** Default max amount of stale time (seconds) server-side before a session is expired */
#define SESSIONS_DEFAULT_MAX_INACTIVE_SECS (300);

/** On every session database access, the time since the last database purge 
is checked. If it's been longer than this interval (in seconds), any expired 
sessions will be purged with soap_session_purge(). A value of 0 means a purge 
will occcur on every database access. A value of -1 means a purge will never 
occur. */
#define SESSION_PURGE_INTERVAL (300)

/** plugin identification for plugin registry */
extern const char sessions_id[];

/** Name used for session ID cookies */
static const char SESSION_COOKIE_NAME[] = "GSOAP_SESSIONID";

/**
@brief A session node. 
*/
struct soap_session
{ 
  struct soap_session *next;
  const char *id;
  struct soap_session_var *session_vars;
  struct soap_session_var *session_vars_tail;
  int num_session_vars;   /* the total number of session_vars we have in this session */
  long cookie_maxage;     /* client time in seconds before the cookie expires (-1 for session cookie) */
  int rolling;            /* whether or not we should set the cookie's maxage on every response */
  time_t last_touched;    /* time that we last touched this session or its vars */
  long max_inactive_secs; /* server time in seconds before inactive session expires (-1 to never expire) 
                             this should usually be at least as high as cookie_maxage */
};

/**
@brief A session variable node.
Session variables are name-value pairs.
*/
struct soap_session_var
{
  struct soap_session_var *next;
  const char *name;
  const char *value;
};

/* plugin registry */
SOAP_FMAC1 int SOAP_FMAC2 sessions(struct soap *soap, struct soap_plugin *plugin, void *arg);

/* gets soap_sessions (first node in sessions linked list) */
SOAP_FMAC1 struct soap_session* SOAP_FMAC2 soap_get_sessions_head();

/* purge expired sessions */
SOAP_FMAC1 void  SOAP_FMAC2 soap_purge_sessions(struct soap *soap, const char *domain, const char *path, time_t when);

/* get/start sessions */
SOAP_FMAC1 struct soap_session* SOAP_FMAC2 soap_start_session(struct soap *soap, const char *domain, const char *path);
SOAP_FMAC1 struct soap_session* SOAP_FMAC2 soap_get_session(struct soap *soap, const char *domain, const char *path);

/* get number of sessions */
SOAP_FMAC1 int SOAP_FMAC2 soap_get_num_sessions(struct soap *soap, const char *domain, const char *path);

/* generate a new session ID */
SOAP_FMAC1 struct soap_session* SOAP_FMAC2 soap_regenerate_session_id(struct soap *soap, const char *domain, const char *path);

/* get/set rolling */
SOAP_FMAC1 int SOAP_FMAC2 soap_get_session_rolling(struct soap *soap, const char *domain, const char *path);
SOAP_FMAC1 int SOAP_FMAC2 soap_set_session_rolling(struct soap *soap, int rolling, const char *domain, const char *path);

/* get/set cookie_maxage */
SOAP_FMAC1 int SOAP_FMAC2 soap_set_session_cookie_maxage(struct soap *soap, long maxage, const char *domain, const char *path);
SOAP_FMAC1 long SOAP_FMAC2 soap_get_session_cookie_maxage(struct soap *soap, const char *domain, const char *path);

/* get/set max_inactive_secs */
SOAP_FMAC1 int SOAP_FMAC2 soap_set_session_max_inactive_secs(struct soap *soap, long max, const char *domain, const char *path);
SOAP_FMAC1 long SOAP_FMAC2 soap_get_session_max_inactive_secs(struct soap *soap, const char *domain, const char *path);

/* end all sessions */
SOAP_FMAC1 void SOAP_FMAC2 soap_end_sessions(struct soap *soap, const char *domain, const char *path);

/* get number of session variables in session */
SOAP_FMAC1 int SOAP_FMAC2 soap_get_num_session_vars(struct soap *soap, const char *domain, const char *path);

/* end single session */
SOAP_FMAC1 void SOAP_FMAC2 soap_end_session(struct soap *soap, const char *domain, const char *path);

/* get/set session variable values */
SOAP_FMAC1 struct soap_session_var* SOAP_FMAC2 soap_set_session_var(struct soap *soap, const char *name, const char *value, const char *domain, const char *path);
SOAP_FMAC1 const char* SOAP_FMAC2 soap_get_session_var(struct soap *soap, const char *name, const char *domain, const char *path);

/* clear single session variable */
SOAP_FMAC1 void SOAP_FMAC2 soap_clr_session_var(struct soap *soap, const char *name, const char *domain, const char *path);

/* clear all session variables */
SOAP_FMAC1 void SOAP_FMAC2 soap_clr_session_vars(struct soap *soap, const char *domain, const char *path);

#ifdef __cplusplus
}
#endif

#endif
