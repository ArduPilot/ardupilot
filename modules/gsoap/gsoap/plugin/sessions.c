/*  
        sessions.c

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
  
/**

@mainpage The Sessions Plugin

@section title HTTP Server Session Management Plugin

[TOC]

By Chris Moutsos and Robert van Engelen.
 
@section overview Overview

The HTTP server session management plugin for gSOAP enables servers to store a
database of sessions to keep track of client data across multiple requests. 
Client data is stored per session as name and value pairs that can be set to 
any valid string.  Sessions are tracked with unique session IDs (sometimes 
called tokens) that are passed between the server and client with HTTP cookies.

The sessions plugin is MT safe.

@section features Features

- Stores sessions identified by random 128-bit session IDs managed with HTTP cookies
- Session validity can be restricted based on domain and path
- Each session stores a list of session variables (name/value string pairs)
- Can easily create, modify, delete, and retrieve session variables 
- Configurable settings
  - Rolling sessions (re-set session ID cookie with every response)
  - Session ID cookie `Max-Age` and `Expire` attribute values
  - Maximum stale time before a session expires server-side
  - Time interval between purging expired sessions
  - Max number of sessions allowed in database
- Thread-safe
- Can be used with C or C++ projects

@section setup Setup

To set up the sessions plugin on the server-side:

-# Make sure to compile your program with plugin/sessions.c and
   plugin/threads.c.
-# Enable cookies by compiling all gSOAP source code with `-DWITH_COOKIES`. 
   Cookies are required for tracking session IDs.
-# Enable HTTPS (compile with `-DWITH_OPENSSL` or `-DWITH_GNUTLS` or link with
   `-lgsoapssl`/`-lgsoapssl++`).

@note
While it is not a requirement to use HTTPS with this plugin, it is **strongly
recommended** to use HTTPS. This is because the session ID cookies can be
intercepted if not encrypted with SSL/TLS. Cookies are automatically sent with
the Secure flag enabled when SSL is enabled. The HttpOnly flag is automatically
enabled for all cookies in order to prevent XSS attacks.

@section usage Basic Usage

Client-side usage is automatic, provided that the client supports cookies and the
server has properly configured the plugin. If the client is a gSOAP 
application, make sure to compile with `-DWITH_COOKIES`.

For server-side usage, register the plugin with:

@code
    #include "plugin/sessions.h"

    struct soap *soap = soap_new();
    soap_register_plugin(soap, sessions);
@endcode

If you are using a server object in C++ generated with soappcpp2 option `-j`:
 
@code
    #include "plugin/sessions.h"

    soap_register_plugin(server.soap, sessions);
@endcode

For brevity, the `struct soap*` version will be used below. Replace `soap`
with your server object's `soap` member if necessary.

Start a new session with:

@code
    struct soap_session *session = soap_start_session(soap, "domain", "path"); 
@endcode

This creates a new session with a random 128-bit session ID and the default
settings (see section: @ref settings). If the client already has a session ID
cookie with name `SESSION_COOKIE_NAME` and the given `domain` and `path`, and
that ID refers to a valid session, that existing session will be returned
instead.

@note
For all of the sessions plugin function calls, `domain` and `path` arguments
are required. These must match the values that were given when the session was
created, otherwise the client's session ID cookie won't be found.  The `domain`
and `path` arguments may be set to `NULL` to use the current values given by
the `soap` context's `cookie_domain` and `cookie_path` members:

@note
| Attribute                   | Value
| --------------------------- | ------------------------------------------------
| `const char *cookie_domain` | MUST be set to the domain (host) of the service
| `const char *cookie_path`   | MAY be set to the default path to the service (defaults to '`/`')

@note
These rules also apply to the cookies used for tracking session IDs.  See the
[gSOAP User Guide](http://genivia.com/doc/soapdoc2.html) for more information
on how gSOAP handles cookies.

For example, to create a session on localhost:8080 that is valid on any path:

@code
    soap->cookie_domain = "localhost:8080";
    struct soap_session *session = soap_start_session(soap, NULL, NULL); 
@endcode
    
To create a new session variable or modify an existing session variable inside that session:

@code
    soap_set_session_var(soap, "session_var_name", "session_var_value", NULL, NULL);
@endcode
    
To access the value of that session variable:

@code
    const char *value = soap_get_session_var(soap, "session_var_name", NULL, NULL);
@endcode

To delete that session variable:

@code
    soap_clr_session_var(soap, "session_var_name", NULL, NULL);
@endcode

Consider changing the session's `rolling`, `cookie_maxage`, 
`max_inactive_secs` settings from their defaults. Read more about these in the 
@ref settings section.

@section example Simple Calculator Server (C) Example

We will show how to extend the simple calculator server example (in C) to 
keep track of the client's last calculated value with a session variable.

Make sure to register the plugin and set the default `domain` and `path`. After
calling `soap_destroy` and `soap_end`, we should also free the internal cookie
database (or call `soap_done`):

@code
    #include "plugin/session.h"
    ...
    struct soap soap;
    soap_init(&soap);
    soap_register_plugin(&soap, sessions);
    soap.cookie_domain = "localhost:8080";
    soap.cookie_path = "/";
    ... // set up the service and serve requests
    soap_destroy(&soap);
    soap_end(&soap);
    soap_free_cookies(&soap);
@endcode

Now in each of our calculator service operations (`ns__add`, `ns__sub`, etc.),
we can update the `lastCalculation` session variable in our current
session:

@code
    int ns__add(struct soap *soap, double a, double b, double *result)
    { 
      (void)soap; *result = a + b;
      
      // make sure we have a session 
      
      if (!soap_get_session(soap, NULL, NULL))
      {
        // no current session, create a new one
        
        if (!soap_start_session(soap, NULL, NULL))
          return SOAP_ERR; // session creation failed
        
        // configure session settings
        soap_set_session_rolling(soap, 1, NULL, NULL);
        soap_set_session_cookie_maxage(soap, 30, NULL, NULL);
        soap_set_session_max_inactive_secs(soap, 30, NULL, NULL);
      }
      
      // update `lastCalculation` session variable 
      
      char calculation[80];
      sprintf(calculation, "add %f %f, result = %f", a, b, *result);
      soap_set_session_var(soap, "lastCalculation", calculation, NULL, NULL);
      
      return SOAP_OK;
    } 
@endcode

If we want to allow the client to access their last calculation, add another 
service operation to retrieve the `lastCalculation` session variable:

@code
    int ns__getLastCalculation(struct soap *soap, char **r)
    { 
      const char *lastCalculation = soap_get_session_var(soap, "lastCalculation", NULL, NULL);
      if (!lastCalculation) 
        lastCalculation = "None";
      if ((*r = (char*)soap_malloc(soap, strlen(lastCalculation)+1)))
        strcpy(*r, lastCalculation);
      return SOAP_OK;
    }
@endcode


@section api Sessions Plugin API

This section describes the Sessions Plugin API.

@note
The term *session ID cookie* refers to a cookie with the name
`SESSION_COOKIE_NAME` (see section: @ref settings), the given `domain` and
`path` (see section: @ref usage), and a random 128-bit session ID as the value.
The term *current session* refers to the session in the database
identified by the client's session ID cookie.

@note
The arguments `domain` and `path` may be set to `NULL` to use the current
values given by `soap->cookie_domain` and `soap->cookie_path` (see section:
@ref usage).

The sessions plugin provides the following API for server-side session
management:

<table>
  <tr><th>Session Plugin Function Description</th></tr>
  <tr><td>
    <em><b>struct soap_session*</b> soap_start_session(<b>struct soap *</b>
    soap, <b>const char *</b>domain, <b>const char *</b>path);</em>\n
    Create a session in the database and a session ID cookie. Behaves the 
    same as `soap_get_session` if the client already has a valid session ID 
    cookie. Returns pointer to the session node, or `NULL` if the session 
    could not be created.
  </td></tr>
  <tr><td>
    <em><b>struct soap_session*</b> soap_get_session(<b>struct soap *</b>soap, 
    <b>const char *</b>domain, <b>const char *</b>path);</em>\n
    Get the current session. Can be used as a 'touch'. Returns pointer to the 
    session node, or `NULL` if the session is not found or is expired.
  </tr></td>
  <tr><td>
    <em><b>int</b> soap_get_num_sessions(<b>struct soap *</b>soap, <b>const 
    char *</b>domain, <b>const char *</b>path);</em>\n
    Get the number of sessions in the database. Returns `-1` if the session 
    is not found or is expired.
  </tr></td>
  <tr><td>
    <em><b>struct soap_session*</b> soap_regenerate_session_id(<b>struct soap *
    </b>soap, <b>const char *</b>domain, <b>const char *</b>path);</em>\n
    Create a new session ID for the current session and update the session ID 
    cookie. Returns pointer to the session node, or `NULL` if the session 
    is not found or is expired.
  </tr></td>
  <tr><td>
    <em><b>int</b> soap_get_num_session_vars(<b>struct soap *</b>soap, <b>const 
    char *</b>domain, <b>const char *</b>path);</em>\n
    Get the number of session variables in the current session. Returns `-1` if 
    the session is not found or is expired.
  </tr></td>
  <tr><td>
    <em><b>void</b> soap_end_session(<b>struct soap *</b>soap, <b>const char *
    </b>domain, <b>const char *</b>path);</em>\n
    Delete the current session from the database and clear the client's 
    session ID cookie.
  </tr></td>
  <tr><td>
    <em><b>void</b> soap_end_sessions(<b>struct soap *</b>soap, <b>const char *
    </b>domain, <b>const char *</b>path);</em>\n
    Delete all sessions in the database and clear the current session ID  
    cookie.
  </tr></td>
  <tr><td>
    <em><b>struct soap_session_var*</b> soap_set_session_var(<b>struct soap *
    </b>soap, <b>const char *</b>name, <b>const char *</b>value, <b>const char 
    *</b>domain, <b>const char *</b>path);</em>\n
    Set a session variable with the given `name` and `value` in the current 
    session, overwring any old `value`. Returns pointer to the session variable 
    node, or `NULL` if the session is not found or is expired.
  </tr></td>
  <tr><td>
    <em><b>const char*</b> soap_get_session_var(<b>struct soap *</b>soap, <b>
    const char *</b>name, <b>const char *</b>domain, <b>const char *</b>path);
    </em>\n
    Get the `value` of the session variable with the given `name` in the 
    current session. Returns `NULL` if the session is not found or is expired.
  </tr></td>
  <tr><td>
    <em><b>void</b> soap_clr_session_var(<b>struct soap *</b>soap, <b>const 
    char *</b>name, <b>const char *</b>domain, <b>const char *</b>path);</em>\n
    Delete the session variable with the given `name` in the current 
    session.
  </tr></td>
  <tr><td>
    <em><b>void</b> soap_clr_session_vars(<b>struct soap</b> *soap, <b>const 
    char</b> *domain, <b>const char</b> *path);</em> \n
    Delete all session variables in the current session.
  </tr></td>
</table>

For managing session settings (see section: @ref settings), the following API is provided:

<table>
  <tr><th>Session Plugin Function Description (Settings / Misc.)</th></tr>
  <tr><td>
    <em><b>int</b> soap_set_session_rolling(<b>struct soap *
    </b>soap, <b>int</b> rolling, <b>const char *</b>domain, <b>const char *
    </b>path);</em>\n
    Set the `rolling` setting for the current session. Zero (non-rolling) means 
    the session ID cookie is only set when the session is first created. 
    Non-zero (rolling) means the session ID cookie will be re-set on every 
    response. If successful, returns SOAP_OK, or SOAP_ERR otherwise.
  </tr></td>
  <tr><td>
    <em><b>int</b> soap_get_session_rolling(<b>struct soap *
    </b>soap, <b>const char *</b>domain, <b>const char *</b>path);</em>\n
    Get the `rolling` setting for the current session. Returns `-1` if the 
    session is not found or is expired. 
  </tr></td>
  <tr><td>
    <em><b>int</b> soap_set_session_cookie_maxage(<b>struct soap *
    </b>soap, <b>long</b> maxage, <b>const char *</b>domain, <b>const char *
    </b>path);</em>\n
    Set the `cookie_maxage` member for the current session. This will be used 
    to set the session ID cookie's `Max-Age` and `Expires` attributes. `-1` means 
    the cookie will have no `Max-Age` or `Expires` attributes. If successful, 
    returns SOAP_OK, or SOAP_ERR otherwise.
  </tr></td>
  <tr><td>
    <em><b>long</b> soap_get_session_cookie_maxage(<b>struct soap *
    </b>soap, <b>const char *</b>domain, <b>const char *</b>path);</em>\n
    Get the `cookie_maxage` member for the current session. Returns `0` if the 
    session is not found or is expired. 
  </tr></td>
  <tr><td>
    <em><b>int</b> soap_set_session_max_inactive_secs(<b>struct soap *
    </b>soap, <b>long</b> max, <b>const char *</b>domain, <b>const char *
    </b>path);</em>\n
    Set the `max_inactive_secs` member for the current session. This is 
    the maximum amount of stale time in seconds before a session is marked 
    as expired (to be purged) on the server-side. `-1` means the session will 
    never expire. If successful, returns SOAP_OK, or SOAP_ERR otherwise.
  </tr></td>
  <tr><td>
    <em><b>long</b> soap_get_session_max_inactive_secs(<b>struct soap *
    </b>soap, <b>const char *</b>domain, <b>const char *</b>path);</em>\n
    Get the `max_inactive_secs` member for the current session. Returns `0`
    if the session is not found or is expired. 
  </tr></td>
  <tr><td>
    <em><b>struct soap_session*</b> soap_get_sessions_head(<b></b>);</em> \n
    Get `soap_sessions`, the first node in the sessions linked list. Not 
    necessary unless low-level list management is needed (see section: @ref 
    internals).
  </tr></td>
</table>

@section settings Settings

In `plugins/session.h`, the following constants are defined:

| Name                                  | Default               | Description
| ------------------------------------- | --------------------- | --------------------------------------------------
| `SESSION_COOKIE_NAME`                 | `"GSOAP_SESSIONID"`   | The name to use for session ID cookies.
| `SESSION_MAX`                         | `8192`                | The max number of sessions allowed in the database. Any attempt to create more sessions than this will fail.
| `SESSIONS_DEFAULT_MAX_INACTIVE_SECS`  | `300`                 | The default time in seconds for sessions' `max_inactive_secs` member. `-1` means the session will never expire server-side.
| `SESSION_PURGE_INTERVAL`              | `300`                 | On every session database access, the time since the last database purge is checked. If it's been longer than this interval (in seconds), any expired sessions will be purged with `soap_purge_sessions()`. `0` means a purge will occcur on every database access. `-1` means a purge will never occur.

Set these values to whatever makes sense for your application.

In each session node, the following settings are declared:

| Name                    | Default                              | Description
| ----------------------- | ------------------------------------ | -----------------------
| `rolling`               | `0`                                  | Non-rolling (`0`) means the session ID cookie is only set when the session is first created. Rolling (non-`0`) means the session ID cookie will be re-set on every response after any part of the session has been touched.
| `cookie_maxage`         | `-1`                                 | This will be used to set the session ID cookie's `Max-Age` and `Expires` attributes. `-1` means the cookie will have no `Max-Age` or `Expires` attributes.
| `max_inactive_secs`     | `SESSIONS_DEFAULT_MAX_INACTIVE_SECS` | This is the maximum amount of stale time in seconds before a session is marked as expired (to be purged) on the server-side. `-1` means the session will never expire server-side. This should probably be set equal to `cookie_maxage` unless that value is `-1`.

@note
These settings should not be changed directly. See the @ref api section for
functions to manage these settings.

@section internals The Internals

The session database is stored as a linked list pointed to by `soap_sessions`
(accessible by calling `soap_get_sessions_head()`). Session nodes are of type
`struct soap_session` and are declared as:

@code
  struct soap_session
  { 
    struct soap_session *next;
    const char *id;
    struct soap_session_var *session_vars;
    struct soap_session_var *session_vars_tail;
    int num_session_vars;   // the total number of session_vars we have in this session 
    long cookie_maxage;     // client time in seconds before the cookie expires (-1 for session cookie) 
    int rolling;            // whether or not we should set the cookie's maxage on every response 
    time_t last_touched;    // time that we last touched this session or its vars 
    long max_inactive_secs; // server time in seconds before inactive session expires (-1 to never expire) 
                            //  this should usually be at least as high as cookie_maxage 
  };
@endcode

The `session_vars` member is a pointer to the session's linked list of session
variables. Session variable nodes are of type `struct soap_session_var` and are
declared as:

@code
    struct soap_session_var
    {
      struct soap_session_var *next;
      const char *name;
      const char *value;
    };
@endcode
  
*/

#include "sessions.h"

#ifdef __cplusplus
extern "C" {
#endif

const char sessions_id[] = SESSIONS_ID;

/** Head node of session database (linked list) */
static struct soap_session *soap_sessions = NULL;

/** Tail node of session database */
static struct soap_session *soap_sessions_tail = NULL;

/** Number of sessions in database */
static int num_sessions = 0;

/** Time that we last purged expired sessions */
static time_t last_purged_sessions;

/** Session database lock */
static MUTEX_TYPE sessions_lock = MUTEX_INITIALIZER;

/******************************************************************************\
 *
 *      Forward decls
 *
\******************************************************************************/

static int sessions_init(struct soap *soap);
static void sessions_delete(struct soap *soap, struct soap_plugin *p);

/******************************************************************************\
 *
 *      Plugin registry
 *
\******************************************************************************/

/**
@brief Plugin registry function, invoked by soap_register_plugin.
*/
SOAP_FMAC1
int
SOAP_FMAC2
sessions(struct soap *soap, struct soap_plugin *p, void *arg)
{ 
  (void)arg;
  p->id = sessions_id;
  /* register the destructor */
  p->fdelete = sessions_delete;
  if (sessions_init(soap))
    return SOAP_EOM; /* return error */
  return SOAP_OK;
}

/******************************************************************************/

/**
@brief Used by plugin registry function.
*/
static int
sessions_init(struct soap *soap)
{ 
  (void)soap;
  last_purged_sessions = time(NULL);  
  
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      Callbacks
 *
\******************************************************************************/

/**
@brief Deletes all sessions. Called when plugin is ended by first caller.
*/
static void
sessions_delete(struct soap *soap, struct soap_plugin *p)
{ 
  (void)p;
  soap_end_sessions(soap, NULL, NULL);
}

/******************************************************************************\
 *
 *      For Internal Plugin Use (NOT THREAD-SAFE ALONE)
 *
\******************************************************************************/

/**
@brief Intended for internal plugin use. Creates a cookie that will be sent to 
the client, named SESSION_COOKIE_NAME with the given value, domain, 
and path.
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_session_cookie_create(struct soap *soap, const char *value, const char *domain, const char *path, long maxage)
{
  /* create session ID cookie */
  soap_set_cookie(soap, SESSION_COOKIE_NAME, value, domain, path);
  soap_set_cookie_session(soap, SESSION_COOKIE_NAME, domain, path);
  soap_set_cookie_expire(soap, SESSION_COOKIE_NAME, maxage, domain, path);
}

/******************************************************************************/

/**
@brief Intended for internal plugin use. Creates a cookie that will be sent to 
the client, named SESSION_COOKIE_NAME with the given domain and path. The value
will be empty and it will be set to expire when the client recieves it. This 
ensures the session ID is no longer in the client's cookie.
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_session_cookie_delete(struct soap *soap, const char *domain, const char *path)
{
  /* delete client-side cookie */
  soap_set_cookie(soap, SESSION_COOKIE_NAME, "", domain, path); 
  soap_set_cookie_session(soap, SESSION_COOKIE_NAME, domain, path);
  soap_set_cookie_expire(soap, SESSION_COOKIE_NAME, 0, domain, path); 
}

/******************************************************************************/

/**
@brief Intended for internal plugin use. Find the current session with 
domain and path. Returns NULL if not found or if the session is expired (for 
normal use, set `when` to `time(NULL)`). Will also clear the cookie if the 
session is expired.
*/
SOAP_FMAC1
struct soap_session*
SOAP_FMAC2
soap_session_internal_find(struct soap *soap, const char *domain, const char *path, time_t when) 
{
  struct soap_session *s = NULL;
  const char *id;
  
  id = soap_cookie_value(soap, SESSION_COOKIE_NAME, domain, path);
  
  if (id) 
  {
    for (s = soap_sessions; s; s = s->next)
      if (s && !strcmp(s->id, id))
        break;
    
    if (!s)
      return NULL;
    
    if (s->max_inactive_secs >= 0 && (s->last_touched + s->max_inactive_secs <= when))
    {
      /* session has expired and has not been purged, or NULL */
      soap_session_cookie_delete(soap, domain, path);
      
      return NULL;
    }
  }
  return s;
}


/******************************************************************************\
 *
 *      Sessions Public API
 *
\******************************************************************************/

/**
@brief Gets soap_sessions (first node in sessions linked list).
*/
SOAP_FMAC1
struct soap_session*
SOAP_FMAC2
soap_get_sessions_head() 
{
  return soap_sessions;
}

/******************************************************************************/

/**
@brief Removes all sessions that have been stale longer than their 
max_inactive_secs allows. -1 max_inactive_secs means the session will 
never expire. The when parameter is usually the current time, i.e. time(NULL) 
to purge sessions that expire after the when time.

This is called internally every time a session or a session_var is accessed, so
it shouldn't ever be necessary to manually call this function.
  
Only will purge when the last time we purged was more than SESSION_PURGE_INTERVAL
seconds ago (defined in sessions.h).
SESSION_PURGE_INTERVAL == -1 means never purge.
SESSION_PURGE_INTERVAL == 0 means always purge.
*/
SOAP_FMAC1
void 
SOAP_FMAC2
soap_purge_sessions(struct soap *soap, const char *domain, const char *path, time_t when)
{
  struct soap_session *s, *t;
  struct soap_session_var *v;
  const char *id;

  if (SESSION_PURGE_INTERVAL == -1)
    return;
  
  MUTEX_LOCK(sessions_lock);
  
  /* If we purge the current session, we'll want to clear
     the cookie, too. So grab the current ID to check against. */
  id = soap_cookie_value(soap, SESSION_COOKIE_NAME, domain, path);
  
  /* not time to purge yet */
  if (SESSION_PURGE_INTERVAL && (last_purged_sessions + SESSION_PURGE_INTERVAL >= when))
  {
    MUTEX_UNLOCK(sessions_lock);

    return;
  }

  last_purged_sessions = when;
  
  s = soap_sessions;
  while (s)
  {
    t = s->next;

    /* delete session if its older than allowed */
    if (s->max_inactive_secs >= 0 && (s->last_touched + s->max_inactive_secs <= when))
    {
      /* clear the cookie if this was the current session */
      if (id && !strcmp(id, s->id))
        soap_session_cookie_delete(soap, domain, path);
      
      /* delete session vars */
      for (v = s->session_vars; v;)
      {
        struct soap_session_var *u;

        if (v->name)
          SOAP_FREE(soap, v->name);
        if (v->value)
          SOAP_FREE(soap, v->value);

        u = v->next;
        SOAP_FREE(soap, v);
        v = u;
      }
      
      /* delete session data */
      if (s->id)
        SOAP_FREE(soap, s->id);
      
      /* deleting not last node? */
      if (t)
      {
        /* adjust tail if this was second to last */
        if (t == soap_sessions_tail)
          soap_sessions_tail = s;

        s->id = t->id;
        s->session_vars = t->session_vars;
        s->session_vars_tail = t->session_vars_tail;
        s->num_session_vars = t->num_session_vars;
        s->cookie_maxage = t->cookie_maxage;
        s->rolling = t->rolling;
        s->last_touched = t->last_touched;
        s->max_inactive_secs = t->max_inactive_secs;
        s->next = t->next;
        SOAP_FREE(soap, t);
      }
      else /* deleting the last node */
      {
        /* set tail to previous node */
        struct soap_session *u = soap_sessions;
        while (u && u->next != s)
          u = u->next;
        soap_sessions_tail = u;  
        if (soap_sessions_tail == NULL)
          soap_sessions = NULL;
        else
          soap_sessions_tail->next = NULL;
        SOAP_FREE(soap, s);
        
        /* to end the loop */
        s = NULL;
      }
  
      num_sessions--;
    }
    else
    {
      /* only move s to the next node if we didn't do a deletion 
        (because we moved the next node onto s if we did do a deletion) */
      s = t;
    }
  }
  
  MUTEX_UNLOCK(sessions_lock);
}

/******************************************************************************/

/**
@brief Gets the number of sessions in the database.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_get_num_sessions(struct soap *soap, const char *domain, const char *path)
{ 
  /* purge first, so num_sessions will not include expired ones */
  soap_purge_sessions(soap, domain, path, time(NULL));
  
  return num_sessions;
}

/******************************************************************************/

/**
@brief Finds the session identified by the cookie with name SESSION_COOKIE_NAME 
and the given domain and path. Updates session's last_touched to now. Returns 
NULL if not found.
*/
SOAP_FMAC1
struct soap_session*
SOAP_FMAC2
soap_get_session(struct soap *soap, const char *domain, const char *path)
{ 
  struct soap_session *s;
  time_t now = time(NULL);

  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);
  
  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s)
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return NULL;
  }
  
  s->last_touched = now;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage);
  
  MUTEX_UNLOCK(sessions_lock);

  return s;
}

/******************************************************************************/

/**
@brief Creates a new session with a random 128-bit ID. A cookie will be created 
with name SESSION_COOKIE_NAME and the given domain and path. If there 
already exists a session ID cookie, the only action will be updating the 
session's last_touched to now. Returns a pointer to the session.
*/
SOAP_FMAC1
struct soap_session*
SOAP_FMAC2
soap_start_session(struct soap *soap, const char *domain, const char *path)
{ 
  struct soap_session *s = NULL;
  time_t now = time(NULL);
  const char *id;
  
  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);
  
  id = soap_cookie_value(soap, SESSION_COOKIE_NAME, domain, path);
  
  if (id)
  {
    for (s = soap_sessions; s; s = s->next)
      if (!strcmp(s->id, id))
        break;
  }
  
  if (!s)
  { 
    /* check if we have too many sessions */
    if (num_sessions >= SESSION_MAX)
    {
      MUTEX_UNLOCK(sessions_lock);
      
      return NULL;
    }
    
    /* create new session */
    s = (struct soap_session*)SOAP_MALLOC(soap, sizeof(struct soap_session));
    if (s)
    { 
      /* generate random 128-bit session ID */
      s->id = (const char*)soap_strdup(NULL, soap_rand_uuid(soap, NULL));
      
      s->session_vars = NULL;
      s->session_vars_tail = NULL;
      s->num_session_vars = 0;
      s->cookie_maxage = -1;
      s->rolling = 0;
      s->last_touched = now;
      s->max_inactive_secs = SESSIONS_DEFAULT_MAX_INACTIVE_SECS;
      
      /* add to end of list */
      if (num_sessions == 0)
        soap_sessions = s;
      else
        soap_sessions_tail->next = s;
      soap_sessions_tail = s;
      soap_sessions_tail->next = NULL;
      
      num_sessions++;
      
      soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage);
    }
  }
  else
  {
    /* session already exists */
    if (s->max_inactive_secs >= 0 && (s->last_touched + s->max_inactive_secs <= now))
    {
      /* session has expired and has not been purged, or NULL */
      soap_session_cookie_delete(soap, domain, path);
    }
    else
    {
      /* update last_touched */
      s->last_touched = now;
    
      if (s->rolling)
        soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
    }
  }
  
  MUTEX_UNLOCK(sessions_lock);
  
  return s;
}

/******************************************************************************/

/**
@brief The session ID for the current session (identified by the cookie with 
name SESSION_COOKIE_NAME and the given domain and path) will be set to a new 
random 128-bit ID. Also updates the cookie. Returns a pointer to the session.
*/
SOAP_FMAC1 
struct soap_session* 
SOAP_FMAC2 
soap_regenerate_session_id(struct soap *soap, const char *domain, const char *path)
{
  struct soap_session *s;
  time_t now = time(NULL);

  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);
  
  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s)
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return NULL;
  }
  
  s->last_touched = now;
  
  if (s->id)
    SOAP_FREE(soap, s->id);
  
  /* generate random 128-bit session ID if one wasn't provided */
  s->id = (const char*)soap_strdup(NULL, soap_rand_uuid(soap, NULL));
  
  /* reset session ID cookie */
  soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage);
  
  MUTEX_UNLOCK(sessions_lock);

  return s;
}

/******************************************************************************/

/**
@brief Frees all sessions in database.
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_end_sessions(struct soap *soap, const char *domain, const char *path)
{ 
  struct soap_session *s;

  MUTEX_LOCK(sessions_lock);
  
  last_purged_sessions = time(NULL);
  
  s = soap_sessions;
  while (s)
  {
    /* delete session_vars */
    struct soap_session *t = s->next;
    struct soap_session_var *v = s->session_vars;

    while (v)
    {
      struct soap_session_var *u;

      if (v->name)
        SOAP_FREE(soap, v->name);
      if (v->value)
        SOAP_FREE(soap, v->value);
    
      u = v->next;
      SOAP_FREE(soap, v);
      v = u;
    }

    /* delete session */
    if (s->id)
      SOAP_FREE(soap, s->id);
    SOAP_FREE(soap, s);
    s = t;
  }
  
  soap_sessions = NULL;
  soap_sessions_tail = NULL;
  num_sessions = 0;
  
  /* clear session ID cookie */
  soap_session_cookie_delete(soap, domain, path);
  
  MUTEX_UNLOCK(sessions_lock);
}

/******************************************************************************/

/**
@brief Frees the current session (identified by the cookie with name 
SESSION_COOKIE_NAME and the given domain and path). Also clears the cookie.
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_end_session(struct soap *soap, const char *domain, const char *path)
{ 
  struct soap_session *s;
  struct soap_session_var *v;
  time_t now = time(NULL);

  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);
  
  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s)
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return;
  }
  
  /* deleting whether expired or not */
  
  /* delete session vars */
  for (v = s->session_vars; v;)
  {
    struct soap_session_var *u;

    if (v->name)
      SOAP_FREE(soap, v->name);
    if (v->value)
      SOAP_FREE(soap, v->value);

    u = v->next;
    SOAP_FREE(soap, v);
    v = u;
  }
  
  /* delete session data */
  if (s->id)
    SOAP_FREE(soap, s->id);
  
  struct soap_session *temp = s->next;
  /* deleting not last node */
  if (temp != NULL)
  {
    /* adjust tail if this was second to last */
    if (temp == soap_sessions_tail)
      soap_sessions_tail = s;

    s->id = temp->id;
    s->session_vars = temp->session_vars;
    s->session_vars_tail = temp->session_vars_tail;
    s->num_session_vars = temp->num_session_vars;
    s->cookie_maxage = temp->cookie_maxage;
    s->last_touched = temp->last_touched;
    s->max_inactive_secs = temp->max_inactive_secs;
    s->next = temp->next;
    SOAP_FREE(soap, temp);
  }
  /* deleting the last node */
  else
  {
    /* set tail to previous node */
    struct soap_session *t = soap_sessions;
    while (t && t->next != s)
      t = t->next;
    soap_sessions_tail = t;  
    if (soap_sessions_tail == NULL)
          soap_sessions = NULL;
    else
      soap_sessions_tail->next = NULL;
    
    SOAP_FREE(soap, s);
  }

  num_sessions--;
  
  /* delete client-side cookie */
  soap_session_cookie_delete(soap, domain, path);
  
  MUTEX_UNLOCK(sessions_lock);
}

/******************************************************************************/

/**
@brief Gets the cookie_maxage for the current session (identified by the cookie 
with name SESSION_COOKIE_NAME and the given domain and path). -1 means the 
cookie is a session cookie. 0 means a session was not found or 
is expired.
*/
SOAP_FMAC1
long
SOAP_FMAC2
soap_get_session_cookie_maxage(struct soap *soap, const char *domain, const char *path)
{ 
  struct soap_session *s;
  time_t now = time(NULL);
  long result;
  
  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);

  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s)
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return 0;
  }
  
  result = s->cookie_maxage;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  MUTEX_UNLOCK(sessions_lock);
  
  return result;
}

/******************************************************************************/

/**
@brief Sets the cookie_maxage for the current session (identified by the 
cookie with name SESSION_COOKIE_NAME and the given domain and path). -1 means 
the cookie will be a session cookie. If successful, returns 
SOAP_OK, or SOAP_ERR otherwise.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_set_session_cookie_maxage(struct soap *soap, long maxage, const char *domain, const char *path)
{ 
  struct soap_session *s;
  time_t now = time(NULL);

  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);  

  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s) 
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return SOAP_ERR;
  }

  s->cookie_maxage = maxage;
  
  soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  MUTEX_UNLOCK(sessions_lock);

  return SOAP_OK;
}

/******************************************************************************/

/**
@brief Gets the max_inactive_secs for the current session (identified by 
the cookie with name SESSION_COOKIE_NAME and the given domain and path). -1 
means the session will never expire. 0 means the session wasn't found or 
is expired.
*/
SOAP_FMAC1
long
SOAP_FMAC2
soap_get_session_max_inactive_secs(struct soap *soap, const char *domain, const char *path)
{ 
  struct soap_session *s;
  time_t now = time(NULL);
  long result;
  
  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);
  
  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s) 
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return 0;
  }
  
  result = s->max_inactive_secs;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  MUTEX_UNLOCK(sessions_lock);
  
  return result;
}

/******************************************************************************/

/**
@brief Sets the max_inactive_secs for the current session (identified by 
the cookie with name SESSION_COOKIE_NAME and the given domain and path). -1 
means the session will never expire. If successful, returns  SOAP_OK, or 
SOAP_ERR otherwise.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_set_session_max_inactive_secs(struct soap *soap, long max, const char *domain, const char *path)
{ 
  struct soap_session *s;
  time_t now = time(NULL);
  
  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);
  
  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s) 
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return SOAP_ERR;
  }
  
  s->max_inactive_secs = max;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  MUTEX_UNLOCK(sessions_lock);
  
  return SOAP_OK;
}

/******************************************************************************/

/**
@brief Gets the `rolling` flag for the current session (identified by the 
cookie with name SESSION_COOKIE_NAME and the given domain and path). -1 means 
the session was not found or is expired.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_get_session_rolling(struct soap *soap, const char *domain, const char *path)
{ 
  struct soap_session *s;
  time_t now = time(NULL);
  int result;

  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);  

  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s) 
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return -1;
  }

  result = s->rolling;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  MUTEX_UNLOCK(sessions_lock);

  return result;
}

/******************************************************************************/

/**
@brief Sets the `rolling` flag for the current session (identified by the 
cookie with name SESSION_COOKIE_NAME and the given domain and path). If 
successful, returns SOAP_OK, or SOAP_ERR otherwise.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_set_session_rolling(struct soap *soap, int rolling, const char *domain, const char *path)
{ 
  struct soap_session *s;
  time_t now = time(NULL);

  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);  
  
  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s) 
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return SOAP_ERR;
  }

  s->rolling = rolling;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  MUTEX_UNLOCK(sessions_lock);

  return SOAP_OK;
}

/******************************************************************************\
 *
 *      Session Variables Public API
 *
\******************************************************************************/

/**
@brief Gets the number of sessions variables in the current session (identified 
by the cookie with name SESSION_COOKIE_NAME and the given domain and path). 
Returns -1 if the session does not exist.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_get_num_session_vars(struct soap *soap, const char *domain, const char *path)
{ 
  struct soap_session *s;
  time_t now = time(NULL);
  int result;
  
  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);
  
  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s)
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return -1;
  }
    
  result = s->num_session_vars;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
    
  MUTEX_UNLOCK(sessions_lock);
  
  return result;
}

/******************************************************************************/

/**
@brief Returns session_var value by name in the current session (identified by 
the cookie with name SESSION_COOKIE_NAME and the given domain and path). 
Returns NULL if not found.
*/
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_get_session_var(struct soap *soap, const char *name, const char *domain, const char *path)
{
  struct soap_session *s;
  struct soap_session_var *v;
  time_t now = time(NULL);
  const char *value;

  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);

  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s)
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return NULL;
  }
  
  s->last_touched = now;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  for (v = s->session_vars; v; v = v->next)
    if (!strcmp(v->name, name))
      break;
    
  value = v ? v->value : NULL;  
    
  MUTEX_UNLOCK(sessions_lock);

  return value; 
}

/******************************************************************************/

/**
@brief Add new session_var with the given name and value to the current session 
(identified by the cookie with name SESSION_COOKIE_NAME and the given domain 
and path). If the name is already used, the value is re-written. If successful, 
returns pointer to  a session_var node in the session's linked list, or NULL 
otherwise.
*/
SOAP_FMAC1
struct soap_session_var*
SOAP_FMAC2
soap_set_session_var(struct soap *soap, const char *name, const char *value, const char *domain, const char *path)
{ 
  struct soap_session *s;
  struct soap_session_var *v;
  time_t now = time(NULL);
  
  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);
  
  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s)
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return NULL;
  }
  
  s->last_touched = now;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  for (v = s->session_vars; v; v = v->next)
    if (!strcmp(v->name, name))
      break;

  if (!v)
  { 
    v = (struct soap_session_var*)SOAP_MALLOC(soap, sizeof(struct soap_session_var));
    if (v)
    { 
      size_t l = strlen(name);
      char *x = (char*)SOAP_MALLOC(soap, l + 1);
      v->name = x;
      if (v->name)
        soap_strcpy(x, l + 1, name);
      l = strlen(value);
      x = (char*)SOAP_MALLOC(soap, l + 1);
      v->value = x;
      if (v->value)
        soap_strcpy(x, l + 1, value); 
        
      /* add to end of list */
      if (s->num_session_vars == 0)
        s->session_vars = v;
      else
        s->session_vars_tail->next = v;
      s->session_vars_tail = v;
      s->session_vars_tail->next = NULL;
     
      s->num_session_vars++;
    }
  }
  else
  {
    size_t l = strlen(value);
    char *x;

    if (v->value)
      SOAP_FREE(soap, v->value);
    x = (char*)SOAP_MALLOC(soap, l + 1);
    v->value = x;
    if (v->value)
      soap_strcpy(x, l + 1, value);
  }
  
  MUTEX_UNLOCK(sessions_lock);
  
  return v;
}

/******************************************************************************/

/**
@brief Frees all session vars in the current session (identified by the cookie 
with name SESSION_COOKIE_NAME and the given domain and path).
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_clr_session_vars(struct soap *soap, const char *domain, const char *path)
{
  struct soap_session *s;
  struct soap_session_var *v;
  time_t now = time(NULL);
  
  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);

  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s)
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return;
  }

  s->last_touched = now;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  v = s->session_vars;
  while (v)
  {
    struct soap_session_var *u;

    if (v->name)
      SOAP_FREE(soap, v->name);
    if (v->value)
      SOAP_FREE(soap, v->value);
  
    u = v->next;
    SOAP_FREE(soap, v);
    v = u;
  }
  
  s->session_vars = NULL;
  s->num_session_vars = 0;
  
  MUTEX_UNLOCK(sessions_lock);
}

/******************************************************************************/

/**
@brief Frees session var with the given name in the current session (identified 
by the cookie with name SESSION_COOKIE_NAME and the given domain and path).
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_clr_session_var(struct soap *soap, const char *name, const char *domain, const char *path)
{
  struct soap_session *s;
  struct soap_session_var *v;
  time_t now = time(NULL);

  soap_purge_sessions(soap, domain, path, now);
  
  MUTEX_LOCK(sessions_lock);

  s = soap_session_internal_find(soap, domain, path, now);
  
  if (!s)
  {
    MUTEX_UNLOCK(sessions_lock);
    
    return;
  }

  s->last_touched = now;
  
  if (s->rolling)
    soap_session_cookie_create(soap, s->id, domain, path, s->cookie_maxage); 
  
  for (v = s->session_vars; v; v = v->next)
    if (!strcmp(v->name, name))
      break;
    
  if (!v) 
  {
    MUTEX_UNLOCK(sessions_lock);

    return;
  }

  /* delete session var data */
  if (v->name)
   SOAP_FREE(soap, v->name);
  if (v->value)
    SOAP_FREE(soap, v->value);  
  
  struct soap_session_var *temp = v->next;
  /* deleting not last node */
  if (temp != NULL)
  {
    /* adjust tail if this was second to last */
    if (temp == s->session_vars_tail)
      s->session_vars_tail = v;

    v->name = temp->name;
    v->value = temp->value;
    v->next = temp->next;
    SOAP_FREE(soap, temp);
  }
  /* deleting the last node */
  else
  {
    /* set tail to previous node */
    struct soap_session_var *t = s->session_vars;
    while (t && t->next != v)
      t = t->next;
    s->session_vars_tail = t;
    if (s->session_vars_tail == NULL)
      s->session_vars = NULL;
    else
      s->session_vars_tail->next = NULL;
      
    SOAP_FREE(soap, v);
  }
  
  s->num_session_vars--;
        
  MUTEX_UNLOCK(sessions_lock);
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif
