/*

        Annotated and simplified stdsoap2.h to generate the user guide

gSOAP XML Web services tools
Copyright (C) 2000-2020, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under ONE of the following licenses:
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
Copyright (C) 2000-2020, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
@file stdsoap2.h
@brief This file defines the common macros, types and functions of the gSOAP API grouped by [modules](modules.html)

The API features are grouped by the following [modules](modules.html):
- \ref group_debug
- \ref group_with
- \ref group_soap
- \ref group_flags
- \ref group_errors
- \ref group_context
- \ref group_callbacks
- \ref group_ssl
- \ref group_io
- \ref group_cookies
- \ref group_s2s
- \ref group_namespace
- \ref group_header
- \ref group_fault
- \ref group_dime
- \ref group_mime
- \ref group_plugin
- \ref group_misc
*/

/**
\defgroup group_debug Debugging and logging
@brief This module defines compile-time flags and functions for run-time debugging and logging

This module defines the following compile-time flags and functions to specify log files:
- `#DEBUG`
- `#SOAP_DEBUG`
- `#DEBUG_STAMP`
- `#SOAP_MEM_DEBUG`
- `::soap_set_recv_logfile`
- `::soap_set_sent_logfile`
- `::soap_set_test_logfile`

Alternatively, the `::logging` plugin can be used without setting `#DEBUG` to efficiently log messages and collect statistics:
- `::soap_set_logging_inbound`
- `::soap_set_logging_outbound`
- `::soap_logging_stats`
- `::soap_reset_logging_stats`

@{
*/

/// User-definable macro to enable debugging and logging
/**
When this macro is defined at compile time (undefined by default), the engine runs in debug mode to produce `RECV.log`, `SENT.log`, and `TEST.log` files for debugging purposes:
- `RECV.log` contains messages received, concatenated
- `SENT.log` contains messages sent, concatenated
- `TEST.log` contains debugging information to identify issues
Debugging with `#DEBUG` incurs significant run-time overhead and should only be used for debugging purposes.

@par Example:

    c++ -D DEBUG -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    ./client
    ls *.log
    RECV.log SENT.log TEST.log

@see `#SOAP_DEBUG`, `#DEBUG_STAMP`, `#SOAP_MEM_DEBUG`, `::soap_set_recv_logfile`, `::soap_set_sent_logfile`, `::soap_set_test_logfile` and the message logging plugin <i>`gsoap/plugin/logging.c`</i> as a faster alternative.

@note The libgsoap, libgsoap++, libgsoapssl and libgsoapssl++ libraries should not be used when a project is (re)compiled with `#DEBUG`, because these libraries are built by default without debugging enabled.  Use `./configure --enable-debug` to rebuild the libraries with `-D DEBUG`.
*/
#define DEBUG

/// User-definable macro, identical behavior as `#DEBUG` but more portable
/**
This macro should be used when the `#DEBUG` macro is reserved by the IDE for other purposes.

@see `#DEBUG`, `#DEBUG_STAMP`, `#SOAP_MEM_DEBUG`, `::soap_set_recv_logfile`, `::soap_set_sent_logfile`, `::soap_set_test_logfile`.
*/
#define SOAP_DEBUG

/// User-definable macro to enable debugging and logging with time stamps
/**
When this macro is defined at compile time (undefined by default), the engine runs in debug mode to produce `RECV.log`, `SENT.log`, and `TEST.log` files with time stamps for debugging purposes:
- `RECV.log` contains messages received, concatenated
- `SENT.log` contains messages sent, concatenated
- `TEST.log` contains debugging information with time stamps to identify issues
This incurs significant run-time overhead and should only be used for debugging purposes.

@par Example:

    c++ -D DEBUG_STAMP -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    ./client
    ls *.log
    RECV.log SENT.log TEST.log

@see `#DEBUG`, `#SOAP_DEBUG`, `#SOAP_MEM_DEBUG`, `::soap_set_recv_logfile`, `::soap_set_sent_logfile`, `::soap_set_test_logfile` and the message logging plugin <i>`gsoap/plugin/logging.c`</i> as a faster alternative.

@note The libgsoap, libgsoap++, libgsoapssl and libgsoapssl++ libraries should not be used when a project is (re)compiled with `#DEBUG_STAMP`, because these libraries are built by default without debugging enabled.
*/
#define DEBUG_STAMP

/// User-definable macro to enable memory debugging without logging
/**
When this macro is defined at compile time (undefined by default), the engine runs in debug mode to detect memory corruption errors but does not produce `RECV.log`, `SENT.log` and `TEST.log` files and avoids the significant run-time overhead of logging.  Use this macro when memory debugging is required without logging overhead.

@par Example:

    c++ -D SOAP_MEM_DEBUG -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    ./client
    ls *.log
    ls: No match.

@see `#DEBUG`, `#SOAP_DEBUG`, `#DEBUG_STAMP`.

@note The libgsoap, libgsoap++, libgsoapssl and libgsoapssl++ libraries should not be used when a project is (re)compiled with `#SOAP_MEM_DEBUG`, because these libraries are built by default without debugging enabled.
*/
#define SOAP_MEM_DEBUG

/// Specify a file name to save messages received
/**
This function sets the specified file path name `logfile` to save all messages received.  Messages are appended to the specified file.  Disables logging when `logfile` is NULL.  This function requires compilation with `#DEBUG`.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
soap_set_recv_logfile(soap, "logs/thread_1_recv.log");
soap_set_sent_logfile(soap, "logs/thread_1_sent.log");
soap_set_test_logfile(soap, "logs/thread_1_test.log");
~~~

@note Requires compilation with `#DEBUG`.

@see `::soap_set_sent_logfile` and the message logging plugin <i>`gsoap/plugin/logging.c`</i> as a faster alternative.
*/
void soap_set_recv_logfile(
    struct soap *soap,   ///< `::soap` context
    const char *logfile) ///< path name of the log file or NULL to disable logging
  ;

/// Specify a file name to save messages sent
/**
This function sets the specified file path name `logfile` to save the messages sent.  Messages are appended to the specified file.  Disables logging when `logfile` is NULL.  This function requires compilation with `#DEBUG`.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
soap_set_recv_logfile(soap, "logs/thread_1_recv.log");
soap_set_sent_logfile(soap, "logs/thread_1_sent.log");
soap_set_test_logfile(soap, "logs/thread_1_test.log");
~~~

@note Requires compilation with `#DEBUG`.

@see `::soap_set_recv_logfile` and the message logging plugin <i>`gsoap/plugin/logging.c`</i> as a faster alternative.
*/
void soap_set_sent_logfile(
    struct soap *soap,   ///< `::soap` context
    const char *logfile) ///< path name of the log file or NULL to disable logging
  ;

/// Specify a file name to save debugging info
/**
This function sets the specified file path name `logfile` to save debugging info generated by the engine and by the generated code. Disables logging when `logfile` is NULL.  This function requires compilation with `#DEBUG`.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
soap_set_recv_logfile(soap, "logs/thread_1_recv.log");
soap_set_sent_logfile(soap, "logs/thread_1_sent.log");
soap_set_test_logfile(soap, "logs/thread_1_test.log");
~~~

@note Requires compilation with `#DEBUG`.
*/
void soap_set_test_logfile(
    struct soap *soap,   ///< `::soap` context
    const char *logfile) ///< path name of the log file or NULL to disable logging
  ;

/// Specify inbound message logging with the `::logging` plugin
/**
This function enables inbound message logging.  Inbound messages are recorded to the specified file descriptor.  Logging is disabled by passing a NULL file descriptor parameter.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/logging.h"

struct soap *soap = soap_new();
FILE *fd = fopen("logs/recv.log", "w");
soap_register_plugin(soap, logging);
soap_set_logging_inbound(soap, fd);
... // send and receive messages
soap_set_logging_inbound(soap, NULL);
fclose(fd);
~~~

@note This function is declared and defined in <i>`gsoap/plugin/logging.h`</i> and <i>`gsoap/plugin/logging.c`</i> and requires the `::logging` plugin and does not require `#DEBUG`.

@see `::soap_set_logging_outbound`, `::soap_logging_stats`, `::soap_reset_logging_stats`.
*/
void soap_set_logging_inbound(
    struct soap *soap, ///< `::soap` context
    FILE *fd)          ///< file descriptor to record inbound messages
  ;

/// Specify outbound message logging with the `::logging` plugin
/**
This function enables outbound message logging.  Outbound messages are recorded to the specified file descriptor.  Logging is disabled by passing a NULL file descriptor parameter.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/logging.h"

struct soap *soap = soap_new();
FILE *fd = fopen("logs/sent.log", "w");
soap_register_plugin(soap, logging);
soap_set_logging_outbound(soap, fd);
... // send and receive messages
soap_set_logging_outbound(soap, NULL);
fclose(fd);
~~~

@note This function is declared and defined in <i>`gsoap/plugin/logging.h`</i> and <i>`gsoap/plugin/logging.c`</i> and requires the `::logging` plugin and does not require `#DEBUG`.

@see `::soap_set_logging_inbound`, `::soap_logging_stats`, `::soap_reset_logging_stats`.
*/
void soap_set_logging_outbound(
    struct soap *soap, ///< `::soap` context
    FILE *fd)          ///< file descriptor to record outbound messages
  ;

/// Collect messaging statistics with the `::logging` plugin
/**
This function collects the recorded messaging statistics, namely the number of bytes received from inbound messages and the number of bytes sent to outbound messages.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/logging.h"

struct soap *soap = soap_new();
soap_register_plugin(soap, logging);
... // send and receive messages
size_t sent, recv;
soap_logging_stats(soap, &sent, &recv);
printf("Bytes sent = %zu bytes received = %zu\n", sent, recv);
soap_reset_logging_stats(soap);
~~~

@note This function is declared and defined in <i>`gsoap/plugin/logging.h`</i> and <i>`gsoap/plugin/logging.c`</i> and requires the `::logging` plugin and does not require `#DEBUG`.

@see `::soap_set_logging_inbound`, `::soap_set_logging_outbound`, `::soap_reset_logging_stats`.
*/
void soap_logging_stats(
    struct soap *soap, ///< `::soap` context
    size_t *sent,      ///< pointer to variable to assign
    size_t *recv)      ///< pointer to variable to assign
  ;

/// Reset messaging statistics with the `::logging` plugin
/**
This function resets the recorded messaging statistics.

@note This function is declared and defined in <i>`gsoap/plugin/logging.h`</i> and <i>`gsoap/plugin/logging.c`</i> and requires the `::logging` plugin and does not require `#DEBUG`.

@see `::soap_set_logging_inbound`, `::soap_set_logging_outbound`, `::soap_logging_stats`.
*/
void soap_reset_logging_stats(struct soap *soap) ///< `::soap` context
  ;

/// The logging plugin registration function
/**
The logging plugin API is declared and defined in <i>`gsoap/plugin/logging.h`</i> and <i>`gsoap/plugin/logging.c`</i>.

@see `::soap_set_logging_inbound`, `::soap_set_logging_outbound`, `::soap_logging_stats`, `::soap_reset_logging_stats`.
*/
int logging(struct soap*, struct soap_plugin*, void*);

/** @} */

/**
\defgroup group_with WITH_MACRO compile-time flags
@brief This module defines the `WITH_MACRO` compile-time flags to configure the engine build

This module defines the following compile-time flags to configure the engine build:
- `#SOAPDEFS_H`
- `#WITH_SOAPDEFS_H`
- `#WITH_COMPAT`
- `#WITH_LEAN`
- `#WITH_LEANER`
- `#WITH_FAST`
- `#WITH_COOKIES`
- `#WITH_INSECURE_COOKIES`
- `#WITH_IPV6`
- `#WITH_IPV6_V6ONLY`
- `#WITH_OPENSSL`
- `#WITH_GNUTLS`
- `#WITH_GZIP`
- `#WITH_ZLIB`
- `#WITH_NTLM`
- `#WITH_C_LOCALE`
- `#WITH_NO_C_LOCALE`
- `#WITH_INCLUDE_XLOCALE_H`
- `#WITH_DOM`
- `#WITH_REPLACE_ILLEGAL_UTF8`
- `#WITH_FASTCGI`
- `#WITH_NOIO`
- `#WITH_NOIDREF`
- `#WITH_NOHTTP`
- `#WITH_NOZONE`
- `#WITH_NOEMPTYNAMESPACES`
- `#WITH_NOEMPTYSTRUCT`
- `#WITH_NOGLOBAL`
- `#WITH_NONAMESPACES`
- `#WITH_CDATA`
- `#WITH_PURE_VIRTUAL`
- `#WITH_DEFAULT_VIRTUAL`
- `#WITH_CASEINSENSITIVETAGS`
- `#WITH_SOCKET_CLOSE_ON_EXIT`
- `#WITH_TCPFIN`
- `#WITH_SELF_PIPE`
- `#WITH_CRTOLF`

@{
*/

/// When this macro is defined at compile time (undefined by default) then the header file specified by this macro is included in the build via stdsoap2.h
/**
This macro specifies the name of a header file to include and can be used to configure the engine build by defining \ref group_with and \ref group_soap.

@par Example:

    cat mydefs.h
    #define WITH_OPENSSL
    #define WITH_DOM
    c++ -D SOAPDEFS_H=mydefs.h -o client stdsoap2.cpp dom.cpp soapC.cpp soapClient.cpp client.cpp -lcrypto -lssl

@see `#WITH_SOAPDEFS_H`.
*/
#define SOAPDEFS_H

/// When this macro is defined at compile time (undefined by default), a user-supplied file named soapdefs.h is included in the build, i.e. soapdefs.h can be used to configure the build by defining compile-time macros and flags
/**
@par Example:

    cat soapdefs.h
    #define WITH_OPENSSL
    #define WITH_DOM
    c++ -D WITH_SOAPDEFS_H -o client stdsoap2.cpp dom.cpp soapC.cpp soapClient.cpp client.cpp -lcrypto -lssl

Alternatively, set `#SOAPDEFS_H` to the header file name to include in your build.  For example:

    cat mydefs.h
    #define WITH_OPENSSL
    #define WITH_DOM
    c++ -D SOAPDEFS_H=mydefs.h -o client stdsoap2.cpp dom.cpp soapC.cpp soapClient.cpp client.cpp -lcrypto -lssl
*/
#define WITH_SOAPDEFS_H

/// When this macro is defined at compile time (undefined by default), removes dependency on C++ `std::string` and `std::iostream` libraries
/**
@par Example:

    c++ -D WITH_COMPAT -c stdsoap2.cpp
*/
#define WITH_COMPAT

/// When this macro is defined at compile time (undefined by default), memory footprint is significantly reduced by removing non-essential features from the engine and from the generated code
/**
@warning `#WITH_LEAN` disables the following features:
- UDP `#SOAP_IO_UDP`
- HTTP keep-alive `#SOAP_IO_KEEPALIVE`
- HTTP authentication with `::soap::userid` and `::soap::passwd`
- HTTP chunked transfers `#SOAP_IO_CHUNK` except HTTP chunked input
- HTTP compression `#SOAP_ENC_ZLIB` except HTTP compressed input
- canonical XML `#SOAP_XML_CANONICAL`, `#SOAP_XML_CANONICAL_NA`
- timeouts `::soap::connect_timeout`, `::soap::send_timeout`, `::soap::recv_timeout` and `::soap::transfer_timeout`
- socket flags `::soap::socket_flags`, `::soap::connect_flags`, `::soap::bind_flags`, `::soap::accept_flags`
- `time_t` serialization as `xsd__dateTime` (use a string to store the date and time instead)
- `::soap_poll` always returns `#SOAP_OK`

@warning When this macro is define at the compiler's command line or in an IDE or by specifying `#SOAPDEFS_H`, then all source code files of an application's project must be recompiled.  Otherwise, `::soap` context corruption errors may occur in the non-recompiled parts of the application, because the size of the `::soap` context has changed.

@par Examples:

    c++ -D WITH_LEAN -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    c++ -D WITH_LEAN -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp
*/
#define WITH_LEAN

/// When this macro is defined at compile time (undefined by default), memory footprint is further reduced from `#WITH_LEAN` by removing non-essential features from the engine and from the generated code
/**
@warning `#WITH_LEANER` disables the following features:
- UDP `#SOAP_IO_UDP`
- HTTP keep-alive `#SOAP_IO_KEEPALIVE`
- HTTP authentication with `::soap::userid` and `::soap::passwd`
- HTTP chunked transfers `#SOAP_IO_CHUNK` except HTTP chunked input
- HTTP compression `#SOAP_ENC_ZLIB` except HTTP compressed input
- canonical XML `#SOAP_XML_CANONICAL`, `#SOAP_XML_CANONICAL_NA`
- timeouts `::soap::connect_timeout`, `::soap::send_timeout`, `::soap::recv_timeout` and `::soap::transfer_timeout`
- socket flags `::soap::socket_flags`, `::soap::connect_flags`, `::soap::bind_flags`, `::soap::accept_flags`
- `time_t` serialization as `xsd__dateTime` (use a string to store the date and time instead)
- `::soap_poll` always returns `#SOAP_OK`
- wide string serializers
- DIME and MIME/MTOM attachments
- XML DOM operations with the DOM API

@warning When this macro is define at the compiler's command line or in an IDE or by specifying `#SOAPDEFS_H`, then all source code files of an application's project must be recompiled.  Otherwise, `::soap` context corruption errors may occur in the non-recompiled parts of the application, because the size of the `::soap` context has changed.

@par Examples:

    c++ -D WITH_LEANER -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    c++ -D WITH_LEANER -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp
*/
#define WITH_LEANER

/// When this macro is defined at compile time uses faster memory allocation at a cost of larger memory blocks allocated (defined by default except with `#WITH_LEAN` and `#WITH_LEANER)`
/**
@par Examples:

    c++ -D WITH_FAST -D WITH_LEANER -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    c++ -D WITH_FAST -D WITH_LEAN -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp

*/
#define WITH_FAST

/// When this macro is defined at compile time (undefined by default), HTTP cookie support is enabled in the engine
/**
Use the following functions to set and get HTTP cookie values:
- `::soap_set_cookie`
- `::soap_set_cookie_expire`
- `::soap_set_cookie_secure`
- `::soap_set_cookie_session`
- `::soap_clr_cookie`
- `::soap_clr_cookie_session`
- `::soap_cookie`
- `::soap_cookie_value`
- `::soap_cookie_expire`
- `::soap_getenv_cookies`
- `::soap_free_cookies`

Client-side HTTP cookie handling is automatic.  The `#WITH_COOKIES` flag is useless without server-side session management and control.  Cookies may require a fair amount of processing overhead and are not needed to implement stateful services, which is typically performed with session IDs in XML/JSON messages or via the URL.

@par Examples:

    c++ -D WITH_COOKIES -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    c++ -D WITH_COOKIES -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp

@see The [HTTP Server Session Management plugin](../../sessions/html/index.html) documentation to use the HTTP session plugin to store a database of sessions to keep track of client data across multiple requests.
*/
#define WITH_COOKIES

/// When this macro is defined at compile time (undefined by default), HTTP cookie support is enabled (as `#WITH_COOKIES`) but allows cookies with their Secure flag set to be sent over insecure channels
/**
Use the following functions to set and get HTTP cookie values:
- `::soap_set_cookie`
- `::soap_set_cookie_expire`
- `::soap_set_cookie_secure`
- `::soap_set_cookie_session`
- `::soap_clr_cookie`
- `::soap_clr_cookie_session`
- `::soap_cookie`
- `::soap_cookie_value`
- `::soap_cookie_expire`
- `::soap_getenv_cookies`
- `::soap_free_cookies`

@par Examples:

    c++ -D WITH_INSECURE_COOKIES -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    c++ -D WITH_INSECURE_COOKIES -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp

@see The [HTTP Server Session Management plugin](../../sessions/html/index.html) documentation to use the HTTP session plugin to store a database of sessions to keep track of client data across multiple requests.
*/
#define WITH_INSECURE_COOKIES

/// When this macro is defined at compile time (undefined by default), IPv6 support is enabled and both IPv4 and IPv6 connections are supported by the engine
/**
Use `::soap::bind_inet6` and `::soap::bind_v6only` at runtime to configure port bindings at the server side:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
soap->bind_inet6 = 1;  // to use AF_INET6 instead of PF_UNSPEC
soap->bind_v6only = 1; // to setsockopt IPPROTO_IPV6 IPV6_V6ONLY
if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  ...
~~~

@par Example:

    c++ -D WITH_IPV6 -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp

*/
#define WITH_IPV6

/// When this macro is defined at compile time (undefined by default), IPv6-only is enabled for port binding at the server side by the engine to ensure that a socket will only use IPv6 without mapping IPv4 to IPv6
/**
Alternatively, compile with `#WITH_IPV6` and set `::soap::bind_v6only` at runtime to do the same as `#WITH_IPV6_V6ONLY`:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
soap->bind_inet6 = 1;  // to use AF_INET6 instead of PF_UNSPEC
soap->bind_v6only = 1; // to setsockopt IPPROTO_IPV6 IPV6_V6ONLY
if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  ...
~~~

@par Example:

    c++ -D WITH_IPV6_ONLY -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp
*/
#define WITH_IPV6_V6ONLY

/// When this macro is defined at compile time (undefined by default), enables linkage with OpenSSL for HTTPS and WS-Security (this macro should also be defined when using plugins that rely on OpenSSL)
/**
Use the following functions to configure SSL/TLS connections and to accept SSL/TLS connections:
- `::soap_ssl_init`
- `::soap_ssl_noinit`
- `::soap_ssl_client_context`
- `::soap_ssl_server_context`
- `::soap_ssl_crl`
- `::soap_ssl_accept`

@par Examples:

    c++ -D WITH_OPENSSL -D WITH_DOM -o client stdsoap2.cpp dom.cpp soapC.cpp soapClient.cpp client.cpp plugin/smdevp.c plugin/mecevp.c plugin/wsseapi.c plugin/threads.c -lcrypto -lssl
    c++ -D WITH_OPENSSL -D WITH_DOM -o server stdsoap2.cpp dom.cpp soapC.cpp soapServer.cpp server.cpp plugin/smdevp.c plugin/mecevp.c plugin/wsseapi.c plugin/threads.c -lcrypto -lssl

*/
#define WITH_OPENSSL

/// When this macro is defined at compile time (undefined by default), enables linking the GNUTLS library to enable HTTPS in the engine
/**
Use the following functions to configure SSL/TLS connections and to accept SSL/TLS connections:
- `::soap_ssl_init`
- `::soap_ssl_noinit`
- `::soap_ssl_client_context`
- `::soap_ssl_server_context`
- `::soap_ssl_crl`
- `::soap_ssl_accept`

@par Examples:

    c++ -D WITH_GNUTLS -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp -lgnutls
    c++ -D WITH_GNUTLS -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp -lgnutls
*/
#define WITH_GNUTLS

/// When this macro is defined at compile time (undefined by default), enables linkage with the zlib library for HTTP message compression (using compress and gzip methods) when the `#SOAP_ENC_ZLIB` mode flag is enabled at runtime
/**
To enable compression with the gzip method and decompression with the compress and gzip methods, use `#SOAP_ENC_ZLIB`:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_ENC_ZLIB);
~~~

@par Examples:

    c++ -D WITH_GZIP -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp -lz
    c++ -D WITH_GZIP -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp -lz

@see `#WITH_ZLIB` to only enable compress but not gzip compression.
*/
#define WITH_GZIP

/// When this macro is defined at compile time (undefined by default), enables linking the zlib library for HTTP message compression when the `#SOAP_ENC_ZLIB` mode flag is enabled at runtime
/**
To enable (de)compression with the compress method use `#SOAP_ENC_ZLIB`:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_ENC_ZLIB);
~~~

@par Examples:

    c++ -D WITH_ZLIB -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp -lz
    c++ -D WITH_ZLIB -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp -lz

@see `#WITH_GZIP` to also enable the gzip compression method in addition to compress.
*/
#define WITH_ZLIB

/// When this macro is defined at compile time (undefined by default), enables linkage with the ntlm library for HTTP NTLM authentication
/**
The libntlm library is available at http://www.nongnu.org/libntlm and required for non-Windows platforms.

@par Examples:

    c++ -D WITH_NTLM -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp -lntlm
    c++ -D WITH_NTLM -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp -lntlm

@see `::soap::ntlm_challenge`.
*/
#define WITH_NTLM

/// When this macro is defined at compile time (defined by default for most platforms), the C locale is enabled in the engine, opposite of `#WITH_NO_C_LOCALE`
/**
This macro prevents floating point values from getting garbled by the standard locale-dependent `strtod` and `sprintf` functions used by the engine, for example when the current locale of the machine redefines the decimal point.
*/
#define WITH_C_LOCALE

/// When this macro is defined at compile time (undefined by default), the use of the C locale is disabled in the engine, the opposite of `#WITH_C_LOCALE`
/**
Defining this macro improves portability of the engine's source code on older platforms that do not have working versions of locale.h and xlocale.h.

@warning When the current locale of the machine redefines the decimal point, floating point values may be rejected by the XML validator and may be garbled in XML and JSON output due to the standard locale-dependent `strtod` and `sprintf` functions.

@par Example:

    c++ -D WITH_NO_C_LOCALE -c stdsoap2.cpp
*/
#define WITH_NO_C_LOCALE

/// When this macro is defined at compile time (defined by default for most platforms), include xlocale.h to define `locale_t` and the `_l` locale-specific functions
#define WITH_INCLUDE_XLOCALE_H

/// When this macro is defined at compile time (undefined by default), enables WS-Security signature verification and XML re-canonicalization, i.e. this macro must be defined when using the gSOAP WSSE plugin for WS-Security
/**
To enable DOM parsing use `#SOAP_XML_DOM`, for example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_DOM);
... // call an XML service operation that deserializes the response
if (soap->dom) // we also got a DOM
{
  const struct soap_dom_element *elt = soap->dom;
  ... // inspect the DOM using the DOM API functions
}
~~~

@par Examples:

    c++ -D WITH_OPENSSL -D WITH_DOM -o client stdsoap2.cpp dom.cpp soapC.cpp soapClient.cpp client.cpp plugin/smdevp.c plugin/mecevp.c plugin/wsseapi.c plugin/threads.c -lcrypto -lssl
    c++ -D WITH_OPENSSL -D WITH_DOM -o server stdsoap2.cpp dom.cpp soapC.cpp soapServer.cpp server.cpp plugin/smdevp.c plugin/mecevp.c plugin/wsseapi.c plugin/threads.c -lcrypto -lssl

@see `#SOAP_XML_DOM` and the gSOAP [XML DOM API documentation](../../dom/html/index.html).
*/
#define WITH_DOM

/// When this macro is defined at compile time (undefined by default), replaces UTF-8 content that is outside the Unicode range with the value of `#SOAP_UNKNOWN_UNICODE_CHAR` (Unicode FFFD) to indicate illegal UTF-8, set `#SOAP_UNKNOWN_UNICODE_CHAR` to another integer code if desired (the value is 0xFFFD by default)
/**
@warning Without this macro the engine silently accepts UTF-8 content that is outside the Unicode character ranges U+0000 to U+D7FF, U+E000 to U+FFFD, U+10000 to U+10FFFF.

@par Example:

    c++ -D WITH_REPLACE_ILLEGAL_UTF8 -c stdsoap2.cpp

@see `#SOAP_UNKNOWN_UNICODE_CHAR`.
*/
#define WITH_REPLACE_ILLEGAL_UTF8

/// When this macro is defined at compile time (undefined by default), enables and configures the engine and generated code to use FastCGI at the server side
/**
Call `::soap_serve` (or the C++ service class `serve` method) to serve CGI and FastCGI requests (only FastCGI requires `#WITH_FASTCGI` but CGI works without this configuration).

@par Example:

    soapcpp2 myservice.h
    c++ -D WITH_FASTCGI -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp
~~~{.cpp}
// example server.cpp
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap_serve(soap);
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
}
... // web method implementations
~~~
*/
#define WITH_FASTCGI

/// When this macro is defined at compile time (undefined by default), removes all IO and socket library calls to replace these with user-supplied callback functions, see the IO callbacks in the user guide
/**
The following IO and socket-related functions should be defined by callbacks implemented by the user as follows:

~~~{.cpp}
#include "soapH.h"

int my_accept(struct soap *soap, SOAP_SOCKET sock, struct sockaddr *addr, int *len) { ... }
int my_connect(struct soap *soap, const char *endpoint, const char *host, int port) { ... }
int my_disconnect(struct soap *soap) { ... }
int my_closesocket(struct soap *soap, SOAP_SOCKET sock) { ... }
int my_shutdownsocket(struct soap *soap, SOAP_SOCKET sock, int how) { ... }
int my_send(struct soap *soap, const char *data, size_t len) { ... }
size_t my_recv(struct soap *soap, char *buf, size_t len) { ... }
int my_poll(struct soap *soap) { ... }

struct soap *soap = soap_new();
soap->faccept = my_accept;
soap->fopen = my_connect;
soap->fclose = my_disconnect;
soap->fclosesocket = my_close;
soap->fshutdownsocket = my_shutdown;
soap->fsend = my_send;
soap->frecv = my_recv;
soap->fpoll = my_poll;
~~~

The socket-related callbacks can be omitted when only basic IO operations are needed to read/write XML with the `::soap::frecv` and `::soap::fsend` callbacks.

@warning When this macro is define at the compiler's command line or in an IDE or by specifying `#SOAPDEFS_H`, then all source code files of an application's project must be recompiled.  Otherwise, `::soap` context corruption errors may occur in the non-recompiled parts of the application, because the size of the `::soap` context has changed.

@par Examples:

    c++ -D WITH_NOIO -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    c++ -D WITH_NOIO -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp

@see `::soap::faccept`, `::soap::fopen`, `::soap::fclose`, `::soap::fclosesocket`, `::soap::fshutdownsocket`, `::soap::fsend`, `::soap::frecv`, `::soap::fpoll`.
*/
#define WITH_NOIO

/// When this macro is defined at compile time (undefined by default), permanently disables XML ID (`id` attribute) and REF (`href` or `ref` attribute) processing rules associated with SOAP 1.1 and SOAP 1.2 "multi-reference" data to serialize object graphs, this is a more aggressive optimization option than the runtime `#SOAP_XML_TREE` flag
/**
Alternatively, set `#SOAP_XML_TREE` at runtime:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_TREE);
~~~

@par Examples:

    c++ -D WITH_NOIDREF -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    c++ -D WITH_NOIDREF -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp

@see `#SOAP_XML_TREE`, `#SOAP_XML_GRAPH`.
*/
#define WITH_NOIDREF

/// When this macro is defined at compile time (undefined by default), permanently removes the HTTP stack to reduce code size
/**
@par Example:

    c++ -D WITH_NOHTTP -o xmlapp stdsoap2.cpp soapC.cpp xmlapp.cpp
*/
#define WITH_NOHTTP

/// When this macro is defined at compile time (undefined by default), removes and ignores the timezone in <i>`xsd:dateTime`</i> values
/**
@par Examples:

    c++ -D WITH_NOZONE -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    c++ -D WITH_NOZONE -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp
*/
#define WITH_NOZONE

/// When this macro is defined at compile time, default empty namespaces are not required to parse XML and are disabled from XML messages when the `#SOAP_XML_DEFAULTNS` mode flag is used
/**
This macro is intended for backward compatibility with old XML parsers and old gSOAP versions that do not support <i>`xmlns=""`</i> empty default namespaces.  When used with the runtime `#SOAP_XML_DEFAULTNS` mode flag, produces XML that lacks <i>`xmlns=""`</i> which should only be used for special cases and is not recommended in general.

@warning This macro should only be used to resolve issues with older tools that do not support <i>`xmlns=""`</i> in XML messages or when XML messages should be parsed and validated that do not include <i>`xmlns=""`</i> that are normally expected in valid XML.
*/
#define WITH_NOEMPTYNAMESPACES

/// When this macro is defined at compile time (undefined by default), omits SOAP Header and Fault serialization code, this flag prevents duplicate definitions in the source code generated by soapcpp2
/// When this macro is defined at compile time (defined for C but undefined for C++), the generated code by the soapcpp2 tool inserts a dummy member in empty structs to allow compilation to succeed with compilers that reject empty structs and unions
#define WITH_NOEMPTYSTRUCT

/// When this macro is defined at compile time (undefined by default), omits SOAP Header and Fault serialization code, this flag prevents duplicate definitions in the source code generated by soapcpp2
/**
This macro is used in the generated files when <b>`soapcpp2 -pname`</b> option <b>`-pname`</b> is used.
*/
#define WITH_NOGLOBAL

/// When this macro is defined at compile time (undefined by default), removes dependence on the global `::namespaces` table, rather users must set the XML namespaces explicitly by calling `::soap_set_namespaces`
/**
Applications developed with soapcpp2 options `-i` or `-j` assign a namespace table automatically to the `::soap` context when the proxy or service object is instantiated.  Other applications must set the namespace table explicitly, before processing XML, for example:

~~~{.cpp}
#include "soapH.h"

struct Namespace my_namespaces[] = {
  { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/", "http://www.w3.org/*soap-envelope",      NULL },
  { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/", "http://www.w3.org/*soap-encoding",      NULL },
  { "xsi",      "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*XMLSchema-instance", NULL },
  { "xsd",      "http://www.w3.org/2001/XMLSchema",          "http://www.w3.org/*XMLSchema",          NULL },
  { "ns",       "http://tempuri.org/ns.xsd",                 NULL,                                    NULL },
  { NULL,       NULL,                                        NULL,                                    NULL }
};

struct soap *soap = soap_new();
soap_set_namespaces(soap, my_namespaces); // use a specific XML namespace binding table for the next call
if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
{
  soap_print_fault(soap, stderr);
}
else
{
  ...
}
soap_set_namespaces(soap, namespaces); // use the default XML namespace binding table, when defined, for the next call
~~~

This example defines SOAP 1.1 namespaces (`SOAP-ENV` and `SOAP-ENC`) to be used by default, but also accepts SOAP 1.2 because of the second URI in the third column.  XML schema instance namespace `xsi` is used with <i>`xsi:type`</i> and <i>`xsi:nil`</i> and the XML schema namespace `xsd` is used with XSD types such as <i>`xsd:string`</i>, which may be used in XML messages.  URI patterns in the third column may contain wildcard strings `*` and wildcard characters `-`.

@par Examples:

    c++ -D WITH_NONAMESPACES -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
    c++ -D WITH_NONAMESPACES -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp
*/
#define WITH_NONAMESPACES

/// When this macro is defined at compile time (undefined by default), retains the parsed CDATA sections in literal XML strings
/**
Literal XML strings are built-in <tt>\ref _XML</tt> types (a typedef of `char*`) in the .h file for soapcpp2, or a user-defined `typedef std::string XML` to define a `XML` literal string.  Literal XML strings contain XML, but CDATA sections are stripped by the XML parser unless this macro is enabled.

@par Example:

    soapcpp2 -C myservice.h
    c++ -D WITH_CDATA -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
~~~{.cpp}
// example myservice.h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
int ns__webmethod(_XML string, _XML *response); // _XML is a literal XML string (a char* type)
~~~
~~~{.cpp}
// example client.cpp file
#include "soapH.h"
#include "example.nsmap"

struct soap *soap = soap_new1(SOAP_XML_INDENT);
if (soap_call_ns__webmethod(soap, endpoint, NULL, "<doc title=\"Example\">Some text</doc>", &r))
  soap_print_fault(soap, stderr);
else
  printf("XML = %s\n", response);
soap_destroy(soap);
soap_end(soap);
soap_free(soap);
~~~
Request and response messages:
~~~{.xml}
<?xml version="1.0" encoding="UTF-8"?>
<SOAP-ENV:Envelope
    xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
    xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xmlns:xsd="http://www.w3.org/2001/XMLSchema"
    xmlns:ns="urn:example">
 <SOAP-ENV:Body>
  <ns:webmethod>
    <doc title="Example">Some text</doc>
  </ns:webmethod>
 </SOAP-ENV:Body>
</SOAP-ENV:Envelope>
~~~
~~~{.xml}
<?xml version="1.0" encoding="UTF-8"?>
<SOAP-ENV:Envelope
    xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
    xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xmlns:xsd="http://www.w3.org/2001/XMLSchema"
    xmlns:ns="urn:example">
 <SOAP-ENV:Body>
  <ns:webmethodResponse>
   <![CDATA[<doc title="Example">Some text</doc>]]>
  </ns:webmethodResponse>
 </SOAP-ENV:Body>
</SOAP-ENV:Envelope>
~~~
Output with `#WITH_CDATA`:

    XML = <![CDATA[<doc title="Example">Some text</doc>]]>
Output without `#WITH_CDATA`:

    XML = <doc title="Example">Some text</doc>
*/
#define WITH_CDATA

/// When this macro is defined at compile time (undefined by default), makes C++ service class operation methods and the copy method pure virtual by defining `#SOAP_PURE_VIRTUAL` = 0
/**
Alternatively, use `#WITH_DEFAULT_VIRTUAL` to make C++ service class operation methods return `#SOAP_NO_METHOD` for rapid Web API prototyping and development.

@note Requires <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b> to generate C++ service classes.

@par Example:

    soapcpp2 -j myservice.h
    c++ -D WITH_PURE_VIRTUAL -o server stdsoap2.cpp soapC.cpp soapexampleService.cpp server.cpp
~~~{.cpp}
// example myservice.h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
struct ns__webmethodResponse {
  char *string;
};
int ns__webmethod(char *string, struct ns__webmethodResponse *response);
~~~
~~~{.cpp}
// example server.cpp file
#include "soapexampleService.h"
#include "example.nsmap"

class MyService : public exampleService {
  public:
    virtual int webmethod(char *string, struct ns__webmethodResponse *response)
    {
      response->string = string;
      return SOAP_OK;
    }
    virtual MyService *copy()
    {
      return new MyService(); // new instance, nothing shared or copied
    }
};
int main()
{
  return MyService().run(PORTNUM);
}
~~~

@see `#WITH_DEFAULT_VIRTUAL`.
*/
#define WITH_PURE_VIRTUAL

/// When this macro is defined at compile time (undefined by default), enables C++ base service classes with default virtual methods returning fault `#SOAP_NO_METHOD`
/**
Use `#WITH_DEFAULT_VIRTUAL` to make C++ service class operation methods return `#SOAP_NO_METHOD` for rapid Web API prototyping and development.

@note Requires <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b> to generate C++ service classes.

@par Example:

    soapcpp2 -j myservice.h
    c++ -D WITH_DEFAULT_VIRTUAL -o server stdsoap2.cpp soapC.cpp soapMyService.cpp server.cpp
~~~{.cpp}
// example myservice.h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
struct ns__webmethodResponse {
  char *string;
};
int ns__webmethod(char *string, struct ns__webmethodResponse *response);
~~~
~~~{.cpp}
// example server.cpp file
#include "soapexampleService.h"
#include "example.nsmap"

class MyService : public exampleService {
  public:
    virtual int webmethod(char *string, struct ns__webmethodResponse *response)
    {
      response->string = string;
      return SOAP_OK;
    }
    // note: other methods (if any) will return SOAP_NO_METHOD
};
int main()
{
  return MyService().run(PORTNUM);
}
~~~
*/
#define WITH_DEFAULT_VIRTUAL

/// When this macro is defined at compile time (undefined by default), enables case insensitive XML tag name parsing and validation
/**
XML is case sensitive, whereas HTML is case insensitive.  XML parsing is always case insensitive unless overruled by thie macro.

@par Example:

    c++ -D WITH_CASEINSENSITIVETAGS -c stdsoap2.cpp
*/
#define WITH_CASEINSENSITIVETAGS

/// When this macro is defined at compile time (undefined by default), prevents a server port from staying in listening mode after exit by internally setting `fcntl(sock, F_SETFD, FD_CLOEXEC)`
/**
@par Example:

    c++ -D WITH_SOCKET_CLOSE_ON_EXIT -c stdsoap2.cpp
*/
#define WITH_SOCKET_CLOSE_ON_EXIT

/// When this macro is defined at compile time (undefined by default), the engine transmits TCP FIN using `shutdown(sock, SHUT_WR)` after all sends are completed and just before the socket closes
/**
@par Example:

    c++ -D WITH_TCPFIN -c stdsoap2.cpp
*/
#define WITH_TCPFIN

/// When this macro is defined at compile time (undefined by default), the engine is configured to enable `::soap_close_connection` using a "self-pipe"
/**
To use `::soap_close_connection` from another thread to terminate a thread that is blocked on socket IO requires `::soap::recv_timeout` and `::soap::send_timeout` set to nonzero timeout values.

@warning When this macro is define at the compiler's command line or in an IDE or by specifying `#SOAPDEFS_H`, then all source code files of an application's project must be recompiled.  Otherwise, `::soap` context corruption errors may occur in the non-recompiled parts of the application, because the size of the `::soap` context has changed.

@par Example:

    c++ -D WITH_SELF_PIPE -o server stdsoap2.cpp soapC.cpp soapServer.cpp server.cpp

@see `::soap_close_connection`.
*/
#define WITH_SELF_PIPE

/// When this macro is defined at compile time (undefined by default), the engine is configured to replace CRLF (carriage return `#xD` followed by line feed `#xA`) by LF and replace single CR by LF
/**
This macro affects C/C++ string serialization to XML.  If `char*`, `wchar_t*`, `std::string`, or `std::wstring` values assigned by an application contain CR, then this macro effectively removes all CR in the normalized XML output by replacing CRLF with LF and each plain CR with a LF.  This macro does not affect parsed and deserialized strings and does not affect <tt>\ref _XML</tt> literal string serialization to XML i.e. strings containing raw XML.

The XML 1.1. standard recommends replacing CRLF by LF and each single CR by LF in XML character data.  This flag is useful when C/C++ string values to serialize to XML may contain CR characters.

@par Example:

    c++ -D WITH_CRTOLF -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
~~~{.cpp}
// compiled with compile-time flag -D WITH_CRTOLF
#include "soapH.h"

struct soap *soap = soap_new();
struct ns__webmethodResponse result;
... //
if (soap_call_ns__webmethod(soap, endpoint, NULL, "Message\r\n", &result))
  soap_print_fault(soap, stderr);
else
  ... // success
~~~
In the example shown above, the message `Message\n` will be sent.
*/
#define WITH_CRTOLF

/** @} */

/**
\defgroup group_soap SOAP_MACRO compile-time values
@brief This module defines the `SOAP_MACRO` compile-time values to configure the engine build

This module defines the following macros with values to configure the engine build:
- `#SOAP_NOTHROW`
- `#SOAP_BUFLEN`
- `#SOAP_HDRLEN`
- `#SOAP_TAGLEN`
- `#SOAP_TMPLEN`
- `#SOAP_MAXALLOCSIZE`
- `#SOAP_MAXARRAYSIZE`
- `#SOAP_MAXDIMESIZE`
- `#SOAP_MAXEINTR`
- `#SOAP_MAXINFLATESIZE`
- `#SOAP_MAXKEEPALIVE`
- `#SOAP_MAXLENGTH`
- `#SOAP_MAXLEVEL`
- `#SOAP_MAXOCCURS`
- `#SOAP_MINDEFLATERATIO`
- `#SOAP_PURE_VIRTUAL`
- `#SOAP_SSL_RSA_BITS`
- `#SOAP_UNKNOWN_CHAR`
- `#SOAP_UNKNOWN_UNICODE_CHAR`
- `#SOAP_LONG_FORMAT`
- `#SOAP_ULONG_FORMAT`
- `#SOAP_SOCKET`
- `#SOAP_SOCKLEN_T`
- `#SOAP_INVALID_SOCKET`
- `#soap_valid_socket`

Integer and float type macros:
- `#LONG64`
- `#ULONG64`
- `#FLT_NAN`
- `#FLT_PINFTY`
- `#FLT_NINFTY`
- `#DBL_NAN`
- `#DBL_PINFTY`
- `#DBL_NINFTY`
- `#soap_isnan`
- `#soap_isinf`

Macros for heap allocation:
- `#SOAP_MALLOC`
- `#SOAP_FREE`
- `#SOAP_NEW`
- `#SOAP_NEW_ARRAY`
- `#SOAP_PLACEMENT_NEW`
- `#SOAP_DELETE`
- `#SOAP_DELETE_ARRAY`
- `#SOAP_MALLOC_UNMANAGED`
- `#SOAP_FREE_UNMANAGED`
- `#SOAP_NEW_UNMANAGED`
- `#SOAP_DELETE_UNMANAGED`

DLL and API export related macros:
- `#SOAP_STD_EXPORTS`
- `#SOAP_FMAC1`
- `#SOAP_FMAC2`
- `#SOAP_FMAC3`
- `#SOAP_FMAC4`
- `#SOAP_FMAC5`
- `#SOAP_FMAC6`
- `#SOAP_CMAC`
- `#SOAP_NMAC`

@{
*/

/// Macro expands to `(std::nothrow)` to prevent the deserializer's memory allocator from throwing an exception but return `#SOAP_EOM` instead, this macro is empty when `#WITH_COMPAT` is enabled or when `#WITH_LEAN` or `#WITH_LEANER` are enabled which means that exceptions may be thrown
/**
It is possible to enable C++ exceptions without detrimental effects by compiling the source code with `#SOAP_NOTHROW` set to an empty value and in that case C++ exception handlers should be used to catch `std::bad_alloc`.

By default, `::soap_new` and the soapcpp2-generated `soap_new_T` functions return NULL when allocation failed.

@warning `::soap_malloc` never throws `std::bad_alloc`, which means that parsing and deserialization of C++ data may not throw `std::bad_alloc` in some cases, but rather the functions called will return `#SOAP_EOM` and `::soap::error` is set to `#SOAP_EOM`.  `::soap_malloc` also returns `#SOAP_EOM` when `#SOAP_MAXALLOCSIZE` is exceeded.

@par Example:

    c++ -D SOAP_NOTHROW="" -o client stdsoap2.cpp soapC.cpp soapClient.cpp client.cpp
~~~{.cpp}
// compiled with compile-time flag -D SOAP_NOTHROW=""
#include "soapH.h"

try
{
  // soap_new() may cause a std::bad_alloc exception
  struct soap *soap = soap_new();
  // MyClass is declared in soapH.h generated by soapcpp2 from a .h file with MyClass
  MyClass *obj = soap_new_MyClass(soap);
  ... // use obj
}
catch (std::bad_alloc)
{
  ... // out of memory error
}
~~~
*/
#define SOAP_NOTHROW (std::nothrow)

/// User-definable size of the input and output message buffer `::soap::buf` (the value is 65536 by default)
/**
This macro defines the size of the input and output message buffer, which is 65536 by default unless the `#WITH_LEAN` or `#WITH_LEANER` compile-time flags are used and the buffer size is 2048.  The size of the buffer affects the performance of socket communications, with larger sizes above 8192 generally improving the performance at the cost of the extra memory required to store the larger `::soap` context.

A default value of 65535 was chosen for OpenVMS TCP/IP stacks that cannot handle 65536 bytes.  A default value of 2048 is used for WinCE platforms.  A default value of 32767 is used for Tandom NonStop platforms.

@warning When the value of this macro is assigned at the compiler's command line or in an IDE or by specifying `#SOAPDEFS_H`, then all source code files of an application's project must be recompiled.  Otherwise, `::soap` context corruption errors may occur in the non-recompiled parts of the application, because the size of the `::soap` context has changed.
*/
#define SOAP_BUFLEN (65536)

/// User-definable maximum length of HTTP headers (the value is 8192 by default)
/**
@warning When the value of this macro is assigned at the compiler's command line or in an IDE or by specifying `#SOAPDEFS_H`, then all source code files of an application's project must be recompiled.  Otherwise, `::soap` context corruption errors may occur in the non-recompiled parts of the application, because the size of the `::soap` context has changed.
*/
#define SOAP_HDRLEN (8192)

/// User-definable maximum length of XML tags and URLs (the value is 1024 by default)
/**
@warning When the value of this macro is assigned at the compiler's command line or in an IDE or by specifying `#SOAPDEFS_H`, then all source code files of an application's project must be recompiled.  Otherwise, `::soap` context corruption errors may occur in the non-recompiled parts of the application, because the size of the `::soap` context has changed.
*/
#define SOAP_TAGLEN (1024)

/// User-definable maximum length of temporary string values stored in `::soap::msgbuf` and `::soap::tmpbuf`, e.g. to hold short strings and brief error messages (1024 by default, must not be less than 1024)
/**
@warning When the value of this macro is assigned at the compiler's command line or in an IDE or by specifying `#SOAPDEFS_H`, then all source code files of an application's project must be recompiled.  Otherwise, `::soap` context corruption errors may occur in the non-recompiled parts of the application, because the size of the `::soap` context has changed.
*/
#define SOAP_TMPLEN (1024)

/// User-definable maximum size of a block of memory that `malloc` can allocate or 0 for no limit (the value is 0 by default)
#define SOAP_MAXALLOCSIZE (0)

/// User-definable macro to protect excessive SOAP array allocation requests by defining a maximum allocation threshold
/**
Deserialized SOAP arrays larger in size than `#SOAP_MAXARRAYSIZE` are not pre-allocated in memory, but deserialized on an element-by-element basis until XML validation contrains kick in (the value is 100000 by default)
*/
#define SOAP_MAXARRAYSIZE (100000)

/// User-definable maximum length of DIME attachments received (the value is 8 MB by default)
/**
DIME attachments sizes are limited to `#SOAP_MAXDIMESIZE`.  Increase this value to allow larger attachments or decrease when resources are limited, this limit is to deny senders to pre-allocate excessive memory at the receiver side without actually sending the whole message, i.e. this threshold protects against DIME-based memory resource usage attacks.
*/
#define SOAP_MAXDIMESIZE (8*1048576)

/// User-definable maximum number of EINTR interrupts to ignore while polling a socket for pending activity, each EINTR ignored may increase the I/O blocking time by at most one second (the value is 10 by default)
#define SOAP_MAXEINTR (10)

/// Trusted inflated content size (1 MB by default), larger content is subject to the `#SOAP_MINDEFLATERATIO` constraint, i.e. if `#SOAP_MINDEFLATERATIO` is 1.0, `#SOAP_MAXINFLATESIZE` is always the max size of uncompressed content
#define SOAP_MAXINFLATESIZE (1*1048576)

/// User-definable maximum iterations in the server-side `::soap_serve` loop (or the C++ service class `serve` method) on HTTP keep-alive connections, assigned to `::soap::max_keep_alive` which can be altered at runtime (the value is 100 by default)
#define SOAP_MAXKEEPALIVE (100)

/// User-definable maximum string content length for strings not already constrained by XML schema validation constraints, zero or negative means unlimited string lengths are allowed (the value is 0 by default)
#define SOAP_MAXLENGTH (0)

/// User-definable maximum XML nesting depth level permitted by the XML parser, must be greater than zero (the value is 10000 by default)
#define SOAP_MAXLEVEL (10000)

/// User-definable maximum number of array or container elements for containers that are not already constrained by XML schema validation constraints, must be greater than zero (the value is 100000 by default)
#define SOAP_MAXOCCURS (100000)

/// Trusted deflation ratio after `#SOAP_MAXINFLATESIZE` is reached, trust when compressed / deflated > `#SOAP_MINDEFLATERATIO` (default is 0.00096899224806 or 1032:1, which is according to the zlib site: "The limit (1032:1) comes from the fact that one length/distance pair can represent at most 258 output bytes. A length requires at least one bit and a distance requires at least one bit, so two bits in can give 258 bytes out, or eight bits in give 1032 bytes out. A dynamic block has no length restriction, so you could get arbitrarily close to the limit of 1032:1."
#define SOAP_MINDEFLATERATIO (1.0/1032.0)

/// Macro is set to `#SOAP_PURE_VIRTUAL` = 0 at compile time when macro `#WITH_PURE_VIRTUAL` is defined
#define SOAP_PURE_VIRTUAL = 0

/// User-definable length of RSA keys used for https connections (the value is 2048 by default)
#define SOAP_SSL_RSA_BITS (2048)

/// User-definable 8 bit integer that represents a character that could not be converted to an ASCII char, i.e. when converting an XML Unicode character (typically UTF-8), this is applicable when the runtime flag `#SOAP_C_UTFSTRING` is not used (the value is 0x7F by default)
#define SOAP_UNKNOWN_CHAR (0x7F)

/// A user-definable integer Unicode value representing a character that replaces an invalid Unicode code point, i.e. when converting an XML invalid Unicode character from UTF-8 (the value is 0xFFFD by default)
#define SOAP_UNKNOWN_UNICODE_CHAR (0xFFFD)

/// User-definable macro that represents the `#LONG64` printf %-format
/**
This macro is "%lld" or "%I64d" on Windows.
*/
#define SOAP_LONG_FORMAT "%lld"

/// User-definable macro that represents the `#ULONG64` printf %-format
/**
This macro is "%llu" or "%I64u" on Windows.
*/
#define SOAP_ULONG_FORMAT "%llu"

/// Macro that defines a portable socket type, usually `int`, but the type may depend on the platform being used
#define SOAP_SOCKET int

/// Macro that defines a portable invalid socket value (usually -1, but the value depends on the OS)
/**
@see `#soap_valid_socket`.
*/
#define SOAP_INVALID_SOCKET (-1)

/// Macro that defines a portable socklen_t type (usually `size_t`, but type depends on the OS)
#define SOAP_SOCKLEN_T size_t

/// Function macro to check if a socket is valid, i.e. not equal to `#SOAP_INVALID_SOCKET`
#define soap_valid_socket(sock) ((sock) != SOAP_INVALID_SOCKET)

/// User-definable macro that represents a portable signed 64 bit integer type
/**
Legacy platforms that do not support 64 bit types can still be used by redefining `#LONG64` to `int`.  For example:

    cc -D LONG64=int -D ULONG64="unsigned int" -D SOAP_LONG_FORMAT='"%d"' -D SOAP_ULONG_FORMAT='"%u"' -o client stdsoap2.c soapC.c soapClient.c client.c

This type is also available as a valid type to use in the .h file for soapcpp2.

@see `#ULONG64`, `#SOAP_LONG_FORMAT`, `#SOAP_ULONG_FORMAT`.
*/
#define LONG64 int64_t

/// User-definable macro that represents a portable unsigned 64 bit integer type
/**
Legacy platforms that do not support 64 bit types can still be used by redefining `#LONG64` to `int`.  For example:

    cc -D LONG64=int -D ULONG64="unsigned int" -D SOAP_LONG_FORMAT='"%d"' -D SOAP_ULONG_FORMAT='"%u"' -o client stdsoap2.c soapC.c soapClient.c client.c

This type is also available as a valid type to use in the .h file for soapcpp2.

@see `#LONG64`, `#SOAP_LONG_FORMAT`, `#SOAP_ULONG_FORMAT`.
*/
#define ULONG64 uint64_t

/// User-definable macro that represents a portable single floating point NaN value (defined by default to a platform-specific NaN value)
#define FLT_NAN

/// User-definable macro that represents a portable single floating point positive infinite value (defined by default to a platform-specific value)
#define FLT_PINFTY

/// User-definable macro that represents a portable single floating point negative infinite value (defined by default to a platform-specific value)
#define FLT_NINFTY

/// User-definable macro that represents a portable double floating point NaN value (defined by default to a platform-specific NaN value)
#define DBL_NAN

/// User-definable macro that represents a portable double floating point positive infinite value (defined by default to a platform-specific value)
#define DBL_PINFTY

/// User-definable macro that represents a portable double floating point negative infinite value (defined by default to a platform-specific value)
#define DBL_NINFTY

/// Macro that returns true if the floating point value is NaN
#define soap_isnan(x)

/// Macro that returns true if the floating point value is infinity
#define soap_isinf(x)

/// User-definable macro to override malloc() for context-managed heap allocation (excluding C++ class instances, see `#SOAP_NEW`)
#define SOAP_MALLOC(soap, size) malloc((size))

/// User-definable macro to override free() for context-managed heap allocation (excluding C++ class instances, see `#SOAP_DELETE`)
#define SOAP_FREE(soap, ptr) free((void*)(ptr))

/// User-definable macro to override malloc() for unmanaged heap allocation
#define SOAP_MALLOC_UNMANAGED(soap, size) malloc((size))

/// User-definable macro to override free() for unmanaged heap allocation
#define SOAP_FREE_UNMANAGED(soap, ptr) free((void*)(ptr))

/// User-definable macro to override C++ new
#define SOAP_NEW(soap, type) new SOAP_NOTHROW (type)

/// User-definable macro to override C++ new for arrays
#define SOAP_NEW_ARRAY(soap, type, n) new SOAP_NOTHROW type[n]

/// User-definable macro to override C++ placement new
#define SOAP_PLACEMENT_NEW(soap, buf, type) new (buf) (type)

/// User-definable macro to override C++ delete
#define SOAP_DELETE(soap, obj, type) delete obj

/// User-definable macro to override C++ delete for arrays
#define SOAP_DELETE_ARRAY(soap, obj, type) delete[] obj

/// User-definable macro to override C++ new for unmanaged allocation of the soap context
#define SOAP_NEW_UNMANAGED(soap) new SOAP_NOTHROW soap

/// User-definable macro to override C++ delete for unmanaged deallocation of the soap context
#define SOAP_DELETE_UNMANAGED(soap) delete soap


/// User-definable macro to enable Windows DLL builds
/**
This macro when set exports global functions and classes by defining `#SOAP_FMAC1`, `#SOAP_FMAC3`, `#SOAP_FMAC5`, and `#SOAP_CMAC` to `__declspec(dllexport)`.

@par Example:

    C:> soapcpp2.exe -penv env.h
    C:> cl /c /I. /EHsc /DWITH_NONAMESPACES /DSOAP_STD_EXPORTS envC.cpp stdsoap2.cpp
    C:> link /LIBPATH ws2_32.lib /OUT:mygsoap.dll /DLL envC.obj stdsoap2.obj
*/
#define SOAP_STD_EXPORTS

/// User-definable macro to annotate global functions
/**
@see `#SOAP_STD_EXPORTS`, `#SOAP_FMAC2`.
*/
#define SOAP_FMAC1

/// User-definable macro to annotate global functions
/**
This macro is used in combination with `#SOAP_FMAC1`.

@see `#SOAP_FMAC1`.
*/
#define SOAP_FMAC2

/// User-definable macro to annotate global serialization functions generated by soapcpp2
/**
The soapcpp2 tool generates serialization functions as follows:

~~~{.cpp}
SOAP_FMAC3 int SOAP_FMAC4 soap_out_int(struct soap *soap, const char *tag, int id, const int *ptr, const char *type);
SOAP_FMAC3 int * SOAP_FMAC4 soap_in_int(struct soap *soap, const char *tag, int *ptr, const char *type);
SOAP_FMAC3 int * SOAP_FMAC4 soap_new_int(struct soap *soap, int num);
SOAP_FMAC3 int SOAP_FMAC4 soap_put_int(struct soap *soap, const int *ptr, const char *tag, const char *type);
SOAP_FMAC3 int * SOAP_FMAC4 soap_get_int(struct soap *soap, int *ptr, const char *tag, const char *type);
~~~

Other serialization functions are generated as well, either as C++ inlines or C defines.

@see `#SOAP_STD_EXPORTS`, `#SOAP_FMAC4`.
*/
#define SOAP_FMAC3

/// User-definable macro to annotate global serialization functions generated by soapcpp2
/**
This macro is used in combination with `#SOAP_FMAC3`.

@see `#SOAP_FMAC3`.
*/
#define SOAP_FMAC4

/// User-definable macro to annotate global service functions generated by soapcpp2
/**
The soapcpp2 tool generates service functions as follows at the client and service sides, respectively:

~~~{.cpp}
SOAP_FMAC5 int SOAP_FMAC6 soap_call_ns__webmethod(struct soap *soap, ...) { ... }
~~~
~~~{.cpp}
SOAP_FMAC5 int SOAP_FMAC6 soap_serve_ns__webmethod(struct soap *soap)
~~~

@see `#SOAP_STD_EXPORTS`, `#SOAP_FMAC6`.
*/
#define SOAP_FMAC5

/// User-definable macro to annotate global service functions generated by soapcpp2
/**
This macro is used in combination with `#SOAP_FMAC5`.

@see `#SOAP_FMAC5`.
*/
#define SOAP_FMAC6

/// User-definable macro to annotate class definitions
/**
The soapcpp2 tool generates struct and class definitions as follows:

~~~{.cpp}
class SOAP_CMAC name { ... };
~~~
*/
#define SOAP_CMAC

/// User-definable macro to annotate namespace table definitions
/**
The soapcpp2 tool generates `::Namespace` tables with XML namespace bindings which are declared as follows:

~~~{.cpp}
SOAP_NMAC struct Namespace namespaces[] = { ... };
~~~
*/
#define SOAP_NMAC

/** @} */

/**
\defgroup group_flags SOAP_MACRO run-time flags
@brief This module defines the `SOAP_MACRO` run-time `::soap_mode` flags to set the engine mode

This module defines the following `::soap_mode` flags:
- `#SOAP_C_MBSTRING`
- `#SOAP_C_NILSTRING`
- `#SOAP_C_NOIOB`
- `#SOAP_C_UTFSTRING`
- `#SOAP_DOM_ASIS`
- `#SOAP_DOM_NODE`
- `#SOAP_DOM_TREE`
- `#SOAP_ENC`
- `#SOAP_ENC_DIME`
- `#SOAP_ENC_LATIN`
- `#SOAP_ENC_MIME`
- `#SOAP_ENC_MTOM`
- `#SOAP_ENC_PLAIN`
- `#SOAP_ENC_SSL`
- `#SOAP_ENC_XML`
- `#SOAP_ENC_ZLIB`
- `#SOAP_IO`
- `#SOAP_IO_BUFFER`
- `#SOAP_IO_CHUNK`
- `#SOAP_IO_FLUSH`
- `#SOAP_IO_KEEPALIVE`
- `#SOAP_IO_LENGTH`
- `#SOAP_IO_STORE`
- `#SOAP_IO_UDP`
- `#SOAP_MIME_POSTCHECK`
- `#SOAP_SEC_WSUID`
- `#SOAP_XML_CANONICAL`
- `#SOAP_XML_CANONICAL_NA`
- `#SOAP_XML_DEFAULTNS`
- `#SOAP_XML_DOM`
- `#SOAP_XML_GRAPH`
- `#SOAP_XML_IGNORENS`
- `#SOAP_XML_INDENT`
- `#SOAP_XML_NIL`
- `#SOAP_XML_NOTYPE`
- `#SOAP_XML_STRICT`
- `#SOAP_XML_TREE`

@{
*/

/// The `::soap_mode` flags to initialize the `::soap` context, flags can be combined with `|` (bit-wise or)
/**
The `::soap_mode` flags are:
- `#SOAP_C_MBSTRING`
- `#SOAP_C_NILSTRING`
- `#SOAP_C_NOIOB`
- `#SOAP_C_UTFSTRING`
- `#SOAP_DOM_ASIS`
- `#SOAP_DOM_NODE`
- `#SOAP_DOM_TREE`
- `#SOAP_ENC`
- `#SOAP_ENC_DIME`
- `#SOAP_ENC_LATIN`
- `#SOAP_ENC_MIME`
- `#SOAP_ENC_MTOM`
- `#SOAP_ENC_PLAIN`
- `#SOAP_ENC_SSL`
- `#SOAP_ENC_XML`
- `#SOAP_ENC_ZLIB`
- `#SOAP_IO`
- `#SOAP_IO_BUFFER`
- `#SOAP_IO_CHUNK`
- `#SOAP_IO_FLUSH`
- `#SOAP_IO_KEEPALIVE`
- `#SOAP_IO_LENGTH`
- `#SOAP_IO_STORE`
- `#SOAP_IO_UDP`
- `#SOAP_MIME_POSTCHECK`
- `#SOAP_SEC_WSUID`
- `#SOAP_XML_CANONICAL`
- `#SOAP_XML_CANONICAL_NA`
- `#SOAP_XML_DEFAULTNS`
- `#SOAP_XML_DOM`
- `#SOAP_XML_GRAPH`
- `#SOAP_XML_IGNORENS`
- `#SOAP_XML_INDENT`
- `#SOAP_XML_NIL`
- `#SOAP_XML_NOTYPE`
- `#SOAP_XML_STRICT`
- `#SOAP_XML_TREE`
*/
typedef int soap_mode;

/// `::soap_mode` IO flags mask to check for `#SOAP_IO_FLUSH`, `#SOAP_IO_BUFFER`, `#SOAP_IO_STORE`, `#SOAP_IO_CHUNK` (for internal use only)
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK);
if ((soap->omode & SOAP_IO) == SOAP_IO_CHUNK)
  ... // HTTP chunking is enabled
~~~
*/
#define SOAP_IO 0x00000003

/// `::soap_mode` IO output flag value to flush the message sent immediately without buffering (the default mode), do not combine this flag with `#SOAP_IO_BUFFER`, `#SOAP_IO_STORE`, `#SOAP_IO_CHUNK`
#define SOAP_IO_FLUSH 0x00000000

/// `::soap_mode` IO output flag value to buffer the message sent in packets of size `#SOAP_BUFLEN`, do not combine this flag with `#SOAP_IO_FLUSH`, `#SOAP_IO_STORE`, `#SOAP_IO_CHUNK`
#define SOAP_IO_BUFFER 0x00000001

/// `::soap_mode` IO output flag value to store messages temporarily before transmission, e.g. to determine message length for transmission over HTTP instead of chunking or two-phase message sends, do not combine this flag with `#SOAP_IO_FLUSH`, `#SOAP_IO_BUFFER`, `#SOAP_IO_CHUNK`
/**
When set as input-mode flag, forces all messages sent to be temporarily stored before transmission.  When set as output-mode flag, forces only the next message to be temporarily stored before transmission.

@note `#SOAP_IO_CHUNK` is preferable over `#SOAP_IO_STORE`, because `#SOAP_IO_STORE` requires more memory and increases response time latency due to the overhead of storing the entire message before transmission.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_STORE); // store messages temporarily before transmission
~~~
*/
#define SOAP_IO_STORE 0x00000002

/// `::soap_mode` IO output flag value to send HTTP chunked messages, buffers the message in packets of size `#SOAP_BUFLEN`, do not combine this flag with `#SOAP_IO_FLUSH`, `#SOAP_IO_BUFFER`, `#SOAP_IO_STORE`
/**
@note `#SOAP_IO_CHUNK` is preferable to use compared to `#SOAP_IO_STORE`, because `#SOAP_IO_STORE` requires more memory and increases response time latency due to the overhead of storing the entire message before transmission.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // enable HTTP chunking
~~~
*/
#define SOAP_IO_CHUNK 0x00000003

/// `::soap_mode` IO input/output flag value to use UDP datagrams, message size is limited to UDP packet size
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_UDP); // use UDP
~~~

@see \ref UDP.
*/
#define SOAP_IO_UDP 0x00000004

/// `::soap_mode` IO output flag value to calculate message length when sending a message without transmission (for internal use only)
/**
@see `::soap_begin_count`, `::soap_end_count`.
*/
#define SOAP_IO_LENGTH 0x00000008

/// `::soap_mode` IO input and output flag value to keep the socket connection alive for `#SOAP_MAXKEEPALIVE` message exchanges per connection (100 by default), enabling HTTP keep-alive connection persistence
/**
This flag activates HTTP connection persistence for client and server applications.  The input and output mode flags should both be set to enable HTTP connection persistance.

Client applications can request HTTP persistance with this flag enabled and may indicate the end of a sequence of messages by disabling this flag immediately prior to the last message exchange with a server, which allows the server to gracefully close the connection.  If the client does not disable this flag, the server will keep the connection open until the connection is reset or times out.

Stand-alone server applications should be multi-threaded to use this flag, because iterative servers cannot serve other clients while connected to one client.  Stand-alone multi-threaded server applications should also use timeouts and limit the number of messages exchanged in a keep-alive connection with `soap::max_keep_alive`, which is set to `#SOAP_MAXKEEPALIVE` (the value is 100 by default).

@warning Do not use `#SOAP_IO_KEEPALIVE` with CGI and FastCGI applications.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); // enable HTTP keep-alive
soap->max_keep_alive = 50;                        // serve 50 max keep-alive exchanges (SOAP_MAXKEEPALIVE by default)
... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
soap_set_mode(soap, SOAP_IO_KEEPALIVE);           // keep connection to the server open
if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
  soap_print_fault(soap, stderr);
else
  ... // success
soap_clr_mode(soap, SOAP_IO_KEEPALIVE);           // connection is still open, but close it afterwards
if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
  soap_print_fault(soap, stderr);
else
  ... // success
~~~

@see `::soap::keep_alive`, `::soap::max_keep_alive`, `#SOAP_MAXKEEPALIVE`.
*/
#define SOAP_IO_KEEPALIVE 0x00000010

/// `::soap_mode` ENC flags mask (for internal use only)
#define SOAP_ENC 0x00000FFF

/// `::soap_mode` ENC input flag value to receive ISO-8859-1 encoded messages (automatically detected when receiving XML)
#define SOAP_ENC_LATIN 0x00000020

/// `::soap_mode` ENC input/output flag value to omit the HTTP headers from messages sent and disable detection and parsing of HTTP headers in messages received
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_ENC_PLAIN); // no HTTP
~~~
*/
#define SOAP_ENC_PLAIN 0x00000040

/// `::soap_mode` ENC input/output flag (deprecated, same as `#SOAP_ENC_PLAIN` for backward compatibility)
#define SOAP_ENC_XML

/// `::soap_mode` ENC output flag value to enable DIME attachments (for internal use only)
/**
@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
struct _xop__Include data;                                  // here we're using gsoap/import/xop.h to store a blob of raw data
data.__ptr = (unsigned char*)soap_malloc(soap, 1024);       // allocate 1024 bytes on the managed heap
memcpy(data.__ptr, image_data);                             // copy an image (or we could just assign to data.__ptr)
data.__size = 1024;                                         // 1024 bytes of data
data.id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
data.type = "image/png";                                    // attachment type
data.options = soap_dime_option(soap, 0, "Picture.png");    // DIME option 0 = "Picture.png" to store file name
... // add data to the message and then send it, which will transmit the data as an attachment
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
unsigned char *ptr = (unsigned char*)image_data;                   // image data to attach
size_t size = 1024;                                                // 1024 bytes of data
const char *id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
const char *type = "image/png";                                    // attachment type
if (soap_set_dime_attachment(soap, ptr, size, type, id, 0, "Picture.png"))
  ... // error attaching
... // send a message, image is attached
~~~

The `#SOAP_ENC_DIME` flag is automatically set when attachments are present in the message to be sent or when a message with DIME attachments is received and does not need to be set explicitly.

DIME attachment sizes are limited to `#SOAP_MAXDIMESIZE`, which is a compile-time constant that can be changed.

@see `#SOAP_ENC_MIME`, `#SOAP_ENC_DIME`, \ref DIME.
*/
#define SOAP_ENC_DIME 0x00000080

/// `::soap_mode` ENC output flag value to enable MIME attachments in messages to be sent, receiving is automatic
/**
The MTOM specification requires SOAP 1.2.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_ENC_MIME);               // enable MIME attachments, not DIME or MTOM
struct _xop__Include data;                                  // here we're using gsoap/import/xop.h to store a blob of raw data
data.__ptr = (unsigned char*)soap_malloc(soap, 1024);       // allocate 1024 bytes on the managed heap
memcpy(data.__ptr, image_data);                             // copy an image (or we could just assign to data.__ptr)
data.__size = 1024;                                         // 1024 bytes of data
data.id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
data.type = "image/png";                                    // attachment type
data.options = "Picture";                                   // attachment description
... // add data to the message and then send it
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_ENC_MIME);                      // enable MIME attachments, not DIME or MTOM
unsigned char *ptr = (unsigned char*)image_data;                   // image data to attach
size_t size = 1024;                                                // 1024 bytes of data
const char *id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
const char *type = "image/png";                                    // attachment type
if (soap_set_mime_attachment(soap, ptr, size, SOAP_MIME_BINARY, type, id, "No location", "Picture))
  ... // error attaching
... // send a message, image is attached
soap_clr_mime(soap);
~~~

@see `#SOAP_ENC_DIME`, `#SOAP_ENC_MTOM`, \ref MIME.
*/
#define SOAP_ENC_MIME 0x00000100

/// `::soap_mode` ENC output flag value to enable MTOM XOP attachments in messages to be sent, receiving is automatic
/**
@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_ENC_MTOM);               // enable MTOM attachments, not DIME
struct _xop__Include data;                                  // here we're using gsoap/import/xop.h to store a blob of raw data
data.__ptr = (unsigned char*)soap_malloc(soap, 1024);       // allocate 1024 bytes on the managed heap
memcpy(data.__ptr, image_data);                             // copy an image (or we could just assign to data.__ptr)
data.__size = 1024;                                         // 1024 bytes of data
data.id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
data.type = "image/png";                                    // attachment type
data.options = "Picture";                                   // attachment description
... // add data to the message and then send it
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_ENC_MTOM);                      // enable MTOM attachments, not DIME
unsigned char *ptr = (unsigned char*)image_data;                   // image data to attach
size_t size = 1024;                                                // 1024 bytes of data
const char *id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
const char *type = "image/png";                                    // attachment type
if (soap_set_mime_attachment(soap, ptr, size, SOAP_MIME_BINARY, type, id, "No location", "Picture))
  ... // error attaching
... // send a message, image is attached
soap_clr_mime(soap);
~~~

@see `#SOAP_ENC_DIME`, `#SOAP_ENC_MIME`, \ref MTOM.
*/
#define SOAP_ENC_MTOM 0x00000200

/// `::soap_mode` ENC output flag value to compress messages sent, requires zlib enabled with compile-time flag `#WITH_GZIP` (or `#WITH_ZLIB` for compression limited to "compress"), detection of compressed messages received and decompression of the messages is automatic
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_ENC_ZLIB); // HTTP compression
~~~
*/
#define SOAP_ENC_ZLIB 0x00000400

/// `::soap_mode` ENC input/output flag value to enable TLS/SSL, e.g. HTTPS, requires OpenSSL or GNUTLS enabled with compile-time flag `#WITH_OPENSSL` or `#WITH_GNUTLS` (for internal use only)
#define SOAP_ENC_SSL 0x00000800

/// `::soap_mode` XML input flag value to enable strict XML validation of messages received
/**
@warning This mode is not recommended for SOAP RPC encoding style messaging.  XML, XML REST, and SOAP/XML document/literal style messages can be validated.

Alternatively, use <i>`soapcpp2 -s`</i> to generate stub and skeleton functions that perform strict XML validation checks.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_STRICT); // strict XML validation
~~~

@see `#WITH_REPLACE_ILLEGAL_UTF8`, `#SOAP_UNKNOWN_UNICODE_CHAR`.
*/
#define SOAP_XML_STRICT 0x00001000

/// `::soap_mode` XML output flag value to enable XML (and JSON) message indentation in messages sent
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_INDENT); // indent XML
~~~
*/
#define SOAP_XML_INDENT 0x00002000

/// `::soap_mode` XML input flag value to ignore XML namespaces in messages received
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_IGNORENS); // ignore xmlns bindings in XML
~~~
*/
#define SOAP_XML_IGNORENS 0x00004000

/// `::soap_mode` XML output flag value to send XML messages with XML default namespaces for elements instead of namespace-qualified elements
/**
@warning This mode is not recommended for SOAP RPC encoding style messaging.  Optionally use this mode with XML, XML REST, and SOAP/XML document/literal style messages.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_DEFAULTNS); // use XML default namespaces with xmlns="..."
~~~

This flag changes the following example XML fragment:

<div class="alt">
~~~{.xml}
<ns:foo xmlns:ns="http://example.org" qux="a" ns:quux="a">
  <ns:bar>y</ns:bar>
  <baz>z<baz>
</ns:foo>
~~~
</div>

into the following fragment with default namespaces for XML elements:

<div class="alt">
~~~{.xml}
<foo xmlns="http://example.org" xmlns:ns="http://example.org" qux="a" ns:quux="a">
  <bar>y<bar>
  <baz xmlns="">z<baz>
</foo>
~~~
</div>

Qualified XML element names are replaced by unqualified element names.  Qualified XML attribute names are not affected and require a prefix with an XML namespace binding.  The `#SOAP_XML_DEFAULTNS` mode flag affects serialization performance somewhat negatively.  To produce normalized cleaner XML use the `#SOAP_XML_CANONICAL` mode flag, but this further negatively affects serialization performance.

@see `#WITH_NOEMPTYNAMESPACES`, `#SOAP_XML_CANONICAL`.

*/
#define SOAP_XML_DEFAULTNS 0x00008000

/// `::soap_mode` XML output flag value to send XML messages in exclusive canonical format as per W3C XML C14N standards, use with the `::soap::c14ninclude` and `::soap::c14nexclude` strings to control the prefixes that are subject to canonicalization by including or excluding specific prefixes
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_CANONICAL); // XML canonicalization, exclusive by default
soap->c14ninclude = "*";                           // optional: make all prefixes inclusive, i.e. inclusive canonicalization
~~~

This flag changes the following example XML fragment:

<div class="alt">
~~~{.xml}
<a:foo xmlns:b="http://example.org" qux="b" b:quux="c" baz="a" xmlns:a="http://www.w3.org">
  <bar xmlns:c="http://www.ietf.org"></bar>
</a:foo>
~~~
</div>

into the following exclusive C14N fragment:

<div class="alt">
~~~{.xml}
<a:foo xmlns:a="http://www.w3.org" xmlns:b="http://example.org" baz="a" qux="b" b:quux="c">
  <bar></bar>
</a:foo>
~~~
</div>

Sending XML is slowed down by canonicalization.  Furthermore, messages may or may not be reduced in size, depending on the additional <i>`xmlns`</i> bindings generated for local element scopes.  Consider for example the following message:

<div class="alt">
~~~{.xml}
<a:foo xmlns:a="http://www.w3.org" xmlns:b="http://example.org">
  <b:bar>1</b:bar>
  <b:bar>2</b:bar>
  <b:bar>3</b:bar>
</a:foo>
~~~
</div>

The message is larger when canonicalized:

<div class="alt">
~~~{.xml}
<a:foo xmlns:a="http://www.w3.org">
  <b:bar xmlns:b="http://example.org">1</b:bar>
  <b:bar xmlns:b="http://example.org">2</b:bar>
  <b:bar xmlns:b="http://example.org">3</b:bar>
</a:foo>
~~~
</div>

@note Because the W3C XML C14N (canonicalization) standards do not cover QName values, inclusive and exclusive C14N appear to be broken when QNames are used.  XML C14N retains xmlns bindings of utilized elements and attributes only, which means that xmlns bindings for prefixes in QNames may be lost in the canonicalization process.  The gSOAP toolkit implements XML C14N with QName normalization to prevent this loss.  Losing xmlns bindings for QName prefixes not only invalidates XML messages, but may also lead to incorrectly bound prefixes to xmlns bindings used in the outer scope of the canonicalized XML.  This poses a security threat when malicious xmlns bindings are placed in the outer scope to capture unbound QName prefix bindings.  The `#SOAP_XML_CANONICAL_NA` mode flag omits QName canonicalization.

@par Example:

<div class="alt">
~~~{.xml}
<a:foo xmlns:a="http://www.w3.org" xmlns:b="http://example.org" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <bar xsi:type="a:Bar" xmlns:a="http://example.org"></bar>
</a:foo>
~~~
</div>

This example XML fragment is canonicalized with `#SOAP_XML_CANONICAL` as:

<div class="alt">
~~~{.xml}
<a:foo xmlns:a="http://www.w3.org">
  <bar xmlns:a="http://example.org" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:type="a:Bar"></bar>
</a:foo>
~~~
</div>

The example XML fragment is canonicalized with `#SOAP_XML_CANONICAL | #SOAP_XML_CANONICAL_NA` as per W3C XML exclusive C14N as:

<div class="alt">
~~~{.xml}
<a:foo xmlns:a="http://www.w3.org">
  <bar xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:type="a:Bar"></bar>
</a:foo>
~~~
</div>

Note that prefix <i>`a`</i> in <i>`"a:Bar"`</i> is now bound to <i>`"http://www.w3.org"`</i> captured by the outer <i>`xmlns:a="http://www.w3.org"`</i>.  XML libraries may rename prefixes to prevent capturing.  The gSOAP serializers do not reuse prefixes, meaning that this problem cannot occur.  However, this does not prevent the namespace prefix to become lost when exclusive C14N is used and namespace bindings are removed because prefixes are not visibly utilized.

@see `#SOAP_XML_CANONICAL_NA`.
*/
#define SOAP_XML_CANONICAL 0x00010000

/// `::soap_mode` XML output flag value to send XML messages in exclusive canonical format as per W3C XML C14N standards, use with the `::soap::c14ninclude` and `::soap::c14nexclude` strings to control the prefixes that are subject to canonicalization by including or excluding specific prefixes
/**
Must be used in combination with `#SOAP_XML_CANONICAL_NA` to emit XML in canonical form such that namespace prefixes in QName values are ignored.  Recommended is to only use `#SOAP_XML_CANONICAL` which is more secure, see the `#SOAP_XML_CANONICAL` notes.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_CANONICAL | SOAP_XML_CANONICAL_NA);
soap->c14ninclude = "*"; // make all prefixes inclusive, i.e. inclusive canonicalization
~~~

@see `#SOAP_XML_CANONICAL`.
*/
#define SOAP_XML_CANONICAL_NA 0x00800000

/// `::soap_mode` XML input/output flag value to serialize C/C++ data structures as XML trees without id-href or id-ref multi-references, duplicates co-referenced data in the XML output and automatically breaks data structure cycles to prevent infinite serialization loops, ignores id and href/ref reference attributes in messages received, this flag is the opposite of `#SOAP_XML_GRAPH`
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_TREE); // no id-href and id-ref multi-reference serialization
~~~

@note This mode flag is always active when the engine is compiled with `#WITH_NOIDREF`.

@see `#SOAP_XML_GRAPH`, `#WITH_NOIDREF`.
*/
#define SOAP_XML_TREE 0x00020000

/// `::soap_mode` XML input/output flag value to serialize C/C++ (cyclic) data structures in XML with id-ref references for co-referenced data and to accurately serialize data structure cycles, accept id-ref references in XML received to (re)construct the C/C++ (co-referenced and cyclic) data structures, this flag is the opposite of `#SOAP_XML_TREE`
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_GRAPH); // id-ref multi-reference serialization
~~~

@note This mode flag is not usable when the engine is compiled with `#WITH_NOIDREF`.

@see `#SOAP_XML_TREE`.
*/
#define SOAP_XML_GRAPH 0x00020000

/// `::soap_mode` XML output flag value to serialize C/C++ NULL pointers in XML as elements with attribute <i>`xsi:nil="true"`</i>
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_NIL); // emit NULL pointers as empty elements with xsi:nil="true"
~~~
*/
#define SOAP_XML_NIL 0x00040000

/// `::soap_mode` XML output flag value to serialize C/C++ data structures in XML without <i>`xsi:type`</i> attributes, even when this may be necessary to distinguish base from derived data types
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_NOTYPE); // no xsi:type attributes ever
~~~
*/
#define SOAP_XML_NOTYPE 0x00080000

/// `::soap_mode` XML input/output flag value to enable DOM node tree construction of the XML received and sent, requires compile-time flag `#WITH_DOM`
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_DOM);
... // call an XML service operation that deserializes the response
if (soap->dom) // we also got a DOM
{
  const struct soap_dom_element *elt = soap->dom;
  ... // inspect the DOM using the DOM API functions (see the gSOAP [XML DOM API documentation](../../dom/html/index.html))
}
~~~
*/
#define SOAP_XML_DOM 0x10000000

/// `::soap_mode` DOM output flag value to disable reformatting of the DOM node tree in XML, displaying the DOM in XML exactly as is
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_DOM_ASIS);
soap_dom_element *dom = soap_new_xsd__anyType(soap); // allocate dom
if ((soap->recvfd = open("doc1.xml", O_RDONLY)) == 0 || soap_read_xsd__anyType(soap, dom))
  ... // an error occurred
if ((soap->sendfd = open("doc2.xml", O_CREAT | O_RDWR, 0644)) == 0 || soap_write_xsd__anyType(soap, dom))
  ... // an error occurred
close(soap->recvfd);
soap->recvfd = 0;
close(soap->sendfd);
soap->sendfd = 1;
~~~

This example requires the soapC.cpp source code generated with soapcpp2 from <i>`gsoap/import/dom.h`</i> as (imported) input in order to use `soap_new_xsd__anyType` and `soap_write_xsd__anyType`.

@see `#SOAP_DOM_NODE`, `#SOAP_DOM_TREE` and the gSOAP [XML DOM API documentation](../../dom/html/index.html).
*/
#define SOAP_DOM_ASIS 0x00400000

/// `::soap_mode` DOM input flag value to always attempt to deserialize embedded serializable C/C++ data structures into the DOM node tree based on matching element name to C/C++ type names and by matching <i>`xsi:type`</i> attributes when present, the opposite of `#SOAP_DOM_TREE`
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_DOM_NODE);
soap_dom_element *dom = soap_new_xsd__anyType(soap); // allocate dom
soap_dom_element *elt = soap_new_xsd__anyType(soap); // allocate child element
int num = 123;
soap_elt(dom, NULL, "data");                     // create <data> element
soap_elt_node(elt, (void*)&num, SOAP_TYPE_int)); // with value 123 serialized as child
soap_add_elt(dom, elt);
if (soap_write_xsd__anyType(soap, dom))
  ... // error, could not write
~~~

This example requires the soapC.cpp source code generated with soapcpp2 from <i>`gsoap/import/dom.h`</i> as (imported) input in order to use `soap_new_xsd__anyType` and `soap_write_xsd__anyType`.

@see `#SOAP_DOM_ASIS`, `#SOAP_DOM_TREE` and the gSOAP [XML DOM API documentation](../../dom/html/index.html).
*/
#define SOAP_DOM_NODE 0x00200000

/// `::soap_mode` DOM input flag value to ignore `id` and <i>`xsi:type`</i> XML attributes and disables deserialization of serializable C/C++ data structures into the DOM node tree, the opposite of `#SOAP_DOM_NODE`
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_DOM_TREE);
soap_dom_element *dom = soap_new_xsd__anyType(soap); // allocate dom
if ((soap->recvfd = open("doc.xml", O_RDONLY)) == 0 || soap_read_xsd__anyType(soap, dom))
  ... // an error occurred
close(soap->recvfd);
soap->recvfd = 0;
~~~

This example requires the soapC.cpp source code generated with soapcpp2 from <i>`gsoap/import/dom.h`</i> as (imported) input in order to use `soap_new_xsd__anyType`, `soap_read_xsd__anyType` and `soap_write_xsd__anyType`.

@see `#SOAP_DOM_ASIS`, `#SOAP_DOM_NODE` and the gSOAP [XML DOM API documentation](../../dom/html/index.html).
*/
#define SOAP_DOM_TREE 0x00100000

/// `::soap_mode` input flag value to ignore array items that are out of bounds when deserializing fixed-size arrays, instead of producing `#SOAP_IOB` errors
/**
@par Example:

~~~{.cpp}
struct ns__webmethodResponse
{
  float coordinates[3];
};
int ns__webmethod(float x, struct ns__webmethodResponse *out);
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_C_NOIOB);
struct ns__webmethodResponse result;
... //
if (soap_call_ns__webmethod(soap, endpoint, NULL, 3, &result))
  ... // deserializing more than 3 array items does not produce a SOAP_IOB error here
else
  ... // success
~~~
*/
#define SOAP_C_NOIOB 0x01000000

/// `::soap_mode` input/output flag value to serialize and deserialize 8-bit C/C++ strings containing UTF-8 encoded wide characters
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_C_UTFSTRING);
~~~
*/
#define SOAP_C_UTFSTRING 0x02000000

/// `::soap_mode` input/output flag value to enable multibyte character support for 8-bit character strings with `wctomb` and `mbtowc` using the current locale
/**
This flag requires macros `HAVE_WCTOMB` and `HAVE_MBTOWC` to be defined.  Otherwise, this flag has no effect.

@warning the `wctomb` and `mbtowc` functions are not thread safe.  It is recommended to use `#SOAP_C_UTFSTRING` instead of `SOAP_C_MBSTRING`.  `#SOAP_C_UTFSTRING` idoes not depend on the current locale.

@see `#SOAP_C_UTFSTRING`.
*/
#define SOAP_C_MBSTRING 0x04000000

/// `::soap_mode` input/output flag value to serialize empty strings as elements with attribute <i>`xsi:nil="true"`</i>
/**
@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_C_NILSTRING); // elements with empty strings have <i>`xsi:nil="true"`</i>
~~~
*/
#define SOAP_C_NILSTRING 0x08000000

/// `::soap_mode` MIME input flag value to check and process additional MIME attachments (for internal use only)
/**
Used by `::soap_post_check_mime_attachments` and `::soap_check_mime_attachments`.
*/
#define SOAP_MIME_POSTCHECK 0x40000000

/// `::soap_mode` SEC output flag value to add wsu:Id attributes to signed parts of messages sent and signed with WS-Security (for internal use only)
#define SOAP_SEC_WSUID 0x80000000

/** @} */

/**
\defgroup group_errors SOAP_MACRO run-time error codes
@brief This module defines the `SOAP_MACRO` run-time `::soap_status` error codes returned by functions and stored in `::soap::error`

@{
*/

/// Status and error codes are int values, a zero value or `#SOAP_OK` (0) means no error, nonzero means error
/**
The `#SOAP_OK` (zero) and nonzero error codes are returned by functions and are also stored in `::soap::error` (`::soap::errnum` stores the value of errno for system-related errors).  A nonzero error code is one of the following error codes or a HTTP error code between 100 and 599 returned by the server when a client connects:
`#SOAP_CLI_FAULT`
`#SOAP_DATAENCODINGUNKNOWN`
`#SOAP_DEL_METHOD`
`#SOAP_DIME_END`
`#SOAP_DIME_ERROR`
`#SOAP_DIME_HREF`
`#SOAP_DIME_MISMATCH`
`#SOAP_DUPLICATE_ID`
`#SOAP_EMPTY`
`#SOAP_END_TAG`
`#SOAP_EOF` (same as EOF)
`#SOAP_EOM`
`#SOAP_ERR` (same as EOF)
`#SOAP_FAULT`
`#SOAP_FD_EXCEEDED`
`#SOAP_FIXED`
`#SOAP_GET_METHOD`
`#SOAP_HDR`
`#SOAP_HREF`
`#SOAP_HTTP_ERROR`
`#SOAP_HTTP_METHOD`
`#SOAP_IOB`
`#SOAP_LENGTH`
`#SOAP_LEVEL`
`#SOAP_MIME_END`
`#SOAP_MIME_ERROR`
`#SOAP_MIME_HREF`
`#SOAP_MISSING_ID`
`#SOAP_MOE`
`#SOAP_MUSTUNDERSTAND`
`#SOAP_NAMESPACE`
`#SOAP_NO_DATA`
`#SOAP_NO_METHOD`
`#SOAP_NO_TAG`
`#SOAP_NTLM_ERROR`
`#SOAP_NULL`
`#SOAP_OCCURS`
`#SOAP_PATCH_METHOD`
`#SOAP_PATTERN`
`#SOAP_PLUGIN_ERROR`
`#SOAP_PROHIBITED`
`#SOAP_PUT_METHOD`
`#SOAP_REQUIRED`
`#SOAP_SSL_ERROR`
`#SOAP_SVR_FAULT`
`#SOAP_SYNTAX_ERROR`
`#SOAP_TAG_MISMATCH`
`#SOAP_TCP_ERROR`
`#SOAP_TYPE`
`#SOAP_UDP_ERROR`
`#SOAP_USER_ERROR`
`#SOAP_UTF_ERROR`
`#SOAP_VERSIONMISMATCH`
`#SOAP_ZLIB_ERROR`

@see `::soap::error`, `::soap_xml_error_check`, `::soap_soap_error_check`, `::soap_http_error_check`, `::soap_dime_error_check`, `::soap_mime_error_check`, `::soap_tcp_error_check`, `::soap_udp_error_check`, `::soap_ssl_error_check`, `soap_zlib_error_check`.
*/
typedef int soap_status;

/// The `::soap_status` code for no error (zero)
#define SOAP_OK

/// A `::soap_status` error code: the service returned a SOAP 1.1 client fault / SOAP 1.2 sender fault to the client
/**
The `::soap::fault` is non-NULL and points to a `::SOAP_ENV__Fault` structure.  Use `::soap_fault_string`, `::soap_fault_subcode` and `::soap_fault_detail` to extract the SOAP Fault string/reason, subcode and the detail XML string (when non-NULL).

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // call a Web service here
if (soap->error == SOAP_CLI_FAULT)
{
  // The server responded with a SOAP 1.1 client fault / SOAP 1.2 sender fault
  const char *s = soap_fault_string(soap);
  const char *d = soap_fault_detail(soap);
  printf("Client fault: %s detail: %s\n", s, d ? d : "(none)");
}
~~~

This error code is also caught by `::soap_soap_error_check`.
*/
#define SOAP_CLI_FAULT

/// A `::soap_status` error code: SOAP 1.2 DataEncodingUnknown fault (unused in practice)
/**
This error code is also caught by `::soap_soap_error_check`.
*/
#define SOAP_DATAENCODINGUNKNOWN

/// A `::soap_status` error code: DIME formatting error or DIME attachment size exceeds `#SOAP_MAXDIMESIZE`
/**
This error code is also caught by `::soap_dime_error_check`.
*/
#define SOAP_DIME_ERROR

/// A `::soap_status` error code: DIME attachment has no href from SOAP body and no DIME callbacks were defined to save the attachment
/**
This error code is also caught by `::soap_dime_error_check`.
*/
#define SOAP_DIME_HREF

/// A `::soap_status` error code: DIME version error
/**
This error code is also caught by `::soap_dime_error_check`.
*/
#define SOAP_DIME_MISMATCH

/// A `::soap_status` error code: end of DIME attachments protocol error
/**
This error code is also caught by `::soap_dime_error_check`.
*/
#define SOAP_DIME_END

/// A `::soap_status` error code: XML element has duplicate id attribute value (applicable to SOAP multi-ref encoding and `#SOAP_XML_GRAPH` serialization)
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_DUPLICATE_ID

/// A `::soap_status` error code: XML element or attribute is empty when a value is required
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_EMPTY

/// A `::soap_status` error code: XML ending tag found when none was expected
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_END_TAG

/// A `::soap_status` error code: unexpected end of file, no input, transmission interrupted or timed out (same value as EOF)
/**
The `#SOAP_EOF` error indicates a transmission error.  Use `::soap::errnum` to determine the source of the error, which is set to the value of `errno` of the failure when the error occurred.  When a transmission timeout occurred, because `::soap::recv_timeout` and/or `::soap::send_timeout` are nonzero, the value of `::soap::errnum` was set to zero to distinguish timeouts from errors.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // call a Web service here
if (soap->error == SOAP_EOF)
{
  if (soap->errnum)
    printf("A transmission error occurred: %s\n", strerror_r(soap->errnum, soap->msgbuf, sizeof(soap->msgbuf)));
  else
    printf("A transmission timeout occurred\n");
}
~~~

This error code is also caught by `::soap_tcp_error_check` and by `::soap_udp_error_check`.
*/
#define SOAP_EOF

/// A `::soap_status` error code: out of memory
/**
This error indicates that a dynamic memory allocation request failed.  The engine does not raise C++ exceptions when allocating memory for objects, but `#SOAP_EOM` is returned (from a function) instead and `::soap::error` is set to `#SOAP_EOM`.  It is possible to enable C++ exceptions without detrimental effects by compiling the source code with `#SOAP_NOTHROW` set to an empty value and in that case C++ exception handlers should be used to catch `std::bad_alloc`.
*/
#define SOAP_EOM

/// A `::soap_status` error code: an unspecified error occurred
#define SOAP_ERR

/// A `::soap_status` error code: the fault code to be returned by a service operation when calling `::soap_sender_fault` (client is at fault) or `::soap_receiver_fault` (server is at fault), and when received, clients set the `::soap::error` code to `#SOAP_CLI_FAULT` or `#SOAP_SVR_FAULT` respectively
/**
This error code is also caught by `::soap_soap_error_check`.
*/
#define SOAP_FAULT

/// A `::soap_status` error code: too many open sockets
#define SOAP_FD_EXCEEDED

/// A `::soap_status` error code: XML element or attribute value is fixed and the parsed value does not match the fixed value
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_FIXED

/// A `::soap_status` error code: an HTTP GET request was received by the service but the GET request callback `::soap::fget` is not implemented
/**
This error code is also caught by `::soap_http_error_check`.

@see `::soap::fget`.
*/
#define SOAP_GET_METHOD

/// A `::soap_status` error code: an HTTP PUT request was received by the service but the PUT request callback `::soap::fput` is not implemented
/**
This error code is also caught by `::soap_http_error_check`.

@see `::soap::fput`.
*/
#define SOAP_PUT_METHOD

/// A `::soap_status` error code: an HTTP PATCH request was received by the service but the PATCH request callback `::soap::fpatch` is not implemented
/**
This error code is also caught by `::soap_http_error_check`.

@see `::soap::fpatch`.
*/
#define SOAP_PATCH_METHOD

/// A `::soap_status` error code: an HTTP DELETE request was received by the service but the DELETE request callback `::soap::fdel` is not implemented
/**
This error code is also caught by `::soap_http_error_check`.

@see `::soap::fdel`.
*/
#define SOAP_DEL_METHOD

/// A `::soap_status` error code: an HTTP request was received by the service that cannot be handled.
#define SOAP_HTTP_METHOD

/// A `::soap_status` error code: HTTP header line is too long, exceeding `#SOAP_HDRLEN` size
#define SOAP_HDR

/// A `::soap_status` error code: reference to object in XML identified by its id attribute is incompatible with the object refered to by the ref or href attribute (applicable to SOAP multi-ref encoding and `#SOAP_XML_GRAPH` serialization)
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_HREF

/// A `::soap_status` error code: an unspecified HTTP error occured
/**
This error code is also caught by `::soap_http_error_check`.
*/
#define SOAP_HTTP_ERROR

/// A `::soap_status` error code: SOAP array index out of bounds
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_IOB

/// A `::soap_status` error code: XML element or attribute value length validation error or `#SOAP_MAXLENGTH` exceeded
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_LENGTH

/// A `::soap_status` error code: XML nesting depth level when parsing XML exceeds `#SOAP_MAXLEVEL`
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_LEVEL

/// A `::soap_status` error code: end of MIME/MTOM attachments protocol error
/**
This error code is also caught by `::soap_mime_error_check`.
*/
#define SOAP_MIME_END

/// A `::soap_status` error code: MIME/MTOM attachment parsing error
/**
This error code is also caught by `::soap_mime_error_check`.
*/
#define SOAP_MIME_ERROR

/// A `::soap_status` error code: MIME/MTOM attachment has no href from SOAP body and no MIME callbacks were defined to save the attachment
/**
This error code is also caught by `::soap_mime_error_check`.
*/
#define SOAP_MIME_HREF

/// A `::soap_status` error code: an XML element with id attribute is missing that should match the element with href/ref attribute (applicable to SOAP multi-ref encoding and `#SOAP_XML_GRAPH` serialization)
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_MISSING_ID

/// A `::soap_status` error code: memory overflow or memory corruption error (applicable in `#DEBUG` mode only)
#define SOAP_MOE

/// A `::soap_status` error code: an XML element is present with a mustUnderstand attribute which must be understood but is not deserialized
/**
This error code is also caught by `::soap_soap_error_check`.
*/
#define SOAP_MUSTUNDERSTAND

/// A `::soap_status` error code: XML namespace name mismatch validation error
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_NAMESPACE

/// A `::soap_status` error code: no data in the HTTP body of the message received
#define SOAP_NO_DATA

/// A `::soap_status` error code: the service request dispatcher did not find a matching service operation for a service request
/**
This error code is also caught by `::soap_soap_error_check`.
*/
#define SOAP_NO_METHOD

/// A `::soap_status` error code: no XML element tag was found when one was expected
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_NO_TAG

/// A `::soap_status` error code: an NTLM authentication handshake error occured
#define SOAP_NTLM_ERROR

/// A `::soap_status` error code: XML element has an <i>`xsi:nil`</i> attribute when the element is not nillable, causing a validation error
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_NULL

/// A `::soap_status` error code: XML element has a minOccurs or maxOccurs constraint validation error or `#SOAP_MAXOCCURS` was exceeded
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_OCCURS

/// A `::soap_status` error code: XML element or attribute value pattern mismatch causes a validation error
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_PATTERN

/// A `::soap_status` error code: failed to register plugin
#define SOAP_PLUGIN_ERROR

/// A `::soap_status` error code: attribute is prohibited but present
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_PROHIBITED

/// A `::soap_status` error code: an HTTP PUT request was received by the service but the PUT request callback `::soap::fput` is not implemented
/**
This error code is also caught by `::soap_http_error_check`.
*/
#define SOAP_PUT_METHOD

/// A `::soap_status` error code: attribute is required but not present
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_REQUIRED

/// A `::soap_status` error code: an SSL error occured
/**
This error code is also caught by `::soap_ssl_error_check`.
*/
#define SOAP_SSL_ERROR

/// A `::soap_status` error code: service returned a SOAP 1.1 server fault / SOAP 1.2 receiver fault to the client
/**
The `::soap::fault` is non-NULL and points to a `::SOAP_ENV__Fault` structure.  Use `::soap_fault_string`, `::soap_fault_subcode` and `::soap_fault_detail` to extract the SOAP Fault string/reason, subcode and the detail XML string (when non-NULL).

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // call a Web service here
if (soap->error == SOAP_SVR_FAULT)
{
  // The server responded with a SOAP 1.1 server fault / SOAP 1.2 receiver fault
  const char *s = soap_fault_string(soap);
  const char *d = soap_fault_detail(soap);
  printf("Server fault: %s detail: %s\n", s, d ? d : "(none)");
}
~~~

This error code is also caught by `::soap_soap_error_check`.
*/
#define SOAP_SVR_FAULT

/// A `::soap_status` error code: an XML syntax error occurred while parsing the input
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_SYNTAX_ERROR

/// A `::soap_status` error code: XML element tag parsed does not match anything that is expected
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_TAG_MISMATCH

/// A `::soap_status` error code: a TCP/IP connection error occured
/**
This error code is also caught by `::soap_tcp_error_check`.
*/
#define SOAP_TCP_ERROR

/// A `::soap_status` error code: XML element or attribute has a mismatching type or value that is causing a validation error
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_TYPE

/// A `::soap_status` error code: a UDP/IP connection error occured or the message too large to store in a UDP packet
/**
This error code is also caught by `::soap_udp_error_check`.
*/
#define SOAP_UDP_ERROR

/// A `::soap_status` error code: soap::user not set to non-NULL
#define SOAP_USER_ERROR

/// A `::soap_status` error code: a UTF-8 decoding error occured
/**
This error code is also caught by `::soap_xml_error_check`.
*/
#define SOAP_UTF_ERROR

/// A `::soap_status` error code: SOAP version mismatch or no SOAP message is received
/**
This error code is also caught by `::soap_soap_error_check`.
*/
#define SOAP_VERSIONMISMATCH

/// A `::soap_status` error code: a zlib error occured
/**
This error code is also caught by `::soap_zlib_error_check`.
*/
#define SOAP_ZLIB_ERROR

/// Check for XML parsing and validation errors, returns true if the specified error code is an XML error
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

/// Check for SOAP protocol faults and errors, returns true if the specified error code is a SOAP protocol error
#define soap_soap_error_check(e) \
((e) == SOAP_CLI_FAULT || \
 (e) == SOAP_SVR_FAULT || \
 (e) == SOAP_VERSIONMISMATCH || \
 (e) == SOAP_MUSTUNDERSTAND || \
 (e) == SOAP_FAULT || \
 (e) == SOAP_NO_METHOD || \
 (e) == SOAP_DATAENCODINGUNKNOWN)

/// Check for HTTP protocol errors, returns true if the specified error code is an HTTP protocol error or an HTTP status code between 100 and 599 returned by an HTTP server, but note that HTTP status 100 to 202 should not be considered errors but informative codes
#define soap_http_error_check(e) \
((e) == SOAP_HTTP_ERROR || \
 (e) == SOAP_NO_DATA || \
 ((e) >= SOAP_GET_METHOD && (e) <= SOAP_HTTP_METHOD) || \
 ((e) >= 100 && (e) < 600))

/// Check for DIME protocol errors, returns true if the specified `::soap_status` error code is a DIME protocol error
#define soap_dime_error_check(e) \
  ((e) == SOAP_DIME_ERROR || \
   (e) == SOAP_DIME_HREF || \
   (e) == SOAP_DIME_MISMATCH || \
   (e) == SOAP_DIME_END)

/// Check for MIME/MTOM protocol errors, returns true if the specified `::soap_status` error code is a MIME/MTOM protocol error
#define soap_mime_error_check(e) \
  ((e) == SOAP_MIME_ERROR || \
   (e) == SOAP_MIME_HREF || \
   (e) == SOAP_MIME_END)

/// Check for TCP protocol errors, returns true if the specified error code is a TCP error, when true use `::soap::errnum` to retrieve the `errno` value of the failure to determine the cause
#define soap_tcp_error_check(e) ((e) == SOAP_EOF || (e) == SOAP_TCP_ERROR)

/// Check for UDP protocol errors, returns true if the specified error code is a UDP error, when true use `::soap::errnum` to retrieve the `errno` value of the failure to determine the cause
#define soap_udp_error_check(e) ((e) == SOAP_EOF || (e) == SOAP_UDP_ERROR)

/// Check for SSL/TLS protocol errors, returns true if the specified error code is a SSL/TLS error, when true use `::soap::errnum` to retrieve the `errno` value of the failure to determine the cause
#define soap_ssl_error_check(e) ((e) == SOAP_SSL_ERROR)

/// Check for zlib library errors, returns true if the specified error code is a zlib error
#define soap_zlib_error_check(e) ((e) == SOAP_ZLIB_ERROR)

/// An internal `::soap_status` error code to signal that an HTTP response must not be produced
#define SOAP_STOP

/// An internal `::soap_status` error code to signal that an HTTP form is present and no HTTP response must be produced
#define SOAP_FORM

/// A special `::soap_status` error code to signal that a custom HTTP response is present and no HTTP response must be produced
/**
@ingroup group_io
This code is used with `::soap_response` to return a HTML response message with a HTTP content type "text/html".
*/
#define SOAP_HTML

/// A special `::soap_status` error code to signal that a custom file-based HTTP response is present and no HTTP response must be produced
/**
@ingroup group_io
This code is used with `::soap_response` to return a file-based response message with a HTTP content type specified by the `::soap::http_content` string variable.  The `::soap_response` function normally returns HTTP 200 OK, but the HTTP status code can be specified as `#SOAP_FILE + status` where `status` is a HTTP status code between 200 and 599.
*/
#define SOAP_FILE

/** @} */

/**
\defgroup group_context Context with engine state
@brief This module defines the `::soap` \ref soap "context structure with the engine state" and functions to allocate, initialize, copy and delete contexts

@{
*/

/// Context with the engine state
/**
The `::soap` context should be passed as the first parameter to all gSOAP functions and should only be used by a single thread at a time.  Each thread should use a copy of the context created with `::soap_copy` or with the `::soap::soap` copy constructor.

To allocate a new `::soap` context, use one of these three allocators that take `::soap_mode` parameters:

~~~{.cpp}
struct soap *soap1 = soap_new();
struct soap *soap2 = soap_new1(input_and_output_mode);
struct soap *soap3 = soap_new2(input_mode, output_mode);
~~~

Alternatively, use constructors in C++ as follows:

~~~{.cpp}
struct soap *soap1();
struct soap *soap2(input_and_output_mode);
struct soap *soap3(input_mode, output_mode);
~~~

To copy the `::soap` context, for example to be used by another thread, use:

~~~{.cpp}
struct soap *soap2 = soap_copy(soap1);
~~~

Alternatively, use the copy constructor in C++ as follows:

~~~{.cpp}
struct soap *soap2 = new soap(soap1);
~~~

To free the `::soap` context or a copy of a `::soap` context, use:

~~~{.cpp}
soap_free(soap1);
~~~

To stack-allocate a `::soap` context (i.e. as opposed to heap-allocating it as shown above), initialize the `::soap` context with one of these three initializers that take `::soap_mode` parameters:

~~~{.cpp}
struct soap soap1, soap2, soap3;
soap_init(&soap1);
soap_init1(&soap2, input_and_output_mode);
soap_init2(&soap3, input_mode, output_mode);
~~~

Alternatively, use constructors in C++:

~~~{.cpp}
struct soap soap1();
struct soap soap2(input_and_output_mode);
struct soap soap3(input_mode, output_mode);
~~~

Finalization of the `::soap` context is automatically done by the C++ destructor.  In C, you should finalize a stack-allocated `::soap` context before it is reclaimed by the stack:

~~~{.cpp}
soap_done(&soap1);
~~~

Before freeing or finalizating a `::soap` context, you may want to delete all data allocated in managed heap memory with these two calls in this specific order:

~~~{.cpp}
soap_destroy(soap1);
soap_end(soap1);
~~~

Alternatively, in C++ you can simply invoke one method to perform both calls at once:

~~~{.cpp}
soap1.destroy();
~~~
*/
struct soap {
  /// Construct a `::soap` context (C++ only, in C use `::soap_new` or `::soap_init`)
  soap();
  /// Construct a `::soap` context with the specified input and output `::soap_mode` flags (C++ only, in C use `::soap_new1` or `::soap_init1`))
  soap(soap_mode input_and_output_mode) ///< input and output `::soap_mode` flags
    ;
  /// Construct a `::soap` context with the specified input and output `::soap_mode` flags (C++ only, in C use `::soap_new2` or `::soap_init2`))
  soap(
      soap_mode input_mode,  ///< input `::soap_mode` flags
      soap_mode output_mode) ///< output `::soap_mode` flags
    ;
  /// Copy constructor (C++ only, in C use `::soap_copy` or `::soap_copy_context`)
  soap(const struct soap &) ///< the `::soap` context to copy
    ;
  /// Assignment constructor (C++ only, in C use `::soap_copy` or `::soap_copy_context`)
  struct soap& operator=(const struct soap &) ///< the `::soap` context to copy
    /// @returns this `::soap` context
    ;
  /// Delete all dynamically-allocated objects and data managed by this `::soap` context (C++ only, in C use `::soap_end`)
  void destroy();
  /// Finalize and delete the `::soap` context by finalizing with soap_done(this) (C++ only, in C use `::soap_free` or `::soap_done`)
  ~soap();
  /// The `::soap` context input `::soap_mode` flags that are set at context initialization and set or cleared with `::soap_set_imode` or `::soap_clr_imode`, respectively
  /**
  @see `::soap::omode`, `::soap_set_omode`, `::soap_set_mode`, `::soap_clr_omode`, `::soap_clr_mode`.
  */
  soap_mode imode;
  /// The `::soap` context output `::soap_mode` flags that are set at context initialization and set or cleared with `::soap_set_omode` or `::soap_clr_omode`, respectively
  /**
  @see `::soap::imode`, `::soap_set_omode`, `::soap_set_mode`, `::soap_clr_omode`, `::soap_clr_mode`.
  */
  soap_mode omode;
  /// OpenSSL context pointer
  /**
  This pointer is non-NULL after calling `::soap_ssl_server_context` or `::soap_ssl_client_context` and points to the OpenSSL context which can be configured with OpenSSL API functions such as `SSL_CTX_set_cipher_list`.

  @note Requires compilation with `#WITH_OPENSSL`.

  @see `::soap_ssl_server_context`, `::soap_ssl_client_context`.
  */
  SSL_CTX *ctx;
  /// SOAP version (0 = no SOAP, 1 = SOAP 1.1, 2 = SOAP 1.2)
  /**
  The SOAP version is determined from the SOAP or XML message received.  The version is automatically set before sending messages when SOAP 1.1 or SOAP 1.2 namespaces are defined in the `::Namespace` table or when SOAP namespaces are omitted from the table.  The version can be explicitly set or overruled by calling `::soap_set_version`.

  @see `::soap::encodingStyle`, `::soap_set_version`.
  */
  short version;
  /// User-definable <i>`SOAP-ENV:encodingStyle`</i> URI value
  /**
  This URI string value is pre-defined by the engine depending on the SOAP protocol version used, setting this to NULL means no SOAP encodingStyle, setting this to "" means that the engine will set the encodingStyle URI according to the SOAP version used.

  @see `::soap::version`, `::soap_set_version`.
  */
  const char *encodingStyle;
  /// User-definable <i>`SOAP-ENV:actor`</i> (SOAP 1.1) or <i>`SOAP-ENV:role`</i> (SOAP 1.2) attribute value of all <i>`SOAP-ENV:mustUnderstand`</i> attributed header elements
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  ... // context initializations
  // add a SOAP Header to the message
  SOAP_ENV__Header header;
  header->ns__required_value = 123; // a header element marked mustUnderstand
  soap->header = &header;
  // add SOAP-ENV:role="http://www.w3.org/2002/06/soap-envelope/role/next";
  soap->actor = "http://www.w3.org/2002/06/soap-envelope/role/next";
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  @see `::SOAP_ENV__Header`.
  */
  const char *actor;
  /// User-definable <i>`xml:lang`</i> attribute value of <i>`SOAP-ENV:Text`</i> to output the SOAP Fault string/reason (the value is `en` by default)
  /**
  @see `::SOAP_ENV__Reason`.
  */
  const char *lang;
  /// User-definable maximum number of keep-alive message exchanges per connection (the value is `#SOAP_MAXKEEPALIVE` by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); // enable HTTP keep-alive
  soap->max_keep_alive = 50;                        // 50 max keep-alive exchanges (SOAP_MAXKEEPALIVE by default)
  ~~~

  @see `#SOAP_IO_KEEPALIVE`, `::soap_serve`, `::soap::keep_alive`.
  */
  int max_keep_alive;
  /// HTTP keep-alive flag (try to enable when -1, disabled when 0) and counter (enabled when >0)
  /**
  @see `#SOAP_IO_KEEPALIVE`, `::soap_serve`, `::soap_closesock`, `::soap::max_keep_alive`.
  */
  int keep_alive;
  /// User-definable string to control the XML namespace prefixes that are subject to XML canonicalization with the `#SOAP_XML_CANONICAL` output mode flag, specified by space-separated prefixes in the string, or `*` to specify that all prefixes are inclusive, or NULL when unused
  const char *c14ninclude;
  /// User-definable string to control the XML namespace prefixes that are subject to XML canonicalization with `#SOAP_XML_CANONICAL` output mode flag, specified by space-separated prefixes in the string, or NULL when unused
  const char *c14nexclude;
  /// The `::soap` context HTTP status code received at the client side (100 to 599), HTTP header method received at the server side (`#SOAP_POST`, `#SOAP_PATCH`, `#SOAP_GET`, `#SOAP_PUT`, `#SOAP_DEL`, `#SOAP_HEAD`, `#SOAP_OPTIONS`), or the HTTP method to use for sending a message with `::soap_connect_command` or with `::soap_response` (`#SOAP_POST`, `#SOAP_POST_FILE`, `#SOAP_PATCH`, `#SOAP_GET`, `#SOAP_PUT`, `#SOAP_PUT`, `#SOAP_DEL`, `#SOAP_CONNECT`, `#SOAP_HEAD`, `#SOAP_OPTIONS`)
  /**
  @see `::soap_connect_command`, `::soap_GET`, `::soap_PUT`, `::soap_PATCH`, `::soap_POST`, `::soap_DELETE`, `::soap_response`.
  */
  int status;
  /// The `::soap` context `::soap_status` (int) error code of the last operation or `#SOAP_OK` (zero)
  /**
  @see `::soap_status`, `::soap::errnum`.
  */
  int error;
  /// The `errno` value of the last failed IO operation
  /**
  The `::soap::errnum` value is set to the value or `errno` when a `#SOAP_EOF` or `#SOAP_TCP_ERROR` error occurred.  This allows for reporting the error condition with `::soap_print_fault`, `::soap_stream_fault`, and `::soap_sprint_fault`.  For the `#SOAP_EOF` error, `::soap::errnum` is set to zero when IO operations timed out, when a client's connection attempt to a server timed out, or when a server-side `::soap_accept` timed out.

  @see `::soap::error`, `::soap::connect_timeout`, `::soap::accept_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`.
  */
  int errnum;
  /// The `::soap::header` points to a `::SOAP_ENV__Header` structure with the SOAP Header that was received or that can be populated by the user to be sent, or NULL when no SOAP Header is present
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  ... // context initializations
  // add a SOAP Header to the message
  SOAP_ENV__Header header;
  header->ns__someHeaderValue = 123;
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
  {
    soap_print_fault(soap, stderr);
  }
  else
  {
    if (soap->header) // received a SOAP_ENV__Header?
      ... // yes, inspect SOAP_ENV__Header headers
  }
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  @see `::SOAP_ENV__Header`, `::soap::actor`.
  */
  struct SOAP_ENV__Header *header;
  /// The `::soap::fault` points to a `::SOAP_ENV__Fault` structure with the SOAP Fault that was received or that can be populated by the user to be sent, or NULL when no SOAP Fault is present
  struct SOAP_ENV__Fault *fault;
  /// User-definable variable that may point to user-specified data of any type to pass the data through to callbacks and plugins
  void *user;
  /// The `::soap::mustUnderstand` flag is set when a SOAP Header element carries a <i>`SOAP-ENV:mustUnderstand`</i> attribute that is true
  short mustUnderstand;
  /// The `::soap::null` flag is set when an element carries a <i>`xsi:nil`</i> attribute that is true
  short null;
  /// The `::soap::body` flag is set when an element has element content during XML parsing or when a HTTP message has a body when parsing an HTTP header
  short body;
  /// User-definable XML declaration prolog (<i>`<?xml version="1.0" encoding="UTF-8"?>`</i> by default)
  const char *prolog;
  /// User-definable string that specifies the HTTP cookie domain of the running server
  const char *cookie_domain;
  /// User-definable  string that specifies the HTTP cookie path of the running server
  const char *cookie_path;
  /// User-definable maximum number of active cookies allowed to be set with `::soap_set_cookie` before cookie memory is reused (the value is 32 by default)
  int cookie_max;
  /// The cookie store is a linked list of cookies
  struct soap_cookie *cookies;
  /// String with HTTP content type header value received, can also be assigned to specify a content type header for `::soap_connect_command` with `#SOAP_POST_FILE`, `#SOAP_PUT` and `#SOAP_PATCH` or for `::soap_response` with `#SOAP_FILE` or for `::soap_PUT`, `::soap_PATCH`, `::soap_POST`.  The `::soap::http_content` string is reset to NULL after each HTTP invocation and has to be set again when required.
  /**
  @see `::soap_connect_command`, `::soap_PUT`, `::soap_PATCH`, `::soap_POST`, `::soap_response`.
  */
  const char *http_content;
  /// User-definable string that specifies an extra HTTP header or headers when separated by CRLF, to include in the next HTTP request (client side) or to include with the HTTP response (server side)
  /**
  @par Examples:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  ... // context initializations
  soap->http_extra_header = "X-CUSTOM-RequestMessageNumber: 1\r\nX-CUSTOM-TotalRequests: 5";
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  ~~~{.cpp}
  #include "soapH.h"
  int main()
  {
    struct soap *soap = soap_new();
    if (soap_ssl_server_context(soap, ...))
      exit(EXIT_FAILURE);
    ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
  }
  int ns__webmethod(struct soap *soap, ...)
  {
    ... // process the request data and populate the response data
    soap->http_extra_header = "X-CUSTOM-ResponseMessageNumber: 1";
    return SOAP_OK;
  }
  ~~~

  @see `::soap_GET`, `::soap_PUT`, `::soap_PATCH`, `::soap_POST`, `::soap_DELETE`, `::soap::fparsehdr`.
  */
  const char *http_extra_header;
  /// The socket set by `::soap_bind` (or the C++ service class `bind` method) to serve as the master socket bound to a specified port, or `#SOAP_INVALID_SOCKET` when unassigned
  /**
  @see `::soap_bind`, `::soap_valid_socket`.
  */
  SOAP_SOCKET master;
  /// The socket set by `::soap_accept` (or the C++ service class `accept` method) or `::soap_connect` or `::soap_connect_command` (or the C++ proxy methods) when successful, or `#SOAP_INVALID_SOCKET` when unassigned
  /**
  @see `::soap_accept`, `::soap_connect`, `::soap_connect_command`, `::soap_valid_socket`.
  */
  SOAP_SOCKET socket;
  /// The file descriptor to read data from when no socket communications are set (`::soap::socket` == `#SOAP_INVALID_SOCKET`) and `::soap:is` == NULL, default value is 0 (stdin)
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new1(SOAP_DOM_TREE);
  struct ns__someElement data;
  if ((soap->recvfd = open("doc.xml", O_RDONLY)) == 0)
    ... // an error occurred
  // parse and deserialize XML into a ns__someElement structure
  if (soap_read_ns__someElement(soap, &data))
    ... // an error occurred
  close(soap->recvfd);
  soap->recvfd = 0;
  ~~~
  */
  int recvfd;
  /// The file descriptor to write data to when no socket communications are set (`::soap::socket` == `#SOAP_INVALID_SOCKET`) and `::soap:os` == NULL, default value is 1 (stdout)
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new1(SOAP_DOM_TREE);
  struct ns__someElement data;
  ... // populate the data
  if ((soap->sendfd = open("doc.xml", O_CREAT | O_RDWR, 0644)) == 0)
    ... // an error occurred
  // serialize ns__someElement structure as XML
  if (soap_write_ns__someElement(soap, &data))
    ... // an error occurred
  close(soap->sendfd);
  soap->sendfd = 1;
  ~~~
  */
  int sendfd;
  /// The source to read data from when non-NULL, which in C++ is a `std::istream` object and in C is a 0-terminated string to be read, default value is NULL
  /**
  @par Examples:

  ~~~{.cpp}
  // C example
  #include "soapH.h"
  struct soap *soap = soap_new1(SOAP_DOM_TREE);
  struct ns__someElement data;
  soap->is = "<ns:someElement xmlns:ns="urn:example"><text>example</text></ns:someElement>\n";
  // parse and deserialize XML into a ns__someElement structure
  if (soap_read_ns__someElement(soap, &data))
    ... // an error occurred
  soap->is = NULL;
  ~~~

  ~~~{.cpp}
  // C++ example
  #include "soapH.h"
  struct soap *soap = soap_new1(SOAP_DOM_TREE);
  struct ns__someElement data;
  std::stringstream ss("<ns:someElement xmlns:ns="urn:example"><text>example</text></ns:someElement>\n");
  soap->is = &ss;
  // parse and deserialize XML into a ns__someElement structure
  if (soap_read_ns__someElement(soap, &data))
    ... // an error occurred
  soap->is = NULL;
  ~~~
  */
  SOAP_SOURCE *is;
  /// The sink to write data to when non-NULL, which in C++ is a `std::ostream` object and in C is a pointer to a `char*` string variable that will be set to point to a managed 0-terminated string with the data, default value is NULL
  /**
  @par Examples:

  ~~~{.cpp}
  // C example
  #include "soapH.h"
  struct soap *soap = soap_new1(SOAP_DOM_TREE);
  struct ns__someElement data;
  char *cs = NULL;
  ... // populate the data
  soap->os = &cs;
  // serialize ns__someElement structure as XML
  if (soap_write_ns__someElement(soap, &data))
    ... // an error occurred
  soap->os = NULL;
  const char *xml = cs; // string allocated and managed by the context
  ~~~

  ~~~{.cpp}
  // C++ example
  #include "soapH.h"
  struct soap *soap = soap_new1(SOAP_DOM_TREE);
  struct ns__someElement data;
  ... // populate the data
  std::stringstream ss;
  soap->os = &ss;
  // serialize ns__someElement structure as XML
  if (soap_write_ns__someElement(soap, &data))
    ... // an error occurred
  soap->os = NULL;
  std::string xml = ss.str();
  ~~~
  */
  SOAP_SINK *os;
  /// User-definable maximum message length that is permitted to be received, zero means unlimited (the value is 2GB by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  ~~~
  */
  ULONG64 recv_maxlength;
  /// User-definable timeout to send or receive an entire message, positive timeout values are seconds, negative timeout values are microseconds, zero means no timeout (the value is 0 by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  ~~~

  @see `::soap::error`, `::soap::errnum`, `::soap::connect_timeout`, `::soap::accept_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::recv_maxlength`.
  */
  int transfer_timeout;         /* user-definable, when > 0, sets socket total transfer timeout in seconds, < 0 in usec */
  /// User-definable timeout to receive a packet of data, positive timeout values are seconds, negative timeout values are microseconds, zero means no timeout (the value is 0 by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  ~~~

  @see `::soap::error`, `::soap::errnum`, `::soap::connect_timeout`, `::soap::accept_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`.
  */
  int recv_timeout;
  /// User-definable timeout to send a packet of data, positive timeout values are seconds, negative timeout values are microseconds, zero means no timeout (the value is 0 by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  ~~~

  @see `::soap::error`, `::soap::errnum`, `::soap::connect_timeout`, `::soap::accept_timeout`, `::soap::recv_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`.
  */
  int send_timeout;
  /// User-definable timeout when waiting to accept a request from a client at the server-side with `::soap_accept` (or the C++ service class `accept` method), positive timeout values are seconds, negative timeout values are microseconds, zero means no timeout (the value is 0 by default)
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  int main()
  {
    struct soap *soap = soap_new();
    soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
    soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
    soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
    soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
    if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
    {
      while (1)
      {
        if (soap_valid_socket(soap_accept(soap)))
        {
          if (soap_serve(soap))
            soap_print_fault(soap, stderr);
        }
        else if (soap->errnum) // accept failed, try again after 1 second
        {
          soap_print_fault(soap, stderr);
          sleep(1);
        }
        else // accept timed out, quit looping
        {
          break;
        }
        soap_destroy(soap);
        soap_end(soap);
      }
    }
    soap_free(soap);
  }
  ~~~

  @see `::soap::error`, `::soap::errnum`, `::soap::connect_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`.
  */
  int accept_timeout;
  /// User-definable timeout when waiting to connect to a server at the client-side, positive timeout values are seconds, negative timeout values are microseconds, zero means no timeout (the value is 0 by default)
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  int main()
  {
    struct soap *soap = soap_new();
    soap->connect_timeout = 30;                 // 30 seconds max connect stall time
    soap->connect_retry = 4;                    // retry 4 more times (waiting 1, 2, 4, 8 seconds between retries)
    soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
    soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
    soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
    if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    {
      soap_print_fault(soap, stderr);
      if (soap->errnum == 0) // timed out, exit program
        exit(EXIT_FAILURE);
    }
    else
    {
      ... // success
    }
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
  }
  ~~~

  @see `::soap::error`, `::soap::errnum`, `::soap::connect_retry`, `::soap::accept_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`.
  */
  int connect_timeout;
  /// User-definable number of retries to attempt at the client side when connecting to a server fails with `::SOAP_TCP_ERROR`, using exponential backoff delays between reconnects, maxed at 32 seconds: 1, 2, 4, 8, 16, 32, 32, 32, ... seconds for retry values 1 to 8 respectively (the retry value is 0 by default, meaning no retry)
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  int main()
  {
    struct soap *soap = soap_new();
    soap->connect_timeout = 30;                 // 30 seconds max connect stall time
    soap->connect_retry = 4;                    // retry 4 more times (waiting 1, 2, 4, 8 seconds between retries)
    soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
    soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
    soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
    if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    {
      soap_print_fault(soap, stderr);
      if (soap->errnum == 0) // timed out, exit program
        exit(EXIT_FAILURE);
    }
    else
    {
      ... // success
    }
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
  }
  ~~~

  @see `::soap::error`, `::soap::errnum`, `::soap::connect_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`, `::SOAP_TCP_ERROR`.
  */
  int connect_retry;
  /// User-definable maximum XML and JSON nesting level permitted, initially set to `#SOAP_MAXLEVEL` (the value is 10000 by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->maxlevel = 10; // limit XML nesting depth to 10 (10000 by default)
  ~~~

  @see `::soap::recv_maxlength`, `::soap::maxlength`, `::soap::maxoccurs`.
  */
  unsigned int maxlevel;
  /// User-definable maximum string length parsed from XML and JSON, initially set to `#SOAP_MAXLENGTH`, zero or negative means unlimited (0 by default)
  /**
  The length of a string is the number of characters it contains.  Multi-byte strings with UTF-8 content (enabled with `#SOAP_C_UTFSTRING`) contain up to the specified number of multi-byte characters.  The byte length depends on the UTF-8 encoding.  The specified limit applies to strings that are not subject to string length schema validation constraints, to ensure that schema validation is not affected.

  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->maxlength = 256; // limit string lengths to 256 characters (unlimited by default)
  ~~~

  @see `::soap::recv_maxlength`, `::soap::maxlevel`, `::soap::maxoccurs`.
  */
  long maxlength;
  /// User-definable maximum array and container size (maximum item occurrence constraint) as parsed from XML and JSON, except when specifie XML schema validation constraints permit greater sizes, initially set to `#SOAP_MAXOCCURS` (the value is 100000 by default)
  /**
  The specified occurrence limit applies to arrays and containers that are not subject to occurrence validation constraints (`minOccurs` and `maxOccurs`, to ensure that schema validation is not affected.

  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->maxoccurs = 100; // limit arrays and containers to 100 items (100000 by default)
  ~~~

  @see `::soap::recv_maxlength`, `::soap::maxlevel`, `::soap::maxlength`.
  */
  size_t maxoccurs;
  /// User-definable socket `send` and `recv` flags, for example assign `MSG_NOSIGNAL` to disable sigpipe (the value is 0 by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->socket_flags = MSG_NOSIGNAL; // no sigpipe (this is not portable)
  ~~~

  Other ways to disable sigpipe:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->connect_flags = SO_NOSIGPIPE;
  soap->bind_flags = SO_NOSIGPIPE;
  ~~~

  ~~~{.cpp}
  #include <signal.h>
  void sigpipe_handle(int) { }

  signal(SIGPIPE, sigpipe_handle);
  ~~~

  @see `::soap::connect_flags`, `::soap::bind_flags`, `::soap::accept_flags`.
  */
  int socket_flags;
  /// User-definable `setsockopt` level `SOL_SOCKET` flags when connecting `::soap::socket` to a server (the value is 0 by default)
  /**
  @par Examples:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->connect_flags = SO_LINGER;
  soap->linger_time = 5;
  ~~~

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->connect_flags = SO_BROADCAST;
  ~~~

  @see `::soap::socket_flags`, `::soap::bind_flags`, `::soap::accept_flags`.
  */
  int connect_flags;
  /// User-definable `setsockopt` level `SOL_SOCKET` flags when binding `::soap::master` socket (the value is 0 by default)
  /**
  @par Example

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->bind_flags = SO_REUSEADDR; // immediate port reuse when binding
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG))
    ... // success
  ~~~

  @see `::soap::socket_flags`, `::soap::connect_flags`, `::soap::accept_flags`, `#WITH_IPV6`, `#WITH_IPV6_V6ONLY`, `#SOAP_IO_UDP`, `::soap::bind_inet6`, `::soap::bind_v6only`, `::soap::rcvbuf`, `::soap::sndbuf`, `::soap::master`.
  */
  int bind_flags;
  /// User-definable flag, when nonzero uses `AF_INET6` instead of `PF_UNSPEC` when binding the `::soap::master` socket in `::soap_bind` (or the C++ service class `bind` method), to remap IPv4 to IPv6 addresses, meaningful only when used with `#WITH_IPV6`
  /**
  @par Example:

  ~~~{.cpp}
  // requires compilation with compile-time flag -D WITH_IPV6
  struct soap *soap = soap_new();
  soap->bind_inet6 = 1;
  soap->bind_v6only = 1;
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)) // IPv6 address binding
    ... // success
  ~~~

  @see `::soap::bind_v6only`.

  @note Requires compilation with `#WITH_IPV6`.
  */
  short bind_inet6;
  /// User-definable flag, when nonzero enables `IPPROTO_IPV6` `setsockopt` `IPV6_V6ONLY` when binding the `::soap::master` socket with `::soap_bind` (or the C++ service class `bind` method), meaningful only when used with `#WITH_IPV6`
  /**
  @par Example:

  ~~~{.cpp}
  // requires compilation with compile-time flag -D WITH_IPV6
  struct soap *soap = soap_new();
  soap->bind_inet6 = 1;
  soap->bind_v6only = 1;
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)) // IPv6 address binding
    ... // success
  ~~~

  @see `::soap::bind_inet6`.

  @note Requires compilation with `#WITH_IPV6`.
  */
  short bind_v6only;
  /// User-definable `setsockopt` level `SOL_SOCKET` flags (0 by default), when nonzero sets the `::soap::socket` flags when accepting a request with `::soap_accept` (or the C++ service class `accept` method)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->accept_flags = SO_NOSIGPIPE; // no sigpipe (this is not portable)
  ~~~

  Other ways to disable sigpipe:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->socket_flags = MSG_NOSIGNAL;
  ~~~

  ~~~{.cpp}
  #include <signal.h>
  void sigpipe_handle(int) { }

  signal(SIGPIPE, sigpipe_handle);
  ~~~

  @see `::soap::socket_flags`, `::soap::connect_flags`, `::soap::bind_flags`.
  */
  int accept_flags;
  /// User-definable linger time value, requires the `SO_LINGER` `setsockopt` flag value to be assigned to `::soap::socket_flags`
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->connect_flags = SO_LINGER;
  soap->linger_time = 5;
  ~~~
  */
  unsigned short linger_time;
  /// User-definable value to set `SO_RCVBUF` `setsockopt` (the value is `#SOAP_BUFLEN` by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->rcvbuf = 32768; // setsockopt SO_RCVBUF size
  ~~~
  */
  int rcvbuf;
  /// User-definable value to set `SO_SNDBUF` `setsockopt` (the value is `#SOAP_BUFLEN` by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->sndbuf = 32768; // setsockopt SO_SNDBUF size
  ~~~
  */
  int sndbuf;
  /// User-definable value to set `SO_KEEPALIVE` `setsockopt` (0 by default unless the `#SOAP_IO_KEEPALIVE` mode flag is set)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->tcp_keep_alive = 1;   // setsockopt SO_KEEPALIVE
  soap->tcp_keep_idle = 30;   // time in seconds the connection needs to remain idle before TCP starts sending keepalive probes
  soap->tcp_keep_intvl = 120; // time in seconds between individual keepalive probes
  soap->tcp_keep_cnt = 5;     // maximum number of keepalive probes TCP should send before dropping the connection
  ~~~

  HTTP keep-alive together with TCP `SO_KEEPALIVE` is enabled with:

  ~~~{.cpp}
  struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); // enable HTTP keep-alive
  soap->max_keep_alive = 50;  // 50 max keep-alive exchanges for SOAP_IO_KEEPALIVE (SOAP_MAXKEEPALIVE by default)
  soap->tcp_keep_idle = 30;   // time in seconds the connection needs to remain idle before TCP starts sending keepalive probes
  soap->tcp_keep_intvl = 120; // time in seconds between individual keepalive probes
  soap->tcp_keep_cnt = 5;     // maximum number of keepalive probes TCP should send before dropping the connection
  ~~~

  @see `#SOAP_IO_KEEPALIVE`, `::soap::tcp_keep_idle`, `::soap::tcp_keep_intvl` and `::soap::tcp_keep_cnt`.
  */
  int tcp_keep_alive;
  /// User-definable value to set `TCP_KEEPIDLE` `setsockopt` (the value is 0 by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->tcp_keep_alive = 1;   // setsockopt SO_KEEPALIVE
  soap->tcp_keep_idle = 30;   // time in seconds the connection needs to remain idle before TCP starts sending keepalive probes
  soap->tcp_keep_intvl = 120; // time in seconds between individual keepalive probes
  soap->tcp_keep_cnt = 5;     // maximum number of keepalive probes TCP should send before dropping the connection
  ~~~

  HTTP keep-alive with TCP `SO_KEEPALIVE` is enabled with:

  ~~~{.cpp}
  struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); // enable HTTP keep-alive
  soap->max_keep_alive = 50;  // 50 max keep-alive exchanges (SOAP_MAXKEEPALIVE by default)
  soap->tcp_keep_idle = 30;   // time in seconds the connection needs to remain idle before TCP starts sending keepalive probes
  soap->tcp_keep_intvl = 120; // time in seconds between individual keepalive probes
  soap->tcp_keep_cnt = 5;     // maximum number of keepalive probes TCP should send before dropping the connection
  ~~~

  @see `::soap::tcp_keep_alive`, `::soap::tcp_keep_intvl` and `::soap::tcp_keep_cnt`.
  */
  unsigned int tcp_keep_idle;
  /// User-definable value to set `TCP_KEEPINTVL` `setsockopt` (the value is 0 by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->tcp_keep_alive = 1;   // setsockopt SO_KEEPALIVE
  soap->tcp_keep_idle = 30;   // time in seconds the connection needs to remain idle before TCP starts sending keepalive probes
  soap->tcp_keep_intvl = 120; // time in seconds between individual keepalive probes
  soap->tcp_keep_cnt = 5;     // maximum number of keepalive probes TCP should send before dropping the connection
  ~~~

  HTTP keep-alive with TCP `SO_KEEPALIVE` is enabled with:

  ~~~{.cpp}
  struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); // enable HTTP keep-alive
  soap->max_keep_alive = 50;  // 50 max keep-alive exchanges (SOAP_MAXKEEPALIVE by default)
  soap->tcp_keep_idle = 30;   // time in seconds the connection needs to remain idle before TCP starts sending keepalive probes
  soap->tcp_keep_intvl = 120; // time in seconds between individual keepalive probes
  soap->tcp_keep_cnt = 5;     // maximum number of keepalive probes TCP should send before dropping the connection
  ~~~

  @see `::soap::tcp_keep_alive`, `::soap::tcp_keep_idle` and `::soap::tcp_keep_cnt`.
  */
  unsigned int tcp_keep_intvl;
  /// User-definable value to set `TCP_KEEPCNT` `setsockopt` (the value is 0 by default)
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->tcp_keep_alive = 1;   // setsockopt SO_KEEPALIVE
  soap->tcp_keep_idle = 30;   // time in seconds the connection needs to remain idle before TCP starts sending keepalive probes
  soap->tcp_keep_intvl = 120; // time in seconds between individual keepalive probes
  soap->tcp_keep_cnt = 5;     // maximum number of keepalive probes TCP should send before dropping the connection
  ~~~

  HTTP keep-alive with TCP `SO_KEEPALIVE` is enabled with:

  ~~~{.cpp}
  struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); // enable HTTP keep-alive
  soap->max_keep_alive = 50;  // 50 max keep-alive exchanges (SOAP_MAXKEEPALIVE by default)
  soap->tcp_keep_idle = 30;   // time in seconds the connection needs to remain idle before TCP starts sending keepalive probes
  soap->tcp_keep_intvl = 120; // time in seconds between individual keepalive probes
  soap->tcp_keep_cnt = 5;     // maximum number of keepalive probes TCP should send before dropping the connection
  ~~~

  @see `::soap::tcp_keep_alive`, `::soap::tcp_keep_idle` and `::soap::tcp_keep_intvl`.
  */
  unsigned int tcp_keep_cnt;
  /* in_addr_t in6addr->sin6_scope_id IPv6 value */
  /// User-definable value to set `sockaddr_in6::sin6_scope_id` when nonzero
  /**
  This value is used by the engine for UDP multicast messaging at the client side, sets `sockaddr_in6::sin6_scope_id` to `::soap::ipv6_multicast_if` when nonzero.

  @see `#SOAP_IO_UDP`, `::soap::ipv4_multicast_if`, `::soap::ipv4_multicast_ttl`.
  */
  unsigned int ipv6_multicast_if;
  /// User-definable value to set `setsockopt` level `IPPROTO_IP` to `IP_MULTICAST_IF` when non-NULL
  /**
  This value is used by the engine for UDP multicast messaging at the client side, sets `setsockopt` level `IPPROTO_IP` to `IP_MULTICAST_IF` with value `::soap::ipv4_multicast_if` when non-NULL.

  @see `#SOAP_IO_UDP`, `::soap::ipv6_multicast_if`, `::soap::ipv4_multicast_ttl`.
  */
  char* ipv4_multicast_if;
  /// User-definable value to set `setsockopt` level `IPPROTO_IP` to `IP_MULTICAST_TTL` when nonzero
  /**
  This value is used by the engine for UDP multicast messaging at the client side, sets `setsockopt` level `IPPROTO_IP` to `IP_MULTICAST_TTL` with value `::soap::ipv4_multicast_ttl` when nonzero.

  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new(); 
  in_addr_t addr = inet_addr("1.2.3.4"); // optional 
  soap->send_timeout = 5;                // 5 seconds max socket delay
  soap->connect_flags = SO_BROADCAST;    // required for broadcast 
  soap->ipv4_multicast_if = &addr;       // optional for IPv4, see setsockopt IPPROTO_IP IP_MULTICAST_IF 
  soap->ipv6_multicast_if = addr;        // optional for IPv6, multicast sin6_scope_id 
  soap->ipv4_multicast_ttl = 1;          // optional, see setsockopt IPPROTO_IP, IP_MULTICAST_TTL 
  ~~~

  Refer to the socket options for `IPPROTO_IP` `IP_MULTICAST_TTL` to limit the lifetime of the packet.  Multicast datagrams are sent with a default value of 1, to prevent them to be forwarded beyond the local network.  This parameter can be set between 1 to 255.

  @see `#SOAP_IO_UDP`, `::soap::ipv6_multicast_if`, `::soap::ipv4_multicast_if`.
  */
  unsigned char ipv4_multicast_ttl;
  /// User-definable client address to bind to before connecting to a server, when non-NULL (Windows: n/a)
  /**
  When non-NULL, sets the client address specified as IPv4 or IPv6 or as a host address to bind to before connecting to a server.  The value is reset to NULL after connecting successfully or unsuccessfully to the server.

  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->client_addr = "12.34.56.78"; // client binds to 12.34.56.78 when connecting to endpoint
  soap->client_port = 8080; // optional, to also specify a port to bind to
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  @note Cannot be used on Windows platforms.

  @see `::soap::client_addr_ipv6`, `::soap::client_port`, `::soap::client_interface`.
  */
  const char *client_addr;
  /// User-definable client address to bind to before connecting to a server, when non-NULL, requires `#WITH_IPV6`
  /**
  When non-NULL and `soap::client_addr` is non-NULL and when connecting to a IPv6 server, sets the client address specified as IPv6 or as a host address to bind to before connecting to the server.  The value is reset to NULL after connecting successfully or unsuccessfully to the server.

  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->client_addr = "12.34.56.78"; // client binds to 12.34.56.78 when connecting to a IPv4 endpoint
  soap->client_addr_ipv6 = "2001:db8:0:1; // but the client binds 2001:db8:0:1 when connecting to a IPv6 endpoint
  soap->client_port = 8080; // optional, to also specify a port to bind to
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  @note Requires compilation with `#WITH_IPV6` or `#WITH_IPV6_V6ONLY`.

  @see `::soap::client_addr`, `::soap::client_port`, `::soap::client_interface`.
  */
  const char *client_addr_ipv6;
  /// User-definable client port to bind to before connecting to a server, when non-negative
  /**
  When non-negative, executes a `bind` with this port number before connecting to a server.  The value is reset to -1 after connecting successfully or unsuccessfully to the server.

  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  soap->client_port = 18000; // client binds to port 18000 when connecting to endpoint
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  @see `::soap::client_addr`, `::soap::client_addr_ipv6`, `::soap::client_interface`.
  */
  int client_port;
  /// User-definable client interface address to override when connecting to a server, when non-NULL (Windows: n/a)
  /**
  When non-NULL, sets the client address before connecting to a server.  The value is reset to NULL after connecting successfully or unsuccessfully to the server.  Does not bind the address, unlike `::soap::client_addr` and `::soap::client_addr_ipv6`.

  @note Cannot be used on Windows platforms.

  @see `::soap::client_addr`, `::soap::client_addr_ipv6`, `::soap::client_addr_ipv6`.
  */
  const char *client_interface;
  /// User-definable compression level for gzip compression (0=none, 1=fast to 9=best) default level is 6
  /**
  @see `#WITH_GZIP`, `#WITH_ZLIB`, `#SOAP_ENC_ZLIB`, `::soap::z_ratio_in`, `::soap::z_ratio_out`.
  */
  unsigned short z_level;
  /// The compression ratio = compressed.size/uncompressed.size of the compressed message received
  /**
  @see `#WITH_GZIP`, `#WITH_ZLIB`, `#SOAP_ENC_ZLIB`, `::soap::z_level`, `::soap::z_ratio_out`.
  */
  float z_ratio_in;
  /// The compression ratio = compressed.size/uncompressed.size of the compressed message sent
  /**
  @see `#WITH_GZIP`, `#WITH_ZLIB`, `#SOAP_ENC_ZLIB`, `::soap::z_level`, `::soap::z_ratio_in`.
  */
  float z_ratio_out;
  /// User-definable HTTP authorization bearer token value to be sent by the client, server side receives this string when the client sends authorization bearer
  /**
  @par Example:

  ~~~{.cpp}
  struct soap *soap = soap_new();
  if (soap_ssl_client_context(soap, ...))
    exit(EXIT_FAILURE);
  soap->bearer = "bearer-token";
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  ~~~{.cpp}
  int main()
  {
    struct soap *soap = soap_new();
    if (soap_ssl_server_context(soap, ...))
      exit(EXIT_FAILURE);
    ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
  }
  int ns__webmethod(struct soap *soap, ...)
  {
    if (!soap->bearer || strcmp(soap->bearer, "bearer-token"))
      return 401; // Unauthorized
    ... // process the request data and populate the response data
    return SOAP_OK;
  }
  ~~~

  @see `::soap_ssl_client_context`, `::soap_ssl_server_context`, `::soap::userid`, `::soap::passwd`, `::soap::ntlm_challenge` and the [HTTP digest plugin](../../httpda/html/httpda.html) documentation.
  */
  const char *bearer;
  /// User-definable HTTP and NTLM authorization user id string for HTTP basic and NTLM authentication by the client, server side receives this string when the client uses HTTP basic authentication, for HTTP digest authentication see the gSOAP HTTP digest authentication plugin
  /**
  @par Examples:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  if (soap_ssl_client_context(soap, ...))
    exit(EXIT_FAILURE);
  soap->userid = "user-id"; // HTTP basic auth credential
  soap->passwd = "user-pw"; // HTTP basic auth credential
  soap->authrealm = "server-domain"; // if known in advance, see example below
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  if (soap_ssl_client_context(soap, ...))
    exit(EXIT_FAILURE);
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...) == 401) // Unauthorized, got soap->authrealm
  {
    soap->userid = "user-id"; // HTTP basic auth credential
    soap->passwd = "user-pw"; // HTTP basic auth credential
    if (soap_call_ns__webmethod(soap, endpoint, NULL, ...)) // try again
      soap_print_fault(soap, stderr);
    else
      ... // success
  }
  else
  {
    ... // success
  }
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  ~~~{.cpp}
  #include "soapH.h"
  int main()
  {
    struct soap *soap = soap_new();
    if (soap_ssl_server_context(soap, ...))
      exit(EXIT_FAILURE);
    ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
  }
  int ns__webmethod(struct soap *soap, ...)
  {
    if (!soap->userid ||
        !soap->passwd ||
        !soap->authrealm ||
        strcmp(soap->userid, "user-id") ||
        strcmp(soap->passwd, "user-pw") ||
        strcmp(soap->authrealm, "server-domain"))
      return 401; // Unauthorized
    ... // process the request data and populate the response data
    return SOAP_OK;
  }
  ~~~

  @see `::soap::passwd`, `::soap::authrealm`, `::soap_ssl_client_context`, `::soap::bearer`, `::soap::ntlm_challenge`, `::soap::http_extra_header`, `::soap::proxy_userid`, `::soap::proxy_passwd` and the [HTTP digest plugin](../../httpda/html/httpda.html) documentation.
  */
  const char *userid;
  /// User-definable HTTP and NTLM authorization password string required for HTTP basic and NTLM authentication by the client, server side receives this string when the client uses HTTP basic authentication
  /**
  @see `::soap::userid`, `::soap::authrealm`.
  */
  const char *passwd;
  /// The HTTP and NTLM authorization realm/domain string received by the client with the `WWW-Authenticate` or `Proxy-Authenticate` HTTP headers, user-definable on the server side to send a `WWW-Authenticate` header to require authentication (service operation should return 401 to respond with "Unauthorized"), also serves as NTLM domain value
  /**
  @see `::soap::userid`, `::soap::passwd`.
  */
  const char *authrealm;
  /// User-definable NTLM authentication challenge key string
  /**
  @par Examples:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...) == 401) // Unauthorized, got soap->authrealm with NTLM domain
  {
    soap->userid = "user-id"; // NTLM auth credential
    soap->passwd = "user-pw"; // NTLM auth credential
    if (soap_call_ns__webmethod(soap, endpoint, NULL, ...)) // try again
      soap_print_fault(soap, stderr);
    else
      ... // success
  }
  else
  {
    ... // success
  }
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  soap->ntlm_challenge = ""; // pretend we've been challenged
  soap->userid = "user-id";  // NTLM credential
  soap->passwd = "user-pw";  // NTLM credential
  soap->authrealm = "server-domain"; // NTLM domain
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  @note Requires compilation with `#WITH_NTLM` and linking libntlm available at http://www.nongnu.org/libntlm for non-Windows platforms.

  @see `::soap::bearer`, `::soap::userid`, `::soap::passwd`, `::soap::proxy_userid`, `::soap::proxy_passwd`.
  */
  const char *ntlm_challenge;
  /// User-definable proxy host name string which should be set to connect through an HTTP proxy
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  soap->proxy_host = "proxy-domain-or-IP";
  soap->proxy_port = 3128;
  soap->proxy_userid = "proxy-id";
  soap->proxy_passwd = "proxy-pw";
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  @see `::soap::bearer`, `::soap::userid`, `::soap::passwd`, `::soap::ntlm_challenge`, `::soap::http_extra_header`, and the [HTTP digest plugin](../../httpda/html/httpda.html) documentation.
  */
  const char *proxy_host;
  /// User-definable proxy port which should be set to connect through an HTTP proxy (the value is 8080 by default)
  /**
  @see `::soap::proxy_host`.
  */
  int proxy_port;
  /// User-definable proxy authorization user id string to authenticate and connect to an HTTP proxy
  /**
  @see `::soap::proxy_host`.
  */
  const char *proxy_userid;
  /// User-definable proxy authorization password string to authenticate and connect to an HTTP proxy
  /**
  @see `::soap::proxy_host`.
  */
  const char *proxy_passwd;
  /// The `X-Forwarding-For` HTTP header string value received
  const char *proxy_from;
  /// The IPv4 address in numeric form of the client as received on the server side by `::soap_accept` (or the C++ service class `accept` method), possibly set to zero when `#WITH_IPV6` is used
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  int main()
  {
    struct soap *soap = soap_new();
    soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
    ... // context initializations
    if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
    {
      while (1)
      {
        if (soap_valid_socket(soap_accept(soap)))
        {
          printf("Client host = %s port = %d path = %s ip = %d.%d.%d.%d\n", soap->host, soap->port, soap->path, soap->ip >> 24, (soap->ip >> 16) & 0xFF, (soap->ip >> 8) & 0xFF, soap->ip & 0xFF);
          if (soap_serve(soap))
            soap_print_fault(soap, stderr);
        }
        else if (soap->errnum) // accept failed, try again after 1 second
        {
          soap_print_fault(soap, stderr);
          sleep(1);
        }
        else // accept timed out, quit looping
        {
          break;
        }
        soap_destroy(soap);
        soap_end(soap);
      }
    }
    soap_free(soap);
  }
  ~~~

  @see `::soap::host`, `::soap::port`, `::soap::ip6`, `::soap_bind`, `::soap_accept`, `::soap_ssl_accept`, `::soap_serve`.
  */
  unsigned int ip;
  /// The IPv6 address in numeric form (upper ip6[0] to lower ip6[3]) of the client as received on the server side by `::soap_accept` (or the C++ service class `accept` method), requires `#WITH_IPV6`
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  int main()
  {
    struct soap *soap = soap_new();
    soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
    ... // context initializations
    if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
    {
      while (1)
      {
        if (soap_valid_socket(soap_accept(soap)))
        {
          printf("Client host = %s port = %d path = %s ", soap->host, soap->port, soap->path);
          if (soap->ip)
            printf("ip4 = %d.%d.%d.%d\n", soap->ip >> 24, (soap->ip >> 16) & 0xFF, (soap->ip >> 8) & 0xFF, soap->ip & 0xFF);
          else
            printf("ip6 = %.8x%.8x%.8x%.8x\n", soap->ip6[0], soap->ip6[1], soap->ip6[2], soap->ip6[3]);
          if (soap_serve(soap))
            soap_print_fault(soap, stderr);
        }
        else if (soap->errnum) // accept failed, try again after 1 second
        {
          soap_print_fault(soap, stderr);
          sleep(1);
        }
        else // accept timed out, quit looping
        {
          break;
        }
        soap_destroy(soap);
        soap_end(soap);
      }
    }
    soap_free(soap);
  }
  ~~~

  @note Requires compilation with `#WITH_IPV6` or `#WITH_IPV6_V6ONLY`.

  @see `::soap::host`, `::soap::port`, `::soap::ip`, `::soap_bind`, `::soap_accept`, `::soap_ssl_accept`, `::soap_serve`.
  */
  unsigned int ip6[4];
  /// The client port connected to as received on the server side by `::soap_accept` (or the C++ service class `accept` method)
  /**
  @see `::soap::ip`, `::soap::ip6`.
  */
  int port;
  /// The endpoint URL as received on the server side
  /**
  On the server side, the URL endpoint string is extracted from the HTTP header by `::soap::fparse` called by `::soap_begin_recv` and consists of the concatenated string of `::soap::host`, `::soap::port`, and `::soap::path` to form a valid URL.

  @see `::soap::ip`, `::soap::ip6`, `::soap::host`, `::soap::port`, `::soap::path`, `::soap_query`.
  */
  char endpoint[SOAP_TAGLEN];
  /// The host IP address of the client, as received on the server side
  /**
  On the server side, the host string is the IPv4 or IPv6 address, depending on `#WITH_IPV6`.  On the client side, the host string is extracted from `::soap::endpoint`, i.e. the endpoint URL of the server.

  @see `::soap::ip`, `::soap::ip6`, `::soap::endpoint`, `::soap::port`, `::soap::path`, `::soap_query`.
  */
  char host[SOAP_TAGLEN];
  /// The client request path as received on the server side
  /**
  The URL path string is extracted from the HTTP header by `::soap::fparse` called by `::soap_begin_recv` and starts with a "/".

  @see `::soap::ip`, `::soap::ip6`, `::soap::endpoint`, `::soap::host`, `::soap::port`, `::soap_query`.
  */
  char path[SOAP_TAGLEN];
  /// User-definable string to override the host name or IP address in the HTTP header when connecting at the client side
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new1(SOAP_IO_CHUNK);
  ... // initializations
  soap->override_host = "server-domain-or-IP";
  soap->override_port = 1234;
  if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  ~~~

  @see `::soap::override_port`.
  */
  const char *override_host;
  /// User-definable port number to override the port address in the HTTP header when connecting at the client side
  /**
  @see `::soap::override_host`.
  */
  int override_port;
  /// The CORS Origin HTTP header string value received
  /**
  CORS is automatic at the server side.  The server internally calls the `::soap::fopt` callback to serve the OPTION method CORS request, which returns HTTP 200 OK with CORS headers.  The default value of the CORS Access-Control-Allow-Origin header is "*".

  At the client side, CORS requires the HTTP OPTIONS method with CORS headers.  Use the following code to send HTTP OPTIONS with CORS headers to a server:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  soap->origin = "http://example.com";
  soap->cors_method = "POST"; // request method is POST
  soap->cors_header = "...";  // list of comma-separated request headers (may be omitted)
  if (soap_connect_command(soap, SOAP_OPTIONS, endpoint, NULL)
   || soap_end_send(soap)
   || soap_recv_empty_response(soap))
    soap_print_fault(soap, stderr);
  else if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
    soap_print_fault(soap, stderr);
  else
    ... // success
  ~~~

  @see `::soap::cors_origin`, `::soap::cors_allow`, `::soap::cors_method`, `::soap::cors_header`, `::soap::cors_methods`, `::soap::cors_header`.
  */
  const char *origin;
  /// The CORS Access-Control-Allow-Origin HTTP header string value received or the user-definable value to be returned by the server when set
  /**
  @see `::soap::origin`.
  */
  const char *cors_origin;
  /// User-definable CORS Access-Control-Allow-Origin HTTP header string default value (the value is `*` by default)
  /**
  CORS is automatic at the server side.  At the client side, CORS requires the HTTP OPTIONS method with CORS headers.

  @see `::soap::origin`.
  */
  const char *cors_allow;
  /// The CORS Access-Control-Request-Method HTTP header string received
  /**
  CORS is automatic at the server side.  At the client side, CORS requires the HTTP OPTIONS method with CORS headers.

  @see `::soap::origin`.
  */
  const char *cors_method;
  /// The CORS Access-Control-Request-Headers HTTP header string received
  /**
  CORS is automatic at the server side.  At the client side, CORS requires the HTTP OPTIONS method with CORS headers.

  @see `::soap::origin`.
  */
  const char *cors_header;
  /// User-definable CORS Access-Control-Request-Methods HTTP header string to be returned by the server
  /**
  CORS is automatic at the server side.  At the client side, CORS requires the HTTP OPTIONS method with CORS headers.

  @see `::soap::origin`.
  */
  const char *cors_methods;
  /// User-definable CORS Access-Control-Request-Headers HTTP header string to be returned by the server
  /**
  CORS is automatic at the server side.  At the client side, CORS requires the HTTP OPTIONS method with CORS headers.

  @see `::soap::origin`.
  */
  const char *cors_headers;
  /// User-definable floating point format string (`%.9G` by default, the printed format should not exceed 1023 bytes)
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  soap->float_format = "%g";
  soap->double_format = "%g";
  soap->long_double_format = "%Lg";
  ~~~

  */
  const char *float_format;
  /// User-definable double floating point format string (`%.17lG` by default, the printed format should not exceed 1023 bytes)
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  soap->float_format = "%g";
  soap->double_format = "%g";
  soap->long_double_format = "%Lg";
  ~~~

  */
  const char *double_format;
  /// User-definable long double floating point format string (NULL by default and defined by the long_double.c custom serializer, the printed format should not exceed 1023 bytes)
  /**
  @par Example:

  ~~~{.cpp}
  #include "soapH.h"
  struct soap *soap = soap_new();
  soap->float_format = "%g";
  soap->double_format = "%g";
  soap->long_double_format = "%Lg";
  ~~~

  @note Requires `#import "custom/long_double.h" in the .h file for soapcpp2 to generate code that supports `long double` and compiling <i>`gsoap/custom/long_double.c`</i> to serialize `long double`.
  */
  const char *long_double_format;
  /// User-definable format string to generate DIME content IDs
  /**
  The format string should contain a `%d` or `%x`.  The default format string is "cid:id%d".
  */
  const char *dime_id_format;
  /// DOM tree received
  /**
  This pointer points to the DOM tree received when `#SOAP_XML_DOM` mode is enabled and the engine is configured with `#WITH_DOM`.

  @see `#WITH_DOM`, `#SOAP_XML_DOM`.
  */
  struct soap_dom_element *dom;
  /// DIME attachments received
  /**
  This structure contains a linked list of DIME attachments received.
  */
  struct soap_dime dime;
  /// MIME attachments received
  /**
  This structure contains a linked list of MIME attachments received.
  */
  struct soap_mime mime;
  /// Internal index that keeps track of the current position in the `::soap::buf` buffer after receiving data into the buffer
  size_t bufidx;
  /// Internal index that keeps track of the length of the data available in the `::soap::buf` buffer, does not exceed `#SOAP_BUFLEN`
  size_t buflen;
  /// Internal buffer with partial data received or partial data to be sent, where the data occupies `::soap::buflen` bytes
  char buf[SOAP_BUFLEN];
  /// Internal buffer to hold short messages, URLs and HTTP/MIME header lines, must have at least `#SOAP_TMPLEN` = 1024 bytes of space allocated
  char msgbuf[SOAP_TMPLEN];
  /// Internal buffer to hold temporary strings such as string representations of primitive values, XML tag names, HTTP header lines and so on, must have at least `#SOAP_TMPLEN` = 1024 bytes of space allocated
  char tmpbuf[SOAP_TMPLEN];
  /// Message length counter value of the message received and counter value of the HTTP content length header to send a message
  /**
  @see `::soap_begin_count`, `::soap_end_count`, `#SOAP_IO_LENGTH`, `#SOAP_IO_CHUNK`.
  */
  ULONG64 count;
  /// The HTTP content length header value received or 0 when HTTP transfer encoding is chunked
  ULONG64 length;
  /// Callback that populates and then sends HTTP headers from the client-side to a connected HTTP server
  /**
  @ingroup group_callbacks
  This callback is called at the client side by the engine to send HTTP headers to the connected server.  The parameters `host`, `port`, and `path` were micro-parsed from the `endpoint` prior to passing them to this callback.  Parameter `action` is the SOAP Action header.  Parameter `count` is the length of the HTTP body with the message or 0 when HTTP chunking is used.  This callback sends the headers with POST by default, or when `::soap::status` == `#SOAP_POST` or `::soap::status` == `#SOAP_POST_FILE`. Alternatively, sends the HTTP headers with GET when `::soap::status` == `#SOAP_GET`, PATCH when `::soap::status` == `#SOAP_PATCH`, PUT when `::soap::status` == `#SOAP_PUT`, DELETE when `::soap::status` == `#SOAP_DEL`, CONNECT when `::soap::status` == `#SOAP_CONNECT`, HEAD when `::soap::status` == `#SOAP_HEAD` or OPTIONS when `::soap::status` == `#SOAP_OPTIONS`.  Extra HTTP headers are added when `::soap::http_extra_header` is set to one or more header lines separated by CRLF.  When redefining this callback, use function `::soap_send` to write the header contents.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap:fpost` is `http_post`.
  @param soap `::soap` context
  @param endpoint URL of the endpoint connected to (string)
  @param host URL host of the endpoint connected to (string)
  @param port URL port of the endpoint connected to (int)
  @param path URL path of the endpoint connected to (string)
  @param action SOAP Action or NULL (string)
  @param count HTTP content-length or 0 for HTTP chunked transfers (size_t)
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fpost)(struct soap *soap, const char *endpoint, const char *host, int port, const char *path, const char *action, ULONG64 count);
  /// Callback that populates and then sends HTTP headers from the server-side to a connected client
  /**
  @ingroup group_callbacks
  This callback is called at the server side by the engine to send the HTTP headers to the connected client.  The parameter `status` should be an HTTP status error code or `#SOAP_OK` (200 OK) or `#SOAP_HTML` or `#SOAP_FILE`.  Using `#SOAP_HTML` sets the content-type header to `text/html; charset=utf-8`.  Using `#SOAP_FILE` sets the content-type header to the value of `::soap::http_content`.  Extra HTTP headers are added when `::soap::http_extra_header` is set to one or more header lines separated by CRLF.  When redefining this callback, use function `::soap_send` to write the header contents.  Returns `#SOAP_OK` or a `::soap_status` error code.  The built-in function assigned to `::soap::fresponse` is `http_response`.
  @param soap `::soap` context
  @param status HTTP status code (> 100) or `#SOAP_OK` (200 OK), or `#SOAP_HTML` or `#SOAP_FILE`
  @param count HTTP content-length or 0 for HTTP chunked transfers
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fresponse)(struct soap *soap, int status, ULONG64 count);
  /// Callback that sends a single HTTP header given a key-value pair
  /**
  @ingroup group_callbacks
  This callback is called by `::soap::fpost` and `::soap::fresponse` to send an HTTP header with a key and an optional value.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fposthdr` is `http_post_header`.
  @param soap `::soap` context
  @param key HTTP header key (string)
  @param val optional HTTP header value (string), omitted when NULL
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fposthdr)(struct soap *soap, const char *key, const char *val);
  /// Callback that reads and parses HTTP and MIME headers
  /**
  @ingroup group_callbacks
  This callback is called by the engine (as a client or server) to read and parse HTTP headers or MIME headers.  When redefined, this function should read or skip the entire HTTP header to reach the message body.  Function `::soap_getline` is used by this callback to read each header line into an internal buffer `::soap::msgbuf` with `::soap_getline(soap, soap->msgbuf, sizeof(soap->msgbuf))`.  Returns `#SOAP_OK`, or a gSOAP error code.  The built-in function assigned to `::soap::fparse` is `http_parse`.
  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code

  @see `::soap::fparsehdr`.
  */
  int (*fparse)(struct soap *soap);
  /// Callback that consumes an HTTP header that consists of a key-value pair
  /**
  @ingroup group_callbacks
  This callback is called by `::soap::fparse`, consumes an HTTP header that is split in a key-value pair and updates the `::soap` context state accordingly.  The context is updated with the HTTP header information received, but HTTP headers are not literally retained by the engine.  Returns `#SOAP_OK` or `#SOAP_STOP` to prevent further reading of the HTTP body, or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fparsehdr` is `http_parse_header`.

  @note This callback can be used to parse (custom) HTTP headers, which is typically done by plugins.

  @see `::soap::fparse`, `::soap::http_extra_header`, `::soap::user`.

  @par Example:

  ~~~{.cpp}
  int main()
  {
    struct soap *soap = soap_new();
    soap->user = (void*)soap->fparsehdr; // to call the engine's fparsehdr()
    soap->fparsehdr = parse_header;
    ... //
  }

  int parse_header(struct soap *soap, const char *key, const char *val)
  {
    ... // use key and val, then pass the key-val to the engine:
    return ((int(*)(struct soap*, const char*, const char*))(soap->user))(soap, key, val);
  }
  ~~~

  @param soap `::soap` context
  @param key HTTP header key received (non-NULL string)
  @param val HTTP header value received (non-NULL string) or an empty string
  @returns `#SOAP_OK`, `#SOAP_STOP` or a `::soap_status` error code
  */
  int (*fparsehdr)(struct soap *soap, const char *key, const char *val);
  /// Callback to implement logic at the server-side to serve responses to HTTP GET requests from clients
  /**
  @ingroup group_callbacks
  This callback is called by the service dispatcher when an HTTP GET request is pending.  Redefine this callback to respond to HTTP GET requests with content, see the `::http_get` HTTP GET plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fget` is the internal static function `http_get` that returns the `#SOAP_GET_METHOD` error.

  @see `::http_get`, `::soap::user`.

  @par Example

  ~~~{.cpp}
  int main()
  {
    struct soap *soap = soap_new();
    soap->user = ... // set this to pass data to the callback
    soap->fget = http_get;
    ... //
  }

  // return a service.wsdl WSDL document on HTTP GET service path ending in ?wsdl
  int http_get(struct soap * soap)
  {
    FILE *fd = NULL;
    char *s = strchr(soap->path, '?'); // soap->path has the URL path of soap->endpoint
    if (s == NULL || strcmp(s, "?wsdl") != 0) 
      return SOAP_GET_METHOD;  // return GET method not available error
    fd = fopen("service.wsdl", "r"); // open WSDL file to copy 
    if (!fd) 
      return 404; // return HTTP not found error 
    soap->http_content = "text/xml"; // HTTP header with text/xml content 
    soap_response(soap, SOAP_FILE); 
    while (1)
    {
      size_t r = fread(soap->tmpbuf, 1, sizeof(soap->tmpbuf), fd); 
      if (r == 0) 
        break; 
      if (soap_send_raw(soap, soap->tmpbuf, r)) 
        break; // can't send
    } 
    fclose(fd); 
    return soap_end_send(soap); 
  }
  ~~~

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fget)(struct soap *soap);
  /// Callback to implement logic at the server-side to serve responses to HTTP PUT requests from clients
  /**
  @ingroup group_callbacks
  This callback is called by the service dispatcher when an HTTP PUT request is pending.  Redefine this callback to respond to HTTP PUT requests, see the `::http_post` HTTP POST plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` error code.  The built-in function assigned to `::soap::fput` is the internal static function `http_put` that returns the `#SOAP_PUT_METHOD` error.

  @see `::http_post`, `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fput)(struct soap *soap);
  /// Callback to implement logic at the server-side to serve responses to HTTP PATCH requests from clients
  /**
  @ingroup group_callbacks
  This callback is called by the service dispatcher when an HTTP PATCH request is pending.  Redefine this callback to respond to HTTP PATCH requests, see the `::http_post` HTTP POST plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` error code.  The built-in function assigned to `::soap::fpatch` is the internal static function `http_patch` that returns the `#SOAP_PATCH_METHOD` error.

  @see `::http_post`, `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fpatch)(struct soap *soap);
  /// Callback to implement logic at the server-side to serve responses to HTTP DELETE requests from clients
  /**
  @ingroup group_callbacks
  This callback is called by the service dispatcher when an HTTP DELETE request is pending.  Redefine this callback to respond to HTTP DELETE requests, see the `::http_post` HTTP POST plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fdel` is the internal static function `http_del` that returns the `#SOAP_DEL_METHOD` error.

  @see `::http_post`, `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fdel)(struct soap *soap);
  /// Callback to implement logic at the server-side to serve responses to HTTP OPTION requests from clients
  /**
  @ingroup group_callbacks
  Called by the service dispatcher when an HTTP OPTION request is pending.  Redefine this callback to respond to HTTP OPTION requests, see the `::http_post` HTTP POST plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fopt` is the internal static function `http_200` that returns HTTP 200 OK.

  @see `::http_post`, `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fopt)(struct soap *soap);
  /// Callback to implement logic at the server-side to serve responses to HTTP HEAD requests from clients
  /**
  @ingroup group_callbacks
  This callback is called by the service dispatcher when an HTTP HEAD request is pending.  Redefine this callback to respond to HTTP HEAD requests more specifically.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fhead` is the internal static function `http_200` that returns HTTP 200 OK.

  @see `::http_opt`, `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fhead)(struct soap *soap);
  /// Callback to implement logic at the server-side to handle HTML forms, such as done by the callbacks provided by the HTTP FORM handler plugin
  /**
  @ingroup group_callbacks
  This callback is called by the HTTP FORM handler plugin to parse HTML forms received with HTTP POST and PUT requests, see the <i>`;:http_form`</i> HTTP FORM plugin for more details.  The HTTP body with the form data should be parsed by this callback, otherwise HTTP keep-alive messages will end up out of sync as a result of the current position not being advanced to the end of the HTTP body.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fform`.

  @see `::http_post`, `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fform)(struct soap *soap);
  /// Callback to inspect the SOAP Header received before the rest of the message with the SOAP Body is consumed
  /**
  @ingroup group_callbacks
  This callback is called immediately after parsing a SOAP Header into the `::soap::header` structure.  The SOAP Header structure `::soap::header` can be inspected by this function and verified or rejected before the rest of the message with the SOAP Body is consumed.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fheader`.

  @see `::http_form`, `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fheader)(struct soap *soap);
  /// Callback to catch unrecognized XML elements and overrides `#SOAP_XML_STRICT` validation errors for these
  /**
  @ingroup group_callbacks
  This callback is called when an unrecognized XML element was encountered on the input that could be ignored depending on some specified logic.  The `tag` parameter is the offending XML element tag name string.  The callback should return `#SOAP_OK` to ignore the element or return an `::soap_status` error code such as `#SOAP_TAG_MISMATCH` to trigger a validation error.  This callback also overrides `mustUnderstand` attributes on unrecognized SOAP Header elements that normally raise faults.  It is strongly recommended that the callback returns `#SOAP_MUSTUNDERSTAND` when `::soap::mustUnderstand` != `0`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fignore`.

  @note This callback was not called in gSOAP versions prior to 2.8.54 when `#SOAP_XML_STRICT` is set.

  @see `::soap::user`.

  @par Example:

  ~~~{.cpp}
  int main()
  {
    struct soap *soap = soap_new();
    soap->fignore = ignore;
    ... //
  }

  int ignore(struct soap *soap, const char *tag) 
  { 
    // do not ignore mustUnderstand="true" 
    if (!soap->mustUnderstand)
    {
      // tags <ns:someElement> can be safely ignored
      if (soap_match_tag(soap, tag, "ns:someElement") == SOAP_OK)
        return SOAP_OK;
    }
    return SOAP_MUSTUNDERSTAND; 
  }
  ~~~

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fignore)(struct soap *soap, const char *tag);
  /// Callback to validate strings against XML regex patterns
  /**
  @ingroup group_callbacks
  This callback is called to validate a string against an XML regex pattern.  Patterns use XML schema regex syntax.  This callback allows user-defined pattern validation that is normally disabled.  Returns `#SOAP_OK` when the string matches the pattern or `#SOAP_TYPE` when the string does not match.  No built-in function is assigned to `::soap::fsvalidate`.

  @see `::soap::fwvalidate`,`::soap::user`.

  @param soap `::soap` context
  @param pattern regex in XML schema syntax
  @param string to match pattern against
  @returns `#SOAP_OK` (match) or `#SOAP_TYPE` (mismatch) or a `::soap_status` error code
  */
  int (*fsvalidate)(struct soap *soap, const char *pattern, const char *string);
  /// Callback to validate wide strings against XML regex patterns
  /**
  @ingroup group_callbacks
  This callback is called to validate a wide string against an XML regex pattern.  Patterns use XML schema regex syntax.  This callback allows user-defined pattern validation that is normally disabled.  Returns `#SOAP_OK` when the string matches the pattern or `#SOAP_TYPE` when the string does not match.  No built-in function is assigned to `::soap::fwvalidate`.

  @see `::soap::fsvalidate`, `::soap::user`.

  @param soap `::soap` context
  @param pattern regex in XML schema syntax
  @param string to match pattern against
  @returns `#SOAP_OK` (match) or `#SOAP_TYPE` (mismatch) or a `::soap_status` error code
  */
  int (*fwvalidate)(struct soap *soap, const char *pattern, const wchar_t *string);
  /// Callback to inspect or override fault code or fault string messages
  /**
  @ingroup group_callbacks
  This callback is called by the engine when an error is raised to allow inspection or overriding of the fault code or fault string messages before the error is reported or transmitted.  No built-in function is assigned to `::soap::fseterror`.

  @see `::soap::user`.

  @param soap `::soap` context
  @param faultcode pointer to a string with the fault code message or NULL, can be reassigned
  @param faultstring pointer to a string with the fault string message or NULL, can be reassigned
  */
  void (*fseterror)(struct soap *soap, const char **faultcode, const char **faultstring);
  /// Callback that opens a socket connection to a server endpoint
  /**
  @ingroup group_callbacks
  This callback is called by the engine at the client-side by `::soap_connect` or `::soap_connect_command` to open a TCP or UDP connection to a server specified at an endpoint.  Parameters `host` and `port` are micro-parsed from `endpoint` before being passed to `::soap::fopen`.  Returns a valid socket or `#SOAP_INVALID_SOCKET` with a `::soap::error` set to a `::soap_status` (int) error code and `::soap::errnum` set to `errno` of the connection failure.  The built-in function assigned to `::soap::fopen` is `tcp_connect`.

  @see `::soap::faccept`, `::soap::user`.

  @param soap `::soap` context
  @param endpoint URL of the endpoint to connect to (string)
  @param host URL host of the endpoint to connect to (string)
  @param port URL port of the endpoint to connect to (int)
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  SOAP_SOCKET (*fopen)(struct soap *soap, const char *endpoint, const char *host, int port);
  /// Callback that waits for and accepts a socket connection requested by a client
  /**
  @ingroup group_callbacks
  This callback is called by `::soap_accept` (or the C++ service class `accept` method) to wait for and accept a socket connection requested by a client.  Returns a valid socket or `#SOAP_INVALID_SOCKET` when an error occurred and sets `::soap::error` to a `::soap_status` value.  The built-in function assigned to `::soap::faccept` is `tcp_accept`.

  @see `::soap::fopen`, `::soap::user`.

  @param soap `::soap` context
  @param sock master socket
  @param addr points to a `sockaddr` structure to be populated
  @param len points to the length of the `sockaddr` structure, the length may be reduced by the callback function with the actual size of the `sockaddr` structure populated
  @returns socket or `#SOAP_INVALID_SOCKET` if an error occurred
  */
  SOAP_SOCKET (*faccept)(struct soap *soap, SOAP_SOCKET sock, struct sockaddr *addr, int *len);
  /// Callback that closes the current socket connection
  /**
  @ingroup group_callbacks
  This callback is called by the engine at the client-side to close the current socket connection before a new socket connection is established.  This callback may be called multiple times (e.g. by the engine and by plugins) to close the same socket `::soap::socket`.  Checks internally if `::soap::socket` == `#SOAP_INVALID_SOCKET` before closing, which means that the socket was already closed.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fclose` is `tcp_disconnect`.

  @see `::soap::fopen`, `::soap::faccept`, `::soap::user`.

  @param soap `::soap` context
  @returns socket or `#SOAP_INVALID_SOCKET` if an error occurred
  */
  int (*fclose)(struct soap *soap);
  /// Callback that resolves a host name by address translation
  /**
  @ingroup group_callbacks
  This callback is called by `::soap_bind` (or the C++ service class `bind` method) at the server-side and by `::soap_connect` or `::soap_connect_command` at the client-side with a host `name` parameter to resolve to address `inaddr` by address translation.  When successful sets parameter `inaddr` and returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fresolve` is `tcp_gethost`.

  @see `::soap::user`.

  @param soap `::soap` context
  @param name host name (string)
  @param inaddr points to in_addr structure to set
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fresolve)(struct soap *soap, const char *name, struct in_addr *inaddr);
  /// Callback that overrides the client-side connecting operations
  /**
  @ingroup group_callbacks
  This callback is called by the engine to optionally override client-side connecting.  The parameters `host` and `port` were micro-parsed from the `endpoint` prior to passing them to this callback.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fconnect`.

  @see `::soap::user`.

  @param soap `::soap` context
  @param endpoint URL of the endpoint connected to (string)
  @param host URL host of the endpoint connected to (string)
  @param port URL port of the endpoint connected to (int)
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fconnect)(struct soap *soap, const char *endpoint, const char *host, int port);
  /// Callback that executes disconnect logic before closing
  /**
  @ingroup group_callbacks
  This callback is called by the engine `::soap_closesock` before the `::soap::fclose` callback is called to shutdown/disconnect.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fdisconnect`.

  @see `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fdisconnect)(struct soap *soap);
  /// Callback that closes a given socket
  /**
  @ingroup group_callbacks
  This callback is called to close a socket by the engine.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fclosesocket` is `tcp_closesocket`.

  @see `::soap::user`.

  @param soap `::soap` context
  @param sock socket to close
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fclosesocket)(struct soap *soap, SOAP_SOCKET sock);
  /// Callback that shuts down a given socket
  /**
  @ingroup group_callbacks
  This callback is called to shut down a socket by the engine.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fshutdownsocket` is `tcp_shutdownsocket`.

  @see `::soap::user`.

  @param soap `::soap` context
  @param sock socket to shut down
  @param how `SHUT_RD` (=0), `SHUT_WR` (=1) or `SHUT_RDWR` (=2)
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fshutdownsocket)(struct soap *soap, SOAP_SOCKET sock, int how);
  /// Callback that blocks until activity is detected on the `::soap::socket` or `::soap::master` socket, times out when `::soap::send_timeout` or `::soap::recv_timeout` are set.
  /**
  @ingroup group_callbacks
  This callback is called by the engine to wait for activity on the `::soap::socket` or `::soap::master` socket using `poll` or `select`.  Times out when `::soap::send_timeout` or `::soap::recv_timeout` are nonzero.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fpoll` is `::soap_poll`.

  @see `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fpoll)(struct soap *soap);
  /// Callback that receives bytes of data into the given buffer
  /**
  @ingroup group_callbacks
  This callback is called by the engine to receive (or read) data into a specified buffer `buf` and `len`.  The source for the data to read by this callback is `::soap::is` when non-NULL, `::soap::socket` when valid, or `::soap::recvfd`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::frecv` is `frecv`.

  @see `::soap::user`.

  @param soap `::soap` context
  @param buf buffer to fill with bytes to be read (string)
  @param len maximum size of the buffer (size_t)
  @returns the nonzero number of bytes that were placed in the buffer or 0 to indicate EOF was reached (size_t)
  */
  size_t (*frecv)(struct soap *soap, char *buf, size_t len);
  /// Callback that sends the given bytes of data
  /**
  @ingroup group_callbacks
  This callback is called by the engine to send (or write) data specified by `data` bytes of length `len`.  The sink for the data to be sent to is typically `::soap::socket`, `::soap::sendfd` or `::soap::os`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fsend` is `fsend`.

  @see `::soap::user`.

  @param soap `::soap` context
  @param data bytes to be send (string)
  @param len number of bytes to be send (size_t)
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fsend)(struct soap *soap, const char *data, size_t len);
  /// Callback executed by the engine at the server side immediately after a server operation successfully completed
  /**
  @ingroup group_callbacks
  This callback is called after each successful completion of a server operation in the server loop.  Executes immediately after sending the response to a client and before the next keep-alive server loop iteration when enabled with `#SOAP_IO_KEEPALIVE`.  This callback can be used to reclaim resources in the keep-alive server loop, for example managed memory can be reclaimed by calling `::soap_destroy` and `::soap_end` in that order and all deserialized and other dynamically-allocated data managed by the context will be deallocated.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fserveloop`.

  @see `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fserveloop)(struct soap *soap);
  /// Callback to override dynamic memory allocation and management
  /**
  @ingroup group_callbacks
  This callback can be used to override memory allocation and management done by `::soap_malloc` in C.  Memory allocated via this callback will not be managed and not be automatically released by the engine.  Instead, the application using this callback should release allocated memory.  All allocations done by `::soap_malloc` are replaced with a call to `::soap::fmalloc`.  However, no other allocations, such as `::soap_new` and `soap_new_T` for C++ classes, are affected.  This callback is therefore not useful for C++ applications.  Returns a pointer to dynamically allocated memory or NULL on failure to allocate.  No built-in function is assigned to `::soap::fmalloc`.

  @warning Deprecated since 2.8.72.  Define `#SOAP_MALLOC` and `#SOAP_FREE` instead.

  @see `::soap::user`.

  @param soap `::soap` context
  @param size number of bytes to allocate
  @returns pointer to allocated memory or NULL on failure to allocate (out of memory)
  */
  void *(*fmalloc)(struct soap *soap, size_t size);
  /// Callback to open a streaming DIME attachment for reading
  /**
  @ingroup group_callbacks
  This callback is called by the engine to start sending a streaming DIME attachment.  This callback opens a stream to start reading the attachment data to send.  The actual data stream will be read in chunks using the `::soap::fdimeread` callback until no more data is available and the `::soap::fdimereadclose` callback is called to close the stream.  The `handle` parameter contains the value of the `__ptr` member variable of the attachment struct/class with data (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members), which should be a pointer to specific information such as a file descriptor or a pointer to a some application-specific data to be passed to this callback.  Both the `__ptr` and `__size` members of the attachment struct/class should have been set by the application prior to the serialization of the message with attachments.  If the `__size` is zero and HTTP chunking is enabled (with `#SOAP_IO_CHUNK`), then chunked DIME attachments are sent, see `::soap::fdimeread`.  The `id`, `type` and `options` parameters are the `id` (optional ID), `type` (a MIME type) and `options` (DIME options are set with `::soap_dime_option`) of the attachment struct/class, respectively, of which at least one member should be non-NULL.  The callback should return the `handle` parameter value or another pointer value, which is passed as the new `handle` parameter to `::soap::fdimeread` and `::soap::fdimereadclose` callbacks.  When an error occurred in this callback, the callback should return NULL and set `::soap::error` to an error code, e.g. using `::soap_receiver_fault`.  The callback may return NULL and set `::soap::error` to `#SOAP_OK` when this specific DIME attachment should not to be streamed and the engine will simply skip it.

  @see `#SOAP_ENC_DIME`, `::soap::user`.

  @par Example:

  ~~~{.cpp}
  void * dime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *options)
  {
    return (void*)fopen((char*)handle, "rb");
  }
  size_t dime_read(struct soap *soap, void *handle, char *buf, size_t len)
  {
    return fread(buf, 1, len, (FILE*)handle);
  }
  void dime_read_close(struct soap *soap, void *handle)
  {
    fclose((FILE*)handle);
  }

  struct soap *soap = soap_new();
  soap->fdimereadopen = dime_read_open;
  soap->fdimeread = dime_read;
  soap->fdimereadclose = dime_read_close;

  struct _xop__Include data;                                  // here we're using gsoap/import/xop.h to store a blob of raw data
  data.__ptr = "Picture.png";                                 // file name to open for streaming with dime_read_open
  data.__size = 1024;                                         // file has 1024 bytes of data
  data.id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
  data.type = "image/png";                                    // attachment type
  data.options = soap_dime_option(soap, 0, "Picture.png");    // DIME option 0 = "Picture.png" to store file name
  ... // add data to the message and then send it, which will stream the DIME attachment content from Picture.png
  ~~~

  This mechanism also works for DIME attachments attached with `::soap_set_dime_attachment`.

  The maximum size of DIME attachments that the engine allows to be received is limited to `#SOAP_MAXDIMESIZE`.  Increase this size as necessary.

  @param soap `::soap` context
  @param handle the value of the `__ptr` member variable of the attachment struct/class with data
  @param id the value of the `id` member variable of the attachment struct/class with data
  @param type the value of the `type` member variable of the attachment struct/class with data
  @param options the value of the `options` member variable of the attachment struct/class with data
  @returns a handle or NULL when an error occurred (`::soap::error` is nonzero) or when the attachment should be skipped (`::soap::error` is `#SOAP_OK`)
  */
  void *(*fdimereadopen)(struct soap *soap, void *handle, const char *id, const char *type, const char *options);
  /// Callback to read data in a DIME attachment stream
  /**
  @ingroup group_callbacks
  This callback is called by the engine to read a chunk of attachment data to transmit.  The `handle` parameter contains the handle returned by the `::soap::fdimereadopen` callback.  The `buf` parameter is the buffer of length `len` into which a chunk of data should be written by the callback.  The actual amount of data written into the buffer may be less than `len` and this actual amount should be returned by the callback.  A return value of zero indicates an error and `::soap::error` should be set.  The `__size` member variable of the attachment struct/class with data (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members) should be set by the application prior to the serialization of the message with attachments.  The value of `__size` indicates the total size of the attachment data to be transmitted.  If the `__size` member variable is zero and HTTP chunking is enabled (with `#SOAP_IO_CHUNK`), then DIME chunked transfers are activated by the engine, which is more flexible since the attachment data size does not need to be determined in adance.  To use DIME chunked transfers, enable HTTP chunking with `#SOAP_IO_CHUNK` (also `#SOAP_IO_STORE` can be used, but this buffers the entire message in memory before transmission) and set the `__size` member variable of the attachment struct/class to zero.  When DIME attachment chunking is enabled, this callback should completely fill the `buf` buffer with `len` bytes unless the last data chunk is reached and fewer bytes are returned.

  @see `#SOAP_ENC_DIME`, `::soap::user`.

  @par Example:

  See the example provided with the documentation for `::soap::fdimereadopen`.  To enable chunked DIME attachments, replace the last part of the example with:

  ~~~{.cpp}
  struct soap *soap = soap_new1(SOAP_IO_CHUNK);

  struct _xop__Include data;                                  // here we're using gsoap/import/xop.h to store a blob of raw data
  data.__ptr = "Picture.png";                                 // file name to open for streaming with dime_read_open
  data.__size = 0;                                            // zero size means chunked DIME attachments are sent
  data.id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
  data.type = "image/png";                                    // attachment type
  data.options = soap_dime_option(soap, 0, "Picture.png");    // DIME option 0 = "Picture.png" to store file name
  ... // add data to the message and then send it, which will stream the DIME attachment content from Picture.png
  ~~~

  @param soap `::soap` context
  @param handle the value of the handle returned by `::soap::fdimereadopen`
  @param buf buffer to fill
  @param len length of the buffer in bytes
  @returns the number of bytes written to the buffer
  */
  size_t (*fdimeread)(struct soap *soap, void *handle, char *buf, size_t len);
  /// Callback to close a DIME attachment stream after reading
  /**
  @ingroup group_callbacks
  This callback is called by the engine to close the DIME attachment stream after reading.  The `handle` parameter contains the handle returned by the `::soap::fdimereadopen` callback.

  @see `#SOAP_ENC_DIME`, `::soap::fdimereadopen`, `::soap::fdimeread`, `::soap::user`.

  @par Example:

  See the examples provided with the documentation for `::soap::fdimereadopen` and `::soap::fdimeread`.

  @param soap `::soap` context
  @param handle the value of the of the handle returned by `::soap::fdimereadopen`
  */
  void (*fdimereadclose)(struct soap *soap, void *handle);
  /// Callback to open a streaming DIME attachment for writing
  /**
  @ingroup group_callbacks
  Called by the to start receiving a streaming DIME attachment.  This callback opens a stream to start writing the attachment data received.  The actual data stream will be written in chunks using the `::soap::fdimewrite` callback until no more data is available and the `::soap::fdimewriteclose` callback is called to close the stream.  The `id`, `type` and `options` parameters are the `id`, `type` and `options` of the attachment struct/class (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members), respectively.  The callback should return a handle, which is passed to the `::soap::fdimewrite` and `::soap::fdimewriteclose` callbacks.  The `__ptr` member variable of the attachment struct/class is set by the engine to the value of this handle.  The `__size` member variable is set to the size of the attachment received.  The maximum DIME attachment size received is limited by `#SOAP_MAXDIMESIZE`.

  @see `#SOAP_ENC_DIME`, `::soap::user`.

  @par Example:

  ~~~{.cpp}
  void * dime_write_open(struct soap *soap, const char *id, const char *type, const char *options)
  {
    if (options)
    {
      FILE *handle = NULL;
      char *name;
      size_t len = ((unsigned char)options[2] << 8) | ((unsigned char)options[3]); // get option string length
      name = (char*)soap_malloc(soap, len + 1);
      strncpy(name, options + 4, len); // get file name from options (as an example)
      name[len] = '\0';
      handle = fopen(name, "wb");
      if (!handle)
      {
        soap->error = SOAP_EOF; // could not open file for writing
        soap->errnum = errno;   // to report errno value
        return NULL;
      }
    }
    else
    {
      soap->error = soap_receiver_fault(soap, "Cannot save to file: no file name was present in the attachment", NULL);
    }
    return (void*)handle;
  }
  int dime_write(struct soap *soap, void *handle, const char *buf, size_t len)
  {
    while (len)
    {
      size_t nwritten = fwrite(buf, 1, len, (FILE*)handle);
      if (!nwritten)
      {
        soap->errnum = errno;
        return SOAP_EOF;
      }
      len -= nwritten;
      buf += nwritten;
    }
    return SOAP_OK;
  }
  void dime_write_close(struct soap *soap, void *handle)
  {
    fclose((FILE*)handle);
  }

  struct soap *soap = soap_new();
  soap->fdimewriteopen = dime_write_open;
  soap->fdimewrite = dime_write;
  soap->fdimewriteclose = dime_write_close;

  ... // when a service responds to the client request with DIME attachment(s), the attachment data is saved via the callbacks
  ~~~

  The maximum size of DIME attachments that the engine allows to be received is limited to `#SOAP_MAXDIMESIZE`.  Increase this size as necessary.

  @param soap `::soap` context
  @param id the value of the `id` member variable of the attachment struct/class with data
  @param type the value of the `type` member variable of the attachment struct/class with data
  @param options the value of the `options` member variable of the attachment struct/class with data
  @returns a handle or NULL when an error occurred (`::soap::error` should be nonzero)
  */
  void *(*fdimewriteopen)(struct soap *soap, const char *id, const char *type, const char *options);
  /// Callback to write data in a DIME attachment stream
  /**
  @ingroup group_callbacks
  This callback is called by the engine to write a chunk of attachment data received.  The `handle` parameter contains the handle returned by the `::soap::fdimewriteopen` callback.  The `buf` parameter contains the data of length `len`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.

  @see `#SOAP_ENC_DIME`, `::soap::user`.

  @param soap `::soap` context
  @param handle the value of the handle returned by `::soap::fdimewriteopen`
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fdimewrite)(struct soap *soap, void*, const char*, size_t);
  /// Callback to close a DIME attachment stream after writing
  /**
  @ingroup group_callbacks
  This callback is called by the engine to close the DIME attachment stream after writing.  The `handle` parameter contains the handle returned by the `::soap::fdimewriteopen` callback.

  @see `#SOAP_ENC_DIME`, `::soap::user`.

  @param soap `::soap` context
  @param handle the value of the of the handle returned by `::soap::fdimewriteopen`
  */
  void (*fdimewriteclose)(struct soap *soap, void *handle);
  /// Callback to open a streaming MIME/MTOM attachment for reading
  /**
  @ingroup group_callbacks
  This callback is called by the engine to start sending a streaming MIME/MTOM attachment.  This callback opens a stream to start reading the attachment data to send.  The actual data stream will be read in chunks using the `::soap::fmimeread` callback until no more data is available and the `::soap::fmimereadclose` callback is called to close the stream.  The `handle` parameter contains the value of the `__ptr` member variable of the attachment struct/class with data (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members), which should be a pointer to specific information such as a file descriptor or a pointer to a some application-specific data to be passed to this callback.  Both the `__ptr` and `__size` members of the attachment struct/class should have been set by the application prior to the serialization of the message with attachments.  If the `__size` is zero and HTTP chunking is enabled (with `#SOAP_IO_CHUNK`), then chunked MIME/MTOM attachments are sent, see `::soap::fmimeread`.  The `id`, `type` and `options` parameters are the `id` (an optional ID), `type` (a MIME type) and `options` (a descriptive string) of the attachment struct/class, respectively, of which at least one member should be non-NULL.  The callback should return the `handle` parameter value or another pointer value, which is passed as the new `handle` parameter to `::soap::fmimeread` and `::soap::fmimereadclose` callbacks.  When an error occurred in this callback, the callback should return NULL and set `::soap::error` to an error code, e.g. using `::soap_receiver_fault`.  The callback may return NULL and set `::soap::error` to `#SOAP_OK` when this specific MIME/MTOM attachment should not to be streamed and the engine will simply skip it.

  @see `#SOAP_ENC_DIME`, `::soap::user`.

  @par Example:

  ~~~{.cpp}
  void * mime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *options)
  {
    return (void*)fopen((char*)handle, "rb");
  }
  size_t mime_read(struct soap *soap, void *handle, char *buf, size_t len)
  {
    return fread(buf, 1, len, (FILE*)handle);
  }
  void mime_read_close(struct soap *soap, void *handle)
  {
    fclose((FILE*)handle);
  }

  struct soap *soap = soap_new1(SOAP_ENC_MTOM);
  soap->fmimereadopen = mime_read_open;
  soap->fmimeread = mime_read;
  soap->fmimereadclose = mime_read_close;

  struct _xop__Include data;                                  // here we're using gsoap/import/xop.h to store a blob of raw data
  data.__ptr = "Picture.png";                                 // file name to open for streaming with mime_read_open
  data.__size = 1024;                                         // file has 1024 bytes of data
  data.id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
  data.type = "image/png";                                    // attachment type
  data.options = "Picture.png";                               // we store the file name with the attachment description
  ... // add data to the message and then send it, which will stream the MIME/MTOM attachment content from Picture.png
  ~~~

  To enable chunked MIME/MTOM attachments, replace the last part of the example with:

  ~~~{.cpp}
  struct soap *soap = soap_new1(SOAP_IO_CHUNK);

  struct _xop__Include data;                                  // here we're using gsoap/import/xop.h to store a blob of raw data
  data.__ptr = "Picture.png";                                 // file name to open for streaming with dime_read_open
  data.__size = 0;                                            // zero size means chunked MIME/MTOM attachments are sent
  data.id = soap_strdup(soap, soap_rand_uuid(soap, "uuid:")); // attachment id (optional, can use NULL)
  data.type = "image/png";                                    // attachment type
  data.options = "Picture.png";                               // we store the file name with the attachment description
  ... // add data to the message and then send it, which will stream the MIME/MTOM attachment content from Picture.png
  ~~~

  This mechanism also works for MIME/MTOM attachments that are explicitly attached with `::soap_set_mime_attachment`.

  @see `#SOAP_ENC_MIME`, `#SOAP_ENC_MTOM`, `::soap_set_mime_attachment`.

  @param soap `::soap` context
  @param handle the value of the `__ptr` member variable of the attachment struct/class with data
  @param id the value of the `id` member variable of the attachment struct/class with data
  @param type the value of the `type` member variable of the attachment struct/class with data
  @param options the value of the `options` member variable of the attachment struct/class with data
  @returns a handle or NULL when an error occurred (`::soap::error` is nonzero) or when the attachment should be skipped (`::soap::error` is `#SOAP_OK`)
  */
  void *(*fmimereadopen)(struct soap *soap, void*, const char*, const char*, const char*);
  /// Callback to read data in a MIME/MTOM attachment stream
  /**
  @ingroup group_callbacks
  This callback is called by the engine to read a chunk of attachment data to transmit.  The `handle` parameter contains the handle returned by the `::soap::fmimereadopen` callback.  The `buf` parameter is the buffer of length `len` into which a chunk of data should be written by the callback.  The actual amount of data written into the buffer may be less than `len` and this actual amount should be returned by the callback.  A return value of zero indicates an error and `::soap::error` should be set.  The `__size` member variable of the attachment struct/class with data (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members) should be set by the application prior to the serialization of the message with attachments.  The value of `__size` indicates the total size of the attachment data to be transmitted.  If the `__size` member variable is zero and HTTP chunking is enabled (with `#SOAP_IO_CHUNK`), then MIME/MTOM chunked transfers are activated by the engine, which is more flexible since the attachment data size does not need to be determined in advance.  To use MIME/MTOM chunked transfers, enable HTTP chunking with `#SOAP_IO_CHUNK` (also `#SOAP_IO_STORE` can be used, but this buffers the entire message in memory before transmission) and set the `__size` member variable of the attachment struct/class to zero.  When MIME/MTOM attachment chunking is enabled, this callback should completely fill the `buf` buffer with `len` bytes unless the last data chunk is reached and fewer bytes are returned.

  @see `#SOAP_ENC_MIME`, `#SOAP_ENC_MTOM`, `::soap::fmimereadopen`, `::soap::user`.

  @par Example:

  See the example provided with the documentation for `::soap::fmimereadopen`.

  @param soap `::soap` context
  @param handle the value of the handle returned by `::soap::fmimereadopen`
  @param buf buffer to fill
  @param len length of the buffer in bytes
  @returns the number of bytes written to the buffer
  */
  size_t (*fmimeread)(struct soap *soap, void *handle, char *buf, size_t len);
  /// Callback to close a MIME/MTOM attachment stream after reading
  /**
  @ingroup group_callbacks
  This callback is called by the engine to close the MIME/MTOM attachment stream after reading.  The `handle` parameter contains the handle returned by the `::soap::fmimereadopen` callback.

  @see `#SOAP_ENC_MIME`, `#SOAP_ENC_MTOM`, `::soap::fmimereadopen`, `::soap::user`.

  @par Example:

  See the example provided with the documentation for `::soap::fmimereadopen`.

  @param soap `::soap` context
  @param handle the value of the of the handle returned by `::soap::fmimereadopen`
  */
  void (*fmimereadclose)(struct soap *soap, void *handle);
  /// Callback to open a streaming MIME/MTOM attachment for writing
  /**
  @ingroup group_callbacks
  Called by the to start receiving a streaming MIME/MTOM attachment.  This callback opens a stream to start writing the attachment data received.  The actual data stream will be written in chunks using the `::soap::fmimewrite` callback until no more data is available and the `::soap::fmimewriteclose` callback is called to close the stream.  The `id`, `type` and `options` parameters are the `id`, `type` and `options` of the attachment struct/class (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members), respectively.  The callback should return a handle, which is passed to the `::soap::fmimewrite` and `::soap::fmimewriteclose` callbacks.  The `__ptr` member variable of the attachment struct/class is set by the engine to the value of this handle.  The `__size` member variable is set to the size of the attachment received.

  @see `#SOAP_ENC_MIME`, `#SOAP_ENC_MTOM`, `::soap::fmimereadopen`, `::soap::user`.

  @par Example:

  ~~~{.cpp}
  void * mime_write_open(struct soap *soap, const char *id, const char *type, const char *options)
  {
    if (options)
    {
      FILE *handle = NULL;
      char *name;
      size_t len = ((unsigned char)options[2] << 8) | ((unsigned char)options[3]); // get option string length
      name = (char*)soap_malloc(soap, len + 1);
      strncpy(name, options + 4, len); // get file name from options (as an example)
      name[len] = '\0';
      handle = fopen(name, "wb");
      if (!handle)
      {
        soap->error = SOAP_EOF; // could not open file for writing
        soap->errnum = errno;   // to report errno value
        return NULL;
      }
    }
    else
    {
      soap->error = soap_receiver_fault(soap, "Cannot save to file: no file name was present in the attachment", NULL);
    }
    return (void*)handle;
  }
  int mime_write(struct soap *soap, void *handle, const char *buf, size_t len)
  {
    while (len)
    {
      size_t nwritten = fwrite(buf, 1, len, (FILE*)handle);
      if (!nwritten)
      {
        soap->errnum = errno;
        return SOAP_EOF;
      }
      len -= nwritten;
      buf += nwritten;
    }
    return SOAP_OK;
  }
  void mime_write_close(struct soap *soap, void *handle)
  {
    fclose((FILE*)handle);
  }

  struct soap *soap = soap_new();
  soap->fmimewriteopen = mime_write_open;
  soap->fmimewrite = mime_write;
  soap->fmimewriteclose = mime_write_close;

  ... // when a service responds to the client request with MIME/MTOM attachment(s), the attachment data is saved via the callbacks
  ~~~

  @param soap `::soap` context
  @param id the value of the `id` member variable of the attachment struct/class with data
  @param type the value of the `type` member variable of the attachment struct/class with data
  @param options the value of the `options` member variable of the attachment struct/class with data
  @returns a handle or NULL when an error occurred (`::soap::error` should be nonzero)
  */
  void *(*fmimewriteopen)(struct soap *soap, void *handle, const char *id, const char *type, const char *description, enum soap_mime_encoding encoding);
  /// Callback to write data in a MIME attachment stream
  /**
  @ingroup group_callbacks
  This callback is called by the engine to write a chunk of attachment data received.  The `handle` parameter contains the handle returned by the `::soap::fmimewriteopen` callback.  The `buf` parameter contains the data of length `len`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.
  @see `#SOAP_ENC_MIME`, `#SOAP_ENC_MTOM`, `::soap::user`.

  @param soap `::soap` context
  @param handle the value of the handle returned by `::soap::fmimewriteopen`
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fmimewrite)(struct soap *soap, void *handle, const char *buf, size_t len);
  /// Callback to close a MIME/MTOM attachment stream after writing
  /**
  @ingroup group_callbacks
  This callback is called by the engine to close the MIME/MTOM attachment stream after writing.  The `handle` parameter contains the handle returned by the `::soap::fmimewriteopen` callback.

  @see `#SOAP_ENC_MIME`, `#SOAP_ENC_MTOM`, `::soap::user`.

  @param soap `::soap` context
  @param handle the value of the of the handle returned by `::soap::fmimewriteopen`
  */
  void (*fmimewriteclose)(struct soap *soap, void *handle);
  /// Callback to initialize the OpenSSL library
  /**
  @ingroup group_callbacks
  This callback is called to initialize the OpenSSL or GNUTLS context for HTTPS connections configured with the parameters passed to `::soap_ssl_client_context` and `::soap_ssl_server_context`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fsslauth` is `ssl_auth_init`.

  @see `::soap::user`.

  @param soap `::soap` context
  @returns `#SOAP_OK` or a `::soap_status` error code
  */
  int (*fsslauth)(struct soap *soap);
  /// Callback to manage the verification of the certificate provided by a peer (typically a server)
  /**
  @ingroup group_callbacks
  This callback is called by the engine to manage the verification of the certificate provided by a peer, such as the certificate provided by a server connected over HTTPS or to verify the certificate included with a WS-Security message.  To require certificate verification of a server connected via HTTPS, use `::soap_ssl_client_context` with `#SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION`.  To require certificate verification of a client connected to a server, use `::soap_ssl_server_context` with `#SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION`.  The `ok` parameter of this callback indicates whether the verification of the certificate in question passed (`ok` == 1) or failed (`ok` == 0) as determined by the OpenSSL library based on the `::soap_ssl_client_context` or `::soap_ssl_server_context` configuration.  If the callback returns 1 then the handshake is continued and the connection maybe established.  To return 1 when `ok` == 0 requires resetting the error state with `X509_STORE_CTX_set_error(store, X509_V_OK)`.  If the callback returns 0 then the handshake is immediately terminated with "verification failed" and a verification failure alert is sent to the peer.  The built-in function assigned to `::soap::fsslverify` is `ssl_verify_callback` or when `#SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE` is used `ssl_verify_callback_allow_expired_certificate`.

  @see `::soap::user`.

  @par Example:

  ~~~{.cpp}
  int ssl_verify_callback_allow_self_signed_cert(int ok, X509_STORE_CTX *store)
  {
    if (!ok && X509_STORE_CTX_get_error(store) == X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN)
    {
      X509_STORE_CTX_set_error(store, X509_V_OK);
      ok = 1;
    }
    return ok;
  }

  struct soap *soap = soap_new();
  soap->fsslverify = ssl_verify_callback_allow_self_signed_cert;
  ~~~

  @param ok when 1: the certificate passed, when 0: the certificate did not pass
  @returns 1 to pass and 0 to fail

  @see `::soap_ssl_client_context`.
  */
  int (*fsslverify)(int ok, X509_STORE_CTX *store);
};

/// A built-in string type containing literal XML content in UTF-8 format
typedef char *_XML;

/// A built-in string type containing normalized QName contents
typedef char *_QName;

/// Allocate and initialize a new `::soap` context
/**
This function allocates and initializes a new context.

There is no need to call `::soap_init` to initialize the context allocated with `::soap_new`, since `::soap_new` initializes the allocated context.  To change the input/output mode flags, use `::soap_set_mode` and `::soap_clr_mode`.

@note C++ proxy and service classes generated by <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b> have an internal `::soap` context that is either a base class (option <b>`-i`</b>) or a member variable pointing to a `::soap` context (option <b>`-j`</b>).  The C++ proxy and service classes allocate and deallocate this context, which means that `::soap_new` and `::soap_free` are not required.  For example:
~~~{.cpp}
#include "soapexampleProxy.h"

exampleProxy proxy(SOAP_XML_INDENT);
... // use proxy or proxy.soap (option -j)
proxy.destroy(); // delete managed C++ objects and memory
~~~
~~~{.cpp}
#include "soapexampleService.h"

exampleService service(SOAP_XML_INDENT);
... // use service or service.soap (option -j)
service.destroy(); // delete managed C++ objects and memory
~~~

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // send and receive messages etc.
soap_destroy(soap); // delete managed C++ objects
soap_end(soap);     // delete managed memory
soap_free(soap);    // free the context
~~~

@see `::soap_new1`, `::soap_new2`, `::soap_copy`, `::soap_destroy`, `::soap_end`, `::soap_free`.
*/
struct soap *soap_new()
  /// @returns pointer to allocated and initialized `::soap` context or NULL when out of heap memory
  ;

/// Allocate and initialize a new `::soap` context with input and output `::soap_mode` flags
/**
This function allocates and initializes a new context with the specified input and output `::soap_mode` flags.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_XML_INDENT);
... // send and receive messages etc.
soap_destroy(soap); // delete managed C++ objects
soap_end(soap);     // delete managed memory
soap_free(soap);    // free the context
~~~

@note There is no need to call `::soap_init` to initialize the context allocated with `::soap_new1`.  To change the input/output mode flags, use `::soap_set_mode` and `::soap_clr_mode`.

@see `::soap_new`, `::soap_new2`, `::soap_copy`, `::soap_destroy`, `::soap_end`, `::soap_free`.
*/
struct soap *soap_new1(soap_mode input_and_output_mode) ///< input and output `::soap_mode` flags
  /// @returns pointer to allocated and initialized `::soap` context or NULL when out of heap memory
  ;

/// Allocate and initialize a new `::soap` context with separate input and output `::soap_mode` flags
/**
This function allocates and initializes a new context with the specified input and output `::soap_mode` flags.  The separation of input and output mode flags is only useful for the `::SOAP_XML_TREE` flag that affects both input and output behaviors.

@note There is no need to call `::soap_init` to initialize the context allocated with `::soap_new2`.

@see `::soap_new`, `::soap_new1`, `::soap_copy`, `::soap_destroy`, `::soap_end`, `::soap_free`.
*/
struct soap *soap_new2(
    soap_mode input_mode,  ///< input `::soap_mode` flags
    soap_mode output_mode) ///< output `::soap_mode` flags
  /// @returns pointer to allocated and initialized `::soap` context or NULL when out of heap memory
  ;

/// Initialize a stack-allocated `::soap` context
/**
This function initializes a context.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap soap;
soap_init(&soap);
... // send and receive messages etc.
soap_destroy(&soap); // delete managed C++ objects
soap_end(s&oap);     // delete managed memory
soap_done(&soap);    // finalize the context
~~~

The context can be re-initialized for reuse after `::soap_done` by calling `::soap_init`.

@note Initialization should be done at most once before calling `::soap_done`.  To change the input/output mode flags, use `::soap_set_mode` and `::soap_clr_mode`.

@see `::soap_init1`, `::soap_new`, `::soap_copy`, `::soap_destroy`, `::soap_end`, `::soap_done`.
*/
void soap_init(struct soap *soap) ///< `::soap` context to initialize
  ;

/// Initialize a stack-allocated `::soap` context with input and output `::soap_mode` flags
/**
This function initializes a context with the specified input and output `::soap_mode` flags.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap soap;
soap_init1(&soap, SOAP_XML_INDENT);
... // send and receive messages etc.
soap_destroy(&soap); // delete managed C++ objects
soap_end(s&oap);     // delete managed memory
soap_done(&soap);    // finalize the context
~~~

The context can be re-initialized for reuse after `::soap_done` by calling `::soap_init`.

@note Initialization should be done at most once before calling `::soap_done`.  To change the input/output mode flags, use `::soap_set_mode` and `::soap_clr_mode`.

@see `::soap_init`, `::soap_new`, `::soap_copy`, `::soap_destroy`, `::soap_end`, `::soap_done`.
*/
void soap_init1(
    struct soap *soap,               ///< `::soap` context to initialize
    soap_mode input_and_output_mode) ///< input and output `::soap_mode` flags
  ;

/// Initialize a stack-allocated `::soap` context with input and output `::soap_mode` flags
/**
This function initializes a context with the specified input and output `::soap_mode` flags.

@note Initialization should be done at most once before calling `::soap_done`.  To change the input/output mode flags, use `::soap_set_mode` and `::soap_clr_mode`.

@see `::soap_init`, `::soap_new`, `::soap_copy`, `::soap_destroy`, `::soap_end`, `::soap_done`.
*/
void soap_init2(
    struct soap *soap,     ///< `::soap` context to initialize
    soap_mode input_mode,  ///< input `::soap_mode` flags
    soap_mode output_mode) ///< output `::soap_mode` flags
  ;

/// Set input and output `::soap_mode` flags of the given `::soap` context
void soap_set_mode(
    struct soap *soap,               ///< `::soap` context
    soap_mode input_and_output_mode) ///< input and output `::soap_mode` flags
  ;

/// Set input `::soap_mode` flags of the given `::soap` context
void soap_set_imode(
    struct soap *soap,    ///< `::soap` context
    soap_mode input_mode) ///< input `::soap_mode` flags
  ;

/// Set output `::soap_mode` flags of the given `::soap` context
void soap_set_omode(
    struct soap *soap,     ///< `::soap` context
    soap_mode output_mode) ///< output `::soap_mode` flags
  ;

/// Clear input and output `::soap_mode` flags of the given `::soap` context
void soap_clr_mode(
    struct soap *soap,               ///< `::soap` context
    soap_mode input_and_output_mode) ///< input and output `::soap_mode` flags
  ;

/// Clear input `::soap_mode` flags of the given `::soap` context
void soap_clr_imode(
    struct soap *soap,    ///< `::soap` context
    soap_mode input_mode) ///< input `::soap_mode` flags
  ;

/// Clear output `::soap_mode` flags of the given `::soap` context
void soap_clr_omode(
    struct soap *soap,     ///< `::soap` context
    soap_mode output_mode) ///< output `::soap_mode` flags
  ;

/// Allocate and initialize a new `::soap` context as a copy of the given `::soap` context
/**
This function allocates a new context and copies the state of the specified context except for the heap-allocated data managed by the specified context.  After the copy the contexts do not share any data and can therefore be used by separate threads without requiring synchronization or mutex locking.

@note C++ proxy and service classes generated by <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b> have an internal `::soap` context that is either a base class (option <b>`-i`</b>) or a member variable pointing to a `::soap` context (option <b>`-j`</b>).  For convenience, use the `copy` member function instead of `::soap_copy` and delete the copied instance with `delete`.  For example:
~~~{.cpp}
#include "soapexampleService.h" // generated by soapcpp2 option -j

int main()
{
  exampleService service(SOAP_XML_INDENT);
  service.soap->bind_flags = SO_REUSEADDR; // immediate port reuse
  service.soap->accept_timeout = 3600;     // let soap_accept time out after 1 hour
  ... // further initialize service.soap
  if (soap_valid_socket(service.bind(NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      if (soap_valid_socket(service.accept()))
      {
        THREAD_TYPE tid;
        exampleService *tservice = service.copy();
        if (!tservice)
          soap_closesock(service.soap);
        else
          while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tservice))
            sleep(1); // failed, try again
      }
      else if (service.soap->errnum) // accept failed, try again after 1 second
      {
        service.soap_print_fault(stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      service.destroy();
    }
  }
}
void *process_request(exampleService *service)
{
  THREAD_DETACH(THREAD_ID);
  service.serve();
  service.destroy();
  delete service;
  return NULL;
}
~~~

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/threads.h"

int main()
{
  struct soap *soap = soap_new1(SOAP_XML_INDENT);
  soap->bind_flags = SO_REUSEADDR;            // immediate port reuse
  soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      if (soap_valid_socket(soap_accept(soap)))
      {
        THREAD_TYPE tid;
        struct soap *tsoap = soap_copy(soap);
        if (!tsoap)
          soap_force_closesock(soap);
        else
          while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap))
            sleep(1); // failed, try again
      }
      else // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}
void *process_request(struct soap *soap)
{
  THREAD_DETACH(THREAD_ID);
  soap_serve(soap);
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}
~~~

@see `::soap_copy_context`, `::soap_copy_stream`, `::soap_free_stream`, `::soap_delegate_deletion`, `::soap_destroy`, `::soap_end`, `::soap_free`.
*/
struct soap *soap_copy(struct soap *soap) ///< `::soap` context to copy
  /// @returns pointer to allocated and initialized `::soap` context or NULL when out of heap memory
  ;

/// Copy a given `::soap` context to an uninitialized destination `::soap` context
/**
This function copies the state of the specified context to another uninitialized context (i.e. overriding it).  If the destination context is initialized or active then call `::soap_done` first to clean it up before overriding it.  The entire state is copied except for the heap-allocated data managed by the specified context.  After the copy the contexts do not share any data and can therefore be used by separate threads without requiring synchronization or mutex locking.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // send and receive messages etc.
struct soap temp;
soap_copy_context(&temp, soap); 
...
~~~

@see `::soap_copy`, `::soap_copy_stream`, `::soap_free_stream`, `::soap_delegate_deletion`, `::soap_destroy`, `::soap_end`, `::soap_free`.
*/
void soap_copy_context(
    struct soap *soap_destination, ///< destination `::soap` context to initialize
    struct soap *soap_source)      ///< source `::soap` context to copy
  ;

/// Copy the input/output stream state of the given `::soap` context to another context
/**
This function copies the input/output state of the specified source context to the specified destination context.  Both contexts will share the same input/output streams , i.e. `::soap::is`, `::soap::os`, `::soap::socket`, `::soap::recvfd` and `::soap::sendfd` are shared and the current message buffer `::soap::buf` content is copied.  The destination context is set to the source context `::soap_mode` flags and timeouts.  To move the input/output state of one context to another, use this function and then call `::soap_free_stream` on the source context to clear its input/output state.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
struct soap *temp = soap_new();
... // send and receive messages etc.
// move the input/output state to another context
soap_copy_stream(temp, soap); 
soap_free_stream(soap);
soap_closesock(soap); // has no effect
... // send and receive messages etc.
soap_closesock(temp); // closes socket, if open
~~~

@see `::soap_free_stream`.
*/
void soap_copy_stream(
    struct soap *soap_destination, ///< destination `::soap` context
    struct soap *soap_source)      ///< source `::soap` context
  ;

/// Free the input/output stream state of the given `::soap` context
/**
@see `::soap_copy_stream`.
*/
void soap_free_stream(struct soap *soap) ///< `::soap` context
  ;

/// Finalize and free the given `::soap` context from unmanaged heap memory
/**
This function finalizes and frees the specified context.  The finalization is done with `::soap_done` before releasing its memory.  This function does not free memory managed by the context.  To free memory managed by the context use `::soap_destroy` and `::soap_end`, or `::soap::destroy` to call both.

@note C++ proxy and service classes generated by <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b> have an internal `::soap` context that is either a base class (option <b>`-i`</b>) or a member variable pointing to a `::soap` context (option <b>`-j`</b>).  The C++ proxy and service classes allocate and deallocate this context, which means that `::soap_new` and `::soap_free` are not required.  For example:
~~~{.cpp}
#include "soapexampleProxy.h"

exampleProxy proxy(SOAP_XML_INDENT);
... // use proxy or proxy.soap (option -j)
proxy.destroy(); // delete managed C++ objects and memory
~~~
~~~{.cpp}
#include "soapexampleService.h"

exampleService service(SOAP_XML_INDENT);
... // use service or service.soap (option -j)
service.destroy(); // delete managed C++ objects and memory
~~~

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // send and receive messages etc.
soap_destroy(soap);
soap_end(soap);
soap_free(soap);
~~~

@see `::soap_new`, `::soap_new1`, `::soap_new2`, `::soap_done`, `::soap_destroy`, `::soap_end`, `::soap::destroy`.
*/
void soap_free(struct soap *soap) ///< `::soap` context to free
  ;

/// Finalize the given `::soap` context, i.e. when the `::soap` context is stack allocated, automatically invoked in C++ by the `::soap` destructor on the `::soap` context to delete
/**
This function finalizes the specified context.  This function does not free memory managed by the context.  To free memory managed by the context use `::soap_destroy` and `::soap_end`, or `::soap::destroy` to call both.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap soap;
soap_init(&soap);
... // send and receive messages etc.
soap_destroy(&soap);
soap_end(&soap);
soap_done(&soap);
~~~

@see `::soap_free`, `::soap_destroy`, `::soap_end`, `::soap::destroy`.
*/
void soap_done(struct soap *soap) ///< `::soap` context to finalize
  ;

/// Allocate a block of heap memory managed by the specified `::soap` context
/**
This function allocates a block of memory from the heap managed by the specified `::soap` context. All such blocks allocated are deleted with a single call to `::soap_end`.  Returns a pointer to the allocated block of memory or NULL when out of memory without setting `::soap::error`.

@note The soapcpp2 tool generates `soap_new_T` functions for all serialiable types `T`.  The `soap_new_T` functions allocate and default initializes the type `T` or an array of items of type `T`.  Recommended is to use these more powerful `soap_new_T` functions instead of `::soap_malloc`.  For example:
~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
struct ns__someElement *data = soap_new_ns__someElement(soap);       // allocate managed object
struct ns__someElement *array = soap_new_ns__someElement(soap, 100); // allocate array of 100 managed objects
... // send and receive messages etc.
soap_destroy(soap); // deletes data, array, and other managed C++ objects
soap_end(soap);     // delete managed memory
soap_free(soap);    // free the context
~~~
The soapcpp2 tool also generates `soap_default_T` functions to default initialize the type `T`.  For example:
~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
struct ns__someElement data;
soap_default_ns__someElement(soap, &data); // default initializes all public members
...
~~~
but objects of classes should use their `soap_default` method instead of the `soap_default_T` function.


@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
char *s = (char*)soap_malloc(soap, 80); // allocate 80 bytes of memory managed by the context
strcpy(s, "Hello");                     // copy a string into it
... // send and receive messages etc.
soap_destroy(soap); // delete managed C++ objects
soap_end(soap);     // delete managed memory
soap_free(soap);    // free the context
~~~

@see `::soap_strdup`, `::soap_wstrdup`, `::soap_unlink`, `::soap_delegate_deletion`, `::soap_destroy`, `::soap_end`, and the [C and C++ XML data bindings](../../databinding/html/index.html) documentation.
*/
void * soap_malloc(
    struct soap *soap, ///< `::soap` context
    size_t len)        ///< length of the block to allocate in number of bytes
  /// @returns pointer to the allocated block of memory or NULL on failure to allocate (out of memory)
  ;

/// Copy a string to managed memory
/**
This function copies the specified wide string to memory managed by the specified context.  Returns a copy of the string or NULL when the specified string is NULL or when the function failed to allocate memory.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
// allocate and assign a string in memory managed by the context
char *s = soap_strdup(soap, "Hello");
... // send and receive messages etc.
soap_destroy(soap); // delete managed C++ objects
soap_end(soap);     // delete managed memory
soap_free(soap);    // free the context
~~~

@see `::soap_malloc`, `::soap_wstrdup`, `::soap_unlink`, `::soap_delegate_deletion`, `::soap_destroy`, `::soap_end`.
*/
char * soap_strdup(
    struct soap *soap,  ///< `::soap` context
    const char *string) ///< string to copy to managed memory
  /// @returns copy of string or NULL when the specified string is NULL or on failure to allocate (out of memory)
  ;

/// Copy a wide string to managed memory
/**
This function copies the specified wide string to managed memory.  Returns a copy of the wide string or NULL when the specified wide string is NULL or when the function failed to allocate memory.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
// allocate and assign a wide string in memory managed by the context
wchar_t *s = soap_wstrdup(soap, L"Hello");
... // send and receive messages etc.
soap_destroy(soap); // delete managed C++ objects
soap_end(soap);     // delete managed memory
soap_free(soap);    // free the context
~~~

@see `::soap_malloc`, `::soap_strdup`, `::soap_unlink`, `::soap_delegate_deletion`, `::soap_destroy`, `::soap_end`.
*/
wchar_t * soap_wstrdup(
    struct soap *soap,     ///< `::soap` context
    const wchar_t *string) ///< wide string to copy to managed memory or NULL
  /// @returns copy of wide string or NULL when the specified wide string is NULL or on failure to allocate (out of memory)
  ;

/// Unlink a block of heap memory managed by the specified `::soap` context, to release the memory explicitly later
/**
This function removes a managed block of memory from the managing `::soap` context.  This memory is not released but rather should be released explicitly later by the application logic using `free` or `delete`.  Returns `#SOAP_OK` when successful or `#SOAP_ERR` when the block is not managed by the specified context.
*/
int soap_unlink(
    struct soap *soap, ///< `::soap` context
    const void *ptr)   ///< pointer to the block of managed heap memory to unlink
  /// @returns `#SOAP_OK` when successful or `#SOAP_ERR` when the block is not managed by the specified context
  ;

/// Delete all dynamically-allocated C++ objects managed by the specified `::soap` context
/**
This function deletes all dynamically-allocated C++ objects managed by the specified `::soap` context, i.e. data allocated with `soap_new_T` calls.  This call should be followed by `::soap_end` to delete all other dynamically-allocated data managed by the `::soap` context.  Or just invoke `::soap::destroy` to delete objects and data and release the freed memory back to the heap.

@note C++ proxy and service classes generated by <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b> have an internal `::soap` context that is either a base class (option <b>`-i`</b>) or a member variable pointing to a `::soap` context (option <b>`-j`</b>).  For convenience, use the `destroy` member function instead of `::soap_destroy` and `::soap_end`.  For example:
~~~{.cpp}
#include "soapexampleProxy.h"

exampleProxy proxy(SOAP_XML_INDENT);
... // use proxy or proxy.soap (option -j)
proxy.destroy(); // delete managed C++ objects and memory
~~~
~~~{.cpp}
#include "soapexampleService.h"

exampleService service(SOAP_XML_INDENT);
... // use service or service.soap (option -j)
service.destroy(); // delete managed C++ objects and memory
~~~

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // send and receive messages etc.
soap_destroy(soap); // delete managed C++ objects
soap_end(soap);     // delete managed memory
soap_free(soap);    // free the context
~~~

@see `::soap_malloc`, `::soap_end`, `::soap::destroy`.
*/
void soap_destroy(struct soap *soap) ///< `::soap` context
  ;

/// Explicitly dealllocates a block of managed memory that is managed by the specified `::soap` context and release the free memory back to the heap
/**
This function deallocates a managed block of memory from the managing `::soap` context and releases the free memory back to the heap.  This frees data allocated with `::soap_malloc` and C++ objects allocated and instantiated with the `soap_new_T` functions.  Normally this function should not be used to individually deallocate managed objects and data but rather `::soap_destroy` and `::soap_end` should be used to deallocate all objects and data managed by the context, which is much more efficient.
*/
void soap_dealloc(
    struct soap *soap, ///< `::soap` context
    void *ptr)         ///< pointer to the block of managed heap memory to deallocate
  ;

/// Delete temporary data
/**
This function deallocates temporary data such as buffers and hash tables but leaves deserialized managed data intact.
*/
void soap_free_temp(struct soap *soap) ///< `::soap` context
  ;

/// Delete all data from heap memory managed by the specified `::soap` context and release the freed memory back to the heap
/**
This function deletes all dynamically-allocated data managed by the specified `::soap` context, i.e. data allocated with `::soap_malloc`.  This call suffices to delete all managed data from C applications and release the freed memory back to the heap.  C++ applications however should call `::soap_destroy` first before `::soap_end` or just invoke `::soap::destroy` (or C++ proxy and service class member function `destroy`) to delete objects and data and release the freed memory back to the heap.

@note C++ proxy and service classes generated by <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b> have an internal `::soap` context that is either a base class (option <b>`-i`</b>) or a member variable pointing to a `::soap` context (option <b>`-j`</b>).  For convenience, use the `destroy` member function instead of `::soap_destroy` and `::soap_end`.  For example:
~~~{.cpp}
#include "soapexampleProxy.h"

exampleProxy proxy(SOAP_XML_INDENT);
... // use proxy or proxy.soap (option -j)
proxy.destroy(); // delete managed C++ objects and memory
~~~
~~~{.cpp}
#include "soapexampleService.h"

exampleService service(SOAP_XML_INDENT);
... // use service or service.soap (option -j)
service.destroy(); // delete managed C++ objects and memory
~~~

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // send and receive messages etc.
soap_destroy(soap); // delete managed C++ objects
soap_end(soap);     // delete managed memory
soap_free(soap);    // free the context
~~~

@see `::soap_malloc`, `::soap_destroy`, `::soap::destroy`.
*/
void soap_end(struct soap *soap) ///< `::soap` context
  ;

/// Delegate the deletion of all managed objects and data from the specified `::soap` context to another `::soap` context
/**
This function moves all dynamically-allocated data managed by the specified `::soap` context to the target context `soap_to` for deletion by the target context using `::soap_destroy` and `::soap_end`.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
struct soap *temp = soap_new(); // temp context to manage data
if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
{
  soap_print_fault(soap, stderr);
}
else
{
  // success, response contains deserialized data
  soap_delegate_deletion(soap, temp); // deserialized data is managed by temp context
}
soap_destroy(soap); // clean up 'soap' context
soap_end(soap);     // clean up 'soap' context
... // use deserialized data managed by 'temp' context, reuse 'soap' context as needed
soap_destroy(temp); // clean up 'temp' context, deletes deserialized data
soap_end(temp);     // clean up 'temp' context, deletes deserialized data
soap_free(temp);
soap_free(soap);
~~~

@see `::soap_copy`, `::soap_copy_context`, `::soap_destroy`, `::soap_end`.
*/
void soap_delegate_deletion(
    struct soap *soap,    ///< source `::soap` context
    struct soap *soap_to) ///< target `::soap` context
  ;

/// Set SOAP version (0 = no SOAP, 1 = SOAP 1.1, 2 = SOAP 1.2)
/**
This function sets (or overrides) the SOAP version to use when sending a message.  This function can be used prior to a client-side call to select the SOAP version to use for the request message (assuming the generated code does not fix the version already) or in a service operation to select the SOAP version of the response message.  The response message of a service operation normally uses the same SOAP version of the SOAP request message received.
*/
void soap_set_version(
    struct soap *soap, ///< `::soap` contexr
    short version)     ///< SOAP version (0 = REST (no SOAP), 1 = SOAP 1.1, 2 = SOAP 1.2)
  ;

/** @} */

/**
\defgroup group_callbacks Callback functions
@brief This module defines the callback functions of the `::soap` context to modify its behavior, as is done by plugins

HTTP callbacks:
- `::soap::fpost`
- `::soap::fresponse`
- `::soap::fposthdr`
- `::soap::fparse`
- `::soap::fparsehdr`
- `::soap::fget`
- `::soap::fput`
- `::soap::fdel`
- `::soap::fopt`
- `::soap::fhead`
- `::soap::fform`

XML and SOAP callbacks:
- `::soap::fheader`
- `::soap::fignore`
- `::soap::fsvalidate`
- `::soap::fwvalidate`
- `::soap::fseterror`

Socket connection callbacks:
- `::soap::fopen`
- `::soap::faccept`
- `::soap::fclose`
- `::soap::fresolve`
- `::soap::fconnect`
- `::soap::fdisconnect`
- `::soap::fclosesocket`
- `::soap::fshutdownsocket`
- `::soap::fpoll`

IO callbacks:
- `::soap::frecv`
- `::soap::fsend`

Server keep-alive loop callback:
- `::soap::fserveloop`

Memory allocation callback:
- `::soap::fmalloc`

Streaming DIME attachment callbacks:
- `::soap::fdimereadopen`
- `::soap::fdimeread`
- `::soap::fdimereadclose`
- `::soap::fdimewriteopen`
- `::soap::fdimewrite`
- `::soap::fdimewriteclose`

Streaming MIME/MTOM attachment callbacks:
- `::soap::fmimereadopen`
- `::soap::fmimeread`
- `::soap::fmimereadclose`
- `::soap::fmimewriteopen`
- `::soap::fmimewrite`
- `::soap::fmimewriteclose`

OpenSSL and GNUTLS SSL/TLS callbacks:
- `::soap::fsslauth`
- `::soap::fsslverify`

To pass user-specified data to callbacks and plugins, assign a value to the `::soap::user` variable of the context which can be accessed within the callback or plugin.

@{
*/

/** @} */

/**
\defgroup group_ssl SSL/TLS context and functions
@brief This module defines functions to set the SSL/TLS context for HTTPS and WS-Security

@{
*/

/// SSL/TLS (HTTPS) client and server context flags for `::soap_ssl_client_context` and `::soap_ssl_server_context`, respectively, flags can be combined with `|` (bit-wise or)
/**
The SSL/TLS flags are:
- `#SOAP_SSL_DEFAULT`
- `#SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE`
- `#SOAP_SSL_NO_AUTHENTICATION`
- `#SOAP_SSL_NO_DEFAULT_CA_PATH`
- `#SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION`
- `#SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION`
- `#SOAP_SSL_RSA`
- `#SOAP_SSL_SKIP_HOST_CHECK`
- `#SOAP_SSLv3_TLSv1`
- `#SOAP_SSLv3`
- `#SOAP_TLSv1`
- `#SOAP_TLSv1_0`
- `#SOAP_TLSv1_1`
- `#SOAP_TLSv1_2`
- `#SOAP_TLSv1_3`
*/
typedef unsigned short soap_ssl_flags;

/// `::soap_ssl_flags` flag with `#SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION` and `#SOAP_TLSv1` enabled by default
/**
@see `::soap_ssl_server_context`, `::soap_ssl_client_context`.
*/
#define SOAP_SSL_DEFAULT (SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION | SOAP_TLSv1)

/// `::soap_ssl_flags` flag value to allow self-signed and expired certificates and those without CRL to be used for authentication
/**
@see `::soap_ssl_server_context`, `::soap_ssl_client_context`.
*/
#define SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE

/// `::soap_ssl_flags` flag value to disable authentication of the peer
/**
This flag should be used sparingly such as for testing only.

@see `::soap_ssl_server_context`, `::soap_ssl_client_context`.
*/
#define SOAP_SSL_NO_AUTHENTICATION

/// `::soap_ssl_flags` flag value to prevent OpenSSL from calling `SSL_CTX_set_default_verify_paths`
/**
@see `::soap_ssl_server_context`, `::soap_ssl_client_context`.
*/
#define SOAP_SSL_NO_DEFAULT_CA_PATH

/// `::soap_ssl_flags` flag for servers to require clients to authenticate to servers during the HTTPS handshake
/**
This flag requires clients connected to the server to authenticate to the server.  The `::soap_ssl_server_context` must specify `cafile` and/or `capath` parameters with certificates (CA root and/or server certificates) to authenticate clients.

@see `::soap_ssl_server_context`.
*/
#define SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION

/// `::soap_ssl_flags` flag for clients to require servers to authenticate to clients during the HTTPS handshake
/**
This flag requires connected servers to authenticate to the client.  The `::soap_ssl_client_context` must specify `cafile` and/or `capath` parameters with certificates (CA root and/or server certificates) to authenticate the server.

@see `::soap_ssl_client_context`.
*/
#define SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION

/// `::soap_ssl_flags` flag value to use RSA instead of DH (automatically set when no DH parameter is specified)
#define SOAP_SSL_RSA

/// `::soap_ssl_flags` flag for clients to allow clients to skip common name checks against the host name of the server
#define SOAP_SSL_SKIP_HOST_CHECK

/// `::soap_ssl_flags` flag value to enable SSL v3 but disable TLS, should be used for legacy purposes only
/**
This flag can be used in combination with other SSL and TLS protocol flags, using `|` (bit-wise or).

@see `#SOAP_SSLv3`, `#SOAP_SSLv3_TLSv1`, `#SOAP_TLSv1`, `#SOAP_TLSv1_0`, `#SOAP_TLSv1_1`, `#SOAP_TLSv1_2`, `#SOAP_TLSv1_3`.
*/
#define SOAP_SSLv3

/// `::soap_ssl_flags` flag value to enable both SSL v3 and TLS 1 (TLS 1.0 to max)
/**
This flag can be used in combination with other SSL and TLS protocol flags, using `|` (bit-wise or).

@see `#SOAP_SSLv3`, `#SOAP_SSLv3_TLSv1`, `#SOAP_TLSv1`, `#SOAP_TLSv1_0`, `#SOAP_TLSv1_1`, `#SOAP_TLSv1_2`, `#SOAP_TLSv1_3`.
*/
#define SOAP_SSLv3_TLSv1

/// `::soap_ssl_flags` flag value to enable TLS v1 (TLS 1.0 and higher) and to disable SSL v3
/**
This flag can be used in combination with other SSL and TLS protocol flags, using `|` (bit-wise or).

@see `#SOAP_SSLv3`, `#SOAP_SSLv3_TLSv1`, `#SOAP_TLSv1`, `#SOAP_TLSv1_0`, `#SOAP_TLSv1_1`, `#SOAP_TLSv1_2`, `#SOAP_TLSv1_3`.
*/
#define SOAP_TLSv1

/// `::soap_ssl_flags` flag value to enable TLS 1.0
/**
This flag can be used in combination with other SSL and TLS protocol flags, using `|` (bit-wise or).

@see `#SOAP_SSLv3`, `#SOAP_SSLv3_TLSv1`, `#SOAP_TLSv1`, `#SOAP_TLSv1_0`, `#SOAP_TLSv1_1`, `#SOAP_TLSv1_2`, `#SOAP_TLSv1_3`.
*/
#define SOAP_TLSv1_0

/// `::soap_ssl_flags` flag value to enable TLS 1.1
/**
This flag can be used in combination with other SSL and TLS protocol flags, using `|` (bit-wise or).

@see `#SOAP_SSLv3`, `#SOAP_SSLv3_TLSv1`, `#SOAP_TLSv1`, `#SOAP_TLSv1_0`, `#SOAP_TLSv1_1`, `#SOAP_TLSv1_2`, `#SOAP_TLSv1_3`.
*/
#define SOAP_TLSv1_1

/// `::soap_ssl_flags` flag value to enable TLS 1.2
/**
This flag can be used in combination with other SSL and TLS protocol flags, using `|` (bit-wise or).

@see `#SOAP_SSLv3`, `#SOAP_SSLv3_TLSv1`, `#SOAP_TLSv1`, `#SOAP_TLSv1_0`, `#SOAP_TLSv1_1`, `#SOAP_TLSv1_2`, `#SOAP_TLSv1_3`.
*/
#define SOAP_TLSv1_2

/// `::soap_ssl_flags` flag value to enable TLS 1.3
/**
This flag can be used in combination with other SSL and TLS protocol flags, using `|` (bit-wise or).

@see `#SOAP_SSLv3`, `#SOAP_SSLv3_TLSv1`, `#SOAP_TLSv1`, `#SOAP_TLSv1_0`, `#SOAP_TLSv1_1`, `#SOAP_TLSv1_2`, `#SOAP_TLSv1_3`.
*/
#define SOAP_TLSv1_3

/// Initialize the SSL/TLS library
/**
This function initializes the SSL/TLS library's global state and should be called just once per application.  This function is called by the engine when the SSL/TLS library is used but not yet initialized.  It is strongly recommended to call `::soap_ssl_init` once in a multi-threaded application before any threads are started.  Furthermore, all OpenSSL versions prior to 1.1.0 require mutex locks to be explicitly set up in your code for multi-threaded applications by calling `CRYPTO_thread_setup()` and `CRYPTO_thread_cleanup()`.

@warning If an application initializes the SSL/TLS library such as OpenSSL by loading algorithms, then `::soap_ssl_noinit` should be used instead before a `::soap` context is used to ensure that OpenSSL is not initialized twice.

@see `::soap_ssl_init`, `::CRYPTO_thread_setup`, `::CRYPTO_thread_cleanup`.
*/
void soap_ssl_init(void)
  ;

/// Do not initalized the SSL/TLS library
/**
This function prevents (re)initialization of the SSL/TLS library's global state and should be called when an application initializes the SSL/TLS library such as OpenSSL by loading algorithms.  OpenSSL may misbehave when `SSL_library_init` and `OpenSSL_add_all_algorithms` are called more than once, e.g. by the application and again by the gSOAP engine.  The latter is avoided by calling `::soap_ssl_noinit` before threads are started and before a `::soap` context is created.
*/
void soap_ssl_noinit(void)
  ;

/// Initialize the server-side SSL/TLS context
/**
This function initializes the server-side SSL/TLS context of OpenSSL or GNUTLS library.  The `flags` parameter initializes the context with a combination of `::soap_ssl_flags`.  The `keyfile` parameter when non-NULL is the server's private key PEM file, typically concatenated with its certificate in the PEM file.  The `password` parameter when non-NULL is used to unlock the password-protected private key in the key file.  The `cafile` parameter when non-NULL is used to authenticate clients when the `#SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION` flag is used and contains the client or CA certificate(s).  Alternatively, a directory name `capath` can be specified to point to a directory with the certificate(s).  The `dhfile` parameter when non-NULL is a file with DH parameters to use DH instead of RSA.  Alternatively, the `dhfile` parameter can be a numeric string value greater than 512 to let the engine generate the DH parameters, but beware this can take a while to execute.  The `randfile` parameter when non-NULL can be used to seed the PRNG using the specified file with random data.  The `sid` parameter when non-NULL is used for server-side session caching using a specified unique name per server.  Returns `#SOAP_OK` or a `::soap_status` error code.

All strings passed to this function except `sid` must be persistent in memory until the SSL/TLS context is implicitly deleted when the `::soap` context is deleted.

All OpenSSL versions prior to 1.1.0 require mutex locks to be explicitly set up in your code for multi-threaded applications by calling `CRYPTO_thread_setup()` and `CRYPTO_thread_cleanup()`.

After `::soap_ssl_server_context` initialization you can select a specific cipher list using OpenSSL function `SSL_CTX_set_cipher_list(soap->ctx, "...")`.  When client authentication is required with CRLs, you can use `::soap_ssl_crl` to specify a CRL file and to use any CRLs provided with SSL/TLS handshakes.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
if (soap_ssl_server_context(soap,
      SOAP_SSL_DEFAULT, // authenticate, use TLSv1.0 to 1.3
      "server.pem",     // private key and certificate
      "password",       // password to read server.pem
      NULL,
      NULL,
      NULL,
      NULL,
      "my_unique_server_id123" // server identification to enable SSL session caching to speed up TLS
      ))
{
  soap_print_fault(soap, stderr);
  exit(EXIT_FAILURE);
}
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
if (soap_ssl_server_context(soap,
      SOAP_TLSv1_1 | SOAP_TLSv1_2, // authenticate, use TLSv1.1 or TLSv1.2 only
      "server.pem",                // private key and certificate
      "password",                  // password to read server.pem
      NULL,
      NULL,
      "dh1024.pem",                // use DH with 1024 bits
      NULL,
      "my_unique_server_id123"     // identification to enable SSL session caching to speed up TLS
      ))
{
  soap_print_fault(soap, stderr);
  exit(EXIT_FAILURE);
}
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
if (soap_ssl_server_context(soap,
      SOAP_TLSv1_1 | SOAP_TLSv1_2, // authenticate, use TLSv1.1 or TLSv1.2 only
      "server.pem",                // private key and certificate
      "password",                  // password to read server.pem
      NULL,
      NULL,
      "1024",                      // generate DH with 1024 bits (takes a while to execute)
      NULL,
      "my_unique_server_id123"     // identification to enable SSL session caching to speed up TLS
      ))
{
  soap_print_fault(soap, stderr);
  exit(EXIT_FAILURE);
}
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
if (soap_ssl_server_context(soap,
      SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION, // require clients to authenticate
      "server.pem",                           // private key and certificate
      "password",                             // password to read server.pem
      "cacert.pem",                           // certificate to authenticate clients
      NULL,
      NULL,
      NULL,
      soap_rand_uuid(soap, NULL) // identification to enable SSL session caching to speed up TLS
      )
 || soap_ssl_crl(soap, "crl.pem"))            // specify CRLs
{
  soap_print_fault(soap, stderr);
  exit(EXIT_FAILURE);
}
// specify a cipher list
SSL_CTX_set_cipher_list(soap->ctx, "HIGH:!aNULL:!eNULL:!EXPORT:!DES:!MD5:!PSK:!RC4:!DH");
~~~

@note Requires compilation with `#WITH_OPENSSL` or `#WITH_GNUTLS`.

@see `#SOAP_SSL_RSA_BITS`, `::soap_ssl_client_context`, `::soap_ssl_crl`.
*/
int soap_ssl_server_context(
    struct soap *soap,    ///< `::soap` context
    soap_ssl_flags flags, ///< SSL/TLS context initialization flags
    const char *keyfile,  ///< private key file in PEM format or NULL
    const char *password, ///< password to unlock the private key or NULL
    const char *cafile,   ///< file with certificates in PEM format or NULL
    const char *capath,   ///< directory to certificates
    const char *dhfile,   ///< file with DH parameters or numeric string value to generate DH parameters or NULL
    const char *randfile, ///< file to see the PRNG or NULL
    const char *sid)      ///< a unique server id for session caching or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Initialize the client-side SSL/TLS context
/**
This function initializes the client-side SSL/TLS context of the OpenSSL or GNUTLS library.  The `flags` parameter initializes the context with a combination of `::soap_ssl_flags`.  The `keyfile` parameter when non-NULL is the server's private key PEM file, typically concatenated with its certificate in the PEM file.  The `password` parameter when non-NULL is used to unlock the password-protected private key in the key file.  The `cafile` parameter when non-NULL is used to authenticate clients when the `#SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION` flag is used and contains the client or CA certificate(s).  Alternatively, a directory name `capath` can be specified to point to a directory with the certificate(s).  The `randfile` parameter when non-NULL can be used to seed the PRNG using the specified file with random data.  Returns `#SOAP_OK` or a `::soap_status` error code.

All strings passed to this function must be persistent in memory until the SSL/TLS context is implicitly deleted when the `::soap` context is deleted.

After `::soap_ssl_client_context` initialization you can select a specific cipher list using OpenSSL function `SSL_CTX_set_cipher_list(soap->ctx, "...")`.  When authentication requires the use of CRLs, you can use `::soap_ssl_crl` to specify a CRL file and to use any CRLs provided with SSL/TLS handshakes.

All OpenSSL versions prior to 1.1.0 require mutex locks to be explicitly set up in your code for multi-threaded applications by calling `CRYPTO_thread_setup()` and `CRYPTO_thread_cleanup()`.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
if (soap_ssl_client_context(soap,
      SOAP_SSL_NO_AUTHENTICATION, // do not authenticate the server
      NULL,
      NULL,
      NULL,
      NULL,
      NULL
      ))
{
  soap_print_fault(soap, stderr);
  exit(EXIT_FAILURE);
}
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
if (soap_ssl_client_context(soap,
      SOAP_SSL_DEFAULT, // authenticate, use TLSv1.0 to 1.3
      NULL,
      NULL,
      "cacerts.pem",    // certificates to authenticate servers
      NULL,
      NULL
      ))
{
  soap_print_fault(soap, stderr);
  exit(EXIT_FAILURE);
}
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
if (soap_ssl_client_context(soap,
      SOAP_SSLv3_TLSv1 |                  // authenticate, use SSLv3, TLSv1.0 to 1.3
      SOAP_SSL_SKIP_HOST_CHECK |          // but skip host name checking
      SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE, // and allow expired certificates
      NULL,
      NULL,
      "cacerts.pem",                      // certificates to authenticate servers
      NULL,
      NULL
      ))
{
  soap_print_fault(soap, stderr);
  exit(EXIT_FAILURE);
}
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
if (soap_ssl_client_context(soap,
      SOAP_TLSv1_1 | SOAP_TLSv1_2, // authenticate, use TLSv1.1 or 1.2
      "client.pem",                // private key and certificate (allowing server to authenticate the client)
      "password",                  // password to read client.pem
      "cacerts.pem",               // certificates to authenticate servers
      NULL,
      NULL
      ))
{
  soap_print_fault(soap, stderr);
  exit(EXIT_FAILURE);
}
~~~

~~~{.cpp}
#include "soapH.h"

// define a verification callback that allows self-signed certificates
int ssl_verify_callback_allow_self_signed_cert(int ok, X509_STORE_CTX *store)
{
  if (!ok && X509_STORE_CTX_get_error(store) == X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN)
  {
    X509_STORE_CTX_set_error(store, X509_V_OK);
    ok = 1;
  }
  return ok;
}

struct soap *soap = soap_new();
soap->fsslverify = ssl_verify_callback_allow_self_signed_cert;
if (soap_ssl_client_context(soap,
      SOAP_SSL_DEFAULT, // authenticate, use TLSv1.0 to 1.3
      "client.pem",     // private key and certificate (allowing server to authenticate the client)
      "password",       // password to read client.pem
      "cacerts.pem",    // certificates to authenticate servers
      NULL,
      NULL
      ))
{
  soap_print_fault(soap, stderr);
  exit(EXIT_FAILURE);
}

~~~

@note Requires compilation with `#WITH_OPENSSL` or `#WITH_GNUTLS`.

@see `#SOAP_SSL_RSA_BITS`, `::soap_ssl_server_context`, `::soap_ssl_crl`, `::soap::fsslverify`.
*/
int soap_ssl_client_context(
    struct soap *soap,    ///< `::soap` context
    soap_ssl_flags flags, ///< SSL/TLS context initialization flags
    const char *keyfile,  ///< private key file in PEM format or NULL
    const char *password, ///< password to unlock the private key or NULL
    const char *cafile,   ///< file with certificates in PEM format or NULL
    const char *capath,   ///< directory to certificates
    const char *randfile) ///< file to see the PRNG or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Enable SSL/TLS CRLs
/**
This function enables SSL/TLS CRL checking during the SSL/TLS handshake.  The `crlfile` when non-NULL specifies a file with CRLs in PEM format.  Returns `#SOAP_OK` or a `::soap_status` error code.

@note Requires compilation with `#WITH_OPENSSL` or `#WITH_GNUTLS`.

@see `::soap_ssl_server_context`, `::soap_ssl_client_context`, `::soap_ssl_crl`.
*/
int soap_ssl_crl(
    struct soap *soap,   ///< `::soap` context
    const char *crlfile) ///< CRL file in PEM format or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Accept SSL/TLS connection
/**
This function should be called after calling `::soap_accept` to perform the SSL/TLS handshake with a connected client.  This function enforces HTTPS connections that are initialized with `::soap_ssl_server_context`.  Returns `#SOAP_OK` or a `::soap_status` error code.

@par Examples:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  if (soap_ssl_server_context(soap, SOAP_SSL_DEFAULT, "server.pem", "password", NULL, NULL, NULL, NULL, NULL))
    exit(EXIT_FAILURE);
  soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      if (soap_valid_socket(soap_accept(soap)))
      {
        if (soap_ssl_accept(soap) || soap_serve(soap))
          soap_print_fault(soap, stderr);
      }
      else if (soap->errnum) // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}
~~~

~~~{.cpp}
#include "soapH.h"
#include "plugin/threads.h"

int main()
{
  struct soap *soap = soap_new();
  soap->bind_flags = SO_REUSEADDR;            // immediate port reuse
  soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      if (soap_valid_socket(soap_accept(soap)))
      {
        THREAD_TYPE tid;
        struct soap *tsoap = soap_copy(soap);
        if (!tsoap)
          soap_force_closesock(soap);
        else
          while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap))
            sleep(1); // failed, try again
      }
      else if (soap->errnum) // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}

void *process_request(struct soap *soap)
{
  THREAD_DETACH(THREAD_ID);
  if (soap_ssl_accept(soap) == SOAP_OK)
    soap_serve(soap);
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}
~~~

@note Requires compilation with `#WITH_OPENSSL` or `#WITH_GNUTLS` and SSL server context initialization with `::soap_ssl_server_context`.

@see `#WITH_OPENSSL`, `#WITH_GNUTLS`, `::soap_ssl_server_context`, `::soap_ssl_crl`, `::soap_bind`, `::soap_accept`.
*/
int soap_ssl_accept(struct soap *soap)
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Setup function for OpenSSL versions prior to 1.1.1 to set locks for multi-threaded SSL/TLS applications
/**
The `CRYPTO_thread_setup()` and `CRYPTO_thread_setup` cleanup functions should be defined in the source code of a multi-threaded SSL/TLS application as follows, requiring gsoap/plugin/threads.h:

~~~{.cpp}
#include "plugin/threads.h"

struct CRYPTO_dynlock_value
{
  MUTEX_TYPE mutex;
};

static MUTEX_TYPE *mutex_buf = NULL;

static struct CRYPTO_dynlock_value *dyn_create_function(const char *file, int line)
{
  struct CRYPTO_dynlock_value *value;
  (void)file; (void)line;
  value = (struct CRYPTO_dynlock_value*)OPENSSL_malloc(sizeof(struct CRYPTO_dynlock_value));
  if (value)
    MUTEX_SETUP(value->mutex);
  return value;
}

static void dyn_lock_function(int mode, struct CRYPTO_dynlock_value *l, const char *file, int line)
{
  (void)file; (void)line;
  if (mode & CRYPTO_LOCK)
    MUTEX_LOCK(l->mutex);
  else
    MUTEX_UNLOCK(l->mutex);
}

static void dyn_destroy_function(struct CRYPTO_dynlock_value *l, const char *file, int line)
{
  (void)file; (void)line;
  MUTEX_CLEANUP(l->mutex);
  OPENSSL_free(l);
}

static void locking_function(int mode, int n, const char *file, int line)
{
  (void)file; (void)line;
  if (mode & CRYPTO_LOCK)
    MUTEX_LOCK(mutex_buf[n]);
  else
    MUTEX_UNLOCK(mutex_buf[n]);
}

static unsigned long id_function()
{
  return (unsigned long)THREAD_ID;
}

int CRYPTO_thread_setup()
{
  int i;
  mutex_buf = (MUTEX_TYPE*)OPENSSL_malloc(CRYPTO_num_locks() * sizeof(MUTEX_TYPE));
  if (!mutex_buf)
    return SOAP_EOM;
  for (i = 0; i < CRYPTO_num_locks(); i++)
    MUTEX_SETUP(mutex_buf[i]);
  CRYPTO_set_id_callback(id_function);
  CRYPTO_set_locking_callback(locking_function);
  CRYPTO_set_dynlock_create_callback(dyn_create_function);
  CRYPTO_set_dynlock_lock_callback(dyn_lock_function);
  CRYPTO_set_dynlock_destroy_callback(dyn_destroy_function);
  return SOAP_OK;
}

void CRYPTO_thread_cleanup()
{
  int i;
  if (!mutex_buf)
    return;
  CRYPTO_set_id_callback(NULL);
  CRYPTO_set_locking_callback(NULL);
  CRYPTO_set_dynlock_create_callback(NULL);
  CRYPTO_set_dynlock_lock_callback(NULL);
  CRYPTO_set_dynlock_destroy_callback(NULL);
  for (i = 0; i < CRYPTO_num_locks(); i++)
    MUTEX_CLEANUP(mutex_buf[i]);
  OPENSSL_free(mutex_buf);
  mutex_buf = NULL;
}
~~~

@par Example:

~~~{.cpp}
int main()
{
  soap_ssl_init();
  if (CRYPTO_thread_setup())
    exit(EXIT_FAILURE);
  ... // run the application
  CRYPTO_thread_cleanup();
}

~~~

@see `::soap_ssl_init`, `::CRYPTO_thread_cleanup`.
*/
int CRYPTO_thread_setup();

/// Cleanup function for OpenSSL versions prior to 1.1.1
/**
@see `::soap_ssl_init`, `::CRYPTO_thread_setup`.
*/
int CRYPTO_thread_cleanup();

/** @} */

/**
\defgroup group_io HTTP and IO functions
@brief This module defines functions for HTTP operations and functions for receiving and sending data

This module defines the following client-side functions:
- `::soap_GET`
- `::soap_PUT`
- `::soap_PATCH`
- `::soap_POST`
- `::soap_DELETE`
- `::soap_connect_command`
- `::soap_connect`
- `::soap_recv_empty_response`

This module defines the following server-side functions:
- `::soap_bind`
- `::soap_accept`
- `::soap_ssl_accept` (defined in the \ref group_ssl module)
- `::soap_serve` (generated by soapcpp2)
- `::soap_begin_serve`
- `::soap_serve_request` (generated by soapcpp2)
- `::soap_response`
- `::soap_send_empty_response`

This module defines the following input/output functions:
- `::soap_begin_recv`
- `::soap_end_recv`
- `::soap_begin_send`
- `::soap_end_send`
- `::soap_begin_count`
- `::soap_end_count`
- `::soap_closesock`
- `::soap_force_closesock`
- `::soap_close_connection`
- `::soap_send`
- `::soap_send_raw`
- `::soap_http_has_body`
- `::soap_http_get_body`
- `::soap_getline`
- `::soap_get0`
- `::soap_get1`
- `::soap_poll`
- `::soap_ready`

This module defines the following SOAP message input/output functions:
- `::soap_envelope_begin_in`
- `::soap_envelope_end_in`
- `::soap_envelope_begin_out`
- `::soap_envelope_end_out`
- `::soap_body_begin_in`
- `::soap_body_end_in`
- `::soap_body_begin_out`
- `::soap_body_end_out`
- `::soap_recv_fault`
- `::soap_send_fault`
- `::soap_recv_header`
- `::soap_putheader`
- `::soap_serializeheader`

This module defines three HTTP-related plugins;
- `::http_get` HTTP GET plugin for server-side handling of HTTP GET requests by stand-alone servers as a more capable alternative to setting the `::soap::fget` callback.
- `::http_post` HTTP POST plugin for server-side handling of HTTP POST, PUT, PATCH, and DELETE request by stand-alone servers as a more capable alternative to setting the `::soap::fput`, `::soap::fpatch`, and `::soap::fdel` callbacks.
- `::http_form` HTTP POST form plugin for server-side handling of HTTP application/x-www-form-urlencoded data.
- `::http_pipe` HTTP pipelining plugin to support server-side and client-side pipelined HTTP messages.

@{
*/

/// HTTP GET content from server
/**
This function connects to the server specified by the `endpoint` URL string, using HTTP GET and the HTTP SOAP Action header specified by the `action` string (or NULL).  This call should be followed by `::soap_end_send` to send an empty HTTP body.  Upon successful completion, messages can be received from the server.  Returns `#SOAP_OK` or a `::soap_status` error code.

This function is used by the soapcpp2-generated `soap_GET_T` functions for types `T` to HTTP GET the XML deserialized value of type `T` from a server.

To implement server-side HTTP GET handling use `::soap::fget`.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
char *response = NULL;
size_t response_len;
if (soap_GET(soap, "http://www.example.com/API/GET", NULL)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

@see `::soap::connect_flags`, `::soap::connect_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`, `::soap_PUT`, `::soap_PATCH`, `::soap_POST`, `::soap_DELETE`.
*/
int soap_GET(struct soap *soap, ///< `::soap` context
    const char *endpoint,       ///< URL string
    const char *action)         ///< SOAP Action string or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// HTTP PUT content to server
/**
This function connects to the server specified by the `endpoint` URL string, using HTTP PUT and the HTTP SOAP Action header specified by the `action` string (or NULL).  The HTTP content type of the data sent to the server is specified by `type`.  If the mode is not `#SOAP_IO_CHUNK` (preferred) or `#SOAP_IO_STORE` then this function temporarily sets the mode of the context to `#SOAP_IO_STORE` to compute the HTTP content length.  Upon successful completion, messages can be sent to the server and an empty response should be received by calling `::soap_recv_empty_response`.  Returns `#SOAP_OK` or a `::soap_status` error code.

If the `::soap::http_content` string is non-NULL, this string is used as the HTTP content type to be included in the HTTP header.  Otherwise, the HTTP content header is `text/xml` by default.  The `::soap::http_content` string is automatically reset to NULL after the HTTP POST call and has to be set again for the next call.

This function is used by the soapcpp2-generated `soap_PUT_T` functions for types `T` to HTTP PUT the XML serialized value of type `T` to a server.

To implement server-side HTTP PUT handling use `::soap::fput`.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK (preferred) or SOAP_IO_STORE
if (soap_PUT(soap, "http://www.example.com/API/PUT", NULL, "text/xml; charset=utf-8")
 || soap_send(soap, "<doc title=\"Example\">Some text</doc>\n")
 || soap_end_send(soap)
 || soap_recv_empty_response(soap))
  soap_print_fault(soap, stderr);
soap_closesock(soap);
~~~

@see `::soap::connect_flags`, `::soap::connect_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`, `::soap::http_content`, `::soap_GET`, `::soap_PATCH`, `::soap_POST`, `::soap_DELETE`.
*/
int soap_PUT(
    struct soap *soap,    ///< `::soap` context
    const char *endpoint, ///< URL string
    const char *action,   ///< SOAP Action string or NULL
    const char *type)     ///< HTTP content type string
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// HTTP PATCH content to server
/**
This function connects to the server specified by the `endpoint` URL string, using HTTP PATCH and the HTTP SOAP Action header specified by the `action` string (or NULL).  The HTTP content type of the data sent to the server is specified by `type`.  If the mode is not `#SOAP_IO_CHUNK` (preferred) or `#SOAP_IO_STORE` then this function temporarily sets the mode of the context to `#SOAP_IO_STORE` to compute the HTTP content length.  Upon successful completion, messages can be sent to the server and an empty response should be received by calling `::soap_recv_empty_response`.  Returns `#SOAP_OK` or a `::soap_status` error code.

If the `::soap::http_content` string is non-NULL, this string is used as the HTTP content type to be included in the HTTP header.  Otherwise, the HTTP content header is `text/xml` by default.  The `::soap::http_content` string is automatically reset to NULL after the HTTP POST call and has to be set again for the next call.

This function is used by the soapcpp2-generated `soap_PATCH_T` functions for types `T` to HTTP PATCH the XML serialized value of type `T` to a server.

To implement server-side HTTP PATCH handling use `::soap::fpatch`.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK (preferred) or SOAP_IO_STORE
if (soap_PATCH(soap, "http://www.example.com/API/PATCH", NULL, "application/json; charset=utf-8")
 || soap_send(soap, "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n")
 || soap_end_send(soap)
 || soap_recv_empty_response(soap))
  soap_print_fault(soap, stderr);
soap_closesock(soap);
~~~

@see `::soap::connect_flags`, `::soap::connect_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`, `::soap::http_content`, `::soap_GET`, `::soap_PUT`, `::soap_POST`, `::soap_DELETE`.
*/
int soap_PATCH(
    struct soap *soap,    ///< `::soap` context
    const char *endpoint, ///< URL string
    const char *action,   ///< SOAP Action string or NULL
    const char *type)     ///< HTTP content type string
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// HTTP POST content to server
/**
This function connects to the server specified by the `endpoint` URL string, using HTTP POST and the HTTP SOAP Action header specified by the `action` string (or NULL).  The HTTP content type of the data sent to the server is specified by `type`.  If the mode is not `#SOAP_IO_CHUNK` (preferred) or `#SOAP_IO_STORE` then this function temporarily sets the mode of the context to `#SOAP_IO_STORE` to compute the HTTP content length.  Upon successful completion, messages can be sent to and received from the server.  Returns `#SOAP_OK` or a `::soap_status` error code.

If the `::soap::http_content` string is non-NULL, this string is used as the HTTP content type to be included in the HTTP header.  Otherwise, the HTTP content header is `text/xml` by default.  The `::soap::http_content` string is automatically reset to NULL after the HTTP POST call and has to be set again for the next call.

This function is used by the soapcpp2-generated `soap_POST_send_T` functions for types `T` to HTTP POST the XML serialized value of type `T` to a server.  Use `soap_POST_recv_T` to receive a HTTP POST response deserialized into a value of (another) type `T`.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK (preferred) or SOAP_IO_STORE
char *response = NULL;
size_t response_len;
if (soap_POST(soap, "http://www.example.com/API/POST", NULL, "application/json; charset=utf-8")
 || soap_send(soap, "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n")
 || soap_end_send(soap)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

@see `::soap::connect_flags`, `::soap::connect_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`, `::soap::http_content`, `::soap_GET`, `::soap_PUT`, `::soap_PATCH`, `::soap_DELETE`.
*/
int soap_POST(
    struct soap *soap,    ///< `::soap` context
    const char *endpoint, ///< URL string
    const char *action,   ///< SOAP Action string or NULL
    const char *type)     ///< HTTP content type string
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// HTTP DELETE content from server
/**
This function connects to the server specified by the `endpoint` URL string, using HTTP DELETE.  No messages are sent and received.  Returns `#SOAP_OK` or a `::soap_status` error code.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
if (soap_DELETE(soap, "http://www.example.com/API/DELETE"))
  soap_print_fault(soap, stderr);
~~~

To implement server-side HTTP DELETE handling use `::soap::fdel`.

@see `::soap::connect_flags`, `::soap::connect_timeout`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`, `::soap::recv_maxlength`, `::soap_GET`, `::soap_PUT`, `::soap_PATCH`, `::soap_POST`.
*/
int soap_DELETE(
    struct soap *soap,    ///< `::soap` context
    const char *endpoint) ///< URL string
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// HTTP command methods for `::soap_connect_command`
/**
The choice of options are:
`#SOAP_CONNECT`
`#SOAP_DEL`
`#SOAP_GET`
`#SOAP_HEAD`
`#SOAP_OPTIONS`
`#SOAP_PATCH`
`#SOAP_POST_FILE`
`#SOAP_POST`
`#SOAP_PUT`
*/
typedef int soap_http_command;

/// HTTP POST command code
/**
This code is to be used with `::soap_connect_command` and produces HTTP content type `text/xml; charset=utf-8`.  Use code `#SOAP_POST_FILE` to customize the content type by setting `::soap::http_content`.  Before calling `::soap_connect_command`, either the HTTP content length `::soap::count` must be determined with `::soap_begin_count` and `::soap_end_count`, or the mode of the context should be set to `#SOAP_IO_CHUNK` (preferred) or `#SOAP_IO_STORE`.  The `::soap_POST` function performs the same operation as `::soap_connect_command` with `#SOAP_POST_FILE` and sets the context mode to `#SOAP_IO_STORE` when required.  `::soap_POST` is recommended.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
char *response = NULL;
size_t response_len;
// HTTP GET
if (soap_connect_command(soap, SOAP_GET, endpoint, NULL)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // must use SOAP_IO_CHUNK or SOAP_IO_STORE
char *response = NULL;
size_t response_len;
// HTTP POST request with chunked transfer
if (soap_connect_command(soap, SOAP_POST, endpoint, NULL)
 || soap_send(soap, "<doc title=\"Example\">Some text</doc>\n")
 || soap_end_send(soap)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new(); // no SOAP_IO_CHUNK or SOAP_IO_STORE, see below
const char *request = "<doc title=\"Example\">Some text</doc>\n";
char *response = NULL;
size_t response_len;
// HTTP POST request, here we compute the content length instead of chunked transfer with SOAP_IO_CHUNK or storing the entire message with SOAP_IO_STORE
if (soap_begin_count(soap)
 || soap_send(soap, request)
 || soap_end_count(soap)
 || soap_connect_command(soap, SOAP_POST, endpoint, NULL)
 || soap_send(soap, request)
 || soap_end_send(soap)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

@see `::soap_connect_command`, `::soap_POST`
*/
#define SOAP_POST

/// HTTP POST command code with custom content type
/**
This code is to be used with `::soap_connect_command` and requires `::soap::http_content` to be set to the content type of the data to be sent.  Before calling `::soap_connect_command`, either the HTTP content length `::soap::count` must be determined with `::soap_begin_count` and `::soap_end_count`, or the mode of the context should be set to `#SOAP_IO_CHUNK` (preferred) or `#SOAP_IO_STORE`.  The `::soap_POST` function performs the same operation as `::soap_connect_command` with `#SOAP_POST_FILE` and sets the context mode to `#SOAP_IO_STORE` when required.  `::soap_POST` is recommended.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK (preferred) or SOAP_IO_STORE
char *response = NULL;
size_t response_len;
soap->http_content = "application/json; charset=utf-8";
if (soap_connect_command(soap, "http://www.example.com/API/POST", NULL, SOAP_POST_FILE)
 || soap_send(soap, "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n")
 || soap_end_send(soap)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~
*/
#define SOAP_POST_FILE

/// HTTP GET command code
/**
This code is to be used with `::soap_connect_command`.  The `::soap_GET` performs the same operation as `::soap_connect_command` with `#SOAP_GET`.  `::soap_GET` is recommended.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK or SOAP_IO_STORE
char *response = NULL;
size_t response_len;
if (soap_connect_command(soap, "http://www.example.com/API/GET", NULL, SOAP_GET)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~
*/
#define SOAP_GET

/// HTTP PUT command code with custom content type
/**
This code is to be used with `::soap_connect_command` and requires `::soap::http_content` to be set to the content type of the data to be sent.  The `::soap_PUT` performs the same operation as `::soap_connect_command` with `#SOAP_PUT` and is recommended.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK or SOAP_IO_STORE
soap->http_content = "text/xml; charset=utf-8";
if (soap_connect_command(soap, "http://www.example.com/API/PUT", NULL, SOAP_PUT)
 || soap_send(soap, "<doc title=\"Example\">Some text</doc>\n")
 || soap_end_send(soap)
 || soap_recv_empty_response(soap))
  soap_print_fault(soap, stderr);
soap_closesock(soap);
~~~
*/
#define SOAP_PUT

/// HTTP DEL command code
/**
This code is to be used with `::soap_connect_command`.  The `::soap_DELETE` performs the same operation as `::soap_connect_command` with `#SOAP_DEL`.  `::soap_DELETE` is recommended.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK or SOAP_IO_STORE
if (soap_connect_command(soap, "http://www.example.com/API/DELETE", NULL, SOAP_DEL)
 || soap_recv_empty_response(soap))
  soap_print_fault(soap, stderr);
soap_closesock(soap);
~~~
*/
#define SOAP_DEL

/// HTTP CONNECT command code
/**
This code is to be used with `::soap_connect_command`.
*/
#define SOAP_CONNECT

/// HTTP HEAD command code
/**
This code is to be used with `::soap_connect_command`.
*/
#define SOAP_HEAD

/// HTTP OPTIONS command code
/**
This code is to be used with `::soap_connect_command`.
*/
#define SOAP_OPTIONS

/// HTTP PATCH command code with custom content type
/**
This code is to be used with `::soap_connect_command` and requires `::soap::http_content` to be set to the content type of the data to be sent.  The `::soap_PATCH` function performs the same operation as `::soap_connect_command` with `#SOAP_PATCH`.  `::soap_PATCH` is recommended.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK or SOAP_IO_STORE
soap->http_content = "application/json; charset=utf-8";
if (soap_connect_command(soap, "http://www.example.com/API/PATCH", NULL, SOAP_PATCH)
 || soap_send(soap, "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n")
 || soap_end_send(soap)
 || soap_recv_empty_response(soap))
  soap_print_fault(soap, stderr);
soap_closesock(soap);
~~~
*/
#define SOAP_PATCH

/// Connect to a server
/**
This function connects to the server specified by the `endpoint` URL string, using the HTTP method specified by the `http_command` with a `::soap_http_command` value and using the HTTP SOAP Action header specified by the `action` string (or NULL).  Before calling `::soap_connect_command` with `#SOAP_POST`, `#SOAP_POST_FILE`, `#SOAP_PATCH` or `#SOAP_PUT`, either the HTTP content length `::soap::count` must be determined with `::soap_begin_count` and `::soap_end_count`, or the mode of the context should be set to `#SOAP_IO_CHUNK` (preferred) or `#SOAP_IO_STORE`.  ALso for HTTP methods `#SOAP_POST`, `#SOAP_POST_FILE`, `#SOAP_PATCH` and `#SOAP_PUT` the `::soap::http_content` string should be set before calling `::soap_connect_command`.  Upon successful completion of this function, messages can be sent to and/or received from the server.  Returns `#SOAP_OK` or a `::soap_status` error code.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK or SOAP_IO_STORE
char *response = NULL;
size_t response_len;
soap->http_content = "application/json; charset=utf-8";
// HTTP POST request with chunked transfer
if (soap_connect_command(soap, "http://www.example.com/API/POST", NULL, SOAP_POST_FILE)
 || soap_send(soap, "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n")
 || soap_end_send(soap)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
const char *request = "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n";
char *response = NULL;
size_t response_len;
soap->http_content = "application/json; charset=utf-8";
// HTTP POST request, here we compute the content length instead of chunked transfer with SOAP_IO_CHUNK or storing the entire message with SOAP_IO_STORE
if (soap_begin_count(soap)
 || soap_send(soap, request)
 || soap_end_count(soap)
 || soap_connect_command(soap, "http://www.example.com/API/POST", NULL, SOAP_POST_FILE)
 || soap_send(soap, request)
 || soap_end_send(soap)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

@see `#SOAP_GET`, `#SOAP_POST`, `#SOAP_POST_FILE`, `#SOAP_PATCH`, `#SOAP_PUT`, `::soap::connect_flags`, `::soap::connect_timeout`, `::soap_GET`, `::soap_POST`, `::soap_PUT`, `::soap_PATCH`.
*/
int soap_connect_command(
    struct soap *soap,              ///< `::soap` context
    soap_http_command http_command, ///< `::soap_http_command` with HTTP method code
    const char *endpoint,           ///< URL string
    const char *action)             ///< SOAP Action string or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Connect to a server using HTTP POST
/**
This function connects to the server specified by the `endpoint` URL string, using HTTP POST and the HTTP SOAP Action header specified by the `action` string (or NULL).  Upon successful completion, messages can be sent to and/or received from the server.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK or SOAP_IO_STORE
char *response = NULL;
size_t response_len;
if (soap_connect(soap, endpoint, NULL)
 || soap_send(soap, "<doc title=\"Example\">Some text</doc>\n")
 || soap_end_send(soap)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

@see `::soap::connect_flags`, `::soap::connect_timeout`.
*/
int soap_connect(
    struct soap *soap,    ///< `::soap` context
    const char *endpoint, ///< URL string
    const char *action)   ///< SOAP Action string or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Bind and listen to a port
/**
This function binds to the specified port and starts listening for client requests.  The `host` parameter when non-NULL is the name of the host on which this service runs.  The `port` parameter is the port number to bind, which must not be in use by another service.  The call may also fail if the port was recently in use by this service.  Use `SO_REUSEADDR` with `::soap::bind_flags` to immediately reuse the port, but use this option with caution to prevent "port stealing" attacks.  The `backlog` parameter is used with `listen`, which defines the maximum length for the queue of pending connections.  If a connection request arrives with the queue full, the client may receive an error with an indication of `ECONNREFUSED` or a connection reset.  Alternatively, if the underlying protocol supports retransmission, the request may be ignored so that retries may succeed.  Returns the `::soap::master` socket bound to the port or the invalid socket handle `#SOAP_INVALID_SOCKET` when an error occurred.

@note A small `backlog` value should be used with iterative (i.e. non-multi-threaded) servers to improve fairness among connecting clients, recommended is a `backlog` value between 2 and 10.  A smaller value increases fairness and defends against denial of service, but hampers performance because connection requests may be refused when the queue is full.  Also short `::soap::recv_timeout` and `::soap::send_timeout` values of a few seconds at the most should be used to prevent clients from using connections for long by terminating unacceptably slow message exchanges that exceed the timeout thresholds.  In the worst case, a connecting client may have to wait `backlog` * (`::soap::recv_timeout` + `::soap::send_timeout`) seconds to have the connection accepted by an iterative server when the backlog queue is full, or even longer when message sizes are very large, e.g. several MBs, requiring multiple message data packet exchanges.  Larger `backlog` values can be safely used with multi-threaded servers, such as 50 to 128, where 128 is a typical maximum on most operating systems.  The actual value does not matter when connections are accepted as soon as they arrive by `::soap_accept` and then handled by threads executing `::soap_ssl_accept` (for HTTPS) and `::soap_serve`.

This function effectively deployes a stand-alone server on the specified port.  There are three other alternatives for deploying services:
- CGI and FastCGI, see `#WITH_FASTCGI`.
- IIS web server, see the [ISAPI extension](../../isapi/html/index.html) for gSOAP.
- Apache web server, see the [Apache module](../../apache/html/index.html) for gSOAP.

@par Examples:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->bind_flags = SO_REUSEADDR;            // immediate port reuse
  soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      if (soap_valid_socket(soap_accept(soap)))
      {
        if (soap_serve(soap))
          soap_print_fault(soap, stderr);
      }
      else if (soap->errnum) // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}
~~~

~~~{.cpp}
#include "soapH.h"
#include "plugin/threads.h"

int main()
{
  struct soap *soap = soap_new();
  soap->bind_flags = SO_REUSEADDR;            // immediate port reuse
  soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      if (soap_valid_socket(soap_accept(soap)))
      {
        THREAD_TYPE tid;
        struct soap *tsoap = soap_copy(soap);
        if (!tsoap)
          soap_force_closesock(soap);
        else
          while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap))
            sleep(1); // failed, try again
      }
      else if (soap->errnum) // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}

void *process_request(struct soap *soap)
{
  THREAD_DETACH(THREAD_ID);
  soap_serve(soap);
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}
~~~

@see `#WITH_IPV6`, `#WITH_IPV6_V6ONLY`, `#SOAP_IO_UDP`, `::soap::bind_flags`, `::soap::bind_inet6`, `::soap::bind_v6only`, `::soap::rcvbuf`, `::soap::sndbuf`, `::soap::master`, `::soap_accept`, `::soap_ssl_accept`.
*/
SOAP_SOCKET soap_bind(
    struct soap *soap, ///< `::soap` context
    const char *host,  ///< name of the host or NULL
    int port,          ///< port number to bind
    int backlog)       ///< maximum queue length of pending requests
  /// @returns the `::soap::master` socket value or `#SOAP_INVALID_SOCKET` when an error occurred (check the return value with `#soap_valid_socket`)
  ;

/// Accept a connection with a client
/**
This function accepts a connection requested by a client on a given port that is bound with `::soap_bind` first to set `::soap::master`.  To accept HTTPS connections, call `::soap_ssl_accept` after this function to perform the HTTPS handshake with the client.  Returns the `::soap::socket` connected to the client or the invalid socket handle `#SOAP_INVALID_SOCKET` when an error occurred.

@par Examples:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      if (soap_valid_socket(soap_accept(soap)))
      {
        if (soap_serve(soap))
          soap_print_fault(soap, stderr);
      }
      else if (soap->errnum) // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}
~~~

~~~{.cpp}
#include "soapH.h"
#include "plugin/threads.h"

int main()
{
  struct soap *soap = soap_new();
  soap->bind_flags = SO_REUSEADDR;            // immediate port reuse
  soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
  soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      if (soap_valid_socket(soap_accept(soap)))
      {
        THREAD_TYPE tid;
        struct soap *tsoap = soap_copy(soap);
        if (!tsoap)
          soap_force_closesock(soap);
        else
          while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap))
            sleep(1); // failed, try again
      }
      else if (soap->errnum) // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}

void *process_request(struct soap *soap)
{
  THREAD_DETACH(THREAD_ID);
  soap_serve(soap);
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}
~~~

~~~{.cpp}
// alternative process_request function to accept HTTPS connections
void *process_request(struct soap *soap)
{
  THREAD_DETACH(THREAD_ID);
  if (soap_ssl_accept(soap) == SOAP_OK)
    soap_serve(soap);
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}
~~~

@see `#WITH_IPV6`, `#WITH_IPV6_V6ONLY`, `#SOAP_IO_UDP`, `::soap::accept_flags`, `::soap::bind_flags`, `::soap::bind_inet6`, `::soap::bind_v6only`, `::soap::rcvbuf`, `::soap::sndbuf`, `::soap::master`, `::soap_bind`, `::soap_ssl_accept`.
*/
SOAP_SOCKET soap_accept(struct soap *soap) ///< `::soap` context
  /// @returns the `::soap::socket` value or `#SOAP_INVALID_SOCKET` when an error occurred (check the return value with `#soap_valid_socket`)
  ;

/// Serve a pending request
/**
This function is auto-generated by soapcpp2 and serves the pending request on `::soap::socket` or on standard input/output `::soap::recvfd` and `::soap::sendfd`, e.g. for CGI and FastCGI services.  Returns `#SOAP_OK` or a `::soap_status` error code.

This auto-generated function implements the following behavior:

~~~{.cpp}
#include "soapH.h"

int soap_serve(struct soap *soap)
{
  // keep socket open for HTTP keep-alive for maximum of soap->max_keep_alive iterations when nonzero
  soap->keep_alive = soap->max_keep_alive + 1;
  do
  {
    if (soap->keep_alive > 0 && soap->max_keep_alive > 0)
      soap->keep_alive--;
    // parse HTTP headers and call GET, PUT, PATCH or DELETE callback when set
    if (soap_begin_serve(soap))
    {
      if (soap->error >= SOAP_STOP)  // request was handled by plugin handler
        continue;                    // so continue the keep-alive loop
      return soap->error;
    }
    // call soap_serve_request generated by soapcpp2 to dispatch the SOAP/XML request
    if ((soap_serve_request(soap) || (soap->fserveloop && soap->fserveloop(soap)))
     && soap->error
     && soap->error < SOAP_STOP)
      return soap_send_fault(soap);
  } while (soap->keep_alive);
  return SOAP_OK;
}
~~~

The `::soap_begin_serve` function processes the HTTP headers and XML SOAP headers if present.  The `::soap_serve_request` function is auto-generated by soapcpp2, when applicable, and parses the SOAP/XML request element to dispatch the SOAP/XML request to a service operation.

If soapcpp2 is not used or does not generate `::soap_serve` but `::soap_serve` is required to serve non-SOAP requests, for example to serve JSON or GET/PUT/PATCH/DELETE requests, then the following simplified `::soap_serve` implementation works just fine:

~~~{.cpp}
#include "soapH.h"

int soap_serve(struct soap *ctx)
{
  soap->keep_alive = soap->max_keep_alive + 1;
  do
  {
    if (soap->keep_alive > 0 && soap->max_keep_alive > 0)
      soap->keep_alive--;
    // parse HTTP headers and call GET, PUT, PATCH or DELETE callback when set
    if (soap_begin_serve(soap))
    {
      if (soap->error >= SOAP_STOP) // request was handled by plugin handler
        continue;                   // so continue the keep-alive loop
      return soap->error;
    }
    soap->error = 400;              // Bad Request because we aren't serving anything else
    return soap_send_fault(soap);   // so we report an error
  } while (soap->keep_alive);
  return SOAP_OK;
}
~~~

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
{
  while (1)
  {
    if (soap_valid_socket(soap_accept(soap)))
    {
      if (soap_serve(soap))
        soap_print_fault(soap, stderr);
    }
    else if (soap->errnum) // accept failed, try again after 1 second
    {
      soap_print_fault(soap, stderr);
      sleep(1);
    }
    else // accept timed out, quit looping
    {
      break;
    }
    soap_destroy(soap);
    soap_end(soap);
  }
}
soap_free(soap);
~~~

@see `#WITH_IPV6`, `#WITH_IPV6_V6ONLY`, `#SOAP_IO_UDP`, `::soap::accept_flags`, `::soap::bind_flags`, `::soap::bind_inet6`, `::soap::bind_v6only`, `::soap::rcvbuf`, `::soap::sndbuf`, `::soap::master`, `::soap_bind`, `::soap_accept`, `::soap_ssl_accept`, `::soap::keep_alive`, `::soap::max_keep_alive`.
*/
int soap_serve(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Initiates serving a pending client request
/**
This function processes the HTTP headers and XML SOAP headers if present of the pending request on `::soap::socket` or on standard input/output `::soap::recvfd` and `::soap::sendfd`, e.g. for CGI and FastCGI services.  Returns `#SOAP_OK` or a `::soap_status` error code.

@see `::soap_serve`, `::soap_serve_request`.
*/
int soap_begin_serve(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Process a pending request
/**
This function is auto-generated by soapcpp2 and parses the SOAP/XML request element to dispatch the SOAP/XML request to a service operation.  Returns `#SOAP_OK` or a `::soap_status` error code.

@see `::soap_serve`, `::soap_begin_serve`.
*/
int soap_serve_request(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Initialize the context for server-side sending and send a HTTP response header
/**
This function initializes the context for sending at the server side and sends a HTTP response header with the specified `status` code.  The `status` code parameter controls the behavior of the HTTP header sent and how the message should be send after calling this function:
- `#SOAP_OK` sends a HTTP 200 OK header with HTTP content type "text/xml" and the HTTP content length value of `::soap::count` or chunked transfer encoding if the `#SOAP_IO_CHUNK` mode is set, i.e. chunked transfers avoid having to compute `::soap::count` such as by sending the message twice, the first time to compute the message length using `::soap_begin_count` and `::soap_end_count`.
- HTTP codes 200 to 599 sends the corresponding HTTP header with HTTP content type "text/xml" and the HTTP content length value of `::soap::count` or chunked transfer encoding if the `#SOAP_IO_CHUNK` mode is set.
- `#SOAP_HTML` sends a HTTP 200 OK header with HTTP content type "text/html" and sets the context to `#SOAP_IO_STORE` to accumulate the message in memory to determine the HTTP content length.
- `#SOAP_FILE` sends a HTTP 200 OK header with HTTP content type `::soap::http_content` and if the `#SOAP_IO_CHUNK` mode is not set, sets the context to `#SOAP_IO_STORE` to accumulate the message in memory to determine the HTTP content length.
- `#SOAP_FILE + status` where `status` is a HTTP status code between 200 and 599, sends the corresponding HTTP header with HTTP content type `::soap::http_content` and f the `#SOAP_IO_CHUNK` mode is not set, sets the context to `#SOAP_IO_STORE` to accumulate the message in memory to determine the HTTP content length.
This function returns `#SOAP_OK` or a `::soap_status` error code when an error occurred.

Besides `::soap_response`, other options to return a HTTP status code are:
- Return the value of `::soap_send_empty_response` to produce an empty HTTP response message with the specified HTTP status code.
- Likewise, return a HTTP status code between 200 and 299 directly to produce an empty HTTP response message with the given HTTP status code.
- Return a HTTP status code between 300 and 599 directly to produce an HTTP response message with the given HTTP status code and a SOAP Fault message body.

@par Examples:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->fget = my_get; // HTTP GET handler to serve HTTP GET
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int my_get(struct soap *soap)
{
  soap_set_omode(SOAP_IO_CHUNK); // make sure to chunk the transfers because we do not compute the content length
  // serve HTTP GET request that has path /xml so in this example we respond with XML
  if (strcmp(soap->path, "/xml") == 0)
  {
    if (soap_response(soap, SOAP_OK)
     || soap_send(soap, "<doc title=\"Example\">Some text</doc>\n")
     || soap_end_send(soap))
      return soap_closesock(soap);
    return SOAP_OK;
  }
  // serve HTTP GET request that has path /html so in this example we respond with HTML
  if (strcmp(soap->path, "/html") == 0)
  {
    if (soap_response(soap, SOAP_HTML)
     || soap_send(soap, "<html><head><title>Example</title></head><body>Some text</body></html>\n")
     || soap_end_send(soap))
      return soap_closesock(soap);
    return SOAP_OK;
  }
  // serve HTTP GET request that has path /json so in this example we respond with JSON
  if (strcmp(soap->path, "/json") == 0)
  {
    soap->http_content = "application/json; charset=utf-8";
    if (soap_response(soap, SOAP_FILE)
     || soap_send(soap, "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n")
     || soap_end_send(soap))
      return soap_closesock(soap);
    return SOAP_OK;
  }
  // respond with HTTP 400 Bad Request and HTML message
  soap->http_content = "text/html; charset=utf-8";
  if (soap_response(soap, SOAP_FILE + 400)
   || soap_send(soap, "<html><body>Bad Request</body></html>\n")
   || soap_end_send(soap))
    return soap_closesock(soap);
  return SOAP_OK;
}
~~~

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->fget = my_get; // HTTP GET handler to serve HTTP GET
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int my_get(struct soap *soap)
{
  // serve HTTP GET request, here we compute the content length instead of chunked transfer
  if (soap_begin_count(soap)
   || soap_send(soap, "<doc title=\"Example\">Some text</doc>\n")
   || soap_end_count(soap)
   || soap_response(soap, SOAP_OK)
   || soap_send(soap, "<doc title=\"Example\">Some text</doc>\n")
   || soap_end_send(soap))
    return soap_closesock(soap);
  return SOAP_OK;
}
~~~

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->fget = my_get; // HTTP GET handler to serve HTTP GET
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int my_get(struct soap *soap)
{
  FILE *fd = NULL;
  char *s = strchr(soap->path, '?');
  if (!s || strcmp(s, "?wsdl"))
    return SOAP_GET_METHOD;
  fd = fopen("myservice.wsdl", "rb");
  if (!fd)
    return 404; // return HTTP 404 Not Found
  soap->http_content = "text/xml";
  soap_set_omode(SOAP_IO_CHUNK);
  if (soap_response(soap, SOAP_FILE) == SOAP_OK)
  {
    while (1)
    {
      size_t r = fread(soap->tmpbuf, 1, sizeof(soap->tmpbuf), fd);
      if (!r || soap_send_raw(soap, soap->tmpbuf, r))
        break;
    }
  }
  fclose(fd);
  soap_end_send(soap);
  return soap_closesock(soap);
}
~~~

@see `::soap_bind`, `::soap_accept`, `::soap_serve`, `::soap_send_empty_response`, `::soap::fget`, `::soap::fput`, `::soap::fpatch`, `::soap::fdel`, `::soap::fopt`, `::soap::fhead`, `::soap::http_extra_header`.
*/
int soap_response(
    struct soap *soap, ///< `::soap` context
    int status)        ///< HTTP status code 200 to 599 or `#SOAP_OK` (same as 200 OK) or `#SOAP_HTML` or `#SOAP_FILE`
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Initialize the context for receiving
/**
This function should be called to initialize the context for receiving a message or to begin parsing a document.  This function is called in the server and client-side code generated by soapcpp2, and in the generated `soap_T_read` functions to deserialize data of type `T`.  For example, it is called by `::soap_serve` (via `::soap_begin_serve`) at the server side when a client request is pending.  This function parses HTTP, DIME, MIME and SOAP headers if present and updates the context with the engine state.  Therefore, this function should not be called more than once to initialize the context for receiving.  The source of the data read is `::soap::is` when non-NULL, or `::soap::socket` when valid, or `::soap::recvfd`.  Returns `#SOAP_OK` or a `::soap_status` error code.

@see `::soap::socket`, `::soap::recvfd`, `::soap::is`, `::soap_end_recv`.
*/
int soap_begin_recv(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Finalize the context after receiving
/**
This function should be called to finalize the context after receiving a message or to end parsing a document.  This function is called in the server and client-side code generated by soapcpp2, and in the generated `soap_T_read` functions to deserialize data of type `T`.  Returns `#SOAP_OK` or a `::soap_status` error code.

@see `::soap_begin_recv`.
*/
int soap_end_recv(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Initialize the context for sending
/**
This function should be called to initialize the context for sending a message or to write a document.  This function is called in the server and client-side code generated by soapcpp2, and in the generated `soap_T_write` functions to serialize data of type `T`.  For example, it is called by `::soap_connect` (via `::soap_connect_command`) at the client side when a client connects to a server to send the HTTP headers and message body.  The destination of the data to be send is `::soap::socket` when valid, or `::soap::sendfd` when nonzero or `::soap::os` when non-NULL.  Returns `#SOAP_OK` or a `::soap_status` error code.

@see `::soap::socket`, `::soap::sendfd`, `::soap::os`, `::soap_end_send`.
*/
int soap_begin_send(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Finalize the context after sending
/**
This function should be called to finalize the context after sending a message or to end writing a document.  This function is called in the server and client-side code generated by soapcpp2, and in the generated `soap_T_write` functions to serialize data of type `T`.  Returns `#SOAP_OK` or a `::soap_status` error code.

@see `::soap_begin_send`.
*/
int soap_end_send(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Initialize context to count message length for sending
/**
This function is used to determine the HTTP content length.  This is done by sending the message after calling this function to update `::soap::count`.  To activate message length counting the `#SOAP_IO_LENGTH` mode is enabled, which prevents the message from being sent by not passing the data to the (internal) `::soap::fsend` callback.  HTTP requires either the HTTP content length header or HTTP transfer encoding chunked with `#SOAP_IO_CHUNK`.  Alternatively, the mode of the context can be set to `#SOAP_IO_STORE` to buffer the entire message in memory to determine the message content length.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
char *response = NULL;
size_t response_len;
const char *request = "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n";
// HTTP POST request, soap_POST stores the entire message with SOAP_IO_STORE to determine HTTP content length
if (soap_POST(soap, "http://www.example.com/API/POST", NULL, "application/json; charset=utf-8")
 || soap_send(soap, request)
 || soap_end_send(soap)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
soap->fget = my_get; // HTTP GET handler to serve HTTP GET
... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve

int my_get(struct soap *soap)
{
  // serve HTTP GET request, here we compute the content length instead of chunked transfer with SOAP_IO_CHUNK or storing the entire message with SOAP_IO_STORE
  const char *response = "<doc title=\"Example\">Some text</doc>\n";
  if (soap_begin_count(soap)
   || soap_send(soap, response)
   || soap_end_count(soap)
   || soap_response(soap, SOAP_OK)
   || soap_send(soap, response)
   || soap_end_send(soap))
    return soap_closesock(soap);
  return SOAP_OK;
}
~~~

Same examples but using HTTP transfer encoding chunked:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK);
char *response = NULL;
size_t response_len;
const char *request = "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n";
if (soap_POST(soap, "http://www.example.com/API/POST", NULL, "application/json; charset=utf-8")
 || soap_send(soap, request)
 || soap_end_send(soap)
 || soap_begin_recv(soap)
 || (response = soap_http_get_body(soap, &response_len)) != NULL
 || soap_end_recv(soap))
  soap_print_fault(soap, stderr);
else
  printf("%s", response);
soap_closesock(soap);
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK);
soap->fget = my_get; // HTTP GET handler to serve HTTP GET
... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve

int my_get(struct soap *soap)
{
  const char *response = "<doc title=\"Example\">Some text</doc>\n";
  if (soap_response(soap, SOAP_OK)
   || soap_send(soap, response)
   || soap_end_send(soap))
    return soap_closesock(soap);
  return SOAP_OK;
}
~~~

@see `::soap_end_count`, `#SOAP_IO_LENGTH`, `#SOAP_IO_CHUNK`, `#SOAP_IO_STORE`.
*/
int soap_begin_count(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Finalize context to count message length for sending
/**
This function is used to determine the HTTP content length.  This is done by sending the message after calling this function to update `::soap::count`.  HTTP requires either the HTTP content length header or HTTP transfer encoding chunked with `#SOAP_IO_CHUNK`.  Alternatively, the mode of the context can be set to `#SOAP_IO_STORE` to buffer the entire message in memory to determine the message content length.

@see `::soap_begin_count`, `#SOAP_IO_LENGTH`, `#SOAP_IO_CHUNK`, `#SOAP_IO_STORE`.
*/
int soap_end_count(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Close the socket connection
/**
This function should be called to close `::soap::socket`.  The socket is closed and `::soap::socket` is set to `#SOAP_INVALID_SOCKET` if the socket is valid, keep-alive is not enabled and not currently active, i.e. the socket is closed when `::soap::socket` != `#SOAP_INVALID_SOCKET` and `::soap::keep_alive` == 0.  Therefore, this function keeps the socket connection open when keep-alive is currently active.  This function may be called multiple times but closes the socket just once if the socket connection was open.  This function is called in the server and client-side code generated by soapcpp2.  Returns the current value of `::soap::error` to propagate the error state when used as `return soap_closesock(soap);`.

@see `::soap_force_closesock`, `::soap_close_connection`.
*/
int soap_closesock(struct soap *soap) ///< `::soap` context
  /// @returns the value of `::soap::error` (`#SOAP_OK` or a `::soap_status` error code)
  ;

/// Forcibly close the socket connection
/**
This function immediately closes `::soap::socket` and should only be used when `::soap_closesock` does not suffice.  By contrast, `::soap_closesock` gently finalizes the SSL connection, and when `::soap::keep_alive` == 0 calls `shutdown` and `close` on `::soap::socket`.  By calling `::soap_force_closesock` the socket is forcibly closed immediately and `::soap::socket` is set to `#SOAP_INVALID_SOCKET`, even when keep-alive is currently active.  This function may be called multiple times but closes the socket just once if the socket connection was open.  Returns the current value of `::soap::error` to propagate the error state when used as `return soap_force_closesock(soap);`.

@see `::soap_closesock`, `::soap_close_connection`.
*/
int soap_force_closesock(struct soap *soap) ///< `::soap` context
  /// @returns the value of `::soap::error` (`#SOAP_OK` or a `::soap_status` error code)
  ;

/// Close the connection of the specified context using a self-pipe
/**
This function closes the connection of the specified context, i.e. when it hangs on socket IO and the specified `::soap::recv_timeout` and `::soap::send_timeout` timeouts are not sufficient to release the blocking socket IO.  This function force-closes a connection, which is typically done by another thread that detects a termination condition.  Requires compilation with `#WITH_SELF_PIPE` and requires `::soap::recv_timeout` set to a nonzero timeout value.

Alternatively, `#THREAD_CANCEL` can be used with a thread cleanup function added with `pthread_cleanup_push`.  See `#THREAD_CANCEL` for details.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/threads.h"

struct soap *soap = soap_new();
soap->bind_flags = SO_REUSEADDR;            // immediate port reuse
soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
{
  while (1)
  {
    if (soap_valid_socket(soap_accept(soap)))
    {
      THREAD_TYPE tid;
      struct soap *tsoap = soap_copy(soap);
      if (!tsoap)
        soap_force_closesock(soap);
      else
        while (THREAD_CREATE(&tid, (void*(*)(void*))&accept_request, (void*)tsoap))
          sleep(1); // failed, try again
    }
    else if (soap->errnum) // accept failed, try again after 1 second
    {
      soap_print_fault(soap, stderr);
      sleep(1);
    }
    else // accept timed out, quit looping
    {
      break;
    }
    soap_destroy(soap);
    soap_end(soap);
  }
}
soap_free(soap);

void *accept_request(struct soap *soap)
{
  THREAD_TYPE tid;
  struct soap *tsoap;
  THREAD_DETACH(THREAD_ID);
  // create a new thread that is timed to execute for max 10 seconds
  tsoap = soap_copy(soap);
  if (!tsoap)
  {
    soap_force_closesock(soap);
  }
  else
  {
    while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap))
      sleep(1); // failed, try again
    // allow serving the request by the new thread for up to 30 seconds max
    sleep(30);
    // terminate soap_serve in process_request then wait for it to join
    soap_close_connection(tsoap);
    THREAD_JOIN(tid);
    // clean up the terminated thread context
    soap_free(tsoap);
  }
  // clean up
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}

void *process_request(struct soap *soap)
{
  soap_serve(soap);
  soap_destroy(soap);
  soap_end(soap);
  return NULL;
}
~~~

@note Requires gSOAP 2.8.71 or greater, compilation with `#WITH_SELF_PIPE`, and nonzero `::soap::recv_timeout` and `::soap::send_timeout` timeout values.

@see `::soap_closesock`, `::soap_force_closesock`, `::soap::recv_timeout`, `::soap::send_timeout`, `::soap::transfer_timeout`.
*/
void soap_close_connection(struct soap *soap) ///< `::soap` context
  ;

/// Send a string
/**
This function sends a 0-terminated string to `::soap::os` when non-NULL, or to `::soap::socket` when valid, or or to `::soap::sendfd` when nonzero.

@see `::soap_begin_count`, `::soap_end_count`, `::soap_begin_send`, `::soap_end_send`, `::soap_send_raw`.
*/
int soap_send(
    struct soap *soap, ///< `::soap` context
    const char *s)     ///< 0-terminated string to send
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Send raw bytes
/**
This function sends raw bytes of data to `::soap::os` when non-NULL, or to `::soap::socket` when valid, or or to `::soap::sendfd` when nonzero.

@see `::soap_begin_count`, `::soap_end_count`, `::soap_begin_send`, `::soap_end_send`, `::soap_send_raw`.
*/
int soap_send_raw(
    struct soap *soap, ///< `::soap` context
    const char *s,     ///< data to send
    size_t n)          ///< length of data to send
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Check if HTTP body message is not empty
/**
This function returns nonzero if an HTTP message body is present, zero otherwise.  This function should be called immediately after calling `::soap_begin_recv`.  Note that `::soap_begin_recv` is called at the server side before a HTTP callback is called, such as `::soap::fput` and `::soap::fpatch`.  Callbacks and the `::http_post` HTTP POST plugin handlers should therefore not call `::soap_begin_recv`.  Also, `::soap_begin_recv` is called at the client side by HTTP functions such as `::soap_GET`, after which the HTTP body can then be checked with `::soap_http_has_body` and retrieved with `::soap_http_get_body`.

@see \ref group_callbacks, `::soap_begin_recv`, `::soap_end_recv`, `::soap_http_get_body`, `::soap_GET`, `::soap_POST`, `::soap::fput`, `::soap::fpatch`, `::http_post`.
*/
int soap_http_has_body(struct soap *soap) ///< `::soap` context
  /// @returns nonzero if an HTTP body is present, zero otherwise
  ;

/// Get the HTTP body message as a string
/**
This function parses an HTTP body message into a string, whether or not an HTTP body message is present.  This function should be called immediately after calling `::soap_begin_recv`.  Note that `::soap_begin_recv` is called at the server side before a HTTP callback is called, such as `::soap::fput` and `::soap::fpatch`.  Callbacks and the `::http_post` HTTP POST plugin handlers should therefore not call `::soap_begin_recv`.  Also, `::soap_begin_recv` is called at the client side by HTTP functions such as `::soap_GET`, after which the HTTP body can then be then be checked with `::soap_http_has_body` and retrieved with `::soap_http_get_body`.  This function reads input from from `::soap::is` when non-NULL, or `::soap::socket` when valid, or from `::soap::recvfd`, and sets the `len` pointer parameter to the length of the string read if `len` is not NULL.  After calling this function, `::soap_end_recv` should be called.  Returns the HTTP body as a string allocated in managed memory or returns "" (empty string, since version 2.8.71 or returns NULL for previous versions) when no HTTP message body is present or NULL when an error occurred and sets `::soap::error`.

@par Examples:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new1(SOAP_IO_CHUNK); // use SOAP_IO_CHUNK or SOAP_IO_STORE
  char *response = NULL;
  size_t response_len;
  if (soap_GET(soap, "http://www.example.com/API/GET", NULL)
   || soap_begin_recv(soap)
   || (response = soap_http_get_body(soap, &response_len)) != NULL
   || soap_end_recv(soap))
    soap_print_fault(soap, stderr);
  else
    fwrite(response, 1, response_len, stdout);
  soap_closesock(soap);
}
~~~

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->fput = my_put; // HTTP PUT handler to serve HTTP PUT
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int my_put(struct soap *soap)
{
  size_t len;
  char *message = soap_http_get_body(soap, &len);
  soap_end_recv(soap);
  ... // use the message data
  return soap_send_empty_response(soap, 202); // HTTP 202 Accepted
}
~~~

@see \ref group_callbacks, `::soap_begin_recv`, `::soap_end_recv`, `::soap_http_has_body`, `::soap_GET`, `::soap_POST`, `::soap::fput`, `::soap::fpatch`, `::http_post`.
*/
char * soap_http_get_body(
    struct soap *soap, ///< `::soap` context
    size_t *len)       ///< pointer to the length variable to assign or NULL
  /// @returns HTTP body as a string or NULL when an error occurred
  ;

/// Get a header line
/**
This function stores a header line into the specified buffer `buf` of maxiumum length `len`.  This function should be used to read HTTP and MIME headers, which end with CRLF or LF.  The function handles header continuations (indents).  This function reads input from `::soap::is` when non-NULL, or from `::soap::socket` when valid, or from `::soap::recvfd`.  Returns `#SOAP_OK` or a `::soap_status` error code.
*/
int soap_getline(
    struct soap *soap, ///< `::soap` context
    char *buf,         ///< buffer to fill (string)
    int len)           ///< maximum size of the buffer (int)
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Wide char type
typedef int32_t soap_wchar;

/// Get next byte without consuming it
/**
This function returns the next byte on the input without consuming it, i.e. peeks one byte ahead.  Reads a byte from `::soap::is` when non-NULL, or from `::soap::socket` when valid, or from `::soap::recvfd`.  Returns the next byte or EOF when an error occurred and sets `::soap::error` to a `::soap_status` value and `::soap::errnum` to the `errno` value of the failure.
*/
soap_wchar soap_get0(struct soap *soap) ///< `::soap` context
  /// @returns byte read or EOF when an error occurred
  ;

/// Get next byte
/**
This function returns the next byte on the input.  Reads a byte from `::soap::is` when non-NULL, or from `::soap::socket` when valid, or from `::soap::recvfd`.  Returns the next byte or EOF when an error occurred and sets `::soap::error` to a `::soap_status` value and `::soap::errnum` to the `errno` value of the failure.
*/
soap_wchar soap_get1(struct soap *soap) ///< `::soap` context
  /// @returns byte read or EOF when an error occurred
  ;

/// Poll the connection
/**
This function returns `#SOAP_OK` if the connection is ready to send and receive data, `#SOAP_EOF` otherwise.  Also returns `#SOAP_OK` if the socket connection is closed but data can still be read or written from/to other streams.

@see `::soap_ready`.
*/
int soap_poll(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` (ready), `#SOAP_EOF` (not ready), or `#SOAP_TCP_ERROR` (error)
  ;

/// Check if the connection is ready to receive pending data
/**
This function returns `#SOAP_OK` if the connection is ready to receive pending data, `#SOAP_EOF` otherwise.  Also returns `#SOAP_OK` if the socket connection is closed but data can still be read from other streams.

@par Examples:

~~~{.cpp}
    // file.h interface header file for soapcpp2

    // REST request data
    struct ns__request
    {
      int SKU;
    };

    // REST response data
    struct ns__response
    {
      char *product_name;
    };

    // SOAP web method response data
    struct ns__webmethodResponse
    {
      char *string;
      int number;
    };

    // SOAP web method
    int ns__webmethod(char *string, int number, struct ns__webmethodResponse *response);
~~~

    soapcpp2 -c file.h

~~~{.cpp}
    // SOAP client application (requires 2.8.75 or greater)

    #include "soapH.h"
    #include "ns.nsmap"
    
    int main()
    {
      struct soap *soap = soap_new();
      struct ns__webmethodResponse response;
      // send the SOAP request message of ns__webmethod declared in a .h file for soapcpp2
      if (soap_send_ns__webmethod(soap, endpoint, NULL, "hello", 123))
        ... // error
      // wait on message ready to receive from server
      while (soap_ready(soap) == SOAP_EOF)
      {
        // do some more work here (or sleep 1ms as below)
        usleep(1000);
      }
      if (soap->error)
        ... // error
      // receive the SOAP response message of ns__webmethod declared in a .h file for soapcpp2
      if (soap_recv_ns__webmethod(soap, &response))
        ... // error
      ... // process the response
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
    }
~~~

~~~{.cpp}
    // REST POST client application

    #include "soapH.h"
    #include "ns.nsmap"
    
    int main()
    {
      struct soap *soap = soap_new();
      struct ns__request request;     // serializable type declared in a .h file for soapcpp2
      struct ns__response response;   // serializable type declared in a .h file for soapcpp2
      soap_default_ns__request(soap, &request);
      ... // populate request data
      // POST the REST request message using the ns__request serializer
      if (soap_POST_send_ns__request(soap, endpoint, &request))
        ... // error
      // wait on message ready to receive from server
      while (soap_ready(soap) == SOAP_EOF)
      {
        // do some more work here (or sleep 1ms as below)
        usleep(1000);
      }
      if (soap->error)
        ... // error
      // receive the REST response message using the ns__response serializer
      if (soap_POST_recv_ns__response(soap, &response))
        ... // error
      ... // process the response data
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
    }
~~~

~~~{.cpp}
    // REST GET client application

    #include "soapH.h"
    #include "ns.nsmap"
    
    int main()
    {
      struct soap *soap = soap_new();
      struct ns__response response;   // serializable type declared in a .h file for soapcpp2
      // HTTP GET
      if (soap_GET(soap, "http://www.exanple.org/get?SKU=123", NULL))
        ... // error
      // wait on message ready to receive from server
      while (soap_ready(soap) == SOAP_EOF)
      {
        // do some more work here (or sleep 1ms as below)
        usleep(1000);
      }
      if (soap->error)
        ... // error
      // receive the response message using the ns__response serializer
      if (soap_read_ns__response(soap, &response))
      {
        soap_closesock(soap);
        ... // error
      }
      soap_closesock(soap);
      ... // process the response data
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
    }
~~~

@see `::soap_poll`.
*/
int soap_ready(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` (ready), `#SOAP_EOF` (not ready), or `#SOAP_TCP_ERROR` (error)
  ;

/// Receive an HTTP response message from the server that is assumed to be empty
/**
This function receives an HTTP response message from the server and is typically used when HTTP 202 Accepted or HTTP 200 OK is expected without data.  Therefore the HTTP response message body is assumed to be empty.  If the message is not empty then a fault message with the HTTP body as the fault string will be produced and the `::soap::error` is set to the HTTP status code received.  This function reads input from `::soap::is` when non-NULL, or from `::soap::socket` when valid, or from `::soap::recvfd`.  Returns `#SOAP_OK` or a `::soap_status` error code such as the HTTP status code received.
*/
int soap_recv_empty_response(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Return an HTTP response message with an empty HTTP body from a service back to the client or peer
/**
This function sends an empty response back to the client.  The response includes the specified `status` HTTP status code in the HTTP header but the HTTP body is empty.  This function should be used with HTTP PUT or PATCH requests and "one-way" SOAP messaging with HTTP POST.  This function sends to `::soap::socket` when valid, or to `::soap::os` when non-NULL, or to `::soap::sendfd`.  Returns `#SOAP_STOP` and sets `::soap::error` to `#SOAP_STOP` to halt further response messaging to the client or a `::soap_status` error code when the message could not be sent.

Besides `::soap_send_empty_response`, other options to return an HTTP status code are:
- Use `::soap_response` using `#SOAP_FILE + status` to produce an HTTP response message with the specified HTTP status code.
- Return a HTTP status code between 200 and 299 directly to produce an empty HTTP response message with the given HTTP status code.
- Return a HTTP status code between 300 and 599 directly to produce an HTTP response message with the given HTTP status code and a SOAP Fault message body.

@par Examples:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->fput = my_put; // HTTP PUT handler to serve HTTP PUT
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int my_put(struct soap *soap)
{
  size_t len;
  char *message = soap_http_get_body(soap, &len);
  soap_end_recv(soap);
  ... // use the message data
  return soap_send_empty_response(soap, 202); // HTTP 202 Accepted
}
~~~

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->fput = my_put; // HTTP PUT handler to serve HTTP PUT
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int my_put(struct soap *soap)
{
  if (soap->http_content && soap_tag_cmp(soap->http_content, "text/xml*") == 0)
  {
    soap_dom_element *dom = soap_new_xsd__anyType(soap);
    // get dom, don't use soap_read_xsd__anyType because we are already reading from a socket
    if (soap_in_xsd__anyType(soap, "doc", dom, NULL) == SOAP_OK)
    {
      ... // inspect the DOM using the DOM API functions
      return soap_send_empty_response(soap, 202); // HTTP 202 Accepted
    }
  }
  return soap_send_empty_response(soap, 400); // HTTP 400 Bad Request
}
~~~

@see `::soap_bind`, `::soap_accept`, `::soap_serve`, `::soap_response`, `::soap::fget`, `::soap::fput`, `::soap::fpatch`, `::soap::fdel`, `::soap::fopt`, `::soap::fhead`.
*/
int soap_send_empty_response(
    struct soap *soap,  ///< `::soap` context
    int status)         ///< HTTP status code 200 to 599 or `#SOAP_OK` (same as 200 OK)
  /// @returns `#SOAP_STOP`
  ;

/// Parse the XML <i>`<SOAP-ENV:Envelope>`</i> element opening tag if present
int soap_envelope_begin_in(struct soap *soap) ///< `::soap` context
  ;

/// Parse the XML <i>`</SOAP-ENV:Envelope>`</i> element closing tag if present
int soap_envelope_end_in(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Emit the XML <i>`<SOAP-ENV:Envelope>`</i> element opening tag if `::soap::version` is nonzero
int soap_envelope_begin_out(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;
/// Emit the XML <i>`</SOAP-ENV:Envelope>`</i> element closing tag if `::soap::version` is nonzero
int soap_envelope_end_out(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;
/// Parse the XML <i>`<SOAP-ENV:Body>`</i> element opening tag if present
int soap_body_begin_in(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;
/// Parse the XML <i>`</SOAP-ENV:Body>`</i> element closing tag if present
int soap_body_end_in(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;
/// Emit the XML <i>`<SOAP-ENV:Body>`</i> element opening tag if `::soap::version` is nonzero
int soap_body_begin_out(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;
/// Emit the XML <i>`</SOAP-ENV:Body>`</i> element closing tag if `::soap::version` is nonzero
int soap_body_end_out(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Parse and deserialize the SOAP Fault
/**
This function parses and deserializes the SOAP Fault such that `::soap::fault` points to a `::SOAP_ENV__Fault` structure.  If the specified `check` parameter is nonzero, this function attempts to parse the SOAP Fault that may be present (it checks), but if no <i>`SOAP-ENV:Fault`</i> element is present returns `#SOAP_OK`.  If the specified `check` parameter is zero, this function parses the SOAP Fault that is expected to be present, but if <i>`SOAP-ENV:Fault`</i> element is present returns the HTTP error code received (when between 300 and 599) or the `::soap::error` code.
*/
int soap_recv_fault(
    struct soap *soap, ///< `::soap` context
    int check)         ///< flag to check or expect a SOAP Fault
  /// @returns a nonzero `::soap_status` error code
  ;

/// Return an HTTP error with a SOAP Fault message from a service
/**
This function sends an error response back to the client and is used at the server side by `::soap_serve` after an error occurred to propagate the fault to the client.  This function sends to `::soap::socket` when valid, or to `::soap::os` when non-NULL, or to `::soap::sendfd`.
*/
int soap_send_fault(struct soap *soap) ///< `::soap` context
  /// @returns the value of `::soap::error`, a nonzero error code
  ;

/// Parse and deserialize the SOAP Header
/**
This function parses and deserializes the SOAP Header such that `::soap::header` points to a `::SOAP_ENV__Header` structure.
*/
int soap_recv_header(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Emit the SOAP Header pointed to by `::soap::header`
int soap_putheader(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Serialize the SOAP Header pointed to by `::soap::header` before emitting it with `::soap_putheader`
void soap_serializeheader(struct soap *soap) ///< `::soap` context
  ;

/// The HTTP GET plugin registration function
/**
This function is used to register the `::http_get` HTTP GET plugin with `soap_register_plugin_arg(soap, http_get, http_get_handler)` where the `http_get_handler` is a user-defined function to handle HTTP GET requests.  The HTTP GET plugin API is declared and defined in <i>`gsoap/plugin/httpget.h`</i> and <i>`gsoap/plugin/httpget.c`</i>.  The `::soap::path` string contains the URL path, starting with a leading `/`.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/httpget.h"

int main()
{
  struct soap *soap = soap_new();
  soap_register_plugin_arg(soap, http_get, http_get_handler);
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int http_get_handler(struct soap *soap)
{
  if (!strncmp(soap->path, "/API/GET", 8))
  {
    // get the query key=val pairs from soap->path
    char *query = soap_query(soap); 
    while (query)
    {
      // get next key=val pair
      char *key = soap_query_key(soap, &query);
      char *val = soap_query_key(soap, &query);
      ... // use key and val (key is non-NULL, val may be NULL)
    }
    // return a response message
    soap->http_content = "application/json; charset=utf-8";
    if (soap_response(soap, SOAP_FILE)
     || soap_send(soap, "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n")
     || soap_end_send(soap))
      return soap_closesock(soap);
    return SOAP_OK;
  }
  return 404; // HTTP 404 Not Found
}
~~~

@warning When serving files as responses to requests, we need to be vary careful, because we don't want requests to snoop around in directories and serve files that should be protected from public view.  Therefore, when adding logic to serve files, we must reject request that have `::soap::path` values with a `/` or a `\` .  If these are allowed, then we must at least check for `..` in the path to avoid request from snooping around in higher directories all the way up to the root.  See the <i>`gsoap/samples/webserver/webserver.c`</i> example for details.

@see `::soap_http_get_stats`, `::soap_query`, `::soap::fget`.
*/
int http_get(struct soap*, struct soap_plugin*, void*);

/// Collect access statistics with the `::http_get` plugin
/**
This function sets the specified minutes, hour, and day parameters to point to an array of requests per minute, an array of request per hour, and an array of requests per day of the year.  The minutes array has 60 entries, the hour array has 24 entries, and the day array has 365 entries (a leap year will roll over).  The plugin will collect the stats among a set of server threads if the spawned thread contexts are created with `::soap_copy`.

@note This function is declared and defined in <i>`gsoap/plugin/httpget.h`</i> and <i>`gsoap/plugin/httpget.c`</i> and requires the `::http_get` plugin.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/httpget.h"

int main()
{
  struct soap *soap = soap_new();
  soap_register_plugin_arg(soap, http_get, http_get_handler);
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      size_t stat_get, stat_post, stat_fail, *hist_min, *hist_hour, *hist_day;
      if (soap_valid_socket(soap_accept(soap)))
      {
        if (soap_serve(soap))
          soap_print_fault(soap, stderr);
      }
      else if (soap->errnum) // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_http_get_stats(soap, &stat_get, &stat_post, &stat_fail, &hist_min, &hist_hour, &hist_day);
      // show the number of GET, POST, and failures
      printf("\n#GET = %d #POST = %d failures = %d", stat_get, stat_post, stat_fail);
      // show the stats collected per hour
      printf("\nBy hour: ");
      for (i = 0; i < 24; i++)
        printf("%8d", i);
      printf("\nRequests:");
      for (i = 0; i < 24; i++)
        printf("%8d", hist_hour[i]);
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}
~~~
*/
void soap_http_get_stats(
    struct soap *soap,  ///< `::soap` context
    size_t *stat_get,   ///< points to a variable to assign the number of GET requests
    size_t *stat_post,  ///< points to a variable to assign the number of POST requests
    size_t *stat_fail,  ///< points to a variable to assign the number of failed requests
    size_t **hist_min,  ///< points to a pointer to an array of 60 entries with histogram of requests per minute
    size_t **hist_hour, ///< points to a pointer to an array of 24 entries with histogram of requests per hour
    size_t **hist_day)  ///< points to a pointer to an array of 365 entries with histogram of requests per day
  ;

/// Extract the query string from the URL path at the server side with the `::http_get` plugin
/**
At the server side the value `::soap::path` is set to the URL's path string which may include a query string of the form "?key=val&key=val" with a sequence of key=val pairs, where the value is optional.  Returns the query string or NULL when none is present.  To extract the keys and values from the string returned by this function, use `::soap_query_key` and `::soap_query_val`.

The `::http_get` plugin sets a HTTP GET handler function to serve HTTP GET requests (or NULL to remove a handler) with `soap_register_plugin_arg(soap, http_get, http_get_handler)` and keeps track of the number of GET and POST invocations made and the number of server requests by the minute, by the hour, and by the day.

@note This function is declared and defined in <i>`gsoap/plugin/httpget.h`</i> and <i>`gsoap/plugin/httpget.c`</i> and requires the `::http_get` plugin.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/httpget.h"

int main()
{
  struct soap *soap = soap_new();
  soap_register_plugin_arg(soap, http_get, http_get_handler);
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      size_t *min, *hour, *day;
      if (soap_valid_socket(soap_accept(soap)))
      {
        if (soap_serve(soap))
          soap_print_fault(soap, stderr);
      }
      else if (soap->errnum) // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}

int http_get_handler(struct soap *soap)
{
  if (!strncmp(soap->path, "/API/GET", 8))
  {
    // get the query key=val pairs from soap->path
    char *query = soap_query(soap); 
    while (query)
    {
      // get next key=val pair
      char *key = soap_query_key(soap, &query);
      char *val = soap_query_key(soap, &query);
      ... // use key and val (key is non-NULL, val may be NULL)
    }
    // return a response message
    soap->http_content = "application/json; charset=utf-8";
    if (soap_response(soap, SOAP_FILE)
     || soap_send(soap, "{ \"title\": \"Example\", \"doc\": \"Some text\" }\n")
     || soap_end_send(soap))
      return soap_closesock(soap);
    return SOAP_OK;
  }
  return 404; // HTTP 404 Not Found
}

int ns__webmethod(struct soap *soap, ...)
{
  char *query = soap_query(soap); 
  while (query)
  {
    // get next key=val pair
    char *key = soap_query_key(soap, &query);
    char *val = soap_query_key(soap, &query);
    ... // use key and val (key is non-NULL, val may be NULL)
  }
  ...
}
~~~

@see `::soap_query_key`, `::soap_query_val`, `::soap_http_get_stats`.
*/
char * soap_query(struct soap *soap) ///< `::soap` context
  ;

/// Extract the next query string key at the server side with the `::http_get` plugin
/**
This function takes a pointer to the string variable returned by `::soap_query` and updates it to point to the value (if present), then returns the query key or NULL if no more query key-value pairs are present in the query string.

@note This function is declared and defined in <i>`gsoap/plugin/httpget.h`</i> and <i>`gsoap/plugin/httpget.c`</i> and requires the `::http_get` plugin.

@par Example:

~~~{.cpp}
// get the query key=val pairs from soap->path
char *query = soap_query(soap); 
while (query)
{
  // get next key=val pair
  char *key = soap_query_key(soap, &query);
  char *val = soap_query_key(soap, &query);
  ... // use key and val (key is non-NULL, val may be NULL)
}
~~~

@see `::soap_query`, `::soap_query_val`.
*/
char * soap_query_key(
    struct soap *soap, ///< `::soap` context
    char **query)      ///< points to the string returned by soap_query and soap_query_val
  /// @returns query string key
  ;

/// Extract the next query string value at the server side with the `::http_get` plugin
/**
This function takes a pointer to the string variable returned by `::soap_query` and updates it to point to the next key (if present), then returns the query value or NULL if no value is bound to the key.

@note This function is declared and defined in <i>`gsoap/plugin/httpget.h`</i> and <i>`gsoap/plugin/httpget.c`</i> and requires the `::http_get` plugin.

@par Example:

~~~{.cpp}
// get the query key=val pairs from soap->path
char *query = soap_query(soap); 
while (query)
{
  // get next key=val pair
  char *key = soap_query_key(soap, &query);
  char *val = soap_query_key(soap, &query);
  ... // use key and val (key is non-NULL, val may be NULL)
}
~~~

@see `::soap_query`, `::soap_query_key`.
*/
char * soap_query_val(
    struct soap *soap, ///< `::soap` context
    char **query)      ///< points to the string returned by soap_query and soap_query_key
  /// @returns query string value or NULL
  ;

/// The HTTP POST plugin registration function
/**
This function is used to register the `::http_post` HTTP POST plugin with `soap_register_plugin_arg(soap, http_post, http_post_handler)` where the `http_post_handler` is a table of HTTP POST handler functions and generic POST, PUT, PATCH, and DELETE handler functions.  The HTTP POST plugin API is declared and defined in <i>`gsoap/plugin/httppost.h`</i> and <i>`gsoap/plugin/httppost.c`</i>.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/httppost.h"

int main()
{
  struct soap *soap = soap_new();
  soap_register_plugin_arg(soap, http_post, http_post_handlers);
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
struct http_post_handlers http_post_handlers[] = {
  { "application/json",   json_post_handler },
  { "image/*",            image_post_handler },
  { "text/*",             text_post_handler },
  { "POST",               generic_POST_handler },
  { "PUT",                generic_PUT_handler },
  { "PATCH",              generic_PATCH_handler },
  { "DELETE",             generic_DELETE_handler },
  { NULL }
};
// example image/* POST handler
// note: json_post_handler and text_post_handler are similar to this example
int image_post_handler(struct soap *soap)
{
  const char *buf;
  size_t len;
  // for example, only handle /API/POST paths and content type image/gif
  if (!strncmp(soap->path, "/API/POST", 9))
   || (soap->http_content && soap_tag_cmp(soap->http_content, "image/gif")))
    return 404;
  // get HTTP POST message body
  buf = soap_http_get_body(soap, &len);
  if (!buf)
    return 400; // HTTP 400 Bad Request
  (void)soap_end_recv(soap);
  ... // process image in buf[0..len-1]
  // for example, send image back (HTTP POST returns a response message)
  soap->http_content = "image/gif";
  if (soap_response(soap, SOAP_FILE)
   || soap_send_raw(soap, buf, len)
   || soap_end_send(soap))
    return soap_closesock(soap);
  return SOAP_OK;
}
// example generic PUT handler
int generic_PUT_handler(struct soap *soap)
{
  const char *buf;
  size_t len;
  // for example, only handle /API/PUT paths and content type image/gif
  if (!strncmp(soap->path, "/API/PUT", 8))
   || (soap->http_content && soap_tag_cmp(soap->http_content, "image/gif")))
    return 404;
  // get HTTP PUT message body
  buf = soap_http_get_body(soap, &len);
  if (!buf)
    return 400; // HTTP 400 Bad Request
  (void)soap_end_recv(soap);
  ... // process image in buf[0..len-1]
  return soap_send_empty_response(soap, 200); // HTTP 200 OK
}
~~~
*/
int http_post(struct soap*, struct soap_plugin*, void*);

/// The HTTP POST form plugin registration function
/**
This function is used to register the `::http_form` HTTP POST form plugin to handle application/x-www-form-urlencoded data with `soap_register_plugin_arg(soap, http_form, http_form_handler)` where the `http_form_handler` is a function to parse the application/x-www-form-urlencoded data received by the server.  The HTTP POST form plugin API is declared and defined in <i>`gsoap/plugin/httpform.h`</i> and <i>`gsoap/plugin/httpform.c`</i>.

Handling multipart/related and multipart/form-data at the server side is done by iterating over the `::soap_multipart` linked list containing the MIME attachments received, see the detailed description of \ref group_mime.

@note This plugin requires the `::http_get` plugin for the `::soap_query_key` and `::soap_query_val` functions.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/httpform.h"

int main()
{
  struct soap *soap = soap_new();
  soap_register_plugin_arg(soap, http_form, http_form_handler);
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}

int http_form_handler(struct soap *soap)
{
  if (!strncmp(soap->path, "/API/FORM", 9))
  {
    // get the application/x-www-form-urlencoded data
    char *form = soap_http_get_form(soap); 
    while (form)
    {
      // get next key=val pair
      char *key = soap_query_key(soap, &form);
      char *val = soap_query_key(soap, &form);
      ... // use key and val (key is non-NULL, val may be NULL)
    }
    // return a response message
    if (soap_response(soap, SOAP_HTML)
     || soap_send(soap, "<html><body>Thank you for submitting</body></html>\n")
     || soap_end_send(soap))
      return soap_closesock(soap);
    return SOAP_OK;
  }
  return 404; // HTTP 404 Not Found
}
~~~

@see `::soap_http_get_form`, `::http_get`, `::soap_query_key`, `::soap_query_val`.
*/
int http_form(struct soap*, struct soap_plugin*, void*);

/// Get the HTTP POST application/x-www-form-urlencoded data as a string
/**
This function parses an HTTP body with application/x-www-form-urlencoded data into a string, prepends a `?`, and returns this string allocated in managed memory or NULL when an error occurred and sets `::soap::error`.

@note This function is declared and defined in <i>`gsoap/plugin/httpform.h`</i> and <i>`gsoap/plugin/httpform.c`</i> and requires the `::http_form` plugin.

@see `::http_form`.
*/
char * soap_http_get_form(struct soap *soap) ///< `::soap` context
  ;

/// The HTTP pipelining plugin registration function
/**
This function is used to register the `::http_pipe` HTTP pipelining plugin to support HTTP pipelining.  HTTP pipelining requires `#SOAP_IO_KEEPALIVE` enabled.  At the server side, multiple threads should be used to serve requests, such as with a gSOAP multi-threaded stand-alone server.  HTTP pipelining at the client side requires two threads, one to send request messages and one to receive the response messages, with logic to handle transmission errors and to resend failed request messages until all responses have been received.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/httppipe.h"
#include "plugin/threads.h"

int main()
{
  struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); // HTTP keep-alive
  soap_register_plugin(soap, http_pipe);            // HTTP pipelining
  soap->bind_flags = SO_REUSEADDR;                  // immediate port reuse
  soap->accept_timeout = 3600;                      // exit loop when no request arrives in one hour
  soap->send_timeout = soap_recv_timeout = 5;       // 5 seconds max socket stall time (unlimited by default)
  soap->transfer_timeout = 30;                      // 30 seconds max message transfer time (unlimited by default)
  soap->recv_maxlength = 1048576;                   // limit messages received to 1MB (2GB by default)
  if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
  {
    while (1)
    {
      if (soap_valid_socket(soap_accept(soap)))
      {
        THREAD_TYPE tid;
        struct soap *tsoap = soap_copy(soap);
        if (!tsoap)
          soap_force_closesock(soap);
        else
          while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap))
            sleep(1); // failed, try again
      }
      else if (soap->errnum) // accept failed, try again after 1 second
      {
        soap_print_fault(soap, stderr);
        sleep(1);
      }
      else // accept timed out, quit looping
      {
        break;
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}

void *process_request(struct soap *soap)
{
  THREAD_DETACH(THREAD_ID);
  soap_serve(soap);
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}
~~~

*/
int http_pipe(struct soap*, struct soap_plugin*, void*);

/** @} */

/**
\defgroup group_cookies HTTP cookie functions
@brief This module defines functions to set and get HTTP cookies at the server side

This module defines the cookie structure of cookies stored in the cookie store and server-side functions to set and inspect cookies.  Cookie handling is fully automatic on the client side, when the engine is compiled with `#WITH_COOKIES`.  To avoid "cookie storms" caused by malicious servers that return an unreasonable amount of cookies, the size of the cookie store is limited to `::soap::cookie_max` cookies.  Each `::soap` context has its own independent cookie store.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.

@see The [HTTP Server Session Management plugin](../../sessions/html/index.html) documentation to use the HTTP session plugin to store a database of sessions to keep track of client data across multiple requests.
@{
*/

/// Cookie structure
/**
Each `::soap` context has its own independent cookie store with up to `::soap::cookie_max` cookies.
*/
struct soap_cookie {
  struct soap_cookie *next; ///< next cookie in linked list or NULL
  char *name;               ///< cookie name
  char *value;              ///< cookie value
  char *domain;             ///< cookie domain
  char *path;               ///< cookie path
  ULONG64 expire;           ///< local time to expire (value cast to time_t)
  long maxage;              ///< server-side: seconds to expire
  unsigned int version;     ///< cookie version value
  short secure;             ///< HTTPS secure cookie
  short session;            ///< server-side: this is session cookie
  short env;                ///< server-side: got cookie from a client, do not sent to clients
  short modified;           ///< server-side: client cookie was modified and should be sent to clients
};

/// Add a cookie
/**
This function adds a cookie to the cookie store at the server side, if not already there, with the specified `name` and `value`.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns pointer to the cookie structure in the database or NULL when an error occurred.

@par Example:

~~~{.cpp}
// A CGI service with `::soap_serve` function generated by soapcpp2 from an interface file declaring a ns__webmethod function
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->cookie_domain = "example.com";
  soap->cookie_path = "/";
  soap_getenv_cookies(soap);
  return soap_serve(soap);
}
// Define a ns__webmethod service operation with parameters that uses HTTP cookies to control state
int ns__webmethod(struct soap *soap, ...)
{
  const char *cookie_value = soap_cookie_value(soap, "cookie_name", NULL, NULL);
  if (cookie_value)
    ... // modify the cookie value to advance the state of this (otherwise stateless CGI) service
  else
    cookie_value = "initial_value";
  soap_set_cookie(soap, "cookie_name", cookie_value, NULL, NULL);
  soap_set_cookie_expire(soap, "cookie_name", 60, NULL, NULL); // cookie expires in 60 seconds
  ... // other webmethod logic
  return SOAP_OK;
}
~~~

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.

@see `::soap::cookie_domain`, `::soap::cookie_path`, `::soap::cookie_max`.
*/
struct soap_cookie * soap_set_cookie(
    struct soap *soap,  ///< `::soap` context
    const char *name,   ///< cookie name
    const char *value,  ///< cookie value to set
    const char *domain, ///< cookie domain or NULL
    const char *path)   ///< cookie path or NULL
  /// @returns pointer to cookie or NULL when an error occurred
  ;

/// Set cookie expiration
/**
This function sets the expiration of the specified cookie `name` in seconds and updates the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns `#SOAP_OK` or a `::soap_status` error code.

@see `::soap::cookie_domain`, `::soap::cookie_path`.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.
*/
int soap_set_cookie_expire(
    struct soap *soap,  ///< `::soap` context
    const char *name,   ///< cookie name
    long maxage,        ///< the age to live in seconds
    const char *domain, ///< cookie domain or NULL
    const char *path)   ///< cookie path or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Set cookie secure
/**
This function sets the "secure" property of the specified cookie `name` and updates the cookie store at the server side.  The "secure" property means that this cookie should be sent by the client to the server only when a secure HTTPS connection can be established.  When HTTPS is enabled all cookies are sent by the server to the client with the "secure" property set, which means that this function is generally not needed unless the server is not HTTPS-enabled but cookies must be secure.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns `#SOAP_OK` or a `::soap_status` error code.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.

@see `::soap::cookie_domain`, `::soap::cookie_path`.
*/
int soap_set_cookie_secure(
    struct soap *soap,  ///< `::soap` context
    const char *name,   ///< cookie name
    const char *domain, ///< cookie domain or NULL
    const char *path)   ///< cookie path or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Set session cookie
/**
This function makes the specified cookie `name` a "session cookie" and updates the cookie store at the server side by marking the cookie as a session cookie.  This means that the cookie will be sent to clients that connect to the server.  This function is not needed when a cookie is modified with `::soap_set_cookie_expire`, for example, because modified cookies are always sent back to the client.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns `#SOAP_OK` or a `::soap_status` error code.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.

@see `::soap::cookie_domain`, `::soap::cookie_path`.
*/
int soap_set_cookie_session(
    struct soap *soap,  ///< `::soap` context
    const char *name,   ///< cookie name
    const char *domain, ///< cookie domain or NULL
    const char *path)   ///< cookie path or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Clear cookie
/**
This function deletes the specified cookie `name` from the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.

@see `::soap::cookie_domain`, `::soap::cookie_path`.
*/
void soap_clr_cookie(
    struct soap *soap,  ///< `::soap` context
    const char *name,   ///< cookie name
    const char *domain, ///< cookie domain or NULL
    const char *path)   ///< cookie path or NULL
  ;

/// Clear session cookie
/**
This function clears the session property of the specified cookie `name` and updates the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns `#SOAP_OK` or a `::soap_status` error code.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.

@see `::soap::cookie_domain`, `::soap::cookie_path`.
*/
int soap_clr_cookie_session(
    struct soap *soap,  ///< `::soap` context
    const char *name,   ///< cookie name
    const char *domain, ///< cookie domain or NULL
    const char *path)   ///< cookie path or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Find a cookie
/**
This function returns the cookie structure of the specified cookie `name` or NULL when not found by searching the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.

@see `::soap::cookie_domain`, `::soap::cookie_path`.
*/
struct soap_cookie * soap_cookie(
    struct soap *soap,  ///< `::soap` context
    const char *name,   ///< cookie name
    const char *domain, ///< cookie domain or NULL
    const char *path)   ///< cookie path or NULL
  /// @returns pointer to a `::soap_cookie` structure or NULL when not found
  ;

/// Get cookie value
/**
This function returns the cookie value of the specified cookie `name` or NULL when not found by searching the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.

@see `::soap::cookie_domain`, `::soap::cookie_path`.
*/
const char * soap_cookie_value(
    struct soap *soap,  ///< `::soap` context
    const char *name,   ///< cookie name
    const char *domain, ///< cookie domain or NULL
    const char *path)   ///< cookie path or NULL
  /// @returns cookie value or NULL when not found
  ;

/// Get cookie expiration
/**
This function returns the cookie expiration time `time_t` of the specified cookie `name` or -1 when not found by searching the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.

@see `::soap::cookie_domain`, `::soap::cookie_path`.
*/
time_t soap_cookie_expire(
    struct soap *soap,  ///< `::soap` context
    const char *name,   ///< cookie name
    const char *domain, ///< cookie domain or NULL
    const char *path)   ///< cookie path or NULL
  /// @returns cookie expiration time
  ;

/// Get cookies from the `HTTP_COOKIE` environment variable
/**
This function initializes the cookie store at the server side by reading the `HTTP_COOKIE` environment variable.  This provides a means for a CGI application to read cookies sent by a client.  Returns `#SOAP_OK` or a `::soap_status` error code when the `HTTP_COOKIE` variable was not found.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.
*/
int soap_getenv_cookies(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Free cookies
/**
This function frees the cookie store and deletes all cookies.

To enable HTTP cookies, the engine must be compiled with `#WITH_COOKIES`.
*/
void soap_free_cookies(struct soap *soap) ///< `::soap` context
  ;

/** @} */

/**
\defgroup group_s2s Conversion functions
@brief This module defines conversion functions of values of various types to and from strings

@{
*/

/// Convert a decimal string to a signed 8 bit byte (char) integer value
/**
This function converts the specified decimal string to a signed 8 bit byte (char) integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2byte(
    struct soap *soap,  ///< `::soap` context
    const char *string, ///< string to convert
    char *value)        ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to a signed 16 bit integer value
/**
This function converts the specified decimal string to a signed 16 bit integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2short(
    struct soap *soap,  ///< `::soap` context
    const char *string, ///< string to convert
    short *value)       ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to a signed 32 bit integer value
/**
This function converts the specified decimal string to a signed 32 bit integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2int(
    struct soap *soap,  ///< `::soap` context
    const char *string, ///< string to convert
    int *value)         ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to a signed `long` integer value
/**
This function converts the specified decimal string to a signed `long` integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2long(
    struct soap *soap,  ///< `::soap` context
    const char *string, ///< string to convert
    long *value)        ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to a signed 64 bit integer value
/**
This function converts the specified decimal string to a signed 64 bit integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2LONG64(
    struct soap *soap,  ///< `::soap` context
    const char *string, ///< string to convert
    LONG64 *value)      ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to a float value
/**
This function converts the specified decimal string to a single precision float value.  Also converts `NaN` and `Inf`.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2float(
    struct soap *soap,  ///< `::soap` context
    const char *string, ///< string to convert
    float *value)       ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to a double float value
/**
This function converts the specified decimal string to a double precision float value.  Also converts `NaN` and `Inf`.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2double(
    struct soap *soap,  ///< `::soap` context
    const char *string, ///< string to convert
    double *value)      ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to an unsigned 8 bit byte (unsigned char) integer value
/**
This function converts the specified decimal string to an unsigned 8 bit byte (unsigned char) integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2unsignedByte(
    struct soap *soap,    ///< `::soap` context
    const char *string,   ///< string to convert
    unsigned char *value) ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to an unsigned 16 bit integer value
/**
This function converts the specified decimal string to an unsigned 16 bit integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2unsignedShort(
    struct soap *soap,     ///< `::soap` context
    const char *string,    ///< string to convert
    unsigned short *value) ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to an unsigned 32 bit integer value
/**
This function converts the specified decimal string to an unsigned 32 bit integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2unsignedInt(
    struct soap *soap,   ///< `::soap` context
    const char *string,  ///< string to convert
    unsigned int *value) ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to an unsigned `long` integer value
/**
This function converts the specified decimal string to an unsigned `long` integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2unsignedLong(
    struct soap *soap,    ///< `::soap` context
    const char *string,   ///< string to convert
    unsigned long *value) ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a decimal string to an unsigned 64 bit integer value
/**
This function converts the specified decimal string to an unsigned 64 bit integer value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.
*/
int soap_s2ULONG64(
    struct soap *soap,  ///< `::soap` context
    const char *string, ///< string to convert
    ULONG64 *value)     ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Copy a string (ASCII or UTF-8) to a new string while converting and validating the string contents
/**
This function copies the specified string to a new string allocated in managed memory.  The specified string is validated against the `minlen` and `maxlen` constraints and against the `pattern` XSD regex when non-NULL and when the `::soap::fsvalidate` callback is assigned to perform this check.  The minimum and maximum length constraints are 0 and `::soap::maxlength`, respectively, when `minlen` or `maxlen` are specified as negative values.  The string length verification takes UTF-8 multi-byte encoding into account when the `#SOAP_C_UTFSTRING` mode flag is enabled.  The `flag` parameter controls the conversion as follows: 0 = no conversion, 4 = collapse white space and normalize white space with blanks, 5 = collapse white space.  Returns `#SOAP_OK` or a `::soap_status` error code.
*/
int soap_s2char(
    struct soap *soap,   ///< `::soap` context
    const char *string,  ///< string to convert
    char **value,        ///< pointer to a variable to assign the new string to
    int flag,            ///< controls conversion (0, 4 or 5)
    long minlen,         ///< minimum string length constraint, when non-negative
    long maxlen,         ///< maximum string length constraint, when non-negative
    const char *pattern) ///< XSD regex pattern constraint, when non-NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Copy a string (ASCII or UTF-8) to a new string while converting and validating the string contents
/**
This function copies the specified string to a new string allocated in managed memory.  The specified string is validated against the `minlen` and `maxlen` constraints and against the `pattern` XSD regex when non-NULL and when the `::soap::fsvalidate` callback is assigned to perform this check.  The minimum and maximum length constraints are 0 and `::soap::maxlength`, respectively, when `minlen` or `maxlen` are specified as negative values.  The string length verification takes UTF-8 multi-byte encoding into account when the `#SOAP_C_UTFSTRING` mode flag is enabled.  The `flag` parameter controls the conversion as follows: 0 = no conversion, 4 = collapse white space and normalize white space with blanks, 5 = collapse white space.  Returns `#SOAP_OK` or a `::soap_status` error code.
*/
int soap_s2stdchar(
    struct soap *soap,   ///< `::soap` context
    const char *string,  ///< string to convert
    std::string *value,  ///< pointer to a variable to assign the new string to
    int flag,            ///< controls conversion (0, 4 or 5)
    long minlen,         ///< minimum string length constraint, when non-negative
    long maxlen,         ///< maximum string length constraint, when non-negative
    const char *pattern) ///< XSD regex pattern constraint, when non-NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Copy a string (ASCII or UTF-8) to a new wide string while converting and validating the string contents
/**
This function copies the specified string to a new wide string allocated in managed memory.  When the `#SOAP_C_UTFSTRING` mode flag is enabled the specified string should contain UTF-8 contents or ASCII otherwise.  The specified string is validated against the `minlen` and `maxlen` constraints and against the `pattern` XSD regex when non-NULL and when the `::soap::fsvalidate` callback is assigned to perform this check.  The minimum and maximum length constraints are 0 and `::soap::maxlength`, respectively, when `minlen` or `maxlen` are specified as negative values.  The `flag` parameter controls the conversion as follows: 0 = no conversion, 4 = collapse white space and normalize white space with blanks, 5 = collapse white space.  Returns `#SOAP_OK` or a `::soap_status` error code.

@see `#SOAP_C_UTFSTRING`, `#WITH_REPLACE_ILLEGAL_UTF8`, `#SOAP_UNKNOWN_UNICODE_CHAR`.
*/
int soap_s2wchar(
    struct soap *soap,   ///< `::soap` context
    const char *string,  ///< string to convert
    wchar_t **value,     ///< pointer to a variable to assign the new string to
    int flag,            ///< controls conversion (0, 4 or 5)
    long minlen,         ///< minimum string length constraint, when non-negative
    long maxlen,         ///< maximum string length constraint, when non-negative
    const char *pattern) ///< XSD regex pattern constraint, when non-NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Copy a string (ASCII or UTF-8) to a new wide string while converting and validating the string contents
/**
This function copies the specified string to a new wide string allocated in managed memory.  When the `#SOAP_C_UTFSTRING` mode flag is enabled the specified string should contain UTF-8 contents or ASCII otherwise.  The specified string is validated against the `minlen` and `maxlen` constraints and against the `pattern` XSD regex when non-NULL and when the `::soap::fsvalidate` callback is assigned to perform this check.  The minimum and maximum length constraints are 0 and `::soap::maxlength`, respectively, when `minlen` or `maxlen` are specified as negative values.  The `flag` parameter controls the conversion as follows: 0 = no conversion, 4 = collapse white space and normalize white space with blanks, 5 = collapse white space.  Returns `#SOAP_OK` or a `::soap_status` error code.

@see `#SOAP_C_UTFSTRING`, `#WITH_REPLACE_ILLEGAL_UTF8`, `#SOAP_UNKNOWN_UNICODE_CHAR`.
*/
int soap_s2stdwchar(
    struct soap *soap,   ///< `::soap` context
    const char *string,  ///< string to convert
    std::wstring *value, ///< pointer to a variable to assign the new string to
    int flag,            ///< controls conversion (0, 4 or 5)
    long minlen,         ///< minimum string length constraint, when non-negative
    long maxlen,         ///< maximum string length constraint, when non-negative
    const char *pattern) ///< XSD regex pattern constraint, when non-NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Copy a string (ASCII or UTF-8) to a new QName string while normalizing and validating the string contents
/**
This function copies the specified string to a new QName string allocated in managed memory.  The specified string should be a QName or a space-separated list of QNames.  A given QName is of the form <i>`prefix:suffix`</i> or <i>`"URI":suffix`</i>, where the latter is converted to the former form by this function by replacing the quoted URIs by prefixes using the `::namespaces` table.  The specified string is validated against the `minlen` and `maxlen` constraints and against the `pattern` XSD regex when non-NULL and when the `::soap::fsvalidate` callback is assigned to perform this check.  The minimum and maximum length constraints are 0 and `::soap::maxlength`, respectively, when `minlen` or `maxlen` are specified as negative values.  The string length verification takes UTF-8 multi-byte encoding into account when the `#SOAP_C_UTFSTRING` mode flag is enabled.  Returns `#SOAP_OK` or a `::soap_status` error code.
*/
int soap_s2QName(
    struct soap *soap,   ///< `::soap` context
    const char *string,  ///< string to convert
    char **value,        ///< pointer to a variable to assign the new string to
    long minlen,         ///< minimum string length constraint, when non-negative
    long maxlen,         ///< maximum string length constraint, when non-negative
    const char *pattern) ///< XSD regex pattern constraint, when non-NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Copy a string (ASCII or UTF-8) to a new QName string while normalizing and validating the string contents
/**
This function copies the specified string to a new QName string allocated in managed memory.  The specified string should be a QName or a space-separated list of QNames.  A given QName is of the form <i>`prefix:suffix`</i> or <i>`"URI":suffix`</i>, where the latter is converted to the former form by this function by replacing the quoted URIs by prefixes using the `::namespaces` table.  The specified string is validated against the `minlen` and `maxlen` constraints and against the `pattern` XSD regex when non-NULL and when the `::soap::fsvalidate` callback is assigned to perform this check.  The minimum and maximum length constraints are 0 and `::soap::maxlength`, respectively, when `minlen` or `maxlen` are specified as negative values.  The string length verification takes UTF-8 multi-byte encoding into account when the `#SOAP_C_UTFSTRING` mode flag is enabled.  Returns `#SOAP_OK` or a `::soap_status` error code.
*/
int soap_s2stdQName(
    struct soap *soap,   ///< `::soap` context
    const char *string,  ///< string to convert
    std::string *value,  ///< pointer to a variable to assign the new string to
    long minlen,         ///< minimum string length constraint, when non-negative
    long maxlen,         ///< maximum string length constraint, when non-negative
    const char *pattern) ///< XSD regex pattern constraint, when non-NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Convert a string with <i>`xsd:dateTime`</i> contents to a `time_t` value
/**
This function converts the specified string with <i>`xsd:dateTime`</i> contents to a `time_t` value.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_TYPE` when the string could not be converted.

@see `#WITH_NOZONE`.
*/
int soap_s2dateTime(
    struct soap *soap,  ///< `::soap` context
    const char *string, ///< string to convert
    time_t *value)      ///< pointer to a variable to assign the converted value to
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Encode binary data as a base64-formatted string
/**
This function encodes the specified binary `data` of length `len` as a base64-formatted string stored in the specified `base64` buffer or allocated in managed memory when `base64` is NULL.  Note that `base64` should be large enough to store the encoded string, i.e. (`len` + 2) / 3 * 4 + 1 bytes.  Returns the string or NULL when an error occurred and sets `::soap::error` to a `::soap_status` value.
*/
char * soap_s2base64(
    struct soap *soap,         ///< `::soap` context
    const unsigned char *data, ///< data to encode
    char *base64,              ///< buffer to store base64 string or NULL
    int len)                   ///< length of the data to encode
  /// @returns the converted string or NULL when an error occurred
  ;

/// Encode binary data as a hex-formatted string
/**
This function encodes the specified binary `data` of length `len` as a hex-formatted string stored in the specified `hex` buffer or allocated in managed memory when `hex` is NULL.  Note that `hex` should be large enough to store the encoded string, i.e. 2 * `len` + 1 bytes.  Returns the string or NULL when an error occurred and sets `::soap::error` to a `::soap_status` value.
*/
char * soap_s2hex(
    struct soap *soap,         ///< `::soap` context
    const unsigned char *data, ///< data to encode
    char *hex,                 ///< buffer to store hex string or NULL
    int len)                   ///< length of the data to encode
  /// @returns the converted string or NULL when an error occurred
  ;

/// Convert a signed 8 bit byte (char) integer to a decimal string
/**
This function converts the specified signed 8 bit byte (char) integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_byte2s(
    struct soap *soap, ///< `::soap` context
    char value)        ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert a signed 16 bit integer to a decimal string
/**
This function converts the specified signed 16 bit integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_short2s(
    struct soap *soap, ///< `::soap` context
    short value)       ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert a signed 32 bit integer to a decimal string
/**
This function converts the specified signed 32 bit integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_int2s(
    struct soap *soap, ///< `::soap` context
    int value)         ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert a signed `long` integer to a decimal string
/**
This function converts the specified signed `long` integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_long2s(
    struct soap *soap, ///< `::soap` context
    long value)        ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert a signed 64 bit integer to a decimal string
/**
This function converts the specified signed 64 bit integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_LONG642s(
    struct soap *soap, ///< `::soap` context
    LONG64 value)      ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert a float value to a decimal string
/**
This function converts the specified single precision floating point value to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_float2s(
    struct soap *soap, ///< `::soap` context
    float value)       ///< float value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert a double float value to a decimal string
/**
This function converts the specified double precision floating point value to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_double2s(
    struct soap *soap, ///< `::soap` context
    double value)      ///< double float value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert an unsigned 8 bit byte (char) integer to a decimal string
/**
This function converts the specified unsigned 8 bit byte (char) integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_unsignedByte2s(
    struct soap *soap,   ///< `::soap` context
    unsigned char value) ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert an unsigned 16 bit integer to a decimal string
/**
This function converts the specified unsigned 16 bit integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_unsignedShort2s(
    struct soap *soap,    ///< `::soap` context
    unsigned short value) ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert an unsigned 32 bit integer to a decimal string
/**
This function converts the specified unsigned 32 bit integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_unsignedInt2s(
    struct soap *soap,  ///< `::soap` context
    unsigned int value) ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert an unsigned `long` integer to a decimal string
/**
This function converts the specified unsigned `long` integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_unsignedLong2s(
    struct soap *soap,   ///< `::soap` context
    unsigned long value) ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert an unsigned 64 bit integer to a decimal string
/**
This function converts the specified unsigned 64 bit integer to a decimal string stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.
*/
const char * soap_ULONG642s(
    struct soap *soap, ///< `::soap` context
    ULONG64 value)     ///< integer value to convert to decimal string
  /// @returns the decimal number stored in the `::soap::tmpbuf` string buffer
  ;

/// Convert a wide string to a UTF-8 encoded string
/**
This function converts the specified wide string to a UTF-8 encoded string allocated in managed memory.  Returns the resulting string or NULL when an error occurred and sets `::soap::error` to a `::soap_status` value.

@see `#WITH_REPLACE_ILLEGAL_UTF8`, `#SOAP_UNKNOWN_UNICODE_CHAR`.
*/
const char * soap_wchar2s(
    struct soap *soap,    ///< `::soap` context
    const wchar_t *value) ///< wide string to convert
  /// @returns UTF-8 encoded string allocated in managed memory or NULL when an error occurred
  ;

/// Convert a `time_t` value to a string with <i>`xsd:dateTime`</i> contents
/**
This function converts the specified `time_t` value to a string with <i>`xsd:dateTime`</i> contents stored in the temporary buffer `::soap::tmpbuf`.  Returns `::soap::tmpbuf`.

@see `#WITH_NOZONE`.
*/
const char * soap_dateTime2s(
    struct soap *soap, ///< `::soap` context
    time_t value)      ///< value to convert
  /// @returns the <i>`xsd:dateTime`</i> string stored in the `::soap::tmpbuf` string buffer
  ;

/// Decode a base64-formatted string to binary data
/**
This function decodes the specified `base64` string to binary data.  The `data` parameter is a buffer of length `maxdatalen` to store the decoded data or NULL to dynamically allocate the decoded data in managed memory which is returned.  The length of the decoded data is stored in the length variable pointed to by `datalen` when non-NULL.  Returns the decoded binary string that is 0-terminated if `maxdatalen` is sufficiently large, or returns NULL if an error occurred.
*/
const char * soap_base642s(
    struct soap *soap,  ///< `::soap` context
    const char *base64, ///< base64 string
    char *data,         ///< buffer to store the decoded data or NULL to allocate one
    size_t maxdatalen,  ///< size of the buffer
    int *datalen)       ///< pointer to the length variable to assign or NULL
  /// @returns decoded string or NULL when an error occurred
  ;

/// Decode a hex-formatted string to binary data
/**
This function decodes the specified `hex` string to binary data.  The `data` parameter is a buffer of length `maxdatalen` to store the decoded data or NULL to dynamically allocate the decoded data in managed memory which is returned.  The length of the decoded data is stored in the length variable pointed to by `datalen` when non-NULL.  Returns the decoded binary string that is 0-terminated if `maxdatalen` is sufficiently large, or returns NULL if an error occurred.
*/
const char * soap_hex2s(
    struct soap *soap, ///< `::soap` context
    const char *hex,   ///< hex string
    char *data,        ///< buffer to store the decoded data or NULL to allocate one
    size_t maxdatalen, ///< size of the buffer
    int *datalen)      ///< pointer to the length variable to assign or NULL
  /// @returns decoded string or NULL when an error occurred
  ;

/** @} */

/**
\defgroup group_namespace XML namespace tables
@brief This module defines the `::Namespace` XML namespace structure and function to activate a table

@{
*/

/// Structure of each row in a namespace table
/**
XML namespaces tables define XML namespace prefix-URI pairs for XML generation, parsing and validation.  Each row in the table is defined by this structure.  The last row in the table must be followed by a row with a NULL value to indicate the end of the table.  The first four rows are reserved for the namespaces of `SOAP-ENV`, `SOAP-ENC`, `xsi` and `xsd`.  One or more of these entries can be omitted by using `""` in the first column for the prefix, but beware that XML generation and parsing may fail when an omitted namespace prefix or URI is still used in the XML text.

This structure defines the rows the namespace table `::namespaces` and the namespace table parameter of `::soap_set_namespaces`.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct Namespace namespaces[] = {
  {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/", "http://www.w3.org/*soap-envelope",      NULL},
  {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/", "http://www.w3.org/*soap-encoding",      NULL},
  {"xsi",      "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*XMLSchema-instance", NULL},
  {"xsd",      "http://www.w3.org/2001/XMLSchema",          "http://www.w3.org/*XMLSchema",          NULL},
  {"ns",       "http://tempuri.org/ns.xsd",                 NULL,                                    NULL},
  {NULL,       NULL,                                        NULL,                                    NULL}
};
~~~

This example defines SOAP 1.1 namespaces (`SOAP-ENV` and `SOAP-ENC`) to be used by default, but also accepts SOAP 1.2 because of the second URI in the third column.  XML schema instance namespace `xsi` is used with <i>`xsi:type`</i> and <i>`xsi:nil`</i> and the XML schema namespace `xsd` is used with XSD types such as <i>`xsd:string`</i>, which may be used in XML messages.  URI patterns in the third column may contain wildcard strings `*` and wildcard characters `-`.
*/
struct Namespace {
  /// XML namespace prefix identifier string, use NULL to indicate the end of the namespaces table
  const char *id;
  /// XML namespace URI string
  const char *ns;
  /// an optional XML namespace URI string pattern (`*` is a wildcard string and `-` is a wildcard character) that is permitted to match a parsed URI in place of the first URI, or NULL when not applicable
  const char *in;
  /// Reserved for internal use by the engine only, to switch between URIs such as SOAP 1.1 and 1.2 namespaces based on the URI string pattern when provided in the table
  char *out;
};

/// The global XML namespaces table with entries defined by the `::Namespace` structure and populated in an .nsmap file generated by soapcpp2
/**
This XML namespaces table defines XML namespace prefix-URI pairs for XML generation, parsing and validation.  The last row in the table must be followed by a row with a NULL value to indicate the end of the table.  The requirement to link this global table can be removed at compile time with `#WITH_NONAMESPACES`.
*/
struct Namespace namespaces[];

/// Activates an XML namespace table to generate and resolve xmlns namespace prefixes in XML messages
/**
This function sets the XML namespace table to be used by the engine to generate and resolve xmlns namespace prefixes in XML messages.  Different tables can be set at any time, depending on the XML messaging requirements.  However, XML generation and parsing may fail if prefix-URI bindings are missing in the table.  Tables are generated by soapcpp2 in .nsmap files, which defines a global table `::namespaces` that is used by default by the engines, but can be replaced by the XML namespace table specified with this function.  Returns `#SOAP_OK` or a `::soap_status` error code.

@par Example:
~~~{.cpp}
#include "soapH.h"

struct Namespace my_namespaces[] = {
  { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/", "http://www.w3.org/*soap-envelope",      NULL },
  { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/", "http://www.w3.org/*soap-encoding",      NULL },
  { "xsi",      "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*XMLSchema-instance", NULL },
  { "xsd",      "http://www.w3.org/2001/XMLSchema",          "http://www.w3.org/*XMLSchema",          NULL },
  { "ns",       "http://tempuri.org/ns.xsd",                 NULL,                                    NULL },
  { NULL,       NULL,                                        NULL,                                    NULL }
};

struct soap *soap = soap_new();
soap_set_namespaces(soap, my_namespaces); // use a specific XML namespace binding table for the next call
if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
{
  soap_print_fault(soap, stderr);
}
else
{
  ...
}
soap_set_namespaces(soap, namespaces); // use the default XML namespace binding table, when defined, for the next call
~~~

This example defines SOAP 1.1 namespaces (`SOAP-ENV` and `SOAP-ENC`) to be used by default, but also accepts SOAP 1.2 because of the second URI in the third column.  XML schema instance namespace `xsi` is used with <i>`xsi:type`</i> and <i>`xsi:nil`</i> and the XML schema namespace `xsd` is used with XSD types such as <i>`xsd:string`</i>, which may be used in XML messages.  URI patterns in the third column may contain wildcard strings `*` and wildcard characters `-`.
*/
int soap_set_namespaces(
    struct soap *soap,                  ///< `::soap` context
    const struct Namespace *namespaces) ///< the XML namespace table to activate
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/** @} */

/**
\defgroup group_header Header structure and functions
@brief This module defines the `::SOAP_ENV__Header` structure and `::soap_header` function to allocate the header

@{
*/

/// SOAP Header structure
/**
This structure is generated by the wsdl2h tool from a WSDL with SOAP Header definitions and/or by soapcpp2 to complete the SOAP Header definitions.  The SOAP Header definitions can also be specified manually in the interface header file for soapcpp2.  If no SOAP Header structure is declared in the interface header file input to soapcpp2 then the soapcpp2 tool will generate an empty structure.

A SOAP Header contains meta-data, such as WS-Addressing and WS-Security headers, associated with messages.  SOAP Header elements may be marked with `mustUnderstand` which produces and recognizes <i>`SOAP_ENV:mustUnderstand="true"`</i> in XML to force the receiver to produce an error if the header element was not recognized.

The `//gsoap <prefix> service method-input-header-part:` and `//gsoap <prefix> service method-output-header-part:` directives indicates which member (i.e. XML element) of `::SOAP_ENV__Header` is relevant to the input and output SOAP Headers associated with a service operation.

Because this structure is declared `mutable` (which is a C/C++ extension that only soapcpp2 understands), multiple `::SOAP_ENV__Header` structures in the interface header file input are combined into one structure generated by soapcpp2 for C/C++ compilation.

@par Example:

~~~{.cpp}
// example .h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
struct SOAP_ENV__Header {
  mustUnderstand int *ns__someHeaderValue; // optional element with SOAP_ENV:mustUnderstand="true" attribute
};
//gsoap ns service method-input-header-part: webmethod ns__someHeaderValue
//gsoap ns service method-output-header-part: webmethod ns__someHeaderValue
int ns__webmethod(...);
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // context initializations
// add a SOAP Header to the message
soap->header = NULL; // make sure we allocate a new header
soap_header(soap);   // allocate SOAP_ENV__Header and set soap->header to point to it
int num = 123;
soap->header->ns__someHeaderValue = &num;
if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
{
  soap_print_fault(soap, stderr);
}
else
{
  if (soap->header) // received a SOAP_ENV__Header?
    ... // yes, inspect SOAP_ENV__Header ns__someHeaderValue
}
soap_destroy(soap);
soap_end(soap);
soap_free(soap);
~~~

@see `::soap::header`, `::soap::actor`, `::soap_header`.
*/
struct SOAP_ENV__Header {
};

/// If `::soap::header` is NULL then allocate `::SOAP_ENV__Header` header and set `::soap::header` to point to it
/**
If `::soap::header` is NULL then this function allocates a `::SOAP_ENV__Header` structure in memory managed by the context, default initializes its members, and sets `::soap::header` to point to the allocated structure.

@see `::SOAP_ENV__Header`.
*/
void soap_header(struct soap *soap) ///< `::soap` context
  ;

/** @} */

/**
\defgroup group_fault Fault structure and functions
@brief This module defines the `::SOAP_ENV__Fault` structure and functions to set and get fault information

@{
*/

/// SOAP Fault structure
/**
This structure is generated by the wsdl2h tool from a WSDL with SOAP Fault definitions and/or by soapcpp2 to complete the SOAP Fault definitions.  The SOAP Fault definitions can also be specified manually in the interface header file for soapcpp2.  If no SOAP Fault structure is declared in the interface header file input to soapcpp2 then the soapcpp2 tool will generate an empty structure.

A SOAP Fault contains error information specified by the `::SOAP_ENV__Fault::faultstring` (SOAP 1,1) or `::SOAP_ENV__Fault::SOAP_ENV__Reason` (SOAP 1.2).  The SOAP Fault detail `::SOAP_ENV__Fault::SOAP_ENV__Detail` may include specific elements related to the fault.

The `::SOAP_ENV__Detail` sub-structure of a SOAP Fault is customizable with members that are part of a SOAP Fault that is specific to a service operation.  This structure is generated and populated by wsdl2h with service-specific SOAP Fault details.  The `//gsoap <prefix> service method-fault:` directive indicates which member (i.e. XML element) of `::SOAP_ENV__Detail` is relevant to the SOAP Faults associated with a service operation.

Because the `::SOAP_ENV__Detail` substructure is declared `mutable` (which is a C/C++ extension that only soapcpp2 understands), multiple `struct SOAP_ENV__Detail` structures in the interface header file input are combined into one structure generated by soapcpp2 for C/C++ compilation.

@par Examples:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // call a Web service here
if (soap->error)
{
  const char *s = soap_fault_string(soap);
  const char *d = soap_fault_detail(soap);
  printf("Server fault: %s detail: %s\n", s, d ? d : "(none)");
}
~~~

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new();
... // context initializations
if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
{
  if (soap->fault->detail && soap->fault->detail->__type == SOAP_TYPE_ns__someElement)
  {
    struct ns__someElement *element = (struct ns__someElement*)soap->fault->detail->fault;
    ... // inspect the SOAP Fault detail element
  }
}
else
{
  ... // success
}
soap_destroy(soap);
soap_end(soap);
soap_free(soap);
~~~

~~~{.cpp}
// example .h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
struct ns__someElement {
  char *text;
};
struct SOAP_ENV__Detail {
  char *__any;
  int __type;
  void *fault;
  struct ns__someElement *ns__someElement; // a service-operation specific fault detail
};
//gsoap ns service method-fault: webmethod ns__someElement
int ns__webmethod(...);
~~~

~~~{.cpp}
// example service implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int ns__webmethod(struct soap *soap, ...)
{
  int err = soap_sender_fault(soap, "Invalid request", NULL, NULL); // this is a sender fault to return to the client
  soap_faultdetail(soap); // allocate SOAP Fault detail
  soap->fault->detail->ns__someElement = (struct ns__someElement*)soap_malloc(soap, sizeof(struct ns__someElement));
  soap_default_ns__someElement(soap, soap->fault->detail->ns__someElement);
  soap->fault->detail->ns__someElement->text = "...";
#if 0
  // an alternative way to include the ns__someElement in the SOAP Fault detail can be done as follows:
  soap->fault->detail->__type = SOAP_TYPE_ns__someElement;
  soap->fault->detail->fault = (void*)soap_malloc(soap, sizeof(struct ns__someElement));
  soap_default_ns__someElement(soap, (struct ns__someElement*)soap->fault->detail->fault);
  ((struct ns__someElement*)soap->fault->detail->fault)->text = "...";
#endif
  return err;
}
~~~

@see `#SOAP_CLI_FAULT`, `#SOAP_SVR_FAULT`, `::soap::fault`, `::soap_sender_fault`, `::soap_sender_fault_subcode`, `::soap_receiver_fault`, `::soap_receiver_fault_subcode`, `::soap_print_fault`, `::soap_stream_fault`, `::soap_sprint_fault`, `::soap_print_fault_location`, `::soap_stream_fault_location`, `::soap_fault_string`, `::soap_fault_subcode`, `::soap_fault_detail`, `::soap::lang`.
*/
struct SOAP_ENV__Fault {
  /// Optional element `faultcode` of XSD type <i>`xsd:QName`</i>
  _QName faultcode;
  /// Optional element `faultstring` of XSD type <i>`xsd:string`</i>
  char *faultstring;
  /// Optional element `faultactor` of XSD type <i>`xsd:string`</i>
  char *faultactor;
  /// Optional element `detail` of XSD type <i>`SOAP-ENV:Detail`</i>
  struct SOAP_ENV__Detail *detail;
  /// Optional element <i>`SOAP-ENV:Code`</i> of XSD type <i>`SOAP-ENV:Code`</i>
  struct SOAP_ENV__Code *SOAP_ENV__Code;
  /// Optional element <i>`SOAP-ENV:Reason`</i> of XSD type <i>`SOAP-ENV:Reason`</i>
  struct SOAP_ENV__Reason *SOAP_ENV__Reason;
  /// Optional element <i>`SOAP-ENV:Node`</i> of XSD type <i>`xsd:string`</i>
  char *SOAP_ENV__Node;
  /// Optional element <i>`SOAP-ENV:Role`</i> of XSD type <i>`xsd:string`</i>
  char *SOAP_ENV__Role;
  /// Optional element <i>`SOAP-ENV:Detail`</i> of XSD type <i>`SOAP-ENV:Detail`</i>
  struct SOAP_ENV__Detail *SOAP_ENV__Detail;
};

/// SOAP Fault Code structure
/**
@see `::SOAP_ENV__Fault`.
*/
struct SOAP_ENV__Code {
  /// Optional element <i>`SOAP-ENV:Value`</i> of XSD type <i>`xsd:QName`</i>
  _QName SOAP_ENV__Value;
  /// Optional element <i>`SOAP-ENV:Subcode`</i> of XSD type <i>`SOAP-ENV:Code`</i>
  struct SOAP_ENV__Code *SOAP_ENV__Subcode;
};

/// SOAP Fault Detail structure
/**
@see `::SOAP_ENV__Fault`.
*/
struct SOAP_ENV__Detail {
  /// Any data of some type `T` serialized as `fault` element when its `SOAP_TYPE_T` is assigned to `__type`
  int __type;
  /// Any data of some type `T` serialized as `fault` element when its `SOAP_TYPE_T` is assigned to `__type`
  void *fault;
  /// Any XML content
  _XML __any;
};

/// SOAP Fault Reason structure generated by soapcpp2
/**
@see `::SOAP_ENV__Fault`, `::soap::lang`.
*/
struct SOAP_ENV__Reason {
  /// Optional element <i>`SOAP-ENV:Text`</i> of XSD type <i>`xsd:string`</i>
  char *SOAP_ENV__Text;
};

/// If `::soap::fault` is NULL then allocate `::SOAP_ENV__Fault` header and set `::soap::fault` to point to it
/**
If `::soap::fault` is NULL then this function allocates a `::SOAP_ENV__Fault` structure in memory managed by the context, default initializes its members, and sets `::soap::fault` to point to the allocated structure.  If `::soap::version` == 2 (SOAP 1.2) then also allocates and default initializes `::SOAP_ENV__Fault::SOAP_ENV__Code` and `::SOAP_ENV__Fault::SOAP_ENV__Reason`.  This function is used by `::soap_sender_fault`, `::soap_sender_fault_subcode`, `::soap_receiver_fault` and `::soap_receiver_fault_subcode` to allocate and set a SOAP Fault.

@see `::SOAP_ENV__Fault`, `::soap_sender_fault`, `::soap_sender_fault_subcode`, `::soap_receiver_fault`, `::soap_receiver_fault_subcode`.
*/
void soap_fault(struct soap *soap) ///< `::soap` context
  ;

/// Set SOAP 1.1 client fault / SOAP 1.2 sender fault string and detail
/**
This function assigns a SOAP 1.1 server fault / SOAP 1.2 receiver fault to the `::soap::fault` structure and sets `::soap::error` to `#SOAP_FAULT`, indicating an invalid request by the sender, which the receiver cannot recover from.  The `faultstring` parameter is a human-readable message with the reason of the error.  The `faultdetail` parameter is an XML-formatted string with details or NULL when omitted.  Returns `#SOAP_FAULT`.
*/
int soap_sender_fault(
    struct soap *soap,       ///< `::soap` context
    const char *faultstring, ///< SOAP Fault string with reason
    const char *faultdetail) ///< SOAP Fault detail with XML or NULL
  /// @returns `#SOAP_FAULT`
  ;

/// Set SOAP 1.1 client fault / SOAP 1.2 sender fault subcode, string and detail
/**
This function assigns a SOAP 1.1 server fault / SOAP 1.2 receiver fault to the `::soap::fault` structure and sets `::soap::error` to `#SOAP_FAULT`, indicating an invalid request by the sender, which the receiver cannot recover from.  The `faultcode` parameter is a QName specific to the protocol associated with the fault.  The `faultstring` parameter is a human-readable message with the reason of the error.  The `faultdetail` parameter is an XML-formatted string with details or NULL when omitted.  Returns `#SOAP_FAULT`.
*/
int soap_sender_fault_subcode(
    struct soap *soap,        ///< `::soap` context
    const char *faultsubcode, ///< SOAP Fault subcode with QName
    const char *faultstring,  ///< SOAP Fault string with reason
    const char *faultdetail)  ///< SOAP Fault detail with XML or NULL
  /// @returns `#SOAP_FAULT`
  ;

/// Set SOAP 1.1 server fault / SOAP 1.2 receiver fault string and detail
/**
This function assigns a SOAP 1.1 server fault / SOAP 1.2 receiver fault to the `::soap::fault` structure and sets `::soap::error` to `#SOAP_FAULT`, indicating that the receiver encountered an error in processing the request and may recover from the error later.  The `faultstring` parameter is a human-readable message with the reason of the error.  The `faultdetail` parameter is an XML-formatted string with details or NULL when omitted.  Returns `#SOAP_FAULT`.
*/
int soap_receiver_fault(
    struct soap *soap,       ///< `::soap` context
    const char *faultstring, ///< SOAP Fault string with reason
    const char *faultdetail) ///< SOAP Fault detail with XML or NULL
  /// @returns `#SOAP_FAULT`
  ;

/// Set SOAP 1.1 server fault / SOAP 1.2 receiver fault subcode, string and detail
/**
This function assigns a SOAP 1.1 server fault / SOAP 1.2 receiver fault to the `::soap::fault` structure and sets `::soap::error` to `#SOAP_FAULT`, indicating that the receiver encountered an error in processing the request and may recover from the error later.  The `faultcode` parameter is a QName specific to the protocol associated with the fault.  The `faultstring` parameter is a human-readable message with the reason of the error.  The `faultdetail` parameter is an XML-formatted string with details or NULL when omitted.  Returns `#SOAP_FAULT`.
*/
int soap_receiver_fault_subcode(
    struct soap *soap,        ///< `::soap` context
    const char *faultsubcode, ///< SOAP Fault subcode with QName
    const char *faultstring,  ///< SOAP Fault string with reason
    const char *faultdetail)  ///< SOAP Fault detail with XML or NULL
  /// @returns `#SOAP_FAULT`
  ;

/// Print error message on the specified output
/**
This function prints the error message of the nonzero `::soap::error` value.  The `fd` parameter specifies the output file descriptor, such as stderr.  This function has no effect when `::soap:error` is `#SOAP_OK` (zero).
*/
void soap_print_fault(
    struct soap *soap, ///< `::soap` context
    FILE *fd)          ///< output file descriptor to print to
  ;

/// Print error message on the specified output stream
/**
This function prints the error message of the nonzero `::soap::error` value.  The `os` parameter specifies the output stream such as std::cerr.  This function has no effect when `::soap:error` is `#SOAP_OK` (zero).
*/
void soap_stream_fault(
    struct soap *soap, ///< `::soap` context
    std::ostream& os)  ///< output stream to print to
  ;

/// Print error message to the specified string buffer
/**
This function prints the error message of the nonzero `::soap::error` value to the string buffer specified by the `buf` and `len` parameters.  This function has no effect when `::soap:error` is `#SOAP_OK` (zero).
*/
char * soap_sprint_fault(
    struct soap *soap, ///< `::soap` context
    char *buf,         ///< string buffer to store the message, 0-terminated
    size_t len)        ///< buffer length
  /// @returns the `buf` parameter
  ;

/// Print the location in the message where and when the error occurred
/**
This function prints the location in the inbound message being parsed where the error occurred.  Only part of the message will be printed, depending on the `#SOAP_BUFLEN` size and the size of the actual data packet received into the `::soap::buf` buffer.  This function is usually used after calling `::soap_print_fault` which makes sense when a message processing error occurred such as when `::soap_xml_error_check(soap->error)` is true.  This function has no effect when `::soap:error` is `#SOAP_OK` (zero).
*/
void soap_print_fault_location(
    struct soap *soap, ///< `::soap` context
    FILE *fd)          ///< output file descriptor to print to
  ;

/// Print the location in the message where and when the error occurred
/**
This function prints the location in the inbound message being parsed where the error occurred.  Only part of the message will be printed, depending on the `#SOAP_BUFLEN` size and the size of the actual data packet received into the `::soap::buf` buffer.  This function is usually used after calling `::soap_stream_fault` which makes sense when a message processing error occurred such as when `::soap_xml_error_check(soap->error)` is true.  This function has no effect when `::soap:error` is `#SOAP_OK` (zero).
*/
void soap_stream_fault_location(
    struct soap *soap, ///< `::soap` context
    std::ostream& os)  ///< output stream to print to
  ;

/// Returns the SOAP Fault subcode QName string or NULL when absent
const char * soap_fault_subcode(struct soap *soap)
  /// @returns string or NULL
  ;

/// Returns the SOAP Fault string/reason or NULL when absent
const char * soap_fault_string(struct soap *soap) ///< `::soap` context
  /// @returns string or NULL
  ;

/// Returns the SOAP Fault detail XML string or NULL when absent
const char * soap_fault_detail(struct soap *soap) ///< `::soap` context
  /// @returns string or NULL
  ;

/// Allocates and returns a pointer to the SOAP Fault subcode QName string to set this string
const char ** soap_faultsubcode(struct soap *soap) ///< `::soap` context
  /// @returns pointer to string
  ;

/// Allocates and returns a pointer to the SOAP Fault string/reason to set this string
const char ** soap_faultstring(struct soap *soap) ///< `::soap` context
  /// @returns pointer to string
  ;

/// Allocates and returns a pointer to the SOAP Fault detail XML string to set this string or NULL when not accessible
const char ** soap_faultdetail(struct soap *soap) ///< `::soap` context
  /// @returns pointer to string or NULL
  ;

/** @} */

/**
\defgroup group_dime DIME attachment functions
@brief This module defines functions to set and get DIME attachments

For more information on the DIME protocol, see for example https://msdn.microsoft.com/en-us/library/aa480488.aspx

There are two ways to add DIME attachments to SOAP/XML messages for sending:
- use `::soap_set_dime_attachment` to explicitly add an attachment that contains the specified source of data;
- use `::xsd__base64Binary` or `::_xop__Include` structures in the serializable data of a SOAP/XML message, where the specified data is serialized in DIME attachments automatically when one of the `id`, `type` or `options` member variables are non-NULL.

Both methods also support streaming DIME attachments to send using the `::soap::fdimereadopen`, `::soap::fdimeread`, and `::soap::fdimereadclose` callbacks that fetch the data to send, where a user-defined handle should be specified for the `ptr` parameter of `::soap_set_dime_attachment` or the `::xsd__base64Binary::__ptr` member variable instead of a pointer to the actual data.  This handle can be used by the callbacks to fetch the specific data to transmit.

Receiving DIME attachments attached to SOAP/XML messages is automatic.  The DIME attachments are converted to binary data and stored in the `::xsd__base64Binary` structures that reference the DIME attachments via the `::xsd__base64Binary::id` string, meaning that the `::xsd__base64Binary::__ptr` (points to the data) and `::xsd__base64Binary::__size` (data length) are populated automatically with the DIME binary data.  However, if the streaming DIME callbacks `::soap::fdimewriteopen`, `::soap::fdimewrite`, and `::soap::fdimewriteclose` are defined then the attachments are streamed to these callbacks instead.

For example, to send and receive a SOAP/XML message with DIME attachments using a serializable `::xsd__base64Binary` structure:

~~~{.cpp}
// example .h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
struct xsd__base64Binary {
  unsigned char *__ptr; // pointer to binary data
  int __size;           // size of the binary data
  char *id;             // NULL to generate an id or assign this member variable a unique UUID
  char *type;           // MIME type of the data
  char *options;        // DIME options
};
int ns__webmethod(struct xsd__base64Binary *data, struct xsd__base64Binary *result);
~~~

~~~{.cpp}
// example client implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  struct xsd__base64Binary data, result;
  data.__ptr = ...;  // points to binary image data to send
  data.__size = ...; // size of the image data to send
  data.id = NULL;    // this will be assigned by the engine
  data.type = "image/jpg";
  data.options = soap_dime_option(soap, 0, "Picture.png"); // DIME option 0 = "Picture.png" to store file name
  if (soap_call_ns__webmethod(soap, endpoint, NULL, &data, &result))
    soap_print_fault(soap, stderr);
  else
    ... // success, use the result
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
}
~~~

~~~{.cpp}
// example service implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int ns__webmethod(struct soap *soap, struct xsd__base64Binary *data, struct xsd__base64Binary *result)
{
  // echo back the structure (as a DIME attachment)
  result->__ptr = data->__ptr;
  result->__size = data->__size;
  retult->id = data->id;
  retult->type = data->type;
  retult->options = data->options;
  return SOAP_OK;
}
~~~

Besides receiving the attachments in `::xsd__base64Binary` structures, on the receiving side you can also iterate over the DIME attachments received as follows:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  ... // call soap_call_ns__webmethod etc.
  {
    int n = 0;
    struct soap_multipart *attachment;
    for (attachment = soap->dime.list; attachment; attachment = attachment->next)
    {
      ++n;
      printf("Part %d:\n", n);
      printf("ptr   =%p\n", attachment->ptr);
      printf("size  =%ul\n", attachment->size);
      printf("id    =%s\n", attachment->id ? attachment->id : "");
      printf("type  =%s\n", attachment->type ? attachment->type : "");
      // DIME options are formatted according to the DIME protocol
      if (attachment->options)
      {
        // extract length of first option
        size_t len = ((unsigned char)attachment->options[2] << 8) | ((unsigned char)attachment->options[3]);
        // allocate and copy the first option, which is assumed to be a name
        char *name = (char*)soap_malloc(soap, len + 1);
        strncpy(name, attachment->options + 4, len);
        name[len] = '\0';
        printf("option=%s\n", name);
      }
    }
  }
}
~~~

In C++ you can use an iterator for the last part of this example:

~~~{.cpp}
struct soap *soap = soap_new();
... // call soap_call_ns__webmethod etc.
int n = 0;
for (soap_multipart::iterator i = soap->dime.begin(); i != soap->dime.end(); ++i)
{
  ++n;
  printf("Part %d:\n", n);
  printf("ptr   =%p\n", i->ptr);
  ... // etc
}
~~~

At the server side the code to retrieve the DIME attachments is the same.

@{
*/

/// XSD base64Binary structure with attachment data
/**
@ingroup mime
This structure may be declared in the interface header file for soapcpp2 and defines a XSD base64Binary type with optional DIME/MIME/MTOM attachment data.  This structure is auto-generated by wsdl2h for the XSD base64Binary type to serialize binary data in base64 or as a DIME/MIME/MTOM attachment.  An attachment is serialized when either `id`, `type` or `options` member variables are non-NULL.

@par Example:

~~~{.cpp}
// example .h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
struct xsd__base64Binary {
  unsigned char *__ptr; // pointer to binary data
  int __size;           // size of the binary data
  char *id;             // NULL to generate an id or assign this member variable a unique UUID
  char *type;           // MIME type of the data
  char *options;        // DIME options or an optional description of the MIME attachment
};
int ns__webmethod(struct xsd__base64Binary *data, struct xsd__base64Binary *result);
~~~

~~~{.cpp}
// example client implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  struct xsd__base64Binary data, result;
  data.__ptr = ...;  // points to binary image data to send
  data.__size = ...; // size of the image data to send
  data.id = soap_strdup(soap, soap_rand_uuid(soap, NULL));
  data.type = "image/jpg";
  data.options = "A picture";
  if (soap_call_ns__webmethod(soap, endpoint, NULL, &data, &result))
    soap_print_fault(soap, stderr);
  else
    ... // success, use the result
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
}
~~~

~~~{.cpp}
// example service implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int ns__webmethod(struct soap *soap, struct xsd__base64Binary *data, struct xsd__base64Binary *result)
{
  // echo back the structure (as a MIME attachment)
  result->__ptr = data->__ptr;
  result->__size = data->__size;
  retult->id = data->id;
  retult->type = data->type;
  retult->options = data->options;
  return SOAP_OK;
}
~~~

@see `::xsd__hexBinary`, `::_xop__Include`.
*/
struct xsd__base64Binary {
  unsigned char *__ptr; ///< pointer to binary data
  int __size;           ///< size of the binary data
  char *id;             ///< extra member: NULL to generate an id or assign this member variable a unique UUID
  char *type;           ///< extra member: MIME type of the data
  char *options;        ///< extra member: DIME options or a description of the MIME attachment or NULL
};

/// XOP include structure with attachment data
/**
@ingroup mime
This structure may be declared in the interface header file for soapcpp2 and defines a XOP type with optional MTOM attachment data to support the XML-binary optimized packaging https://www.w3.org/TR/xop10 format.  This structure is pre-defined in <i>`gsoap/import/xop.h`</i> and is used to serialize binary data in base64 or as a DIME/MIME/MTOM attachment.  An attachment is serialized when either `id`, `type` or `options` member variables are non-NULL.

@par Example:

~~~{.cpp}
// example .h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
#import "import/xop.h"
int ns__webmethod(struct _xop__Include *data, struct _xop__Include *result);
~~~

~~~{.cpp}
// example service implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int ns__webmethod(struct soap *soap, struct _xop__Include *data, struct _xop__Include *result)
{
  // strip id, type and options to return base64 data in XML
  result->__ptr = data->__ptr;
  result->__size = data->__size;
  retult->id = NULL;
  retult->type = NULL;
  retult->options = NULL;
  return SOAP_OK;
}
~~~

@see `::xsd__base64Binary`, `::xsd__hexBinary`.
*/
struct _xop__Include {
  unsigned char *__ptr; ///< pointer to binary data
  int __size;           ///< size of the binary data
  char *id;             ///< NULL to generate an id or assign this member variable a unique UUID
  char *type;           ///< MIME type of the data
  char *options;        ///< description of the MIME/MTOM attachment or NULL
};

/// XSD hexBinary structure with attachment data
/**
@ingroup mime
This structure may be declared in the interface header file for soapcpp2 and defines a XSD hexBinary type with optional DIME/MIME/MTOM attachment data.  This structure is auto-generated by wsdl2h for the XSD hexBinary type to serialize binary data in hex or as a DIME/MIME/MTOM attachment.  An attachment is serialized when either `id`, `type` or `options` member variables are non-NULL.

@par Example:

~~~{.cpp}
// example .h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
struct xsd__hexBinary {
  unsigned char *__ptr; // pointer to binary data
  int __size;           // size of the binary data
  char *id;             // NULL to generate an id or assign this member variable a unique UUID
  char *type;           // MIME type of the data
  char *options;        // DIME options or an optional description of the MIME attachment
};
int ns__webmethod(struct xsd__hexBinary *data, struct xsd__hexBinary *result);
~~~

~~~{.cpp}
// example client implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  struct xsd__hexBinary data, result;
  data.__ptr = ...;  // points to binary image data to send
  data.__size = ...; // size of the image data to send
  data.id = soap_strdup(soap, soap_rand_uuid(soap, NULL));
  data.type = "image/jpg";
  data.options = "A picture";
  if (soap_call_ns__webmethod(soap, endpoint, NULL, &data, &result))
    soap_print_fault(soap, stderr);
  else
    ... // success, use the result
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
}
~~~

~~~{.cpp}
// example service implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int ns__webmethod(struct soap *soap, struct xsd__hexBinary *data, struct xsd__hexBinary *result)
{
  // echo back the structure (as a MIME attachment)
  result->__ptr = data->__ptr;
  result->__size = data->__size;
  retult->id = data->id;
  retult->type = data->type;
  retult->options = data->options;
  return SOAP_OK;
}
~~~

@see `::xsd__base64Binary`, `::_xop__Include`.
*/
struct xsd__hexBinary {
  unsigned char *__ptr; ///< pointer to binary data
  int __size;           ///< size of the binary data
  char *id;             ///< extra member: NULL to generate an id or assign this member variable a unique UUID
  char *type;           ///< extra member: MIME type of the data
  char *options;        ///< extra member: DIME options or a description of the MIME attachment or NULL
};

/// Stores a linked list of DIME attachments received
/**
This structure's `::soap_mime::list` member points to a linked list of DIME attachments received.  Other data in this structure (not shown) is used to manage the state of the engine's DIME processing.  The `begin` and `end` member functions return an iterator to iterate over attachments in C++.
*/
struct soap_dime {
  struct soap_multipart *list; ///< list of DIME attachments received

  soap_multipart::iterator begin(); ///< C++ only: an iterator over soap_multipart attachments
  soap_multipart::iterator end();   ///< C++ only: an iterator over soap_multipart attachments
};

/// Enable DIME attachments
/**
This function enables sending DIME attachments.  This function is generally not required because DIME attachments are automatically detected as `::xsd__base64Binary` and `::_xop__Include` structures in the data to serialize as an XML message with the attachments automatically added or DIME attachments can be explicitly added with `::soap_set_dime_attachment`.
*/
int soap_set_dime(struct soap *soap)
  ;

/// Disable DIME attachments
/**
This function disables DIME attachments, unless the data to serialize as an XML message contains attachments defined by `::xsd__base64Binary` and `::_xop__Include` structures.
*/
void soap_clr_dime(struct soap *soap)
  ;

/// Add a DIME attachment to the SOAP/XML message
/**
This function adds a DIME attachment to the XML message to send.  The specified `ptr` points to the data to send of length specified by `size`.  The `type` parameter indicates the MIME type of the data or can be NULL.  The `id` parameter uniquely identifies the attachment in the message, which can be omitted by specifying NULL.  The `option` parameter is an option such as a description of the data and `optype` is a user-defined option type (as per DIME option specification format).  The `ptr` parameter must be persistent.  The `ptr` parameter passed to this function must be persistent in memory until the attachment was sent.  Returns `#SOAP_OK` or a `::soap_status` error code.

When streaming DIME attachments are enabled by defining the `::soap::fdimereadopen`, `::soap::fdimeread`, `::soap::fdimereadclose` then the `ptr` parameter should point to a user-defined structure that is passed to `::soap::fdimereadopen` as the `handle` parameter.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK);
const char *data = ...; // points to data to send
size_t size = ...;      // length of the data
soap->connect_timeout = 30;                 // 30 seconds max connect stall time
soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
soap_set_dime_attachment(soap, data, size, "image/jpg", NULL, 0, "Picture");
if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
{
  soap_print_fault(soap, stderr);
  if (soap->errnum == 0) // timed out, exit program
    exit(EXIT_FAILURE);
}
else
{
  ... // success
}
soap_destroy(soap);
soap_end(soap);
soap_free(soap);
~~~

@see `::xsd__base64Binary`, `::_xop__Include`, `::soap_rand_uuid`, `::soap::fdimereadopen`, `::soap::fdimeread`, `::soap::fdimereadclose`.
*/
int soap_set_dime_attachment(
    struct soap *soap,     ///< `::soap` context
    const char *ptr,       ///< pointer to data
    size_t size,           ///< length of the data
    const char *type,      ///< MIME type of the data or NULL
    const char *id,        ///< content ID of the data or NULL
    unsigned short optype, ///< a 16 bit DIME option type
    const char *option)    ///< one DIME option as a text string or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Creates a DIME option
/**
This function creates a DIME option-formatted string for the `::xsd__base64Binary::options` member variable or `::_xop__Include::options` member variable.
*/
char * soap_dime_option(
    struct soap *soap,     ///< `::soap` context
    unsigned short optype, ///< a 16 bit DIME option type
    const char *option)    ///< one DIME option as a text string
  /// @returns a DIME option-formatted string
  ;

/** @} */

/**
\defgroup group_mime MIME attachment functions
@brief This module defines functions to set and get MIME/MTOM attachments

To enable MIME or MTOM, first initialize the `::soap` context with `#SOAP_ENC_MIME` (for MIME) or `#SOAP_ENC_MTOM` (for MTOM) using `::soap_new1`, `::soap_init1`, `::soap_set_mode`, or `::soap_set_mime` to enable MIME and also to set a start id.  To disable MIME use `::soap_clr_mime`.

There are two ways to add MIME/MTOM attachments to SOAP/XML messages for sending:
- use `::soap_set_mime_attachment` to explicitly add an attachment that contains the specified source of data;
- use `::xsd__base64Binary` (MIME and MTOM) or `::_xop__Include` (MTOM) structures in the serializable data of a SOAP/XML message, where the specified data is serialized in MIME/MTOM attachments automatically when one of the `id`, `type` or `options` member variables are non-NULL.

Both methods also support streaming MIME attachments to send using the `::soap::fmimereadopen`, `::soap::fmimeread`, and `::soap::fmimereadclose` callbacks that fetch the data to send, where a user-defined handle should be specified for the `ptr` parameter of `::soap_set_mime_attachment` or the `::xsd__base64Binary::__ptr` member variable instead of a pointer to the actual data.  This handle can be used by the callbacks to fetch the specific data to transmit.

Receiving MIME/MTOM attachments attached to SOAP/XML messages is automatic.  The MIME/MTOM attachments are converted to binary data and stored in the `::xsd__base64Binary` structures that reference the MIME/MTOM attachments via the `::xsd__base64Binary::id` string, meaning that the `::xsd__base64Binary::__ptr` (points to the data) and `::xsd__base64Binary::__size` (data length) are populated automatically with the MIME/MTOM binary data.  However, if the streaming MIME callbacks `::soap::fmimewriteopen`, `::soap::fmimewrite`, and `::soap::fmimewriteclose` are defined then the attachments are streamed to these callbacks instead.

For example, to send and receive a SOAP/XML message with MIME attachments using a serializable `::xsd__base64Binary` structure:

~~~{.cpp}
// example .h file for soapcpp2
//gsoap ns service name: example
//gsoap ns service namespace: urn:example
struct xsd__base64Binary {
  unsigned char *__ptr; // pointer to binary data
  int __size;           // size of the binary data
  char *id;             // NULL to generate an id or assign this member variable a unique UUID
  char *type;           // MIME type of the data
  char *options;        // DIME options or an optional description of the MIME attachment
};
int ns__webmethod(struct xsd__base64Binary *data, struct xsd__base64Binary *result);
~~~

~~~{.cpp}
// example client implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  struct xsd__base64Binary data, result;
  data.__ptr = ...;  // points to binary image data to send
  data.__size = ...; // size of the image data to send
  data.id = soap_strdup(soap, soap_rand_uuid(soap, NULL));
  data.type = "image/jpg";
  data.options = "A picture";
  if (soap_call_ns__webmethod(soap, endpoint, NULL, &data, &result))
    soap_print_fault(soap, stderr);
  else
    ... // success, use the result
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
}
~~~

~~~{.cpp}
// example service implementation based on the above example .h file for soapcpp2
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int ns__webmethod(struct soap *soap, struct xsd__base64Binary *data, struct xsd__base64Binary *result)
{
  // echo back the structure (as a MIME attachment)
  result->__ptr = data->__ptr;
  result->__size = data->__size;
  retult->id = data->id;
  retult->type = data->type;
  retult->options = data->options;
  return SOAP_OK;
}
~~~

To send MIME attachments without SOAP/XML (i.e. with REST methods), simply add attachments with `::soap_set_mime_attachment`.  The `::soap_end_send` function then adds the attachments before finalizing the message transmission.  To receive MIME attachments without SOAP/XML (i.e. with REST methods), simply call `::soap_begin_recv` (not needed at the server side since this is already called to parse the HTTP header) and `::soap_end_recv`.

For example a client-side multipart-related POST operation that sends a multipart-related message with one MIME attachment and receives a message consisting of multipart-related MIME attachments:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  const char *ptr = "<attached>Some attached text</attached>";
  size_t size = strlen(ptr);
  soap_set_mime(soap, NULL, "body");
  soap_set_mime_attachment(soap, ptr, size, SOAP_MIME_NONE, "text/xml", "attach1", NULL, NULL);
  if (soap_POST(soap, "http://", NULL, "text/xml")
   || soap_send(soap, "<doc title=\"Example\">Some text</doc>")
   || soap_end_send(soap)
   || soap_begin_recv(soap)
   || soap_end_recv(soap))
  {
    soap_print_fault(soap, stderr);
  }
  else
  {
    int n = 0;
    struct soap_multipart *attachment;
    for (attachment = soap->mime.list; attachment; attachment = attachment->next)
    {
      ++n;
      printf("Part %d:\n", n);
      printf("ptr        =%p\n", attachment->ptr);
      printf("size       =%ul\n", attachment->size);
      printf("id         =%s\n", attachment->id ? attachment->id : "");
      printf("type       =%s\n", attachment->type ? attachment->type : "");
      printf("location   =%s\n", attachment->location ? attachment->location : "");
      printf("description=%s\n", attachment->description ? attachment->description : "");
    }
  }
  soap_clr_mime(soap);
}
~~~

In C++ you can use an iterator for the last part of this example:

~~~{.cpp}
struct soap *soap = soap_new();
... // call soap_POST etc
int n = 0;
for (soap_multipart::iterator i = soap->mime.begin(); i != soap->mime.end(); ++i)
{
  ++n;
  printf("Part %d:\n", n);
  printf("ptr        =%p\n", i->ptr);
  ... // etc
}
~~~

At the server side the code to retrieve the REST message sent consisting of a set of multipart-related MIME attachments is the same.  To respond with multipart-related MIME attachments use `::soap_set_mime` and use `::soap_set_mime_attachment` to add attachments.  For example:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->fget = my_get; // HTTP GET handler to serve HTTP GET
  ... // serve requests with soap_bind, soap_accept, soap_ssl_accept, and soap_serve
}
int my_get(struct soap *soap)
{
  const char *ptr = "<attached>Some attached text</attached>";
  size_t size = strlen(ptr);
  soap_set_mime(soap, NULL, "body");
  soap_set_mime_attachment(soap, ptr, size, SOAP_MIME_NONE, "text/xml", "attach1", NULL, NULL);
  soap->http_content = "text/xml";
  if (soap_response(soap, SOAP_FILE)
   || soap_send(soap, "<doc title=\"Example\">Some text</doc>\n")
   || soap_end_send(soap))
    return soap_closesock(soap);
  soap_clr_mime(soap);
  return SOAP_OK;
}
~~~

To disable MIME again, use `::soap_clr_mime`.

@{
*/

/// RFC2045 MIME content transfer encodings
/**
These values are used by the `::soap_set_mime_attachment` function parameter `encoding`.

@warning You are responsible to ensure that the MIME/MTOM content conforms to the encoding specified.  We recommend `#SOAP_MIME_NONE` to transmit any data of any type which is usually the purpose of MIME/MTOM attachments to SOAP/XML.

@see `::soap_set_mime_attachment`.
*/
enum soap_mime_encoding
{
  SOAP_MIME_NONE,             ///< no encoding, raw data content (recommended)
  SOAP_MIME_7BIT,             ///< 7 bit data content
  SOAP_MIME_8BIT,             ///< 8 bit data content
  SOAP_MIME_BINARY,           ///< binary raw data content
  SOAP_MIME_QUOTED_PRINTABLE, ///< data is formatted as quoted printable
  SOAP_MIME_BASE64,           ///< data is formatted in base64
  SOAP_MIME_IETF_TOKEN,       ///< data is an IETF token
  SOAP_MIME_X_TOKEN           ///< data is an X-token
};

/// Add a MIME attachment to the SOAP/XML message
/**
This function adds a MIME attachment to a SOAP/XML message to send.  The specified `ptr` points to the data to send of length specified by `size`.  The `encoding` parameter is a `::soap_mime_encoding` value that is recommended to be specified as `#SOAP_MIME_NONE` to specify that the MIME data content is not encoded in any way (the MIME attachment function simply copies the raw data to the MIME block without encoding).  The `type` parameter is required and indicates the MIME type of the data, such as "image/jpg".  The `id` parameter uniquely identifies the attachment in the message, which can be omitted by specifying NULL.  The `location` parameter specifies a location string or NULL.  The `description` parameter is a string that describes the data or NULL.  Returns `#SOAP_OK` or a `::soap_status` error code.

There are two ways to add MIME/MTOM attachments to SOAP/XML:
- use `::soap_set_mime_attachment` to explicitly add an attachment that contains the specified source of data;
- use `::xsd__base64Binary` or `_xop__Include` structures in the serializable data of a SOAP/XML message, where the specified data is serialized in MIME/MTOM attachments automatically when one of the `id`, `type` or `options` member variables are non-NULL.  This option requires `#SOAP_ENC_MIME` or `#SOAP_ENC_MTOM`.

Both methods support streaming MIME/MTOM attachments, where a user-defined handle instead of the actual data is specified for the `ptr` parameter or the `__ptr` member variable.

@par Example:

~~~{.cpp}
#include "soapH.h"

struct soap *soap = soap_new1(SOAP_IO_CHUNK);
const char *data = ...; // points to data to send
size_t size = ...;      // length of the data
soap->connect_timeout = 30;                 // 30 seconds max connect stall time
soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
soap_set_mime_attachment(soap, data, size, SOAP_MIME_NONE, "image/jpg", NULL, NULL, "Picture");
if (soap_call_ns__webmethod(soap, endpoint, NULL, ...))
{
  soap_print_fault(soap, stderr);
  if (soap->errnum == 0) // timed out, exit program
    exit(EXIT_FAILURE);
}
else
{
  ... // success
}
soap_clr_mime(soap);
soap_destroy(soap);
soap_end(soap);
soap_free(soap);
~~~

@see `::xsd__base64Binary`, `::_xop__Include`, `::soap_rand_uuid`, `::soap::fmimereadopen`, `::soap::fmimeread`, `::soap::fmimereadclose`.
*/
int soap_set_mime_attachment(
    struct soap *soap,                ///< `::soap` context
    const char *ptr,                  ///< pointer to data
    size_t size,                      ///< length of the data
    enum soap_mime_encoding encoding, ///< encoding of the data
    const char *type,                 ///< MIME type of the data
    const char *id,                   ///< content ID of the data or NULL
    const char *location,             ///< location of the data or NULL
    const char *description)          ///< description of the data or NULL
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Enable MIME attachments
/**
This function enables sending MIME attachments.  This function is generally not required when the context is initialized with `#SOAP_ENC_MIME`, because MIME attachments are automatically detected as `::xsd__base64Binary` and `::_xop__Include` structures in the data to serialize as an XML message with the attachments automatically added or MIME attachments can be explicitly added with `::soap_set_mime_attachment`. Parameter `boundary` specifies a MIME boundary string or NULL to have the engine generate a MIME boundary string.  Parameter `start` specifiesthe start content ID for the first MIME body containing the SOAP or XML message.  When NULL, the start ID of the SOAP message is <i>`<SOAP-ENV:Envelope>`</i>.

@see `::soap_set_mime_attachment`, `::soap_clr_mime`, `#SOAP_ENC_MIME`, `#SOAP_ENC_MTOM`.
*/
int soap_set_mime(
    struct soap *soap,    ///< `::soap` context
    const char *boundary, ///< MIME boundary string to use or NULL to generate a random boundary
    const char *start)    ///< string id of the first MIME attachment with the SOAP/XML message or NULL
  ;

/// Disable MIME attachments
/**
This function disables MIME attachments such as after sending a multipart-related message with attachments to switch back to non-multipart-related messaging, unless the data to serialize as a message contains attachments such as `::xsd__base64Binary` for MIME attachments and `::_xop__Include` for MTOM attachments.

@see `::soap_set_mime`, `::soap_set_mime_attachment`, `#SOAP_ENC_MIME`, `#SOAP_ENC_MTOM`.
*/
void soap_clr_mime(struct soap *soap) ///< `::soap` context
  ;

/// Enable post-processing of MIME/MTOM attachments
/**
This function enables post-processing of MTOM/MIME attachments attached to a message and is useful when MIME/MTOM are streamed (asynchronously) by configuring the callbacks `::fmimewriteopen`, `::soap::fmimewrite`, and `::soap::fmimewriteclose` to write the attachment to memory, file or, other resources.  By calling this function, the presence of MIME/MTOM attachments must be explicitly checked after each message is received by calling `::soap_check_mime_attachments`.  When this function returns nonzero (true), `::soap_recv_mime_attachment` must be called repeatedly to retrieve each attachment until this function returns NULL indicating the end of attachments and the channel is closed, or if an error occurred with `::soap::error` set to a nonzero `::soap_status` error code.

If attachments are not referenced by the SOAP/XML message received, then normallly an error will be produced to indicate that attachments exist that were not converted into binary `::xsd__base64Binary` or `::_xop__Include` structures (i.e. deserialized from the message by resolving references to MIME/MTOM attachments).  This error can be avoided by using `::soap_post_check_mime_attachments` to indicate that attachments may appear that cannot be automatically resolved and should be handled explicitly by calling `::soap_check_mime_attachments` and `::soap_recv_mime_attachment`.

@par Examples:

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap_post_check_mime_attachments(soap);
  if (soap_call_ns__webmethod(soap, ...))
  {
    soap_print_fault(soap, stderr); // an error occurred
  }
  else
  {
    if (soap_check_mime_attachments(soap))
    {
      // attachments are present, channel is still open
      int n = 0;
      do
      {
        struct soap_multipart *attachment = soap_recv_mime_attachment(soap, NULL);
        ++n;
        printf("Part %d:\n", n);
        printf("ptr        =%p\n", attachment->ptr);
        printf("size       =%ul\n", attachment->size);
        printf("id         =%s\n", attachment->id ? attachment->id : "");
        printf("type       =%s\n", attachment->type ? attachment->type : "");
        printf("location   =%s\n", attachment->location ? attachment->location : "");
        printf("description=%s\n", attachment->description ? attachment->description : "");
      } while (attachment);
      if (soap->error)
        soap_print_fault(soap, stderr);
    }
  }
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
}
~~~

~~~{.cpp}
#include "soapH.h"

int main()
{
  struct soap *soap = soap_new();
  soap->fmimewriteopen = mime_write_open;
  soap->fmimewrite = mime_write;
  soap->fmimewriteclose = mime_write_close;
  soap_post_check_mime_attachments(soap);
  if (soap_call_ns__webmethod(soap, ...))
  {
    soap_print_fault(soap, stderr); // an error occurred
  }
  else
  {
    if (soap_check_mime_attachments(soap))
    {
      // attachments are present, channel is still open
      int n = 0;
      do
      {
        handle = ...; // a 'handle' to pass to the MIME/MTOM callbacks that process the attachment content
        ++n;
        printf("Part %d:\n", n);
        printf("ptr        =%p\n", attachment->ptr);
        printf("size       =%ul\n", attachment->size);
        printf("id         =%s\n", attachment->id ? attachment->id : "");
        printf("type       =%s\n", attachment->type ? attachment->type : "");
        printf("location   =%s\n", attachment->location ? attachment->location : "");
        printf("description=%s\n", attachment->description ? attachment->description : "");
      } while (attachment);
      if (soap->error)
        soap_print_fault(soap, stderr);
    }
  }
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
}
~~~

@warning When `::soap_post_check_mime_attachments` is used, every message received must be followed by a call to `::soap_check_mime_attachments`.  When this function returns nonzero, `::soap_check_mime_attachments` must be called repeatedly until this function returns NULL.  This sequence of calls is necessary to properly handle messages with and without attachments.  The connection is closed if HTTP keep-alive is not enabled.  With HTTP keep-alive enabled, this sequence of calls allows the next message to be properly received.

@see `::soap_check_mime_attachments`, `::soap_recv_mime_attachment`, `::soap::fmimewriteopen`, `::soap::fmimewrite`, `::soap::fmimewriteclose`.
*/
int soap_post_check_mime_attachments(struct soap *soap) ///< `::soap` context
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Check for a MIME/MTOM attachment
/**
This function checks the presence of a MIME/MTOM attachment after calling a service operation by returning nonzero when attachments are present.  Returns nonzero if attachments are present.  Requires `::soap_post_check_mime_attachments`.

@see `::soap_post_check_mime_attachments`.
*/
int soap_check_mime_attachments(struct soap *soap) ///< `::soap` context
  /// @returns nonzero if MIME/MTOM attachments are present
  ;

/// Stores a linked list of MIME attachments received
/**
This structure's `::soap_mime::list` member points to a linked list of MIME attachments received.  Other data in this structure (not shown) is used to manage the state of the engine's MIME processing.  The `begin` and `end` member functions return an iterator to iterate over attachments in C++.
*/
struct soap_mime {
  struct soap_multipart *list; ///< list of MIME attachments received

  soap_multipart::iterator begin(); ///< C++ only: an iterator over soap_multipart attachments
  soap_multipart::iterator end();   ///< C++ only: an iterator over soap_multipart attachments
};

/// DIME/MIME/MTOM attachment data received by the engine
/**
@see `::soap_post_check_mime_attachments`, `::soap_check_mime_attachments`, `::soap_recv_mime_attachment`.
*/
struct soap_multipart {
  struct soap_multipart *next;      ///< next attachment in the linked list
  const char *ptr;                  ///< points to raw data content
  size_t size;                      ///< size of data content
  const char *id;                   ///< DIME/MIME/MTOM content ID or form data name
  const char *type;                 ///< DIME/MIME/MTOM type (MIME type format)
  const char *options;              ///< DIME options
  enum soap_mime_encoding encoding; ///< MIME Content-Transfer-Encoding
  const char *location;             ///< MIME Content-Location (optional)
  const char *description;          ///< MIME Content-Description (optional)

  typedef soap_multipart_iterator iterator; ///< C++ only: an iterator over soap_multipart attachments
};

/// Get a MIME/MTOM attachment
/**
This function parses an attachment and invokes the MIME callbacks when set.  The `handle` parameter is passed to `fmimewriteopen`.  The handle may contain any data that is extracted from the SOAP message body to guide the redirection of the stream in the callbacks.  Returns a struct with a `char *ptr` member that contains the handle value returned by the `fmimewriteopen` callback, and `char *id`, `char *type`, and `char *description` member variables with the MIME id, type, and description info when present in the attachment.

@see `::soap_post_check_mime_attachments`, `::soap_check_mime_attachments`.
*/
struct soap_multipart * soap_recv_mime_attachment(
    struct soap *soap, ///< `::soap` context
    void *handle)      ///< a handle to pass to the callbacks
  /// @returns MIME attachment data or NULL if no more attachments were found
  ;

/** @} */

/**
\defgroup group_plugin Plugins and plugin registry functions
@brief This module defines plugin registry functions to register plugins

Available plugins:
- `::logging` plugin
- `::http_get` HTTP GET plugin
- `::http_post` HTTP POST plugin
- [HTTP digest authentication plugin](../../httpda/html/httpda.html)
- [HTTP sessions plugin](../../sessions/html/index.html)
- [CURL plugin](../../curl/html/index.html)
- [WinInet plugin](../../wininet/html/index.html)
- [WS-Security plugin](../../wsse/html/wsse.html)
- [WS-Addressing plugin](../../wsa/html/wsa_0.html)
- [WS-ReliableMessaging plugin](../../wsrm/html/wsrm_0.html)
- [WS-Discovery plugin](../../wsdd/html/wsdd_0.html)

Modules and extensions:
- [ISAPI extension](../../isapi/html/index.html)
- [Apache module](../../apache/html/index.html)

Threads and mutex:
- \ref group_threads

Other:
- [XML DOM](../../dom/html/index.html)
- [XML-RPC and JSON](../../xml-rpc-json/html/index.html)

@{
*/

/// Register a plugin
/**
This function registers the specified plugin with the specified engine's context.  The `fcreate` parameter is defined by the plugin library as a plugin registration function that when called initializes the plugin state.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_PLUGIN_ERROR`.

@see `::soap_register_plugin_arg`, `::http_get`, `::http_post`, `::logging`.
*/
int soap_register_plugin(
    struct soap *soap,                                        ///< `::soap` context
    int (*fcreate)(struct soap*, struct soap_plugin*, void*)) ///< plugin registration function
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/// Register a plugin with an argument
/**
This function registers the specified plugin with the specified engine's context.  The `fcreate` parameter is defined by the plugin library as a plugin registration function that when called initializes the plugin state.  The argument `arg` is passed to the plugin registration function.  Returns `#SOAP_OK` or a `::soap_status` error code such as `#SOAP_PLUGIN_ERROR`.

@see `::soap_register_plugin`, `::http_get`, `::http_post`, `::logging`.
*/
int soap_register_plugin_arg(
    struct soap *soap,                                        ///< `::soap` context
    int (*fcreate)(struct soap*, struct soap_plugin*, void*), ///< plugin registration function
    void *arg)                                                ///< argument passed to the plugin registration function
  /// @returns `#SOAP_OK` or a `::soap_status` error code
  ;

/** @} */

/**
\defgroup group_threads Thread and mutex functions
@brief This module defines portable thread and mutex functions

- `#THREAD_TYPE`
- `#THREAD_ID`
- `#THREAD_CREATE(tidptr, funcptr, argptr)`
- `#THREAD_CREATEX(tidptr, funcptr, argptr)` (for Windows)
- `#THREAD_CLOSE(tid)` (for Windows)
- `#THREAD_DETACH(tid)`
- `#THREAD_JOIN(tid)`
- `#THREAD_EXIT`
- `#THREAD_CANCEL(tid)`
- `#MUTEX_TYPE`
- `#MUTEX_INITIALIZER`
- `#MUTEX_SETUP(mx)`
- `#MUTEX_CLEANUP(mx)`
- `#MUTEX_LOCK(mx)`
- `#MUTEX_UNLOCK(mx)`
- `#COND_TYPE`
- `#COND_SETUP(cv)`
- `#COND_CLEANUP(cv)`
- `#COND_SIGNAL(cv)`
- `#COND_WAIT(mx, cv)`

@{
*/

/// Type of a thread (thread ID)
/**
This macro represents a portable thread ID type.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@par Example:

~~~{.cpp}
THREAD_TYPE tid;
while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap)))
  sleep(1); // failed, try again
~~~
*/
#define THREAD_TYPE

/// The thread ID of self
/**
This macro represents the current thread ID, i.e. self.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.
*/
#define THREAD_ID

/// Create a new thread
/**
This macro creates a new thread and runs the specified function with the argument parameter passed to this function.

@warning On Windows platforms `#THREAD_CREATE` uses `_beginthread` which returns a thread ID handle that cannot be joined.  To join a thread use `THREAD_CREATEX(&tid, func, arg)` then `#THREAD_JOIN`.  Don't forget to `THREAD_CLOSE(tid)` afterwards:
~~~{.cpp}
#include "plugin/threads.h"

THREAD_TYPE tid;
while (THREAD_CREATEX(&tid, (void*(*)(void*))&process_request, (void*)tsoap)))
  sleep(1); // failed, try again
... // some other work to do
THREAD_JOIN(tid); // wait for the thread to terminate and join
THREAD_CLOSE(tid); // close handle 
... // some other work to do
~~~

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@par Example:

~~~{.cpp}
#include "plugin/threads.h"

THREAD_TYPE tid;
while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap)))
  sleep(1); // failed, try again
... // some other work to do
THREAD_JOIN(tid); // optional: to wait for the thread to terminate and join (see warning)
... // some other work to do
~~~

@param tidptr pointer to `#THREAD_TYPE` thread ID to assign
@param funcptr function to run by the new thread
@param argptr the argument (a pointer) to pass to the function when called
@returns zero on success and nonzero on failure
*/
#define THREAD_CREATE(tidptr, funcptr, argptr)

/// Create a new joinable thread (Windows only)
/**
On Windows platforms `#THREAD_CREATE` uses `_beginthread` which returns a thread ID handle that cannot be joined.  To join a thread use `THREAD_CREATEX(&tid, func, arg)` then `#THREAD_JOIN`.  Don't forget to `THREAD_CLOSE(tid)` afterwards.
*/
#define THREAD_CREATEX(tidptr, funcptr, argptr)

/// Close the thread ID handle created by `#THREAD_CREATEX` (Windows only)
#define THREAD_CLOSE(tid)

/// Detach a thread
/**
This macro detaches the specified thread.  A detached thread cannot be joined back with the thread that created it.  When a detached thread terminates, its resources are automatically released back to the system without the need for another thread to join with the terminated thread.

This macro has no effect on Windows platforms, see `#THREAD_CREATE`.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@par Example:

~~~{.cpp}
#include "plugin/threads.h"

THREAD_DETACH(THREAD_ID); // detach self
~~~

@param tid `#THREAD_TYPE` thread ID to detach
*/
#define THREAD_DETACH(tid)

/// Join a thread
/**
This macro waits for the termination of the specified thread to join it.

This macro requires `#THREAD_CREATEX` and `#THREAD_CLOSE` on Windows plaforms.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@param tid `#THREAD_TYPE` thread ID to join
*/
#define THREAD_JOIN(tid)

/// Exit the current thread
/**
This macro terminates the current thread.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.
*/
#define THREAD_EXIT

/// Cancel a thread
/**
This macro requests that the specified thread be cancelled.

POSIX threads can be cancelled when currently in a cancellation point, which are certain functions that will terminate the thread when the thread is cancelled.

@warning Cancelling a thread may lead to resource leaks when the thread has no mechanism to clean up its state (memory allocated, files and sockets opened etc.), unless cleanup handlers are defined for each thread, e.g. with `pthread_cleanup_push`.  Even when defining a cleanup function, care must be taken to prevent resource leaks that may be caused by cancellation points that sit between a resouce acquisition operation and its release operation, e.g. between a file open and close operation some read/write functions may be called that are cancellation points.  The gSOAP engine and plugins are designed to maintain the engine state using resource pointers to resources (memory, files, sockets etc.) in the `::soap` context.  The context should be passed to the cleanup function when added with `pthread_cleanup_push` to cleanup the context.  This cleanup function should call `::soap_destroy`, `::soap_end`, and `::soap_free` (in that order).  However, use `#THREAD_CANCEL` at your own risk.  User-defined service operations and other non-gSOAP code may not meet the requirements to perform a safe `#THREAD_CANCEL` unless the cleanup functions are carefully designed.  Alternatively, a simpler approach with a global flag (a flag per thread) may suffice: set the flag by the main thread to indicate termination is requested of the specific thread and check this flag in the user-defined service operations to terminate the service operation with an error, e.g. `return 500`.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@par Example:

~~~{.cpp}
#include "soapH.h"
#include "plugin/threads.h"

struct soap *soap = soap_new();
soap->bind_flags = SO_REUSEADDR;            // immediate port reuse
soap->accept_timeout = 3600;                // exit loop when no request arrives in one hour
soap->send_timeout = soap_recv_timeout = 5; // 5 seconds max socket stall time (unlimited by default)
soap->transfer_timeout = 30;                // 30 seconds max message transfer time (unlimited by default)
soap->recv_maxlength = 1048576;             // limit messages received to 1MB (2GB by default)
if (soap_valid_socket(soap_bind(soap, NULL, PORTNUM, BACKLOG)))
{
  while (1)
  {
    if (soap_valid_socket(soap_accept(soap)))
    {
      THREAD_TYPE tid;
      struct soap *tsoap = soap_copy(soap);
      if (!tsoap)
        soap_force_closesock(soap);
      else
        while (THREAD_CREATE(&tid, (void*(*)(void*))&accept_request, (void*)tsoap))
          sleep(1); // failed, try again
    }
    else if (soap->errnum) // accept failed, try again after 1 second
    {
      soap_print_fault(soap, stderr);
      sleep(1);
    }
    else // accept_timeout timed out, quit looping
    {
      break;
    }
    soap_destroy(soap);
    soap_end(soap);
  }
}
soap_free(soap);

void *accept_request(struct soap *soap)
{
  THREAD_TYPE tid;
  struct soap *tsoap;
  THREAD_DETACH(THREAD_ID);
  // create a new thread that is timed to execute for max 10 seconds
  tsoap = soap_copy(soap);
  if (!tsoap)
  {
    soap_force_closesock(soap);
  }
  else
  {
    while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap))
      sleep(1); // failed, try again
    // allow serving the request by the new thread for up to 30 seconds max
    sleep(30);
    // terminate process_request thread
    THREAD_CANCEL(tid);
  }
  // clean up
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}

void *process_request(struct soap *soap)
{
  // add the cleanup function
  pthread_cleanup_push((void(*)(void*))&cleanup, (void*)soap);
  soap_serve(soap);
  // remove the cleanup function and call it to cleanup the context
  pthread_cleanup_pop(1);
  return NULL;
}

void cleanup(struct soap *soap)
{
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
}
~~~
@param tid `#THREAD_TYPE` thread ID to cancel
*/
#define THREAD_CANCEL(tid)

/// Type of a mutex object
/**
This macro represents a portable mutex object type.

To declare and initialize static mutex objects, see `#MUTEX_INITIALIZER`.

@par Example:

~~~{.cpp}
#include "plugin/threads.h"

MUTEX_TYPE lock;
MUTEX_SETUP(lock);
... // some other work to do
MUTEX_LOCK(lock);
... // critical section
MUTEX_UNLOCK(lock);
... // some other work to do
MUTEX_CLEANUP(lock);
~~~

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.
*/
#define MUTEX_TYPE

/// Mutex initializer object
/**
@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@par Example:

~~~{.cpp}
#include "plugin/threads.h"

static MUTEX_TYPE lock = MUTEX_INITIALIZER;
int main()
{
  ... // some other work to do
  MUTEX_LOCK(lock);
  ... // critical section
  MUTEX_UNLOCK(lock);
  ...
}
~~~
*/
#define MUTEX_INITIALIZER

/// Mutex object initialization
/**
This macro initializes a mutex object.

To declare and initialize static mutex objects, see `#MUTEX_INITIALIZER`.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@par Example:

~~~{.cpp}
#include "plugin/threads.h"

MUTEX_TYPE lock;
MUTEX_SETUP(lock);
... // some other work to do
MUTEX_LOCK(lock);
... // critical section
MUTEX_UNLOCK(lock);
... // some other work to do
MUTEX_CLEANUP(lock);
~~~

@param mx `#MUTEX_TYPE` mutex object
*/
#define MUTEX_SETUP(mx)

/// Mutex object finalization
/**
This macro finalizes a mutex object.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@par Example:

~~~{.cpp}
#include "plugin/threads.h"

MUTEX_TYPE lock;
MUTEX_SETUP(lock);
... // some other work to do
MUTEX_LOCK(lock);
... // critical section
MUTEX_UNLOCK(lock);
... // some other work to do
MUTEX_CLEANUP(lock);
~~~

@param mx `#MUTEX_TYPE` mutex object
*/
#define MUTEX_CLEANUP(mx)

/// Mutex object lock
/**
This macro acquires mutex.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@par Example:

~~~{.cpp}
#include "plugin/threads.h"

MUTEX_TYPE lock;
MUTEX_SETUP(lock);
... // some other work to do
MUTEX_LOCK(lock);
... // critical section
MUTEX_UNLOCK(lock);
... // some other work to do
MUTEX_CLEANUP(lock);
~~~

@param mx `#MUTEX_TYPE` mutex object
*/
#define MUTEX_LOCK(mx)

/// Mutex object unlock
/**
This macro releases mutex.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@par Example:

~~~{.cpp}
#include "plugin/threads.h"

MUTEX_TYPE lock;
MUTEX_SETUP(lock);
... // some other work to do
MUTEX_LOCK(lock);
... // critical section
MUTEX_UNLOCK(lock);
... // some other work to do
MUTEX_CLEANUP(lock);
~~~

@param mx `#MUTEX_TYPE` mutex object
*/
#define MUTEX_UNLOCK(mx)

/// The type of a condition variable
/**
This macro represents a portable condition variable

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.
*/
#define COND_TYPE

/// Condition variable initialization
/**
This macro initializes the specified condition variable.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@param cv `#COND_TYPE` condition variable
*/
#define COND_SETUP(cv)

/// Condition variable finalization
/**
This macro finalizes the specified condition variable.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@param cv `#COND_TYPE` condition variable
*/
#define COND_CLEANUP(cv)

/// Condition variable signal operation
/**
This macro signals the specified condition variable.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@param cv `#COND_TYPE` condition variable
*/
#define COND_SIGNAL(cv)

/// Condition variable wait operation
/**
This macro waits on the specified condition variable and releases the mutex while waiting.

@note This macro is declared in <i>`gsoap/plugin/threads.h`</i> and requires compilation of <i>`gsoap/plugin/threads.c`</i> on Windows platforms.

@param mx `#MUTEX_TYPE` mutex object
@param cv `#COND_TYPE` condition variable
*/
#define COND_WAIT(mx, cv)

/** @} */

/**
\defgroup group_misc Miscellaneous functions
@brief This module defines other useful functions

@{
*/

/// Reset context
/**
This function resets the context to start serialization with the `serialize` functions.  Alternatively, `::soap_begin_send` can be used before calling the `serialize` functions, but this is sometimes not possible so this function can be used instead.

@par Example:

~~~{.cpp}
struct soap *soap = soap_new1(SOAP_XML_GRAPH);
... // further initializations
struct ns__someElement elt;
... // populate elt
soap_begin(soap); // reset context before calling serialize functions
soap_serialize_ns__someElement(soap, &elt); // analyze pointers for multi-ref serialization
~~~

@see `::soap_begin_send`.
*/
void soap_begin(struct soap *soap) ///< `::soap` context
  ;

/// Returns a randomized unique UUID string
/**
This function returns a randomized unique UUID string stored in a temporary buffer.  The UUID string starts with the specified prefix if non-NULL.

@par Example:

~~~{.cpp}
struct soap *soap = soap_new();
const char *uuid = soap_strdup(soap, soap_rand_uuid(soap, NULL));
...
~~~
*/
const char * soap_rand_uuid(
    struct soap *soap,  ///< `::soap` context
    const char *prefix) ///< prefix string or NULL
  /// @returns randomized UUID string stored in a temporary buffer
  ;

/// Compare string to a pattern
/**
This function returns zero when the specified string matches the given pattern, nonzero otherwise.  A pattern consists of two types of wildcard meta-characters: `*` matches any text of any length and `-` matches any single character.  This function is commonly used to match XML tags and XML namespace URIs, such as the URI pattern in the third column of the `::namespaces` table.
*/
int soap_tag_cmp(
    const char *string,  ///< string to match
    const char *pattern) ///< pattern to match
  /// @returns zero (match) or nonzero (no match)
  ;

/// Match an XML tag name
/**
This function returns `#SOAP_OK` when the two specified XML tag names match.  The first tag name is a qualified or unqualified parsed tag name when the parser is pulling XML tags.  The second tag name is an unqualified or namespace-normalized qualified tag name or pattern to match.  A namespace-normalized qualified tag name uses a prefix defined in the `::namespaces` table.  A pattern consists of two types of wildcard meta-characters: `*` matches any text of any length and `-` matches any single character.  Returns `#SOAP_OK` when the tags are matching or `#SOAP_TAG_MISMATCH` when the tags are non-matching.

@warning This function should only be used while the XML parser is actively parsing input, for example in the `::soap::fignore` callback.
*/
int soap_match_tag(
    struct soap *soap, ///< `::soap` context
    const char *tag1,  ///< parsed (un)qualified tag name string to match
    const char *tag2)  ///< (namespace-normalized qualified) tag name string or pattern to match
  /// @returns `#SOAP_OK` (match) or `#SOAP_TAG_MISMATCH` (no match)
  ;

/** @} */

