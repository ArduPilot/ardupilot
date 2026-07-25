/*
        stdsoap2.c[pp] 2.8.114

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

#define GSOAP_LIB_VERSION 208114

#ifdef AS400
# pragma convert(819)   /* EBCDIC to ASCII */
#endif

#if defined(__gnu_linux__) && !defined(_GNU_SOURCE)
# define _GNU_SOURCE 1
#endif

#include "stdsoap2.h"

#if GSOAP_VERSION != GSOAP_LIB_VERSION
# error "GSOAP VERSION MISMATCH IN LIBRARY: PLEASE REINSTALL PACKAGE"
#endif

#if defined(VXWORKS) && defined(WM_SECURE_KEY_STORAGE)
# include <ipcom_key_db.h> /* vxWorks compatible */
#endif

#ifdef __BORLANDC__
# pragma warn -8060
#else
# ifdef WIN32
#  ifdef UNDER_CE
#   pragma comment(lib, "ws2.lib")      /* WinCE */
#  else
#   pragma comment(lib, "Ws2_32.lib")
#  endif
#  pragma warning(disable : 4996) /* disable deprecation warnings */
# endif
#endif

#ifdef __cplusplus
SOAP_SOURCE_STAMP("@(#) stdsoap2.cpp ver 2.8.114 2021-04-20 00:00:00 GMT")
extern "C" {
#else
SOAP_SOURCE_STAMP("@(#) stdsoap2.c ver 2.8.114 2021-04-20 00:00:00 GMT")
#endif

/* 8bit character representing unknown character entity or multibyte data */
#ifndef SOAP_UNKNOWN_CHAR
# define SOAP_UNKNOWN_CHAR (0x7F)
#endif

/* unicode character representing unknown characters outside the XML 1.0 UTF8 unicode space */
#ifdef WITH_REPLACE_ILLEGAL_UTF8
# ifndef SOAP_UNKNOWN_UNICODE_CHAR
#  define SOAP_UNKNOWN_UNICODE_CHAR (0xFFFD)
# endif
#endif

/*      EOF=-1 */
#define SOAP_LT (soap_wchar)(-2) /* XML-specific '<' */
#define SOAP_TT (soap_wchar)(-3) /* XML-specific '</' */
#define SOAP_GT (soap_wchar)(-4) /* XML-specific '>' */
#define SOAP_QT (soap_wchar)(-5) /* XML-specific '"' */
#define SOAP_AP (soap_wchar)(-6) /* XML-specific ''' */

#define soap_coblank(c)         ((c)+1 > 0 && (c) <= 32)

#if defined(WIN32) && !defined(UNDER_CE)
#define soap_hash_ptr(p)        ((size_t)((PtrToUlong(p) >> 3) & (SOAP_PTRHASH - 1)))
#else
#define soap_hash_ptr(p)        ((size_t)(((unsigned long)(p) >> 3) & (SOAP_PTRHASH-1)))
#endif

#ifdef SOAP_DEBUG
static void soap_init_logs(struct soap*);
static void soap_close_logfile(struct soap*, int);
static void soap_set_logfile(struct soap*, int, const char*);
#endif

#ifdef SOAP_MEM_DEBUG
static void soap_init_mht(struct soap*);
static void soap_free_mht(struct soap*);
static void soap_track_unlink(struct soap*, const void*);
#endif

static int soap_set_error(struct soap*, const char*, const char*, const char*, const char*, int);
static int soap_copy_fault(struct soap*, const char*, const char*, const char*, const char*);
static int soap_getattrval(struct soap*, char*, size_t*, soap_wchar);
static void soap_version(struct soap*);
static void soap_free_ns(struct soap*);
static soap_wchar soap_char(struct soap*);
static soap_wchar soap_getpi(struct soap*);
static int soap_isxdigit(int);
static void *fplugin(struct soap*, const char*);
static ULONG64 soap_count_attachments(struct soap*);
static int soap_try_connect_command(struct soap*, int http_command, const char *endpoint, const char *action);
static int soap_init_send(struct soap*);

#ifdef WITH_NTLM
static int soap_ntlm_handshake(struct soap *soap, int command, const char *endpoint, const char *host, int port);
#endif

#ifndef WITH_NOIDREF
static int soap_has_copies(struct soap*, const char*, const char*);
static int soap_type_punned(struct soap*, const struct soap_ilist*);
static int soap_is_shaky(struct soap*, void*);
static void soap_init_iht(struct soap*);
static void soap_free_iht(struct soap*);
#endif
static void soap_init_pht(struct soap*);
static void soap_free_pht(struct soap*);

#ifndef WITH_LEAN
static const char *soap_set_validation_fault(struct soap*, const char*, const char*);
static int soap_isnumeric(struct soap*, const char*);
static struct soap_nlist *soap_push_ns(struct soap *soap, const char *id, const char *ns, short utilized, short isearly);
static void soap_utilize_ns(struct soap *soap, const char *tag, short isearly);
static const wchar_t* soap_wstring(struct soap *soap, const char *s, int flag, long minlen, long maxlen, const char *pattern);
static wchar_t* soap_wcollapse(struct soap *soap, wchar_t *s, int flag, int insitu);
#endif

static const char* soap_string(struct soap *soap, const char *s, int flag, long minlen, long maxlen, const char *pattern);
static char* soap_collapse(struct soap *soap, char *s, int flag, int insitu);
static const char* soap_QName(struct soap *soap, const char *s, long minlen, long maxlen, const char *pattern);

#ifndef WITH_LEANER
static int soap_begin_attachments(struct soap*);
static int soap_end_attachments(struct soap *soap);
static struct soap_multipart *soap_alloc_multipart(struct soap*, struct soap_multipart**, struct soap_multipart**, const char*, size_t);
static int soap_putdimefield(struct soap*, const char*, size_t);
static char *soap_getdimefield(struct soap*, size_t);
static void soap_select_mime_boundary(struct soap*);
static int soap_valid_mime_boundary(struct soap*);
static void soap_resolve_attachment(struct soap*, struct soap_multipart*);
#endif

#ifdef WITH_GZIP
static int soap_getgziphdr(struct soap*);
#endif

#ifdef WITH_OPENSSL
# ifndef SOAP_SSL_RSA_BITS
#  define SOAP_SSL_RSA_BITS 2048
# endif
static int soap_ssl_init_done = 0;
static int ssl_auth_init(struct soap*);
static int ssl_verify_callback(int, X509_STORE_CTX*);
static int ssl_verify_callback_allow_expired_certificate(int, X509_STORE_CTX*);
static int ssl_password(char*, int, int, void *);
#endif

#ifdef WITH_GNUTLS
# ifndef SOAP_SSL_RSA_BITS
#  define SOAP_SSL_RSA_BITS 2048
# endif
static int soap_ssl_init_done = 0;
static int ssl_auth_init(struct soap*);
static const char *ssl_verify(struct soap *soap, const char *host);
# if GNUTLS_VERSION_NUMBER < 0x020b00
#  if defined(HAVE_PTHREAD_H)
#   include <pthread.h>
    /* make GNUTLS thread safe with pthreads */
    GCRY_THREAD_OPTION_PTHREAD_IMPL;
#  elif defined(HAVE_PTH_H)
    #include <pth.h>
    /* make GNUTLS thread safe with PTH */
    GCRY_THREAD_OPTION_PTH_IMPL;
#  endif
# endif
#endif

#ifdef WITH_SYSTEMSSL
static int ssl_auth_init(struct soap*);
static int ssl_recv(int sk, void *s, int n, char *user);
static int ssl_send(int sk, void *s, int n, char *user);
#endif

#if !defined(WITH_NOHTTP) || !defined(WITH_LEANER)
static const char * soap_decode(char*, size_t, const char*, const char*);
#endif

#ifndef WITH_NOHTTP
static soap_wchar soap_getchunkchar(struct soap*);
static const char *http_error(struct soap*, int);
static int http_get(struct soap*);
static int http_put(struct soap*);
static int http_patch(struct soap*);
static int http_del(struct soap*);
static int http_200(struct soap*);
static int http_post(struct soap*, const char*, const char*, int, const char*, const char*, ULONG64);
static int http_send_header(struct soap*, const char*);
static int http_post_header(struct soap*, const char*, const char*);
static int http_response(struct soap*, int, ULONG64);
static int http_parse(struct soap*);
static int http_parse_header(struct soap*, const char*, const char*);
#endif

#ifndef WITH_NOIO

static int fsend(struct soap*, const char*, size_t);
static size_t frecv(struct soap*, char*, size_t);
static int tcp_init(struct soap*);
static const char *tcp_error(struct soap*);

#if !defined(WITH_IPV6)
static int tcp_gethost(struct soap*, const char *addr, struct in_addr *inaddr);
#endif
#if !defined(WITH_IPV6) || defined(WITH_COOKIES)
static int tcp_gethostbyname(struct soap*, const char *addr, struct hostent *hostent, struct in_addr *inaddr);
#endif

static SOAP_SOCKET tcp_connect(struct soap*, const char *endpoint, const char *host, int port);
static SOAP_SOCKET tcp_accept(struct soap*, SOAP_SOCKET, struct sockaddr*, int*);
static int tcp_select(struct soap*, SOAP_SOCKET, int, int);
static int tcp_disconnect(struct soap*);
static int tcp_closesocket(struct soap*, SOAP_SOCKET);
static int tcp_shutdownsocket(struct soap*, SOAP_SOCKET, int);
static const char *soap_strerror(struct soap*);

#define SOAP_TCP_SELECT_RCV 0x1
#define SOAP_TCP_SELECT_SND 0x2
#define SOAP_TCP_SELECT_ERR 0x4
#define SOAP_TCP_SELECT_ALL 0x7
#define SOAP_TCP_SELECT_PIP 0x8

#if defined(WIN32)
  #define SOAP_SOCKBLOCK(fd) \
  { \
    u_long blocking = 0; \
    ioctlsocket(fd, FIONBIO, &blocking); \
  }
  #define SOAP_SOCKNONBLOCK(fd) \
  { \
    u_long nonblocking = 1; \
    ioctlsocket(fd, FIONBIO, &nonblocking); \
  }
#elif defined(VXWORKS)
  #define SOAP_SOCKBLOCK(fd) \
  { \
    u_long blocking = 0; \
    ioctl(fd, FIONBIO, (int)(&blocking)); \
  }
  #define SOAP_SOCKNONBLOCK(fd) \
  { \
    u_long nonblocking = 1; \
    ioctl(fd, FIONBIO, (int)(&nonblocking)); \
  }
#elif defined(__VMS)
  #define SOAP_SOCKBLOCK(fd) \
  { \
    int blocking = 0; \
    ioctl(fd, FIONBIO, &blocking); \
  }
  #define SOAP_SOCKNONBLOCK(fd) \
  { \
    int nonblocking = 1; \
    ioctl(fd, FIONBIO, &nonblocking); \
  }
#elif defined(SYMBIAN)
  #define SOAP_SOCKBLOCK(fd) \
  { \
    long blocking = 0; \
    ioctl(fd, 0/*FIONBIO*/, &blocking); \
  }
  #define SOAP_SOCKNONBLOCK(fd) \
  { \
    long nonblocking = 1; \
    ioctl(fd, 0/*FIONBIO*/, &nonblocking); \
  }
#else
  #define SOAP_SOCKBLOCK(fd) (void)fcntl(fd, F_SETFL, fcntl(fd, F_GETFL)&~O_NONBLOCK);
  #define SOAP_SOCKNONBLOCK(fd) (void)fcntl(fd, F_SETFL, fcntl(fd, F_GETFL)|O_NONBLOCK);
#endif

#endif

static const char soap_env1[42] = "http://schemas.xmlsoap.org/soap/envelope/";
static const char soap_enc1[42] = "http://schemas.xmlsoap.org/soap/encoding/";
static const char soap_env2[40] = "http://www.w3.org/2003/05/soap-envelope";
static const char soap_enc2[40] = "http://www.w3.org/2003/05/soap-encoding";
static const char soap_rpc[35] = "http://www.w3.org/2003/05/soap-rpc";

const union soap_double_nan soap_double_nan = {{0xFFFFFFFF, 0xFFFFFFFF}};
const char soap_base64o[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
const char soap_base64i[81] = "\76XXX\77\64\65\66\67\70\71\72\73\74\75XXXXXXX\00\01\02\03\04\05\06\07\10\11\12\13\14\15\16\17\20\21\22\23\24\25\26\27\30\31XXXXXX\32\33\34\35\36\37\40\41\42\43\44\45\46\47\50\51\52\53\54\55\56\57\60\61\62\63";

#ifndef WITH_LEAN
static const char soap_indent[21] = "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
/* Alternative indentation form for SOAP_XML_INDENT with spaces instead of tabs:
static const char soap_indent[41] = "\n                                       ";
*/
#endif

#ifndef SOAP_CANARY
# define SOAP_CANARY (0xC0DE)
#endif

static const char soap_padding[4] = "\0\0\0";
#define SOAP_STR_PADDING (soap_padding)
#define SOAP_STR_EOS (soap_padding)
#define SOAP_NON_NULL (soap_padding)

#ifndef WITH_LEAN
static const struct soap_code_map html_entity_codes[] = /* entities for XHTML parsing */
{
  { 160, "nbsp" },
  { 161, "iexcl" },
  { 162, "cent" },
  { 163, "pound" },
  { 164, "curren" },
  { 165, "yen" },
  { 166, "brvbar" },
  { 167, "sect" },
  { 168, "uml" },
  { 169, "copy" },
  { 170, "ordf" },
  { 171, "laquo" },
  { 172, "not" },
  { 173, "shy" },
  { 174, "reg" },
  { 175, "macr" },
  { 176, "deg" },
  { 177, "plusmn" },
  { 178, "sup2" },
  { 179, "sup3" },
  { 180, "acute" },
  { 181, "micro" },
  { 182, "para" },
  { 183, "middot" },
  { 184, "cedil" },
  { 185, "sup1" },
  { 186, "ordm" },
  { 187, "raquo" },
  { 188, "frac14" },
  { 189, "frac12" },
  { 190, "frac34" },
  { 191, "iquest" },
  { 192, "Agrave" },
  { 193, "Aacute" },
  { 194, "Acirc" },
  { 195, "Atilde" },
  { 196, "Auml" },
  { 197, "Aring" },
  { 198, "AElig" },
  { 199, "Ccedil" },
  { 200, "Egrave" },
  { 201, "Eacute" },
  { 202, "Ecirc" },
  { 203, "Euml" },
  { 204, "Igrave" },
  { 205, "Iacute" },
  { 206, "Icirc" },
  { 207, "Iuml" },
  { 208, "ETH" },
  { 209, "Ntilde" },
  { 210, "Ograve" },
  { 211, "Oacute" },
  { 212, "Ocirc" },
  { 213, "Otilde" },
  { 214, "Ouml" },
  { 215, "times" },
  { 216, "Oslash" },
  { 217, "Ugrave" },
  { 218, "Uacute" },
  { 219, "Ucirc" },
  { 220, "Uuml" },
  { 221, "Yacute" },
  { 222, "THORN" },
  { 223, "szlig" },
  { 224, "agrave" },
  { 225, "aacute" },
  { 226, "acirc" },
  { 227, "atilde" },
  { 228, "auml" },
  { 229, "aring" },
  { 230, "aelig" },
  { 231, "ccedil" },
  { 232, "egrave" },
  { 233, "eacute" },
  { 234, "ecirc" },
  { 235, "euml" },
  { 236, "igrave" },
  { 237, "iacute" },
  { 238, "icirc" },
  { 239, "iuml" },
  { 240, "eth" },
  { 241, "ntilde" },
  { 242, "ograve" },
  { 243, "oacute" },
  { 244, "ocirc" },
  { 245, "otilde" },
  { 246, "ouml" },
  { 247, "divide" },
  { 248, "oslash" },
  { 249, "ugrave" },
  { 250, "uacute" },
  { 251, "ucirc" },
  { 252, "uuml" },
  { 253, "yacute" },
  { 254, "thorn" },
  { 255, "yuml" },
  {   0, NULL }
};
#endif

#ifndef WITH_NOIO
#ifndef WITH_LEAN
static const struct soap_code_map h_error_codes[] =
{
#ifdef HOST_NOT_FOUND
  { HOST_NOT_FOUND, "Host not found" },
#endif
#ifdef TRY_AGAIN
  { TRY_AGAIN, "Try Again" },
#endif
#ifdef NO_RECOVERY
  { NO_RECOVERY, "No Recovery" },
#endif
#ifdef NO_DATA
  { NO_DATA, "No Data" },
#endif
#ifdef NO_ADDRESS
  { NO_ADDRESS, "No Address" },
#endif
  { 0, NULL }
};
#endif
#endif

#ifndef WITH_NOHTTP
#ifndef WITH_LEAN
static const struct soap_code_map h_http_error_codes[] =
{
  { 100, "Continue" },
  { 101, "Switching Protocols" },
  { 200, "OK" },
  { 201, "Created" },
  { 202, "Accepted" },
  { 203, "Non-Authoritative Information" },
  { 204, "No Content" },
  { 205, "Reset Content" },
  { 206, "Partial Content" },
  { 300, "Multiple Choices" },
  { 301, "Moved Permanently" },
  { 302, "Found" },
  { 303, "See Other" },
  { 304, "Not Modified" },
  { 305, "Use Proxy" },
  { 307, "Temporary Redirect" },
  { 400, "Bad Request" },
  { 401, "Unauthorized" },
  { 402, "Payment Required" },
  { 403, "Forbidden" },
  { 404, "Not Found" },
  { 405, "Method Not Allowed" },
  { 406, "Not Acceptable" },
  { 407, "Proxy Authentication Required" },
  { 408, "Request Time-out" },
  { 409, "Conflict" },
  { 410, "Gone" },
  { 411, "Length Required" },
  { 412, "Precondition Failed" },
  { 413, "Request Entity Too Large" },
  { 414, "Request-URI Too Large" },
  { 415, "Unsupported Media Type" },
  { 416, "Requested range not satisfiable" },
  { 417, "Expectation Failed" },
  { 422, "Unprocessable Entity" },
  { 426, "Upgrade Required" },
  { 428, "Precondition Required" },
  { 429, "Too Many Requests" },
  { 431, "Request Header Fields Too Large" },
  { 500, "Internal Server Error" },
  { 501, "Not Implemented" },
  { 502, "Bad Gateway" },
  { 503, "Service Unavailable" },
  { 504, "Gateway Time-out" },
  { 505, "HTTP Version not supported" },
  { 511, "Network Authentication Required" },
  {   0, NULL }
};
#endif
#endif

#ifdef WITH_OPENSSL
static const struct soap_code_map h_ssl_error_codes[] =
{
#define _SSL_ERROR(e) { e, #e }
  _SSL_ERROR(SSL_ERROR_SSL),
  _SSL_ERROR(SSL_ERROR_ZERO_RETURN),
  _SSL_ERROR(SSL_ERROR_WANT_READ),
  _SSL_ERROR(SSL_ERROR_WANT_WRITE),
  _SSL_ERROR(SSL_ERROR_WANT_CONNECT),
  _SSL_ERROR(SSL_ERROR_WANT_X509_LOOKUP),
  _SSL_ERROR(SSL_ERROR_SYSCALL),
  { 0, NULL }
};
#endif

#ifndef WITH_LEANER
static const struct soap_code_map mime_codes[] =
{
  { SOAP_MIME_7BIT,             "7bit" },
  { SOAP_MIME_8BIT,             "8bit" },
  { SOAP_MIME_BINARY,           "binary" },
  { SOAP_MIME_QUOTED_PRINTABLE, "quoted-printable" },
  { SOAP_MIME_BASE64,           "base64" },
  { SOAP_MIME_IETF_TOKEN,       "ietf-token" },
  { SOAP_MIME_X_TOKEN,          "x-token" },
  { 0,                          NULL }
};
#endif

#ifdef WIN32
static int tcp_done = 0;
#endif

#if (defined(_AIX43) || defined(TRU64) || defined(HP_UX)) && defined(HAVE_GETHOSTBYNAME_R)
#ifndef h_errno
extern int h_errno;
#endif
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static int
fsend(struct soap *soap, const char *s, size_t n)
{
  int nwritten, err;
  SOAP_SOCKET sk;
  soap->errnum = 0;
#if defined(__cplusplus) && !defined(WITH_COMPAT)
  if (soap->os)
  {
    soap->os->write(s, (std::streamsize)n);
    if (soap->os->good())
      return SOAP_OK;
    return SOAP_EOF;
  }
#endif
  sk = soap->sendsk;
  if (!soap_valid_socket(sk))
    sk = soap->socket;
  while (n)
  {
    if (soap_valid_socket(sk))
    {
      if (soap->send_timeout)
      {
        for (;;)
        {
          int r;
#ifdef WITH_SELF_PIPE
#ifdef WITH_OPENSSL
          if (soap->ssl)
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_ALL | SOAP_TCP_SELECT_PIP, soap->send_timeout);
          else
#endif
#ifdef WITH_GNUTLS
          if (soap->session)
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_ALL | SOAP_TCP_SELECT_PIP, soap->send_timeout);
          else
#endif
#ifdef WITH_SYSTEMSSL
          if (soap->ssl)
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_ALL | SOAP_TCP_SELECT_PIP, soap->send_timeout);
          else
#endif
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR | SOAP_TCP_SELECT_PIP, soap->send_timeout);
          if ((r & SOAP_TCP_SELECT_PIP)) /* abort if data is pending on pipe */
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Connection closed by self pipe\n"));
            return SOAP_EOF;
          }
#else
#ifdef WITH_OPENSSL
          if (soap->ssl)
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_ALL, soap->send_timeout);
          else
#endif
#ifdef WITH_GNUTLS
          if (soap->session)
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_ALL, soap->send_timeout);
          else
#endif
#ifdef WITH_SYSTEMSSL
          if (soap->ssl)
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_ALL, soap->send_timeout);
          else
#endif
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, soap->send_timeout);
#endif
          if (r > 0)
            break;
          if (!r)
            return SOAP_EOF;
          err = soap->errnum;
          if (!err)
            return soap->error;
          if (err != SOAP_EAGAIN && err != SOAP_EWOULDBLOCK)
            return SOAP_EOF;
        }
      }
#ifndef WITH_LEAN
      if (soap->transfer_timeout)
      {
        time_t now = time(NULL);
        if ((soap->transfer_timeout > 0 && difftime(now, (time_t)soap->start) > (double)soap->transfer_timeout)
         || (soap->transfer_timeout < 0 && difftime(now, (time_t)soap->start) > -1000000.0 * (double)soap->transfer_timeout))
        return SOAP_EOF;
      }
#endif
#ifdef WITH_OPENSSL
      if (soap->ssl)
        nwritten = SSL_write(soap->ssl, s, (int)n);
      else if (soap->bio)
        nwritten = BIO_write(soap->bio, s, (int)n);
      else
#endif
#ifdef WITH_GNUTLS
      if (soap->session)
        nwritten = gnutls_record_send(soap->session, s, n);
      else
#endif
#ifdef WITH_SYSTEMSSL
      if (soap->ssl)
      {
        err = gsk_secure_socket_write(soap->ssl, (char*)s, n, &nwritten);
        if (err != GSK_OK)
          nwritten = 0;
      }
      else
#endif
#ifndef WITH_LEAN
      if ((soap->omode & SOAP_IO_UDP))
      {
        if (soap->peerlen)
          nwritten = sendto(sk, (char*)s, (SOAP_WINSOCKINT)n, soap->socket_flags, &soap->peer.addr, (SOAP_WINSOCKINT)soap->peerlen);
        else
          nwritten = send(sk, s, (SOAP_WINSOCKINT)n, soap->socket_flags);
        /* retry and back-off algorithm */
        /* TODO: this is not very clear from specs so verify and limit conditions under which we should loop (e.g. ENOBUFS) */
        if (nwritten < 0)
        {
          int udp_repeat;
          int udp_delay;
          if ((soap->connect_flags & SO_BROADCAST))
            udp_repeat = 2; /* SOAP-over-UDP MULTICAST_UDP_REPEAT - 1 */
          else
            udp_repeat = 1; /* SOAP-over-UDP UNICAST_UDP_REPEAT - 1 */
          udp_delay = ((unsigned int)soap_random % 201) + 50; /* UDP_MIN_DELAY .. UDP_MAX_DELAY */
          do
          {
            tcp_select(soap, sk, SOAP_TCP_SELECT_ERR, -1000 * udp_delay);
            if (soap->peerlen)
              nwritten = sendto(sk, (char*)s, (SOAP_WINSOCKINT)n, soap->socket_flags, &soap->peer.addr, (SOAP_WINSOCKINT)soap->peerlen);
            else
              nwritten = send(sk, s, (SOAP_WINSOCKINT)n, soap->socket_flags);
            udp_delay <<= 1;
            if (udp_delay > 500) /* UDP_UPPER_DELAY */
              udp_delay = 500;
          } while (nwritten < 0 && udp_repeat-- > 1);
        }
        if (nwritten < 0)
        {
          err = soap_socket_errno;
          if (err && err != SOAP_EINTR)
          {
            soap->errnum = err;
            return SOAP_EOF;
          }
          nwritten = 0; /* and call write() again */
        }
      }
      else
#endif
#if !defined(AS400)
        nwritten = send(sk, s, (int)n, soap->socket_flags);
#else
        nwritten = send(sk, (void*)s, n, soap->socket_flags);
#endif
      if (nwritten <= 0)
      {
        int r = 0;
        err = soap_socket_errno;
#ifdef WITH_OPENSSL
        if (soap->ssl && (r = SSL_get_error(soap->ssl, nwritten)) != SSL_ERROR_NONE && r != SSL_ERROR_WANT_READ && r != SSL_ERROR_WANT_WRITE)
        {
          soap->errnum = err;
          return SOAP_EOF;
        }
#endif
#ifdef WITH_GNUTLS
        if (soap->session)
        {
          if (nwritten == GNUTLS_E_INTERRUPTED)
            err = SOAP_EINTR;
          else if (nwritten == GNUTLS_E_AGAIN)
            err = SOAP_EAGAIN;
        }
#endif
        if (err == SOAP_EWOULDBLOCK || err == SOAP_EAGAIN)
        {
#if defined(WITH_OPENSSL)
          if (soap->ssl && r == SSL_ERROR_WANT_READ)
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, soap->send_timeout ? soap->send_timeout : -10000);
          else
#elif defined(WITH_GNUTLS)
          if (soap->session && !gnutls_record_get_direction(soap->session))
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, soap->send_timeout ? soap->send_timeout : -10000);
          else
#endif
            r = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, soap->send_timeout ? soap->send_timeout : -10000);
          if (!r && soap->send_timeout)
            return SOAP_EOF;
          if (r < 0)
            return SOAP_EOF;
        }
        else if (err && err != SOAP_EINTR)
        {
          soap->errnum = err;
          return SOAP_EOF;
        }
        nwritten = 0; /* and call write() again */
      }
    }
    else
    {
#ifdef WITH_FASTCGI
      nwritten = fwrite((void*)s, 1, n, stdout);
      fflush(stdout);
#else
#ifdef UNDER_CE
      nwritten = fwrite(s, 1, n, soap->sendfd);
#else
#ifdef WMW_RPM_IO
      /* vxWorks compatible */
      if (soap->rpmreqid)
        nwritten = (httpBlockPut(soap->rpmreqid, (char*)s, n) == 0) ? n : -1;
      else
#endif
#ifdef WIN32
      nwritten = _write(soap->sendfd, s, (unsigned int)n);
#else
      nwritten = write(soap->sendfd, s, (unsigned int)n);
#endif
#endif
#endif
      if (nwritten <= 0)
      {
#ifndef WITH_FASTCGI
        err = soap_errno;
#else
        err = EOF;
#endif
        if (err && err != SOAP_EINTR && err != SOAP_EWOULDBLOCK && err != SOAP_EAGAIN)
        {
          soap->errnum = err;
          return SOAP_EOF;
        }
        nwritten = 0; /* and call write() again */
      }
    }
    n -= nwritten;
    s += nwritten;
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_send_raw(struct soap *soap, const char *s, size_t n)
{
  if (!s || !n)
    return SOAP_OK;
#ifndef WITH_LEANER
  if (soap->fpreparesend && (soap->mode & SOAP_IO) != SOAP_IO_STORE && (soap->mode & SOAP_IO_LENGTH) && (soap->error = soap->fpreparesend(soap, s, n)) != SOAP_OK)
    return soap->error;
  if (soap->ffiltersend && (soap->error = soap->ffiltersend(soap, &s, &n)) != SOAP_OK)
    return soap->error;
#endif
  if ((soap->mode & SOAP_IO_LENGTH))
  {
    soap->count += n;
  }
  else if ((soap->mode & SOAP_IO))
  {
    size_t i = sizeof(soap->buf) - soap->bufidx;
    while (n >= i)
    {
      (void)soap_memcpy((void*)(soap->buf + soap->bufidx), i, (const void*)s, i);
      soap->bufidx = sizeof(soap->buf);
      if (soap_flush(soap))
        return soap->error;
      s += i;
      n -= i;
      i = sizeof(soap->buf);
    }
    (void)soap_memcpy((void*)(soap->buf + soap->bufidx), sizeof(soap->buf) - soap->bufidx, (const void*)s, n);
    soap->bufidx += n;
  }
  else
  {
    return soap_flush_raw(soap, s, n);
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_flush(struct soap *soap)
{
  size_t n = soap->bufidx;
  if (!n)
    return soap->error = soap->fsend(soap, SOAP_STR_EOS, 0); /* force a zero send for HTTP GET and DELETE */
#ifndef WITH_LEANER
  if ((soap->mode & SOAP_IO) == SOAP_IO_STORE)
  {
    int r;
    if (soap->fpreparesend && (r = soap->fpreparesend(soap, soap->buf, n)) != SOAP_OK)
      return soap->error = r;
  }
#endif
  soap->bufidx = 0;
#ifdef WITH_ZLIB
  if ((soap->mode & SOAP_ENC_ZLIB) && soap->d_stream)
  {
    soap->d_stream->next_in = (Byte*)soap->buf;
    soap->d_stream->avail_in = (unsigned int)n;
#ifdef WITH_GZIP
    soap->z_crc = crc32(soap->z_crc, (Byte*)soap->buf, (unsigned int)n);
#endif
    do
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Deflating %u bytes\n", soap->d_stream->avail_in));
      if (deflate(soap->d_stream, Z_NO_FLUSH) != Z_OK)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Unable to deflate: %s\n", soap->d_stream->msg ? soap->d_stream->msg : SOAP_STR_EOS));
        return soap->error = SOAP_ZLIB_ERROR;
      }
      if (!soap->d_stream->avail_out)
      {
        if (soap_flush_raw(soap, soap->z_buf, sizeof(soap->buf)))
          return soap->error;
        soap->d_stream->next_out = (Byte*)soap->z_buf;
        soap->d_stream->avail_out = sizeof(soap->buf);
      }
    } while (soap->d_stream->avail_in);
    return SOAP_OK;
  }
#endif
  return soap_flush_raw(soap, soap->buf, n);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_flush_raw(struct soap *soap, const char *s, size_t n)
{
  if ((soap->mode & SOAP_IO) == SOAP_IO_STORE)
  {
    void *t;
    t = soap_push_block(soap, NULL, n);
    if (!t)
      return soap->error = SOAP_EOM;
    (void)soap_memcpy(t, n, (const void*)s, n);
    return SOAP_OK;
  }
#ifndef WITH_LEANER
  if ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK)
  {
    char t[24];
    (SOAP_SNPRINTF(t, sizeof(t), 20), &"\r\n%lX\r\n"[soap->chunksize ? 0 : 2], (unsigned long)n);
    DBGMSG(SENT, t, strlen(t));
    soap->error = soap->fsend(soap, t, strlen(t));
    if (soap->error)
      return soap->error;
    soap->chunksize += n;
  }
  DBGMSG(SENT, s, n);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Send %u bytes to socket=%d/fd=%d\n", (unsigned int)n, (int)soap->socket, soap->sendfd));
#endif
  return soap->error = soap->fsend(soap, s, n);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_send(struct soap *soap, const char *s)
{
  if (!s)
    return SOAP_OK;
  return soap_send_raw(soap, s, strlen(s));
}

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_send2(struct soap *soap, const char *s1, const char *s2)
{
  if (soap_send(soap, s1))
    return soap->error;
  return soap_send(soap, s2);
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_send3(struct soap *soap, const char *s1, const char *s2, const char *s3)
{
  if (soap_send(soap, s1)
   || soap_send(soap, s2))
    return soap->error;
  return soap_send(soap, s3);
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_query_send_key(struct soap *soap, const char *s)
{
  if (!s)
    return SOAP_OK;
  if (!soap->body && soap_send_raw(soap, "&", 1))
    return soap->error;
  soap->body = 0;
  (void)soap_encode_url(s, soap->msgbuf, (int)sizeof(soap->msgbuf)); /* msgbuf length is max SOAP_TMPLEN or just 1024 bytes */
  return soap_send(soap, soap->msgbuf);
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_query_send_val(struct soap *soap, const char *s)
{
  if (!s)
    return SOAP_OK;
  if (soap_send_raw(soap, "=", 1))
    return soap->error;
  (void)soap_encode_url(s, soap->msgbuf, (int)sizeof(soap->msgbuf)); /* msgbuf length is max SOAP_TMPLEN or just 1024 bytes */
  return soap_send(soap, soap->msgbuf);
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
char *
SOAP_FMAC2
soap_query(struct soap *soap)
{
  return strchr(soap->path, '?');
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
char *
SOAP_FMAC2
soap_query_key(struct soap *soap, char **s)
{
  char *t = *s;
  (void)soap;
  if (t && *t)
  {
    *s = (char*)soap_query_decode(t, strlen(t), t + 1);
    return t;
  }
  return *s = NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
char *
SOAP_FMAC2
soap_query_val(struct soap *soap, char **s)
{
  char *t = *s;
  (void)soap;
  if (t && *t == '=')
  {
    *s = (char*)soap_query_decode(t, strlen(t), t + 1);
    return t;
  }
  return NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_query_decode(char *buf, size_t len, const char *val)
{
  const char *s;
  char *t;
  for (s = val; *s; s++)
    if (*s != ' ' && *s != '=')
      break;
  if (*s == '"')
  {
    t = buf;
    s++;
    while (*s && *s != '"' && len-- > 1)
      *t++ = *s++;
    *t = '\0';
    do s++;
    while (*s && *s != '&' && *s != '=');
  }
  else
  {
    t = buf;
    while (*s && *s != '&' && *s != '=' && len-- > 1)
    {
      switch (*s)
      {
        case '+':
          *t++ = ' ';
          s++;
          break;
        case '\t':
        case '\n':
        case '\r':
        case ' ':
          s++;
          break;
        case '%':
          *t++ = ((s[1] >= 'A' ? (s[1]&0x7) + 9 : s[1] - '0') << 4) + (s[2] >= 'A' ? (s[2]&0x7) + 9 : s[2] - '0');
          s += 3;
          break;
        default:
          *t++ = *s++;
      }
    }
    *t = '\0';
  }
  return s;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static size_t
frecv(struct soap *soap, char *s, size_t n)
{
  int r;
  int retries = 100; /* max 100 retries with non-blocking sockets */
  SOAP_SOCKET sk;
  soap->errnum = 0;
#if defined(__cplusplus) && !defined(WITH_COMPAT)
  if (soap->is) /* recv from C++ stream */
  {
    if (soap->is->good())
      return (size_t)soap->is->read(s, (std::streamsize)n).gcount(); /* downcast to std::streamsize is OK: gcount() returns how much we got in s[] */
    return 0;
  }
#else
  if (soap->is) /* recv from C buffer until NUL */
  {
    size_t l = strlen(soap->is);
    if (l > n)
      l = n;
    (void)soap_memcpy((void*)s, n, soap->is, l);
    soap->is += l;
    return l;
  }
#endif
  sk = soap->recvsk;
  if (!soap_valid_socket(sk))
    sk = soap->socket;
  if (soap_valid_socket(sk))
  {
    for (;;)
    {
#if defined(WITH_OPENSSL) || defined(WITH_SYSTEMSSL)
      int err = 0;
#endif
#ifdef WITH_OPENSSL
      if (soap->recv_timeout && !soap->ssl) /* OpenSSL: sockets are nonblocking so go ahead to read */
#else
      if (soap->recv_timeout)
#endif
      {
        for (;;)
        {
#ifdef WITH_SELF_PIPE
          r = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR | SOAP_TCP_SELECT_PIP, soap->recv_timeout);
          if ((r & SOAP_TCP_SELECT_PIP)) /* abort if data is pending on pipe */
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Connection closed by self pipe\n"));
            return 0;
          }
#else
          r = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, soap->recv_timeout);
#endif
          if (r > 0)
            break;
          if (!r)
            return 0;
          r = soap->errnum;
          if (r != SOAP_EAGAIN && r != SOAP_EWOULDBLOCK)
            return 0;
        }
      }
#ifndef WITH_LEAN
      if (soap->transfer_timeout)
      {
        time_t now = time(NULL);
        if ((soap->transfer_timeout > 0 && difftime(now, (time_t)soap->start) > (double)soap->transfer_timeout)
         || (soap->transfer_timeout < 0 && difftime(now, (time_t)soap->start) > -1000000.0 * (double)soap->transfer_timeout))
        return 0;
      }
#endif
#ifdef WITH_OPENSSL
      if (soap->ssl)
      {
        r = SSL_read(soap->ssl, s, (int)n);
        if (r > 0)
          return (size_t)r;
        err = SSL_get_error(soap->ssl, r);
        if (err != SSL_ERROR_NONE && err != SSL_ERROR_WANT_READ && err != SSL_ERROR_WANT_WRITE)
          return 0;
      }
      else if (soap->bio)
      {
        r = BIO_read(soap->bio, s, (int)n);
        if (r > 0)
          return (size_t)r;
        return 0;
      }
      else
#endif
#ifdef WITH_GNUTLS
      if (soap->session)
      {
        r = (int)gnutls_record_recv(soap->session, s, n);
        if (r >= 0)
          return (size_t)r;
      }
      else
#endif
#ifdef WITH_SYSTEMSSL
      if (soap->ssl)
      {
        err = gsk_secure_socket_read(soap->ssl, s, n, &r);
        if (err == GSK_OK && r > 0)
          return (size_t)r;
        if (err != GSK_OK && err != GSK_WOULD_BLOCK && err != GSK_WOULD_BLOCK_WRITE)
          return 0;
      }
      else
#endif
      {
#ifndef WITH_LEAN
        if ((soap->omode & SOAP_IO_UDP))
        {
          SOAP_SOCKLEN_T k = (SOAP_SOCKLEN_T)sizeof(soap->peer);
          memset((void*)&soap->peer, 0, sizeof(soap->peer));
          r = recvfrom(sk, s, (SOAP_WINSOCKINT)n, soap->socket_flags, &soap->peer.addr, &k);    /* portability note: see SOAP_SOCKLEN_T definition in stdsoap2.h, SOAP_WINSOCKINT cast is safe due to limited range of n in the engine (64K) */
          soap->peerlen = (size_t)k;
#ifdef WITH_IPV6
          soap->ip = 0;
          soap->ip6[0] = 0;
          soap->ip6[1] = 0;
          soap->ip6[2] = 0;
          soap->ip6[3] = 0;
#else
          soap->ip = ntohl(soap->peer.in.sin_addr.s_addr);
          soap->ip6[0] = 0;
          soap->ip6[1] = 0;
          soap->ip6[2] = 0xFFFF;
          soap->ip6[3] = soap->ip;
#endif
        }
        else
#endif
          r = recv(sk, s, (SOAP_WINSOCKINT)n, soap->socket_flags); /* SOAP_WINSOCKINT cast is safe due to limited range of n in the engine (64K) */
        if (r >= 0)
          return (size_t)r;
        r = soap_socket_errno;
        if (r != SOAP_EINTR && r != SOAP_EAGAIN && r != SOAP_EWOULDBLOCK)
        {
          soap->errnum = r;
          return 0;
        }
      }
#if defined(WITH_OPENSSL)
      if (soap->ssl && err == SSL_ERROR_WANT_WRITE)
        r = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, soap->recv_timeout ? soap->recv_timeout : 5);
      else
#elif defined(WITH_GNUTLS)
      if (soap->session && gnutls_record_get_direction(soap->session))
        r = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, soap->recv_timeout ? soap->recv_timeout : 5);
      else
#elif defined(WITH_SYSTEMSSL)
      if (soap->ssl && err == GSK_WOULD_BLOCK_WRITE)
        r = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, soap->recv_timeout ? soap->recv_timeout : 5);
      else
#endif
        r = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, soap->recv_timeout ? soap->recv_timeout : 5);
      if (!r && soap->recv_timeout)
        return 0;
      if (r < 0)
      {
        r = soap->errnum;
        if (r != SOAP_EAGAIN && r != SOAP_EWOULDBLOCK)
          return 0;
      }
      if (retries-- <= 0)
        return 0;
    }
  }
#ifdef WITH_FASTCGI
  return fread(s, 1, n, stdin);
#else
#ifdef UNDER_CE
  return fread(s, 1, n, soap->recvfd);
#else
#ifdef WMW_RPM_IO
  if (soap->rpmreqid)
    r = httpBlockRead(soap->rpmreqid, s, n);
  else
#endif
#ifdef WIN32
    r = _read(soap->recvfd, s, (unsigned int)n);
#else
    r = read(soap->recvfd, s, n);
#endif
  if (r >= 0)
    return (size_t)r;
  soap->errnum = soap_errno;
  return 0;
#endif
#endif
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static soap_wchar
soap_getchunkchar(struct soap *soap)
{
  if (soap->bufidx < soap->buflen)
    return soap->buf[soap->bufidx++];
  soap->bufidx = 0;
  soap->buflen = soap->chunkbuflen = soap->frecv(soap, soap->buf, sizeof(soap->buf));
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Read %u bytes from socket=%d/fd=%d\n", (unsigned int)soap->buflen, (int)soap->socket, soap->recvfd));
  DBGMSG(RECV, soap->buf, soap->buflen);
  if (soap->buflen)
    return soap->buf[soap->bufidx++];
  return EOF;
}
#endif

/******************************************************************************/

static int
soap_isxdigit(int c)
{
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_recv_raw(struct soap *soap)
{
  size_t ret;
#if !defined(WITH_LEANER) || defined(WITH_ZLIB)
  int r;
#endif
#ifdef WITH_ZLIB
  if ((soap->mode & SOAP_ENC_ZLIB) && soap->d_stream)
  {
    if (soap->d_stream->next_out == Z_NULL)
    {
      soap->bufidx = soap->buflen = 0;
      return EOF;
    }
    if (soap->d_stream->avail_in || !soap->d_stream->avail_out)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Inflating\n"));
      soap->d_stream->next_out = (Byte*)soap->buf;
      soap->d_stream->avail_out = sizeof(soap->buf);
      r = inflate(soap->d_stream, Z_NO_FLUSH);
      if (r == Z_NEED_DICT && soap->z_dict)
        r = inflateSetDictionary(soap->d_stream, (const Bytef*)soap->z_dict, soap->z_dict_len);
      if (r == Z_OK || r == Z_STREAM_END)
      {
        soap->bufidx = 0;
        ret = soap->buflen = sizeof(soap->buf) - soap->d_stream->avail_out;
        if (soap->zlib_in == SOAP_ZLIB_GZIP)
          soap->z_crc = crc32(soap->z_crc, (Byte*)soap->buf, (unsigned int)ret);
        if (r == Z_STREAM_END)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Inflated %lu->%lu bytes\n", soap->d_stream->total_in, soap->d_stream->total_out));
          soap->d_stream->next_out = Z_NULL;
        }
        if (ret || r == Z_STREAM_END)
        {
          if (soap->count + ret < soap->count)
            return soap->error = SOAP_EOF;
          soap->count += ret;
          if (soap->recv_maxlength && soap->count > soap->recv_maxlength)
            return soap->error = SOAP_EOF;
          soap->z_ratio_in = (float)soap->d_stream->total_in / (float)soap->d_stream->total_out;
          if (soap->count > SOAP_MAXINFLATESIZE && soap->z_ratio_in < SOAP_MINDEFLATERATIO)
          {
            soap->d_stream->msg = (char*)"caught SOAP_MINDEFLATERATIO explosive decompression guard (remedy: increase SOAP_MAXINFLATESIZE and/or decrease SOAP_MINDEFLATERATIO)";
            return soap->error = SOAP_ZLIB_ERROR;
          }
          DBGLOG(RECV, SOAP_MESSAGE(fdebug, "\n---- decompressed ----\n"));
          DBGMSG(RECV, soap->buf, ret);
          DBGLOG(RECV, SOAP_MESSAGE(fdebug, "\n----\n"));
#ifndef WITH_LEANER
          if (soap->fpreparerecv && (r = soap->fpreparerecv(soap, soap->buf, ret)) != SOAP_OK)
            return soap->error = r;
#endif
          return SOAP_OK;
        }
      }
      else if (r != Z_BUF_ERROR)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Inflate error: %s\n", soap->d_stream->msg ? soap->d_stream->msg : SOAP_STR_EOS));
        soap->d_stream->next_out = Z_NULL;
        return soap->error = SOAP_ZLIB_ERROR;
      }
    }
zlib_again:
    if ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK && !soap->chunksize)
    {
      (void)soap_memcpy((void*)soap->buf, sizeof(soap->buf), (const void*)soap->z_buf, sizeof(soap->buf));
      soap->buflen = soap->z_buflen;
    }
    DBGLOG(RECV, SOAP_MESSAGE(fdebug, "\n---- compressed ----\n"));
  }
#endif
#ifndef WITH_NOHTTP
  if ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK) /* read HTTP chunked transfer */
  {
    for (;;)
    {
      soap_wchar c;
      char *t, tmp[17];
      if (soap->chunksize)
      {
        soap->buflen = ret = soap->frecv(soap, soap->buf, soap->chunksize > sizeof(soap->buf) ? sizeof(soap->buf) : soap->chunksize);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting chunk: read %u bytes\n", (unsigned int)ret));
        DBGMSG(RECV, soap->buf, ret);
        soap->bufidx = 0;
        if (!ret)
        {
          soap->ahead = EOF;
          return EOF;
        }
        soap->chunksize -= ret;
        break;
      }
      if (!soap->chunkbuflen)
      {
        soap->chunkbuflen = ret = soap->frecv(soap, soap->buf, sizeof(soap->buf));
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Read %u bytes (chunked) from socket=%d\n", (unsigned int)ret, (int)soap->socket));
        DBGMSG(RECV, soap->buf, ret);
        soap->bufidx = 0;
        if (!ret)
        {
          soap->ahead = EOF;
          return EOF;
        }
      }
      else
      {
        soap->bufidx = soap->buflen;
      }
      soap->buflen = soap->chunkbuflen;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting chunk size (idx=%u len=%u)\n", (unsigned int)soap->bufidx, (unsigned int)soap->buflen));
      while (!soap_isxdigit((int)(c = soap_getchunkchar(soap))))
      {
        if ((int)c == EOF)
        {
          soap->ahead = EOF;
          return EOF;
        }
      }
      t = tmp;
      do
      {
        *t++ = (char)c;
      } while (soap_isxdigit((int)(c = soap_getchunkchar(soap))) && (size_t)(t - tmp) < sizeof(tmp)-1);
      while ((int)c != EOF && c != '\n')
        c = soap_getchunkchar(soap);
      if ((int)c == EOF)
      {
        soap->ahead = EOF;
        return EOF;
      }
      *t = '\0';
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Chunk size = %s (hex)\n", tmp));
      soap->chunksize = (size_t)soap_strtoul(tmp, &t, 16);
      if (!soap->chunksize)
      {
        soap->bufidx = soap->buflen = soap->chunkbuflen = 0;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "End of chunked message\n"));
        ret = 0;
        soap->ahead = EOF;
        break;
      }
      soap->buflen = soap->bufidx + soap->chunksize;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Moving buf len to idx=%u len=%u (%s)\n", (unsigned int)soap->bufidx, (unsigned int)soap->buflen, tmp));
      if (soap->buflen > soap->chunkbuflen)
      {
        soap->buflen = soap->chunkbuflen;
        soap->chunksize -= soap->buflen - soap->bufidx;
        soap->chunkbuflen = 0;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Passed end of buffer for chunked HTTP (%u bytes left)\n", (unsigned int)(soap->buflen - soap->bufidx)));
      }
      else if (soap->chunkbuflen)
      {
        soap->chunksize = 0;
      }
      ret = soap->buflen - soap->bufidx;
      if (ret)
        break;
    }
  }
  else
#endif
  {
    soap->bufidx = 0;
    soap->buflen = ret = soap->frecv(soap, soap->buf, sizeof(soap->buf));
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Read %u bytes from socket=%d/fd=%d\n", (unsigned int)ret, (int)soap->socket, soap->recvfd));
    DBGMSG(RECV, soap->buf, ret);
  }
#ifdef WITH_ZLIB
  if ((soap->mode & SOAP_ENC_ZLIB))
  {
    (void)soap_memcpy((void*)soap->z_buf, sizeof(soap->buf), (const void*)soap->buf, sizeof(soap->buf));
    soap->d_stream->next_in = (Byte*)(soap->z_buf + soap->bufidx);
    soap->d_stream->avail_in = (unsigned int)ret;
    soap->d_stream->next_out = (Byte*)soap->buf;
    soap->d_stream->avail_out = sizeof(soap->buf);
    r = inflate(soap->d_stream, Z_NO_FLUSH);
    if (r == Z_NEED_DICT && soap->z_dict)
      r = inflateSetDictionary(soap->d_stream, (const Bytef*)soap->z_dict, soap->z_dict_len);
    if (r == Z_OK || r == Z_STREAM_END)
    {
      soap->bufidx = 0;
      soap->z_buflen = soap->buflen;
      soap->buflen = sizeof(soap->buf) - soap->d_stream->avail_out;
      if (soap->zlib_in == SOAP_ZLIB_GZIP)
        soap->z_crc = crc32(soap->z_crc, (Byte*)soap->buf, (unsigned int)soap->buflen);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Inflated %u bytes\n", (unsigned int)soap->buflen));
      if (ret && !soap->buflen && r != Z_STREAM_END)
        goto zlib_again;
      ret = soap->buflen;
      if (r == Z_STREAM_END)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Inflated total %lu->%lu bytes\n", soap->d_stream->total_in, soap->d_stream->total_out));
        soap->d_stream->next_out = Z_NULL;
      }
      soap->z_ratio_in = (float)soap->d_stream->total_in / (float)soap->d_stream->total_out;
      if (soap->count + ret > SOAP_MAXINFLATESIZE && soap->z_ratio_in < SOAP_MINDEFLATERATIO)
      {
        soap->d_stream->msg = (char*)"caught SOAP_MINDEFLATERATIO explosive decompression guard (remedy: increase SOAP_MAXINFLATESIZE and/or decrease SOAP_MINDEFLATERATIO)";
        return soap->error = SOAP_ZLIB_ERROR;
      }
      DBGLOG(RECV, SOAP_MESSAGE(fdebug, "\n---- decompressed ----\n"));
      DBGMSG(RECV, soap->buf, ret);
#ifndef WITH_LEANER
      if (soap->fpreparerecv && (r = soap->fpreparerecv(soap, soap->buf, ret)) != SOAP_OK)
        return soap->error = r;
#endif
    }
    else
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Unable to inflate: (%d) %s\n", r, soap->d_stream->msg ? soap->d_stream->msg : SOAP_STR_EOS));
      soap->d_stream->next_out = Z_NULL;
      return soap->error = SOAP_ZLIB_ERROR;
    }
  }
#endif
#ifndef WITH_LEANER
  if (soap->fpreparerecv
#ifdef WITH_ZLIB
   && soap->zlib_in == SOAP_ZLIB_NONE
#endif
   && (r = soap->fpreparerecv(soap, soap->buf + soap->bufidx, ret)))
    return soap->error = r;
#endif
  if (ret)
  {
    if (soap->count + ret < soap->count)
      return EOF;
    soap->count += ret;
    if (soap->recv_maxlength && soap->count > soap->recv_maxlength)
      return EOF;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Read count=" SOAP_ULONG_FORMAT " (+%lu)\n", soap->count, (unsigned long)ret));
    return SOAP_OK;
  }
  return EOF;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_recv(struct soap *soap)
{
#ifndef WITH_LEANER
  if ((soap->mode & SOAP_ENC_DIME))
  {
    if (soap->dime.buflen)
    {
      char *s;
      int i;
      unsigned char tmp[12];
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DIME hdr for chunked SOAP in DIME is in buffer\n"));
      soap->buflen = soap->dime.buflen;
      soap->dime.buflen = 0;
      soap->dime.chunksize = 0;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Skip padding (%ld bytes)\n", -(long)soap->dime.size&3));
      for (i = -(long)soap->dime.size&3; i > 0; i--)
      {
        soap->bufidx++;
        if (soap->bufidx >= soap->buflen)
          if (soap_recv_raw(soap))
            return EOF;
      }
      if (!(soap->dime.flags & SOAP_DIME_CF))
        return SOAP_OK;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Get DIME hdr for next SOAP in DIME chunk\n"));
      s = (char*)tmp;
      for (i = 12; i > 0; i--)
      {
        *s++ = soap->buf[soap->bufidx++];
        if (soap->bufidx >= soap->buflen)
          if (soap_recv_raw(soap))
            return EOF;
      }
      if ((tmp[0] & 0xF8) != SOAP_DIME_VERSION)
        return soap->error = SOAP_DIME_MISMATCH;
      soap->dime.flags = (tmp[0] & 0x7) | (tmp[1] & 0xF0);
      soap->dime.size = ((size_t)tmp[8] << 24) | ((size_t)tmp[9] << 16) | ((size_t)tmp[10] << 8) | ((size_t)tmp[11]);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Get SOAP in DIME chunk (%u bytes)\n", (unsigned int)soap->dime.size));
      soap->dime.chunksize = soap->dime.size;
      if (soap->buflen - soap->bufidx >= soap->dime.size)
      {
        if ((soap->dime.flags & SOAP_DIME_ME))
        {
          soap->mode &= ~SOAP_ENC_DIME;
        }
        else
        {
          soap->dime.buflen = soap->buflen;
          soap->buflen = soap->bufidx + soap->dime.chunksize;
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Chunked SOAP in DIME (%u bytes buffered)\n", (unsigned int)soap->buflen));
        }
      }
      else
      {
        soap->dime.chunksize -= soap->buflen - soap->bufidx;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Chunked SOAP in DIME (%u bytes in chunk left)\n", (unsigned int)soap->dime.chunksize));
      }
      return SOAP_OK;
    }
    if (soap->dime.chunksize)
    {
      if (soap_recv_raw(soap))
        return EOF;
      if (soap->buflen - soap->bufidx >= soap->dime.chunksize)
      {
        if ((soap->dime.flags & SOAP_DIME_ME))
        {
          soap->dime.chunksize = 0;
          soap->mode &= ~SOAP_ENC_DIME;
        }
        else
        {
          soap->dime.buflen = soap->buflen;
          soap->buflen = soap->bufidx + soap->dime.chunksize;
          soap->dime.chunksize = 0;
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Chunked SOAP in DIME (%u bytes buffered)\n", (unsigned int)soap->buflen));
        }
      }
      else
      {
        soap->dime.chunksize -= soap->buflen - soap->bufidx;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Chunked SOAP in DIME (%u bytes in chunk left)\n", (unsigned int)soap->dime.chunksize));
      }
      return SOAP_OK;
    }
  }
  if (soap->ffilterrecv)
  {
    int err;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Filter recverror = %d bufidx = %lu buflen = %lu\n", soap->recverror, (unsigned long)soap->bufidx, (unsigned long)soap->buflen));
    if (soap->recverror)
    {
      soap->bufidx = soap->buflen = 0;
    }
    else
    {
      soap->bufidx = soap->buflen = 0;
      err = soap->ffilterrecv(soap, soap->buf, &soap->buflen, sizeof(soap->buf));
      if (err)
      {
        if (err == SOAP_EOF)
          return SOAP_EOF;
        return soap->error = err;
      }
      if (soap->buflen)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Filtered output continued %lu bytes\n", (unsigned long)soap->buflen));
        return SOAP_OK;
      }
      soap->recverror = soap_recv_raw(soap);
      soap->buflen -= soap->bufidx; /* chunked may set bufidx > 0 to skip hex chunk length */
    }
    while (soap->ffilterrecv)
    {
      err = soap->ffilterrecv(soap, soap->buf + soap->bufidx, &soap->buflen, sizeof(soap->buf) - soap->bufidx);
      if (err)
      {
        if (err == SOAP_EOF)
          return SOAP_EOF;
        return soap->error = err;
      }
      if (soap->buflen)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Filtered output %lu bytes\n", (unsigned long)soap->buflen));
        soap->buflen += soap->bufidx;
        return SOAP_OK;
      }
      if (soap->recverror)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Returning postponed error %d\n", soap->recverror));
        return soap->recverror;
      }
      soap->recverror = soap_recv_raw(soap);
      soap->buflen -= soap->bufidx; /* chunked may set bufidx > 0 to skip hex chunk length */
    }
  }
  return soap->recverror = soap_recv_raw(soap);
#else
  return soap_recv_raw(soap);
#endif
}

/******************************************************************************/

SOAP_FMAC1
const struct soap_code_map*
SOAP_FMAC2
soap_code(const struct soap_code_map *code_map, const char *str)
{
  if (code_map && str)
  {
    while (code_map->string)
    {
      if (!strcmp(str, code_map->string)) /* case sensitive */
        return code_map;
      code_map++;
    }
  }
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
LONG64
SOAP_FMAC2
soap_code_int(const struct soap_code_map *code_map, const char *str, LONG64 other)
{
  if (code_map)
  {
    while (code_map->string)
    {
      if (!soap_tag_cmp(str, code_map->string)) /* case insensitive */
        return code_map->code;
      code_map++;
    }
  }
  return other;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_code_str(const struct soap_code_map *code_map, long code)
{
  if (!code_map)
    return NULL;
  while (code_map->code != code && code_map->string)
    code_map++;
  return code_map->string;
}

/******************************************************************************/

SOAP_FMAC1
LONG64
SOAP_FMAC2
soap_code_bits(const struct soap_code_map *code_map, const char *str)
{
  LONG64 bits = 0;
  if (code_map)
  {
    while (str && *str)
    {
      const struct soap_code_map *p;
      for (p = code_map; p->string; p++)
      {
        size_t n = strlen(p->string);
        if (!strncmp(p->string, str, n) && soap_coblank((soap_wchar)str[n]))
        {
          bits |= p->code;
          str += n;
          while (*str > 0 && *str <= 32)
            str++;
          break;
        }
      }
      if (!p->string)
        return 0;
    }
  }
  return bits;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_code_list(struct soap *soap, const struct soap_code_map *code_map, long code)
{
  char *t = soap->tmpbuf;
  if (code_map)
  {
    while (code_map->string)
    {
      if ((code_map->code & code))
      {
        const char *s = code_map->string;
        if (t != soap->tmpbuf)
          *t++ = ' ';
        while (*s && t < soap->tmpbuf + sizeof(soap->tmpbuf) - 1)
          *t++ = *s++;
        if (t == soap->tmpbuf + sizeof(soap->tmpbuf) - 1)
          break;
      }
      code_map++;
    }
  }
  *t = '\0';
  return soap->tmpbuf;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_binary_search_string(const char **a, int n, const char *s)
{
  int min = 0, max = n-1;
  while (min <= max)
  {
    int mid = (min+max)/2;
    int r = strcmp(s, a[mid]);
    if (r < 0)
      max = mid - 1;
    else if (r > 0)
      min = mid + 1;
    else
      return mid;
  }
  return -1;
}

/******************************************************************************/

static soap_wchar
soap_char(struct soap *soap)
{
  char tmp[8];
  int i;
  soap_wchar c;
  char *s = tmp;
  for (i = 0; i < (int)sizeof(tmp)-1; i++)
  {
    c = soap_get1(soap);
    if (c == ';' || (int)c == EOF)
      break;
    *s++ = (char)c;
  }
  *s = '\0';
  if (*tmp == '#')
  {
    if (tmp[1] == 'x' || tmp[1] == 'X')
      return (soap_wchar)soap_strtol(tmp + 2, NULL, 16);
    return (soap_wchar)soap_strtol(tmp + 1, NULL, 10);
  }
  if (!strcmp(tmp, "lt"))
    return '<';
  if (!strcmp(tmp, "gt"))
    return '>';
  if (!strcmp(tmp, "amp"))
    return '&';
  if (!strcmp(tmp, "quot"))
    return '"';
  if (!strcmp(tmp, "apos"))
    return '\'';
#ifndef WITH_LEAN
  return (soap_wchar)soap_code_int(html_entity_codes, tmp, (LONG64)SOAP_UNKNOWN_CHAR);
#else
  return SOAP_UNKNOWN_CHAR; /* use this to represent unknown code */
#endif
}

/******************************************************************************/

#ifdef WITH_LEAN
soap_wchar
soap_get0(struct soap *soap)
{
  return (soap->bufidx >= soap->buflen && soap_recv(soap)) ?  EOF : (unsigned char)soap->buf[soap->bufidx];
}
#endif

/******************************************************************************/

#ifdef WITH_LEAN
soap_wchar
soap_get1(struct soap *soap)
{
  return (soap->bufidx >= soap->buflen && soap_recv(soap)) ?  EOF : (unsigned char)soap->buf[soap->bufidx++];
}
#endif

/******************************************************************************/

#ifdef WITH_LEAN
SOAP_FMAC1
soap_wchar
SOAP_FMAC2
soap_getchar(struct soap *soap)
{
  soap_wchar c;
  c = soap->ahead;
  if (c)
  {
    if ((int)c != EOF)
      soap->ahead = 0;
    return c;
  }
  return soap_get1(soap);
}
#else
SOAP_FMAC1
soap_wchar
SOAP_FMAC2
soap_getahead(struct soap *soap)
{
  soap_wchar c = soap->ahead;
  if ((int)c != EOF)
    soap->ahead = 0;
  return c;
}
#endif

/******************************************************************************/

SOAP_FMAC1
soap_wchar
SOAP_FMAC2
soap_get(struct soap *soap)
{
  soap_wchar c;
  c = soap->ahead;
  if (c)
  {
    if ((int)c != EOF)
      soap->ahead = 0;
  }
  else
  {
    c = soap_get1(soap);
  }
  while ((int)c != EOF)
  {
    if (soap->cdata)
    {
      if (c == ']')
      {
        c = soap_get1(soap);
        if (c == ']')
        {
          c = soap_get0(soap);
          if (c == '>')
          {
            soap->cdata = 0;
            c = soap_get1(soap);
            c = soap_get1(soap);
          }
          else
          {
            soap_unget(soap, ']');
            return ']';
          }
        }
        else
        {
          soap_revget1(soap);
          return ']';
        }
      }
      else
      {
        return c;
      }
    }
    switch (c)
    {
      case '<':
        do
        {
          c = soap_get1(soap);
        } while (soap_coblank(c));
        if (c == '!' || c == '?' || c == '%')
        {
          int k = 1;
          if (c == '!')
          {
            c = soap_get1(soap);
            if (c == '[')
            {
              do
              {
                c = soap_get1(soap);
              } while ((int)c != EOF && c != '[');
              if ((int)c == EOF)
                break;
              soap->cdata = 1;
              c = soap_get1(soap);
              continue;
            }
            if (c == '-' && (c = soap_get1(soap)) == '-')
            {
              do
              {
                c = soap_get1(soap);
                if (c == '-' && (c = soap_get1(soap)) == '-')
                  break;
              } while ((int)c != EOF);
            }
          }
          else if (c == '?')
          {
            c = soap_getpi(soap);
          }
          while ((int)c != EOF)
          {
            if (c == '<')
            {
              k++;
            }
            else if (c == '>')
            {
              if (--k <= 0)
                break;
            }
            c = soap_get1(soap);
          }
          if ((int)c == EOF)
            break;
          c = soap_get1(soap);
          continue;
        }
        if (c == '/')
          return SOAP_TT;
        soap_revget1(soap);
        return SOAP_LT;
      case '>':
        return SOAP_GT;
      case '"':
        return SOAP_QT;
      case '\'':
        return SOAP_AP;
      case '&':
        return soap_char(soap) | 0x80000000;
    }
    break;
  }
  return c;
}

/******************************************************************************/

static soap_wchar
soap_getpi(struct soap *soap)
{
  char buf[64];
  char *s = buf;
  size_t i = sizeof(buf);
  soap_wchar c;
  while ((int)(c = soap_getchar(soap)) != EOF && c != '?')
  {
    if (i > 1)
    {
      if (soap_coblank(c))
        c = ' ';
      *s++ = (char)c;
      i--;
    }
  }
  *s = '\0';
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "XML PI <?%s?>\n", buf));
  if (!strncmp(buf, "xml ", 4))
  {
    s = strstr(buf, " encoding=");
    if (s && s[10])
    {
      if (!soap_tag_cmp(s + 11, "iso-8859-*")
       || !soap_tag_cmp(s + 11, "latin*"))
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Switching to latin encoding\n"));
        soap->mode |= SOAP_ENC_LATIN;
      }
      else if (!soap_tag_cmp(s + 11, "utf-8*"))
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Switching to utf-8 encoding\n"));
        soap->mode &= ~SOAP_ENC_LATIN;
      }
    }
  }
  if ((int)c != EOF)
    c = soap_getchar(soap);
  return c;
}

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_move(struct soap *soap, ULONG64 n)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Moving " SOAP_ULONG_FORMAT " bytes forward\n", n));
  for (; n; n--)
    if ((int)soap_getchar(soap) == EOF)
      return SOAP_EOF;
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
ULONG64
SOAP_FMAC2
soap_tell(struct soap *soap)
{
  return soap->count - soap->buflen + soap->bufidx - (soap->ahead != 0);
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_pututf8(struct soap *soap, unsigned long c)
{
  char tmp[24];
  if ((c < 0x7F && c > 0x1F))
  {
    *tmp = (char)c;
    return soap_send_raw(soap, tmp, 1);
  }
#ifdef WITH_REPLACE_ILLEGAL_UTF8
  if (!(c == 0x09 || c == 0x0A || c == 0x0D || (c >= 0x80 && c <= 0xD7FF) || (c >= 0xE000 && c <= 0xFFFD) || (c >= 0x10000 && c <= 0x10FFFF)))
    c = SOAP_UNKNOWN_UNICODE_CHAR;
#endif
#ifndef WITH_LEAN
  if (c > 0x9F)
  {
    char *t = tmp;
    if (c < 0x0800)
    {
      *t++ = (char)(0xC0 | ((c >> 6) & 0x1F));
    }
    else
    {
      if (c < 0x010000)
      {
        *t++ = (char)(0xE0 | ((c >> 12) & 0x0F));
      }
      else
      {
        if (c < 0x200000)
        {
          *t++ = (char)(0xF0 | ((c >> 18) & 0x07));
        }
        else
        {
          if (c < 0x04000000)
          {
            *t++ = (char)(0xF8 | ((c >> 24) & 0x03));
          }
          else
          {
            *t++ = (char)(0xFC | ((c >> 30) & 0x01));
            *t++ = (char)(0x80 | ((c >> 24) & 0x3F));
          }
          *t++ = (char)(0x80 | ((c >> 18) & 0x3F));
        }
        *t++ = (char)(0x80 | ((c >> 12) & 0x3F));
      }
      *t++ = (char)(0x80 | ((c >> 6) & 0x3F));
    }
    *t++ = (char)(0x80 | (c & 0x3F));
    *t = '\0';
  }
  else
#endif
    (SOAP_SNPRINTF(tmp, sizeof(tmp), 20), "&#x%lX;", c);
  return soap_send(soap, tmp);
}

/******************************************************************************/

SOAP_FMAC1
soap_wchar
SOAP_FMAC2
soap_getutf8(struct soap *soap)
{
#ifdef WITH_REPLACE_ILLEGAL_UTF8
  soap_wchar c, c1, c2, c3;
#else
  soap_wchar c, c1, c2, c3, c4;
#endif
  c = soap->ahead;
  if (c >= 0x80)
    soap->ahead = 0;
  else
    c = (soap_wchar)soap_get(soap);
  if (c < 0x80 || c > 0xFF || (soap->mode & SOAP_ENC_LATIN))
    return c;
#ifdef WITH_REPLACE_ILLEGAL_UTF8
  c1 = (soap_wchar)soap_get1(soap);
  if (c <= 0xC1 || (c1 & 0xC0) != 0x80)
  {
    soap_revget1(soap);
    return SOAP_UNKNOWN_UNICODE_CHAR;
  }
  c1 &= 0x3F;
  if (c < 0xE0)
    return (((c & 0x1F) << 6) | c1);
  c2 = (soap_wchar)soap_get1(soap);
  if ((c == 0xE0 && c1 < 0x20) || (c2 & 0xC0) != 0x80)
  {
    soap_revget1(soap);
    return SOAP_UNKNOWN_UNICODE_CHAR;
  }
  c2 &= 0x3F;
  if (c < 0xF0)
    return (((c & 0x0F) << 12) | (c1 << 6) | c2);
  c3 = (soap_wchar)soap_get1(soap);
  if ((c == 0xF0 && c1 < 0x10) || (c == 0xF4 && c1 >= 0x10) || c >= 0xF5 || (c3 & 0xC0) != 0x80)
  {
    soap_revget1(soap);
    return SOAP_UNKNOWN_UNICODE_CHAR;
  }
  return (((c & 0x07) << 18) | (c1 << 12) | (c2 << 6) | (c3 & 0x3F));
#else
  c1 = (soap_wchar)soap_get1(soap);
  if (c < 0xC0 || (c1 & 0xC0) != 0x80)
  {
    soap_revget1(soap);
    /* doesn't look like this is UTF-8, try continue as if ISO-8859-1 */
    return c;
  }
  c1 &= 0x3F;
  if (c < 0xE0)
    return ((soap_wchar)(c & 0x1F) << 6) | c1;
  c2 = (soap_wchar)soap_get1(soap) & 0x3F;
  if (c < 0xF0)
    return ((soap_wchar)(c & 0x0F) << 12) | (c1 << 6) | c2;
  c3 = (soap_wchar)soap_get1(soap) & 0x3F;
  if (c < 0xF8)
    return ((soap_wchar)(c & 0x07) << 18) | (c1 << 12) | (c2 << 6) | c3;
  c4 = (soap_wchar)soap_get1(soap) & 0x3F;
  if (c < 0xFC)
    return ((soap_wchar)(c & 0x03) << 24) | (c1 << 18) | (c2 << 12) | (c3 << 6) | c4;
  return ((soap_wchar)(c & 0x01) << 30) | (c1 << 24) | (c2 << 18) | (c3 << 12) | (c4 << 6) | (soap_wchar)(soap_get1(soap) & 0x3F);
#endif
}

/******************************************************************************/

SOAP_FMAC1
size_t
SOAP_FMAC2
soap_utf8len(const char *s)
{
  size_t l = 0;
  while (*s)
    if ((*s++ & 0xC0) != 0x80)
      l++;
  return l;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_puthex(struct soap *soap, const unsigned char *s, int n)
{
  char d[2 * SOAP_BINARY_BUFLEN], *p = d;
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    soap->dom->text = soap_s2hex(soap, s, NULL, n);
    if (!soap->dom->text)
      return soap->error;
    return SOAP_OK;
  }
#endif
  for (; n > 0; n--)
  {
    int m = *s++;
    p[0] = (char)((m >> 4) + (m > 159 ? '7' : '0'));
    m &= 0x0F;
    p[1] = (char)(m + (m > 9 ? '7' : '0'));
    p += 2;
    if (p - d == sizeof(d))
    {
      if (soap_send_raw(soap, d, sizeof(d)))
        return soap->error;
      p = d;
    }
  }
  if (p != d && soap_send_raw(soap, d, p - d))
    return soap->error;
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
unsigned char*
SOAP_FMAC2
soap_gethex(struct soap *soap, int *n)
{
  size_t l = 0;
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    soap->dom->text = soap_string_in(soap, 0, -1, -1, NULL);
    return (unsigned char*)soap_hex2s(soap, soap->dom->text, NULL, 0, n);
  }
#endif
#ifdef WITH_FAST
  soap->labidx = 0;
  for (;;)
  {
    char *s;
    size_t i, k;
    if (soap_append_lab(soap, NULL, 0))
      return NULL;
    s = soap->labbuf + soap->labidx;
    k = soap->lablen - soap->labidx;
    soap->labidx = soap->lablen;
    for (i = 0; i < k; i++)
    {
      char d1, d2;
      soap_wchar c;
      c = soap_get(soap);
      if (soap_isxdigit(c))
      {
        d1 = (char)c;
        c = soap_get(soap);
        if (soap_isxdigit(c))
        {
          d2 = (char)c;
        }
        else
        {
          soap->error = SOAP_TYPE;
          return NULL;
        }
      }
      else
      {
        unsigned char *p = NULL;
        l = soap->lablen + i - k;
        soap_unget(soap, c);
        if (n)
          *n = (int)l;
        if (soap->maxlength > 0 && l > (size_t)soap->maxlength)
        {
          soap->error = SOAP_LENGTH;
        }
        else
        {
          p = (unsigned char*)soap_malloc(soap, l);
          if (p)
            (void)soap_memcpy((void*)p, l, (const void*)soap->labbuf, l);
        }
        return p;
      }
      *s++ = (char)(((d1 >= 'A' ? (d1 & 0x7) + 9 : d1 - '0') << 4) + (d2 >= 'A' ? (d2 & 0x7) + 9 : d2 - '0'));
    }
    l = soap->lablen;
    if (soap->maxlength > 0 && l > (size_t)soap->maxlength)
    {
      soap->error = SOAP_LENGTH;
      return NULL;
    }
  }
#else
  if (soap_alloc_block(soap) == NULL)
    return NULL;
  for (;;)
  {
    int i;
    char *s = (char*)soap_push_block(soap, NULL, SOAP_BLKLEN);
    if (!s)
    {
      soap_end_block(soap, NULL);
      return NULL;
    }
    for (i = 0; i < SOAP_BLKLEN; i++)
    {
      char d1, d2;
      soap_wchar c = soap_get(soap);
      if (soap_isxdigit(c))
      {
        d1 = (char)c;
        c = soap_get(soap);
        if (soap_isxdigit(c))
          d2 = (char)c;
        else
        {
          soap_end_block(soap, NULL);
          soap->error = SOAP_TYPE;
          return NULL;
        }
      }
      else
      {
        unsigned char *p;
        soap_unget(soap, c);
        if (soap->maxlength > 0 && l > (size_t)soap->maxlength)
        {
          soap->error = SOAP_LENGTH;
          soap_end_block(soap, NULL);
          return NULL;
        }
        if (n)
          *n = (int)soap_size_block(soap, NULL, i);
        p = (unsigned char*)soap_save_block(soap, NULL, NULL, 0);
        return p;
      }
      *s++ = ((d1 >= 'A' ? (d1 & 0x7) + 9 : d1 - '0') << 4) + (d2 >= 'A' ? (d2 & 0x7) + 9 : d2 - '0');
      l++;
    }
    if (soap->maxlength > 0 && l > (size_t)soap->maxlength)
    {
      soap->error = SOAP_LENGTH;
      soap_end_block(soap, NULL);
      return NULL;
    }
  }
#endif
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_putbase64(struct soap *soap, const unsigned char *s, int n)
{
  char d[4 * SOAP_BINARY_BUFLEN], *p = d;
  if (!s)
    return SOAP_OK;
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    soap->dom->text = soap_s2base64(soap, s, NULL, n);
    if (!soap->dom->text)
      return soap->error;
    return SOAP_OK;
  }
#endif
  for (; n > 2; n -= 3, s += 3)
  {
    p[0] = soap_base64o[(s[0] & 0xFC) >> 2];
    p[1] = soap_base64o[((s[0] & 0x03) << 4) | ((s[1] & 0xF0) >> 4)];
    p[2] = soap_base64o[((s[1] & 0x0F) << 2) | ((s[2] & 0xC0) >> 6)];
    p[3] = soap_base64o[s[2] & 0x3F];
    p += 4;
    if (p - d == sizeof(d))
    {
      if (soap_send_raw(soap, d, sizeof(d)))
        return soap->error;
      p = d;
    }
  }
  if (n == 2)
  {
    p[0] = soap_base64o[(s[0] & 0xFC) >> 2];
    p[1] = soap_base64o[((s[0] & 0x03) << 4) | ((s[1] & 0xF0) >> 4)];
    p[2] = soap_base64o[(s[1] & 0x0F) << 2];
    p[3] = '=';
    p += 4;
  }
  else if (n == 1)
  {
    p[0] = soap_base64o[(s[0] & 0xFC) >> 2];
    p[1] = soap_base64o[(s[0] & 0x03) << 4];
    p[2] = '=';
    p[3] = '=';
    p += 4;
  }
  if (p != d && soap_send_raw(soap, d, p - d))
    return soap->error;
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
unsigned char*
SOAP_FMAC2
soap_getbase64(struct soap *soap, int *n, int malloc_flag)
{
  size_t l = 0;
  (void)malloc_flag;
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    soap->dom->text = soap_string_in(soap, 0, -1, -1, NULL);
    return (unsigned char*)soap_base642s(soap, soap->dom->text, NULL, 0, n);
  }
#endif
#ifdef WITH_FAST
  soap->labidx = 0;
  for (;;)
  {
    size_t i, k;
    char *s;
    if (soap_append_lab(soap, NULL, 2))
      return NULL;
    s = soap->labbuf + soap->labidx;
    k = soap->lablen - soap->labidx;
    soap->labidx = 3 * (soap->lablen / 3);
    if (k > 2)
    {
      for (i = 0; i < k - 2; i += 3)
      {
        unsigned long m = 0;
        int j = 0;
        do
        {
          soap_wchar c = soap_get(soap);
          if (c < SOAP_AP)
            c &= 0x7FFFFFFF;
          if (c == '=' || c < 0)
          {
            unsigned char *p = NULL;
            switch (j)
            {
              case 2:
                *s++ = (char)((m >> 4) & 0xFF);
                i++;
                break;
              case 3:
                *s++ = (char)((m >> 10) & 0xFF);
                *s++ = (char)((m >> 2) & 0xFF);
                i += 2;
            }
            l = soap->lablen + i - k;
            if (n)
              *n = (int)l;
            if (soap->maxlength > 0 && l > (size_t)soap->maxlength)
              soap->error = SOAP_LENGTH;
            else
            {
              p = (unsigned char*)soap_malloc(soap, l);
              if (p)
                (void)soap_memcpy((void*)p, l, (const void*)soap->labbuf, l);
            }
            if (c >= 0)
            {
              while ((int)((c = soap_get(soap)) != EOF) && c != SOAP_LT && c != SOAP_TT)
                continue;
            }
            soap_unget(soap, c);
            return p;
          }
          c -= '+';
          if (c >= 0 && c <= 79)
          {
            int b = soap_base64i[c];
            if (b >= 64)
            {
              soap->error = SOAP_TYPE;
              return NULL;
            }
            m = (m << 6) + b;
            j++;
          }
          else if (!soap_coblank(c + '+'))
          {
            soap->error = SOAP_TYPE;
            return NULL;
          }
        } while (j < 4);
        *s++ = (char)((m >> 16) & 0xFF);
        *s++ = (char)((m >> 8) & 0xFF);
        *s++ = (char)(m & 0xFF);
      }
      l = soap->lablen;
      if (soap->maxlength > 0 && l > (size_t)soap->maxlength)
      {
        soap->error = SOAP_LENGTH;
        return NULL;
      }
    }
  }
#else
  if (soap_alloc_block(soap) == NULL)
    return NULL;
  for (;;)
  {
    int i;
    char *s = (char*)soap_push_block(soap, NULL, 3 * SOAP_BLKLEN); /* must be multiple of 3 */
    if (!s)
    {
      soap_end_block(soap, NULL);
      return NULL;
    }
    for (i = 0; i < SOAP_BLKLEN; i++)
    {
      unsigned long m = 0;
      int j = 0;
      do
      {
        soap_wchar c = soap_get(soap);
        if (c < SOAP_AP)
          c &= 0x7FFFFFFF;
        if (c == '=' || c < 0)
        {
          unsigned char *p;
          i *= 3;
          switch (j)
          {
            case 2:
              *s++ = (char)((m >> 4) & 0xFF);
              i++;
              l++;
              break;
            case 3:
              *s++ = (char)((m >> 10) & 0xFF);
              *s++ = (char)((m >> 2) & 0xFF);
              l += 2;
              i += 2;
          }
          if (n)
            *n = (int)soap_size_block(soap, NULL, i);
          if (soap->maxlength > 0 && l > (size_t)soap->maxlength)
          {
            soap->error = SOAP_LENGTH;
            soap_end_block(soap, NULL);
            return NULL;
          }
          p = (unsigned char*)soap_save_block(soap, NULL, NULL, 0);
          if (c >= 0)
          {
            while ((int)((c = soap_get(soap)) != EOF) && c != SOAP_LT && c != SOAP_TT)
              continue;
          }
          soap_unget(soap, c);
          return p;
        }
        c -= '+';
        if (c >= 0 && c <= 79)
        {
          int b = soap_base64i[c];
          if (b >= 64)
          {
            soap->error = SOAP_TYPE;
            return NULL;
          }
          m = (m << 6) + b;
          j++;
        }
        else if (!soap_coblank(c + '+'))
        {
          soap->error = SOAP_TYPE;
          return NULL;
        }
      } while (j < 4);
      *s++ = (char)((m >> 16) & 0xFF);
      *s++ = (char)((m >> 8) & 0xFF);
      *s++ = (char)(m & 0xFF);
      l += 3;
    }
    if (soap->maxlength > 0 && l > (size_t)soap->maxlength)
    {
      soap->error = SOAP_LENGTH;
      soap_end_block(soap, NULL);
      return NULL;
    }
  }
#endif
}

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_xop_forward(struct soap *soap, unsigned char **ptr, int *size, char **id, char **type, char **options)
{
  /* Check MTOM xop:Include element (within hex/base64Binary) */
  /* TODO: this code to be obsoleted with new import/xop.h conventions */
  short body = soap->body; /* should save type too? */
  if (!soap_peek_element(soap))
  {
    if (!soap_element_begin_in(soap, ":Include", 0, NULL))
    {
      if (soap_attachment_forward(soap, ptr, size, id, type, options)
       || (soap->body && soap_element_end_in(soap, ":Include")))
        return soap->error;
    }
    else if (soap->error == SOAP_TAG_MISMATCH)
      soap_retry(soap);
    else
      return soap->error;
  }
  soap->body = body;
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_attachment_forward(struct soap *soap, unsigned char **ptr, int *size, char **id, char **type, char **options)
{
  struct soap_xlist *xp;
  *ptr = NULL;
  *size = 0;
  *id = NULL;
  *type = NULL;
  *options = NULL;
  if (!*soap->href)
    return SOAP_OK;
  *id = soap_strdup(soap, soap->href);
  xp = (struct soap_xlist*)SOAP_MALLOC(soap, sizeof(struct soap_xlist));
  if (!xp)
    return soap->error = SOAP_EOM;
  xp->next = soap->xlist;
  xp->ptr = ptr;
  xp->size = size;
  xp->id = *id;
  xp->type = type;
  xp->options = options;
  soap->xlist = xp;
  return SOAP_OK;
}
#endif

/******************************************************************************/

SOAP_FMAC1
void *
SOAP_FMAC2
soap_memdup(struct soap *soap, const void *s, size_t n)
{
  void *t = NULL;
  if (s)
  {
    t = soap_malloc(soap, n);
    if (t)
      (void)soap_memcpy(t, n, s, n);
  }
  return t;
}

/******************************************************************************/

SOAP_FMAC1
char *
SOAP_FMAC2
soap_strdup(struct soap *soap, const char *s)
{
  char *t = NULL;
  if (s)
  {
    size_t n = strlen(s) + 1;
    if (n > 0)
    {
      t = (char*)soap_malloc(soap, n);
      if (t)
      {
        (void)soap_memcpy((void*)t, n, (const void*)s, n);
        t[n - 1] = '\0';
      }
    }
  }
  return t;
}

/******************************************************************************/

SOAP_FMAC1
wchar_t *
SOAP_FMAC2
soap_wstrdup(struct soap *soap, const wchar_t *s)
{
  wchar_t *t = NULL;
  if (s)
  {
    size_t n = 0, m;
    while (s[n])
      n++;
    n++;
    m = sizeof(wchar_t) * n;
    if (n > 0)
    {
      t = (wchar_t*)soap_malloc(soap, m);
      if (t)
      {
        (void)soap_memcpy((void*)t, m, (const void*)s, m);
        t[n - 1] = L'\0';
      }
    }
  }
  return t;
}

/******************************************************************************/

SOAP_FMAC1
char *
SOAP_FMAC2
soap_strtrim(struct soap *soap, char *s)
{
  (void)soap;
  if (s)
  {
    char *t;
    while ((*s >= 9 && *s <= 13) || *s == 32)
      s++;
    t = s;
    while (*t)
      t++;
    while (--t > s && ((*t >= 9 && *t <= 13) || *t == 32))
      continue;
    t[1] = '\0';
  }
  return s;
}

/******************************************************************************/

SOAP_FMAC1
wchar_t *
SOAP_FMAC2
soap_wstrtrim(struct soap *soap, wchar_t *s)
{
  (void)soap;
  if (s)
  {
    wchar_t *t;
    while ((*s >= 9 && *s <= 13) || *s == 32)
      s++;
    t = s;
    while (*t)
      t++;
    while (--t > s && ((*t >= 9 && *t <= 13) || *t == 32))
      continue;
    t[1] = L'\0';
  }
  return s;
}

/******************************************************************************/

SOAP_FMAC1
struct soap_blist*
SOAP_FMAC2
soap_alloc_block(struct soap *soap)
{
  struct soap_blist *p;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "New block sequence (prev=%p)\n", (void*)soap->blist));
  p = (struct soap_blist*)SOAP_MALLOC(soap, sizeof(struct soap_blist));
  if (!p)
  {
    soap->error = SOAP_EOM;
    return NULL;
  }
  p->next = soap->blist;
  p->head = NULL;
  p->size = 0;
  p->item = 0;
  soap->blist = p;
  return p;
}

/******************************************************************************/

SOAP_FMAC1
void*
SOAP_FMAC2
soap_push_block(struct soap *soap, struct soap_blist *b, size_t n)
{
  struct soap_bhead *p;
  if (!b)
    b = soap->blist;
  if (!b
   || b->size + n < b->size
   || sizeof(struct soap_bhead) + n < n
   || (SOAP_MAXALLOCSIZE > 0 && sizeof(struct soap_bhead) + n > SOAP_MAXALLOCSIZE))
  {
    soap->error = SOAP_EOM;
    return NULL;
  }
  p = (struct soap_bhead*)SOAP_MALLOC(soap, sizeof(struct soap_bhead) + n);
  if (!p)
  {
    soap->error = SOAP_EOM;
    return NULL;
  }
  p->next = b->head;
  b->head = p;
  p->size = n;
  b->size += n;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Push block %p of %u bytes on %lu previous blocks (%lu bytes total)\n", (void*)p, (unsigned int)n, (unsigned long)b->item, (unsigned long)b->size));
  b->item++;
  return (void*)(p + 1); /* skip block header and point to n allocated bytes */
}

/******************************************************************************/

SOAP_FMAC1
void*
SOAP_FMAC2
soap_push_block_max(struct soap *soap, struct soap_blist *b, size_t n)
{
  if (b && b->item >= soap->maxoccurs) /* restrict block array length */
  {
    soap->error = SOAP_OCCURS;
    return NULL;
  }
  return soap_push_block(soap, b, n);
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_pop_block(struct soap *soap, struct soap_blist *b)
{
  struct soap_bhead *p;
  if (!b)
    b = soap->blist;
  if (!b || !b->head)
    return;
  p = b->head;
  b->size -= p->size;
  b->head = p->next;
  b->item--;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Pop block %p (%lu items of %lu bytes total)\n", (void*)p, (unsigned long)b->item, (unsigned long)b->size));
  SOAP_FREE(soap, p);
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_update_pointers(struct soap *soap, const char *dst, const char *src, size_t len)
{
  const void *start = src, *end = src + len;
#ifndef WITH_LEANER
  struct soap_xlist *xp;
#endif
#ifndef WITH_NOIDREF
  if ((soap->version && !(soap->imode & SOAP_XML_TREE)) || (soap->mode & SOAP_XML_GRAPH))
  {
    int i;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Update pointers %p (%lu bytes) -> %p\n", (void*)src, (unsigned long)len, (void*)dst));
    for (i = 0; i < SOAP_IDHASH; i++)
    {
      struct soap_ilist *ip;
      for (ip = soap->iht[i]; ip; ip = ip->next)
      {
        struct soap_flist *fp;
        void *p, **q;
        if (ip->shaky)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Update shaky id='%s'\n", ip->id));
          if (ip->ptr && ip->ptr >= start && ip->ptr < end)
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Update ptr %p -> %p\n", ip->ptr, (void*)((const char*)ip->ptr + (dst-src))));
            ip->ptr = (void*)((const char*)ip->ptr + (dst-src));
          }
          for (q = &ip->link; q; q = (void**)p)
          {
            p = *q;
            if (p && p >= start && p < end)
            {
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Link update id='%s' %p -> %p\n", ip->id, p, (void*)((const char*)p + (dst-src))));
              *q = (void*)((const char*)p + (dst-src));
            }
          }
          for (q = &ip->copy; q; q = (void**)p)
          {
            p = *q;
            if (p && p >= start && p < end)
            {
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Copy chain update id='%s' %p -> %p\n", ip->id, p, (void*)((const char*)p + (dst-src))));
              *q = (void*)((const char*)p + (dst-src));
            }
          }
          for (fp = ip->flist; fp; fp = fp->next)
          {
            if (fp->ptr >= start && fp->ptr < end)
            {
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Copy list update id='%s' target type=%d %p -> %p\n", ip->id, fp->type, fp->ptr, (void*)((char*)fp->ptr + (dst-src))));
              fp->ptr = (void*)((const char*)fp->ptr + (dst-src));
            }
          }
          if (ip->smart && ip->smart >= start && ip->smart < end)
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Smart shared pointer update %p -> %p\n", ip->smart, (void*)((const char*)ip->smart + (dst-src))));
            ip->smart = (void*)((const char*)ip->smart + (dst-src));
          }
        }
      }
    }
  }
#else
  (void)soap; (void)start; (void)end; (void)dst; (void)src;
#endif
#ifndef WITH_LEANER
  for (xp = soap->xlist; xp; xp = xp->next)
  {
    if (xp->ptr && (void*)xp->ptr >= start && (void*)xp->ptr < end)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Update attachment id='%s' %p -> %p\n", xp->id ? xp->id : SOAP_STR_EOS, (void*)xp->ptr, (void*)((char*)xp->ptr + (dst-src))));
      xp->ptr = (unsigned char**)((char*)xp->ptr + (dst-src));
      xp->size = (int*)((char*)xp->size + (dst-src));
      xp->type = (char**)((char*)xp->type + (dst-src));
      xp->options = (char**)((char*)xp->options + (dst-src));
    }
  }
#endif
}

/******************************************************************************/

#ifndef WITH_NOIDREF
static int
soap_has_copies(struct soap *soap, const char *start, const char *end)
{
  int i;
  struct soap_ilist *ip = NULL;
  struct soap_flist *fp = NULL;
  const char *p;
  for (i = 0; i < SOAP_IDHASH; i++)
  {
    for (ip = soap->iht[i]; ip; ip = ip->next)
    {
      for (p = (const char*)ip->copy; p; p = *(const char**)p)
        if (p >= start && p < end)
          return SOAP_ERR;
      for (fp = ip->flist; fp; fp = fp->next)
        if (fp->type == ip->type && (const char*)fp->ptr >= start && (const char*)fp->ptr < end)
          return SOAP_ERR;
    }
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_resolve(struct soap *soap)
{
  int i;
  short flag;
  const char *id;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Resolving forwarded refs\n"));
  for (i = 0; i < SOAP_IDHASH; i++)
  {
    struct soap_ilist *ip;
    for (ip = soap->iht[i]; ip; ip = ip->next)
    {
      if (ip->ptr)
      {
        void **q;
        struct soap_flist *fp, **fpp = &ip->flist;
        if (ip->spine)
          ip->spine[0] = ip->ptr;
        q = (void**)ip->link;
        ip->link = NULL;
        DBGLOG(TEST, if (q) SOAP_MESSAGE(fdebug, "Traversing link chain to resolve id='%s' type=%d\n", ip->id, ip->type));
        while (q)
        {
          void *p = *q;
          *q = ip->ptr;
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "... link %p -> %p\n", (void*)q, ip->ptr));
          q = (void**)p;
        }
        while ((fp = *fpp))
        {
          if (fp->level > 0 && fp->finsert)
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "... insert type=%d link %p -> %p\n", fp->type, fp->ptr, ip->ptr));
            if (ip->spine && fp->level <= SOAP_MAXPTRS)
              fp->finsert(soap, ip->type, fp->type, fp->ptr, fp->index, &ip->spine[fp->level - 1], &ip->smart);
            else if (fp->level == 1)
              fp->finsert(soap, ip->type, fp->type, fp->ptr, fp->index, &ip->ptr, &ip->smart);
            else if (fp->level <= SOAP_MAXPTRS)
            {
              int i;
              ip->spine = (void**)soap_malloc(soap, SOAP_MAXPTRS * sizeof(void*));
              if (!ip->spine)
                return soap->error = SOAP_EOM;
              ip->spine[0] = ip->ptr;
              for (i = 1; i < SOAP_MAXPTRS; i++)
                ip->spine[i] = &ip->spine[i - 1];
              fp->finsert(soap, ip->type, fp->type, fp->ptr, fp->index, &ip->spine[fp->level - 1], &ip->smart);
            }
            *fpp = fp->next;
            SOAP_FREE(soap, fp);
          }
          else
            fpp = &fp->next;
        }
      }
      else if (*ip->id == '#')
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Missing id='%s'\n", ip->id));
        soap_strcpy(soap->id, sizeof(soap->id), ip->id + 1);
        return soap->error = SOAP_MISSING_ID;
      }
    }
  }
  do
  {
    flag = 0;
    id = NULL;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Resolution phase\n"));
    for (i = 0; i < SOAP_IDHASH; i++)
    {
      struct soap_ilist *ip;
      for (ip = soap->iht[i]; ip; ip = ip->next)
      {
        if (ip->copy || ip->flist)
        {
          if (ip->ptr && !soap_has_copies(soap, (const char*)ip->ptr, (const char*)ip->ptr + ip->size))
          {
            struct soap_flist *fp;
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Resolving id='%s' type=%d ptr=%p size=%lu ...\n", ip->id, ip->type, ip->ptr, (unsigned long)ip->size));
            if (ip->copy)
            {
              void *p, **q = (void**)ip->copy;
              DBGLOG(TEST, if (q) SOAP_MESSAGE(fdebug, "Traversing copy chain to resolve id='%s'\n", ip->id));
              ip->copy = NULL;
              do
              {
                DBGLOG(TEST, SOAP_MESSAGE(fdebug, "... copy %p -> %p (%u bytes)\n", ip->ptr, (void*)q, (unsigned int)ip->size));
                p = *q;
                (void)soap_memcpy((void*)q, ip->size, (const void*)ip->ptr, ip->size);
                q = (void**)p;
              } while (q);
              flag = 1;
            }
            while ((fp = ip->flist))
            {
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Resolving forwarded data type=%d target type=%d location=%p level=%u id='%s'\n", ip->type, fp->type, ip->ptr, fp->level, ip->id));
              if (fp->level == 0)
              {
                DBGLOG(TEST, SOAP_MESSAGE(fdebug, "... copy %p -> %p (%lu bytes)\n", ip->ptr, fp->ptr, (unsigned long)ip->size));
                if (fp->finsert)
                  fp->finsert(soap, ip->type, fp->type, fp->ptr, fp->index, ip->ptr, &ip->smart);
                else
                  (void)soap_memcpy((void*)fp->ptr, ip->size, (const void*)ip->ptr, ip->size);
              }
              ip->flist = fp->next;
              SOAP_FREE(soap, fp);
              flag = 1;
            }
          }
          else if (*ip->id == '#')
            id = ip->id;
        }
      }
    }
  } while (flag);
  if (id)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Resolution error: forwarded data for id='%s' could not be propagated, please report this problem to the gSOAP developers\n", id));
    return soap_id_nullify(soap, id);
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Resolution done\n"));
  return SOAP_OK;
}
#endif

/******************************************************************************/

SOAP_FMAC1
size_t
SOAP_FMAC2
soap_size_block(struct soap *soap, struct soap_blist *b, size_t n)
{
  if (!b)
    b = soap->blist;
  if (b->head)
  {
    b->size -= b->head->size - n;
    b->head->size = n;
  }
  return b->size;
}

/******************************************************************************/

SOAP_FMAC1
char*
SOAP_FMAC2
soap_first_block(struct soap *soap, struct soap_blist *b)
{
  struct soap_bhead *p, *q, *r;
  if (!b)
    b = soap->blist;
  p = b->head;
  if (!p)
    return NULL;
  r = NULL;
  do
  {
    q = p->next;
    p->next = r;
    r = p;
    p = q;
  } while (p);
  b->head = r;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "First block %p\n", (void*)(r + 1)));
  return (char*)(r + 1);
}

/******************************************************************************/

SOAP_FMAC1
char*
SOAP_FMAC2
soap_next_block(struct soap *soap, struct soap_blist *b)
{
  struct soap_bhead *p;
  if (!b)
    b = soap->blist;
  p = b->head;
  if (p)
  {
    b->head = p->next;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Next block %p, deleting current block\n", (void*)(b->head ? b->head + 1 : NULL)));
    SOAP_FREE(soap, p);
    if (b->head)
      return (char*)(b->head + 1);
  }
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
size_t
SOAP_FMAC2
soap_block_size(struct soap *soap, struct soap_blist *b)
{
  if (!b)
    b = soap->blist;
  return b->head->size;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_end_block(struct soap *soap, struct soap_blist *b)
{
  struct soap_bhead *p, *q;
  if (!b)
    b = soap->blist;
  if (b)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "End of block sequence, free all remaining blocks\n"));
    for (p = b->head; p; p = q)
    {
      q = p->next;
      SOAP_FREE(soap, p);
    }
    if (soap->blist == b)
      soap->blist = b->next;
    else
    {
      struct soap_blist *bp;
      for (bp = soap->blist; bp; bp = bp->next)
      {
        if (bp->next == b)
        {
          bp->next = b->next;
          break;
        }
      }
    }
    SOAP_FREE(soap, b);
  }
  DBGLOG(TEST, if (soap->blist) SOAP_MESSAGE(fdebug, "Restored previous block sequence\n"));
#ifndef WITH_NOIDREF
  if (!soap->blist && ((soap->version && !(soap->imode & SOAP_XML_TREE)) || (soap->mode & SOAP_XML_GRAPH)))
  {
    int i;
    struct soap_ilist *ip = NULL;
    for (i = 0; i < SOAP_IDHASH; i++)
      for (ip = soap->iht[i]; ip; ip = ip->next)
        ip->shaky = 0;
  }
#endif
}

/******************************************************************************/

SOAP_FMAC1
void*
SOAP_FMAC2
soap_save_block(struct soap *soap, struct soap_blist *b, char *p, int flag)
{
  size_t n;
  char *q, *s;
  if (!b)
    b = soap->blist;
  if (b->size)
  {
    if (!p)
      p = (char*)soap_malloc(soap, b->size);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Save all %lu blocks in contiguous memory space of %u bytes (%p->%p)\n", (unsigned long)b->item, (unsigned int)b->size, (void*)b->head, (void*)p));
    if (p)
    {
      s = p;
      for (q = soap_first_block(soap, b); q; q = soap_next_block(soap, b))
      {
        n = soap_block_size(soap, b);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Copy %u bytes from %p to %p\n", (unsigned int)n, (void*)q, (void*)s));
        if (flag)
          soap_update_pointers(soap, s, q, n);
        (void)soap_memcpy((void*)s, n, (const void*)q, n);
        s += n;
      }
    }
    else
      soap->error = SOAP_EOM;
  }
  soap_end_block(soap, b);
  return p;
}

/******************************************************************************/

SOAP_FMAC1
char *
SOAP_FMAC2
soap_putsizesoffsets(struct soap *soap, const char *type, const int *size, const int *offset, int dim)
{
  int i;
  const char *t = ",%d";
  if (!type)
    return NULL;
  if (soap->version == 2)
    t = " %d";
  if (soap->version != 2 && offset)
  {
    (SOAP_SNPRINTF(soap->type, sizeof(soap->type) - 1, strlen(type) + 20), "%s[%d", type, size[0] + offset[0]);
    for (i = 1; i < dim; i++)
    {
      size_t l = strlen(soap->type);
      (SOAP_SNPRINTF(soap->type + l, sizeof(soap->type) - l - 1, 20), t, size[i] + offset[i]);
    }
  }
  else
  {
    (SOAP_SNPRINTF(soap->type, sizeof(soap->type) - 1, strlen(type) + 20), "%s[%d", type, size[0]);
    for (i = 1; i < dim; i++)
    {
      size_t l = strlen(soap->type);
      (SOAP_SNPRINTF(soap->type + l, sizeof(soap->type) - l - 1, 20), t, size[i]);
    }
  }
  soap_strcat(soap->type, sizeof(soap->type), "]");
  return soap->type;
}

/******************************************************************************/

SOAP_FMAC1
char *
SOAP_FMAC2
soap_putoffsets(struct soap *soap, const int *offset, int dim)
{
  int i;
  soap->arrayOffset[0] = '\0';
  if (soap->version == 1)
  {
    (SOAP_SNPRINTF(soap->arrayOffset, sizeof(soap->arrayOffset) - 1, 20), "[%d", offset[0]);
    for (i = 1; i < dim; i++)
    {
      size_t l = strlen(soap->arrayOffset);
      (SOAP_SNPRINTF(soap->arrayOffset + l, sizeof(soap->arrayOffset) - l - 1, 20), ",%d", offset[i]);
    }
    soap_strcat(soap->arrayOffset, sizeof(soap->arrayOffset), "]");
  }
  return soap->arrayOffset;
}

/******************************************************************************/

SOAP_FMAC1
size_t
SOAP_FMAC2
soap_size(const int *size, int dim)
{
  int i;
  size_t n = 0;
  if (size[0] <= 0)
    return 0;
  n = (size_t)size[0];
  for (i = 1; i < dim; i++)
  {
    if (size[i] <= 0)
      return 0;
    n *= (size_t)size[i];
  }
  return (size_t)n;
}

/******************************************************************************/

SOAP_FMAC1
size_t
SOAP_FMAC2
soap_getsizes(const char *attr, int *size, int dim)
{
  size_t i, k, n;
  if (!*attr)
    return 0;
  i = strlen(attr);
  n = 1;
  do
  {
    for (; i > 0; i--)
      if (attr[i - 1] == '[' || attr[i - 1] == ',' || attr[i - 1] == ' ')
        break;
    n *= k = (size_t)soap_strtoul(attr + i, NULL, 10);
    size[--dim] = (int)k;
    if (n > SOAP_MAXARRAYSIZE)
      return 0;
  } while (dim > 0 && i-- > 1 && attr[i] != '[');
  return n;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_getoffsets(const char *attr, const int *size, int *offset, int dim)
{
  int i, j = 0;
  if (offset)
  {
    for (i = 0; i < dim && attr && *attr; i++)
    {
      attr++;
      j *= size[i];
      j += offset[i] = (int)soap_strtol(attr, NULL, 10);
      attr = strchr(attr, ',');
    }
  }
  else
  {
    for (i = 0; i < dim && attr && *attr; i++)
    {
      attr++;
      j *= size[i];
      j += (int)soap_strtol(attr, NULL, 10);
      attr = strchr(attr, ',');
    }
  }
  return j;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_getposition(const char *attr, int *pos)
{
  int i, n;
  if (!*attr)
    return -1;
  n = 0;
  i = 1;
  do
  {
    pos[n++] = (int)soap_strtol(attr + i, NULL, 10);
    while (attr[i] && attr[i] != ',' && attr[i] != ']')
      i++;
    if (attr[i] == ',')
      i++;
  } while (n < SOAP_MAXDIMS && attr[i] && attr[i] != ']');
  return n;
}

/******************************************************************************/

SOAP_FMAC1
struct soap_nlist *
SOAP_FMAC2
soap_push_namespace(struct soap *soap, const char *id, const char *ns)
{
  struct soap_nlist *np = NULL;
  struct Namespace *p;
  short i = -1;
  size_t n, k;
  n = strlen(id);
  k = strlen(ns) + 1;
  p = soap->local_namespaces;
  if (p)
  {
    for (i = 0; p->id; p++, i++)
    {
      if (p->ns && !strcmp(ns, p->ns))
        break;
      if (p->out)
      {
        if (!strcmp(ns, p->out))
          break;
      }
      else if (p->in)
      {
        if (!soap_tag_cmp(ns, p->in))
        {
          if (SOAP_MAXALLOCSIZE <= 0 || k <= SOAP_MAXALLOCSIZE)
            p->out = (char*)SOAP_MALLOC(soap, k);
          if (p->out)
            soap_strcpy(p->out, k, ns);
          break;
        }
      }
    }
    if (!p->id)
      i = -1;
  }
  if (i >= 0)
    k = 0;
  if (sizeof(struct soap_nlist) + n + k > n && (SOAP_MAXALLOCSIZE <= 0 || sizeof(struct soap_nlist) + n + k <= SOAP_MAXALLOCSIZE))
    np = (struct soap_nlist*)SOAP_MALLOC(soap, sizeof(struct soap_nlist) + n + k);
  if (!np)
  {
    soap->error = SOAP_EOM;
    return NULL;
  }
  np->next = soap->nlist;
  soap->nlist = np;
  np->level = soap->level;
  np->index = i;
  soap_strcpy((char*)np->id, n + 1, id);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Push namespace binding (level=%u,index=%hd) '%s'='%s'\n", soap->level, i, id, ns));
  if (i < 0)
  {
    np->ns = np->id + n + 1;
    soap_strcpy((char*)np->ns, k, ns);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Push NOT OK: no match found for '%s' in namespace mapping table (added to stack anyway)\n", ns));
  }
  else
  {
    np->ns = NULL;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Push OK ('%s' matches '%s' in namespace table)\n", id, p->id));
  }
  return np;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_pop_namespace(struct soap *soap)
{
  struct soap_nlist *np, *nq;
  for (np = soap->nlist; np && np->level >= soap->level; np = nq)
  {
    nq = np->next;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Pop namespace binding (level=%u) '%s' level=%u\n", soap->level, np->id, np->level));
    SOAP_FREE(soap, np);
  }
  soap->nlist = np;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_match_namespace(struct soap *soap, const char *id1, const char *id2, size_t n1, size_t n2)
{
  struct soap_nlist *np = soap->nlist;
  const char *s;
  while (np && (strncmp(np->id, id1, n1) || np->id[n1]))
    np = np->next;
  if (np)
  {
    if (!(soap->mode & SOAP_XML_IGNORENS) && (n2 > 0 || !np->ns || *np->ns))
    {
      if (np->index < 0
       || ((s = soap->local_namespaces[np->index].id) && (strncmp(s, id2, n2) || (s[n2] && s[n2] != '_'))))
        return SOAP_NAMESPACE;
    }
    return SOAP_OK;
  }
  if (n1 == 0)
    return n2 == 0 || (soap->mode & SOAP_XML_IGNORENS) ? SOAP_OK : SOAP_NAMESPACE;
  if ((n1 == 3 && n1 == n2 && !strncmp(id1, "xml", 3) && !strncmp(id1, id2, 3))
   || (soap->mode & SOAP_XML_IGNORENS))
    return SOAP_OK;
  return soap->error = SOAP_SYNTAX_ERROR;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_current_namespace_tag(struct soap *soap, const char *tag)
{
  struct soap_nlist *np;
  const char *s;
  if (!tag || !strncmp(tag, "xml", 3))
    return NULL;
  np = soap->nlist;
  s = strchr(tag, ':');
  if (!s)
  {
    while (np && *np->id) /* find default namespace, if present */
      np = np->next;
  }
  else
  {
    while (np && (strncmp(np->id, tag, s - tag) || np->id[s - tag]))
      np = np->next;
    if (!np)
      soap->error = SOAP_NAMESPACE;
  }
  if (np)
  {
    if (np->index >= 0)
      return soap->namespaces[np->index].ns;
    if (np->ns)
    {
      s = np->ns;
      if (*s)
        return soap_strdup(soap, s);
      do
        np = np->next;
      while (np && *np->id); /* find if there is any other default namespace */
      if (np)
        return soap_strdup(soap, s);
    }
  }
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_current_namespace_att(struct soap *soap, const char *tag)
{
  struct soap_nlist *np;
  const char *s;
  if (!tag || !strncmp(tag, "xml", 3))
    return NULL;
  s = strchr(tag, ':');
  if (!s)
    return NULL;
  np = soap->nlist;
  while (np && (strncmp(np->id, tag, s - tag) || np->id[s - tag]))
    np = np->next;
  if (!np)
    soap->error = SOAP_NAMESPACE;
  if (np)
  {
    if (np->index >= 0)
      return soap->namespaces[np->index].ns;
    if (np->ns && *np->ns)
      return soap_strdup(soap, np->ns);
  }
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_tag_cmp(const char *s, const char *t)
{
  const char *a = NULL;
  const char *b = NULL;
  for (;;)
  {
    int c1 = *s;
    int c2 = *t;
    if (!c1 || c1 == '"')
      break;
    if (c2 != '-')
    {
      if (c1 < c2)
      {
        if (c1 >= 'A' && c1 <= 'Z')
          c1 += 'a' - 'A';
      }
      else if (c1 > c2)
      {
        if (c2 >= 'A' && c2 <= 'Z')
          c2 += 'a' - 'A';
      }
      if (c2 == '*')
      {
        c2 = *++t;
        if (!c2)
          return 0;
        a = s;
        b = t;
        continue;
      }
      if (c1 != c2)
      {
        if (!a)
          return 1;
        s = ++a;
        t = b;
        continue;
      }
    }
    s++;
    t++;
  }
  if (*t == '*' && !t[1])
    return 0;
  return *t;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_match_tag(struct soap *soap, const char *tag1, const char *tag2)
{
  const char *s, *t;
  int err;
  if (!tag1 || !tag2 || !*tag2)
    return SOAP_OK;
  s = strchr(tag1, ':');
  t = strchr(tag2, ':');
  if (t)
  {
    if (s)
    {
      if (t[1] && SOAP_STRCMP(s + 1, t + 1))
        return SOAP_TAG_MISMATCH;
      if (t != tag2 && !(soap->mode & SOAP_XML_IGNORENS))
      {
        err = soap_match_namespace(soap, tag1, tag2, s - tag1, t - tag2);
        if (err)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Tags '%s' and '%s' match but namespaces differ\n", tag1, tag2));
          if (err == SOAP_NAMESPACE)
            return SOAP_TAG_MISMATCH;
          return err;
        }
      }
    }
    else if (!t[1])
    {
      if ((soap->mode & SOAP_XML_IGNORENS) || soap_match_namespace(soap, tag1, tag2, 0, t - tag2))
        return SOAP_TAG_MISMATCH;
    }
    else if (SOAP_STRCMP(tag1, t + 1))
    {
      return SOAP_TAG_MISMATCH;
    }
    else if (t != tag2)
    {
      err = soap_match_namespace(soap, tag1, tag2, 0, t - tag2);
      if (err)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Tags '%s' and '%s' match but namespaces differ\n", tag1, tag2));
        if (err == SOAP_NAMESPACE)
          return SOAP_TAG_MISMATCH;
        return err;
      }
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Tags and (default) namespaces match: '%s' '%s'\n", tag1, tag2));
    return SOAP_OK;
  }
  if (s)
  {
    if (!(soap->mode & SOAP_XML_IGNORENS) || SOAP_STRCMP(s + 1, tag2)) /* always fails (except when ignoring ns) */
      return SOAP_TAG_MISMATCH;
  }
  else if (SOAP_STRCMP(tag1, tag2)
#ifndef WITH_NOEMPTYNAMESPACES
        || ((soap->mode & SOAP_XML_STRICT) && !(soap->mode & SOAP_XML_IGNORENS) && soap_match_namespace(soap, tag1, tag2, 0, 0)) /* strict checking: default namespace must be null namespace */
#endif
      )
  {
    return SOAP_TAG_MISMATCH;
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Tags match: '%s' '%s'\n", tag1, tag2));
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_match_att(struct soap *soap, const char *tag1, const char *tag2)
{
  const char *s, *t;
  int err;
  if (!tag1 || !tag2 || !*tag2)
    return SOAP_OK;
  s = strchr(tag1, ':');
  t = strchr(tag2, ':');
  if (t)
  {
    if (s)
    {
      if (t[1] && SOAP_STRCMP(s + 1, t + 1))
        return SOAP_TAG_MISMATCH;
      if (t != tag2 && !(soap->mode & SOAP_XML_IGNORENS))
      {
        err = soap_match_namespace(soap, tag1, tag2, s - tag1, t - tag2);
        if (err)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Atts '%s' and '%s' match but namespaces differ\n", tag1, tag2));
          if (err == SOAP_NAMESPACE)
            return SOAP_TAG_MISMATCH;
          return err;
        }
      }
    }
    else if (!t[1] || t != tag2 || SOAP_STRCMP(tag1, t + 1))
      return SOAP_TAG_MISMATCH;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Atts and (default) namespaces match: '%s' '%s'\n", tag1, tag2));
    return SOAP_OK;
  }
  if (s)
  {
    if (!(soap->mode & SOAP_XML_IGNORENS) || SOAP_STRCMP(s + 1, tag2)) /* always fails (except when ignoring ns) */
      return SOAP_TAG_MISMATCH;
  }
  else if (SOAP_STRCMP(tag1, tag2))
    return SOAP_TAG_MISMATCH;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Atts match: '%s' '%s'\n", tag1, tag2));
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_match_array(struct soap *soap, const char *type)
{
  if (type && *soap->arrayType)
  {
    if (soap->version == 1 || !strchr(type, '['))
    {
      if (soap_match_tag(soap, soap->arrayType, type)
        && soap_match_tag(soap, soap->arrayType, "xsd:anyType")
        && soap_match_tag(soap, soap->arrayType, "xsd:ur-type"))
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SOAP array type mismatch: '%s' '%s'\n", soap->arrayType, type));
        return SOAP_TAG_MISMATCH;
      }
    }
  }
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      SSL/TLS
 *
\******************************************************************************/

#ifdef WITH_OPENSSL
SOAP_FMAC1
int
SOAP_FMAC2
soap_rand()
{
  int r;
  if (!soap_ssl_init_done)
    soap_ssl_init();
#if OPENSSL_VERSION_NUMBER < 0x10100000L
  RAND_pseudo_bytes((unsigned char*)&r, sizeof(int));
#else
  RAND_bytes((unsigned char*)&r, sizeof(int));
#endif
  return r;
}
#endif

/******************************************************************************/

#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS) || defined(WITH_SYSTEMSSL)
SOAP_FMAC1
int
SOAP_FMAC2
#if defined(VXWORKS) && defined(WM_SECURE_KEY_STORAGE)
soap_ssl_server_context(struct soap *soap, unsigned short flags, const char *keyfile, const char *keyid, const char *password, const char *cafile, const char *capath, const char *dhfile, const char *randfile, const char *sid)
#else
soap_ssl_server_context(struct soap *soap, unsigned short flags, const char *keyfile, const char *password, const char *cafile, const char *capath, const char *dhfile, const char *randfile, const char *sid)
#endif
{
  int err;
  soap->keyfile = keyfile;
#if defined(VXWORKS) && defined(WM_SECURE_KEY_STORAGE)
  soap->keyid = keyid; /* vxWorks compatible */
#endif
  soap->password = password;
  soap->cafile = cafile;
  soap->capath = capath;
#ifdef WITH_OPENSSL
  soap->dhfile = dhfile;
  soap->randfile = randfile;
  if (!soap->fsslverify)
    soap->fsslverify = ssl_verify_callback;
#endif
  soap->ssl_flags = flags | (dhfile == NULL ? SOAP_SSL_RSA : 0);
#ifdef WITH_GNUTLS
  (void)randfile; (void)sid;
  if (dhfile)
  {
    char *s = NULL;
    int n = (int)soap_strtoul(dhfile, &s, 10);
    if (!soap->dh_params)
      gnutls_dh_params_init(&soap->dh_params);
    /* if dhfile is numeric, treat it as a key length to generate DH params which can take a while */
    if (n >= 512 && s && *s == '\0')
      gnutls_dh_params_generate2(soap->dh_params, (unsigned int)n);
    else
    {
      unsigned int dparams_len;
      unsigned char dparams_buf[1024];
      FILE *fd = fopen(dhfile, "r");
      if (!fd)
        return soap_set_receiver_error(soap, "SSL/TLS error", "Invalid DH file", SOAP_SSL_ERROR);
      dparams_len = (unsigned int)fread(dparams_buf, 1, sizeof(dparams_buf), fd);
      fclose(fd);
      gnutls_datum_t dparams = {
        dparams_buf, dparams_len };
      if (gnutls_dh_params_import_pkcs3(soap->dh_params, &dparams, GNUTLS_X509_FMT_PEM))
        return soap_set_receiver_error(soap, "SSL/TLS error", "Invalid DH file", SOAP_SSL_ERROR);
    }
  }
  else
  {
#if GNUTLS_VERSION_NUMBER < 0x030300
    if (!soap->rsa_params)
      gnutls_rsa_params_init(&soap->rsa_params);
    gnutls_rsa_params_generate2(soap->rsa_params, SOAP_SSL_RSA_BITS);
#endif
  }
  if (soap->session)
  {
    gnutls_deinit(soap->session);
    soap->session = NULL;
  }
  if (soap->xcred)
  {
    gnutls_certificate_free_credentials(soap->xcred);
    soap->xcred = NULL;
  }
#endif
#ifdef WITH_SYSTEMSSL
  (void)randfile; (void)sid;
  if (soap->ctx)
    gsk_environment_close(&soap->ctx);
#endif
  err = soap->fsslauth(soap);
#ifdef WITH_OPENSSL
  if (!err)
  {
    if (sid)
      SSL_CTX_set_session_id_context(soap->ctx, (unsigned char*)sid, (unsigned int)strlen(sid));
    else
      SSL_CTX_set_session_cache_mode(soap->ctx, SSL_SESS_CACHE_OFF);
  }
#endif
  return err;
}
#endif

/******************************************************************************/

#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS) || defined(WITH_SYSTEMSSL)
SOAP_FMAC1
int
SOAP_FMAC2
#if defined(VXWORKS) && defined(WM_SECURE_KEY_STORAGE)
soap_ssl_client_context(struct soap *soap, unsigned short flags, const char *keyfile, const char *keyid, const char *password, const char *cafile, const char *capath, const char *randfile)
#else
soap_ssl_client_context(struct soap *soap, unsigned short flags, const char *keyfile, const char *password, const char *cafile, const char *capath, const char *randfile)
#endif
{
  soap->keyfile = keyfile;
#if defined(VXWORKS) && defined(WM_SECURE_KEY_STORAGE)
  soap->keyid = keyid; /* vxWorks compatible */
#endif
  soap->password = password;
  soap->cafile = cafile;
  soap->capath = capath;
  soap->ssl_flags = SOAP_SSL_CLIENT | flags;
#ifdef WITH_OPENSSL
  soap->dhfile = NULL;
  soap->randfile = randfile;
  if (!soap->fsslverify)
    soap->fsslverify = (flags & SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE) == 0 ? ssl_verify_callback : ssl_verify_callback_allow_expired_certificate;
#endif
#ifdef WITH_GNUTLS
  (void)randfile;
  if (soap->session)
  {
    gnutls_deinit(soap->session);
    soap->session = NULL;
  }
  if (soap->xcred)
  {
    gnutls_certificate_free_credentials(soap->xcred);
    soap->xcred = NULL;
  }
#endif
#ifdef WITH_SYSTEMSSL
  (void)randfile;
  if (soap->ctx)
    gsk_environment_close(&soap->ctx);
#endif
  return soap->fsslauth(soap);
}
#endif

/******************************************************************************/

#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)
SOAP_FMAC1
int
SOAP_FMAC2
soap_ssl_crl(struct soap *soap, const char *crlfile)
{
#ifdef WITH_OPENSSL
  if (crlfile && soap->ctx)
  {
#if OPENSSL_VERSION_NUMBER > 0x00907000L
    X509_STORE *store = SSL_CTX_get_cert_store(soap->ctx);
    if (*crlfile)
    {
      int ret;
      X509_LOOKUP *lookup = X509_STORE_add_lookup(store, X509_LOOKUP_file());
      if (!lookup)
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't create X509_LOOKUP object", SOAP_SSL_ERROR);
      ret = X509_load_crl_file(lookup, crlfile, X509_FILETYPE_PEM);
      if (ret <= 0)
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read CRL PEM file", SOAP_SSL_ERROR);
    }
    X509_STORE_set_flags(store, X509_V_FLAG_CRL_CHECK | X509_V_FLAG_CRL_CHECK_ALL);
#endif
  }
  else
    soap->crlfile = crlfile; /* activate later when store is available */
#endif
#ifdef WITH_GNUTLS
  if (crlfile && soap->xcred)
  {
    if (*crlfile)
      if (gnutls_certificate_set_x509_crl_file(soap->xcred, crlfile, GNUTLS_X509_FMT_PEM) < 0)
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read CRL PEM file", SOAP_SSL_ERROR);
  }
  else
  {
    soap->crlfile = crlfile; /* activate later when xcred is available */
  }
#endif
  return SOAP_OK;
}
#endif

/******************************************************************************/

#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)
SOAP_FMAC1
void
SOAP_FMAC2
soap_ssl_init()
{
  /* Note: for multi-threaded applications, the main program should call soap_ssl_init() before any threads are started */
  if (!soap_ssl_init_done)
  {
    soap_ssl_init_done = 1;
#ifdef WITH_OPENSSL
#if OPENSSL_VERSION_NUMBER < 0x10100000L
    SSL_library_init();
    OpenSSL_add_all_algorithms(); /* we keep ciphers and digests for the program's lifetime */
#ifndef WITH_LEAN
    SSL_load_error_strings();
#endif
#endif
#if !defined(WIN32) && !defined(CYGWIN) && !defined(__MINGW32__) && !defined(__MINGW64__)
    if (!RAND_load_file("/dev/urandom", 1024))
#else
    if (1)
#endif
    {
      /* if /dev/urandom does not exist we need to do at least some pertubations to seed the OpenSSL PRNG */
      char buf[1024];
      RAND_seed(buf, sizeof(buf));
#ifdef HAVE_RANDOM
      srandom((unsigned long)time(NULL));
#else
      srand((unsigned int)time(NULL));
#endif
      do 
      {
#ifdef HAVE_RANDOM
        long r = random(); /* we actually do no use random() anywhere, except to further seed the OpenSSL PRNG */
        RAND_seed(&r, sizeof(long));
#else
        int r = rand(); /* we actually do no use rand() anywhere, except when random() is not available and to further seed the OpenSSL PRNG */
        RAND_seed(&r, sizeof(int));
#endif
      } while (!RAND_status());
    }
#endif
#ifdef WITH_GNUTLS
# if GNUTLS_VERSION_NUMBER < 0x020b00
#  if defined(HAVE_PTHREAD_H)
    gcry_control(GCRYCTL_SET_THREAD_CBS, &gcry_threads_pthread);
#  elif defined(HAVE_PTH_H)
    gcry_control(GCRYCTL_SET_THREAD_CBS, &gcry_threads_pth);
#  endif
    gcry_control(GCRYCTL_ENABLE_QUICK_RANDOM, 0);
    gcry_control(GCRYCTL_DISABLE_SECMEM, 0);
    gcry_control(GCRYCTL_INITIALIZATION_FINISHED, 0); /* libgcrypt init done */
# endif
# if GNUTLS_VERSION_NUMBER < 0x030300
    gnutls_global_init();
# endif
#endif
  }
}
#endif

/******************************************************************************/

#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)
SOAP_FMAC1
void
SOAP_FMAC2
soap_ssl_noinit()
{
  /* Call this first to bypass SSL init is SSL is already initialized elsewhere */
  soap_ssl_init_done = 1;
}
#endif

/******************************************************************************/

#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_ssl_error(struct soap *soap, int ret, int err)
{
#ifdef WITH_OPENSSL
  const char *msg = soap_code_str(h_ssl_error_codes, err);
  if (!msg)
    return ERR_error_string(err, soap->msgbuf);
  (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), strlen(msg) + 1), "%s\n", msg);
  if (ERR_peek_error())
  {
    unsigned long r;
    while ((r = ERR_get_error()))
    {
      size_t l = strlen(soap->msgbuf);
      ERR_error_string_n(r, soap->msgbuf + l, sizeof(soap->msgbuf) - l);
      l = strlen(soap->msgbuf);
      if (l + 1 < sizeof(soap->msgbuf))
      {
        soap->msgbuf[l++] = '\n';
        soap->msgbuf[l] = '\0';
      }
      if (ERR_GET_REASON(r) == SSL_R_CERTIFICATE_VERIFY_FAILED && l < sizeof(soap->msgbuf))
      {
        const char *s = X509_verify_cert_error_string(SSL_get_verify_result(soap->ssl));
        (SOAP_SNPRINTF(soap->msgbuf + l, sizeof(soap->msgbuf) - l, strlen(s)), "%s", s);
      }
    }
  }
  else
  {
    size_t l = strlen(soap->msgbuf);
    switch (ret)
    {
      case 0:
        soap_strcpy(soap->msgbuf + l, sizeof(soap->msgbuf) - l, "EOF was observed that violates the SSL/TLS protocol. The client probably provided invalid authentication information.");
        break;
      case -1:
        {
          const char *s = strerror(soap_errno);
          (SOAP_SNPRINTF(soap->msgbuf + l, sizeof(soap->msgbuf) - l, strlen(s) + 42), "Error observed by underlying SSL/TLS BIO: %s", s);
        }
        break;
    }
  }
  ERR_clear_error();
  return soap->msgbuf;
#endif
#ifdef WITH_GNUTLS
  (void)soap;
  (void)err;
  return gnutls_strerror(ret);
#endif
}
#endif

/******************************************************************************/

#ifdef WITH_SYSTEMSSL
static int
ssl_recv(int sk, void *s, int n, char *user)
{
  (void)user;
  return recv(sk, s, n, 0);
}
#endif

/******************************************************************************/

#ifdef WITH_SYSTEMSSL
static int
ssl_send(int sk, void *s, int n, char *user)
{
  (void)user;
  return send(sk, s, n, 0);
}
#endif

/******************************************************************************/

#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS) || defined(WITH_SYSTEMSSL)
static int
ssl_auth_init(struct soap *soap)
{
#ifdef WITH_OPENSSL
#if OPENSSL_VERSION_NUMBER >= 0x10101000L
  int minv = 0, maxv = 0;
#endif
  long flags = SSL_OP_ALL;
  int mode;
#if defined(VXWORKS) && defined(WM_SECURE_KEY_STORAGE)
  EVP_PKEY* pkey; /* vxWorks compatible */
#endif
  if (!soap_ssl_init_done)
    soap_ssl_init();
  ERR_clear_error();
  if (!soap->ctx)
  {
#if OPENSSL_VERSION_NUMBER >= 0x10100000L
    /* TLS_method: a TLS/SSL connection established may understand the SSLv3, TLSv1, TLSv1.1 and TLSv1.2 protocols. */
    soap->ctx = SSL_CTX_new(TLS_method());
#else
    /* SSLv23_method: a TLS/SSL connection established may understand the SSLv3, TLSv1, TLSv1.1 and TLSv1.2 protocols. */
    soap->ctx = SSL_CTX_new(SSLv23_method());
#endif
    if (!soap->ctx)
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't setup context", SOAP_SSL_ERROR);
    /* The following alters the behavior of SSL read/write: */
#if 0
    SSL_CTX_set_mode(soap->ctx, SSL_MODE_ENABLE_PARTIAL_WRITE | SSL_MODE_AUTO_RETRY);
#endif
  }
  if (soap->randfile)
  {
    if (!RAND_load_file(soap->randfile, -1))
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't load randomness", SOAP_SSL_ERROR);
  }
  if (soap->cafile || soap->capath)
  {
    if (!SSL_CTX_load_verify_locations(soap->ctx, soap->cafile, soap->capath))
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read CA PEM file", SOAP_SSL_ERROR);
    if (soap->cafile && (soap->ssl_flags & SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION))
      SSL_CTX_set_client_CA_list(soap->ctx, SSL_load_client_CA_file(soap->cafile));
  }
  if (!(soap->ssl_flags & SOAP_SSL_NO_DEFAULT_CA_PATH))
  {
    if (!SSL_CTX_set_default_verify_paths(soap->ctx))
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read default CA PEM file and/or directory", SOAP_SSL_ERROR);
  }
  if (soap->crlfile)
  {
    if (soap_ssl_crl(soap, soap->crlfile))
      return soap->error;
  }
/* This code assumes a typical scenario with key and cert in one PEM file, see alternative code below */
#if 1
  if (soap->keyfile)
  {
    if (!SSL_CTX_use_certificate_chain_file(soap->ctx, soap->keyfile))
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't find or read certificate in private key PEM file", SOAP_SSL_ERROR);
    if (soap->password)
    {
      SSL_CTX_set_default_passwd_cb_userdata(soap->ctx, (void*)soap->password);
      SSL_CTX_set_default_passwd_cb(soap->ctx, ssl_password);
    }
#ifndef WM_SECURE_KEY_STORAGE
    if (!SSL_CTX_use_PrivateKey_file(soap->ctx, soap->keyfile, SSL_FILETYPE_PEM))
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read private key PEM file", SOAP_SSL_ERROR);
#endif
  }
#else
/* Suggested alternative approach to check the key file for cert only when cafile==NULL */
  if (soap->password)
  {
    SSL_CTX_set_default_passwd_cb_userdata(soap->ctx, (void*)soap->password);
    SSL_CTX_set_default_passwd_cb(soap->ctx, ssl_password);
  }
  if (!soap->cafile)
  {
    if (soap->keyfile)
    {
      if (!SSL_CTX_use_certificate_chain_file(soap->ctx, soap->keyfile))
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't find or read certificate in private key PEM file", SOAP_SSL_ERROR);
      if (!SSL_CTX_use_PrivateKey_file(soap->ctx, soap->keyfile, SSL_FILETYPE_PEM))
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read private key PEM file", SOAP_SSL_ERROR);
    }
  }
  else /* use cafile for (server) cert and keyfile for (server) key */
  {
    if (!SSL_CTX_use_certificate_chain_file(soap->ctx, soap->cafile))
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read CA PEM file", SOAP_SSL_ERROR);
    if (soap->keyfile)
      if (!SSL_CTX_use_PrivateKey_file(soap->ctx, soap->keyfile, SSL_FILETYPE_PEM))
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read private key PEM file", SOAP_SSL_ERROR);
  }
#endif
#if defined(VXWORKS) && defined(WM_SECURE_KEY_STORAGE)
  /* vxWorks compatible */
  pkey = ipcom_key_db_pkey_get(soap->keyid);
  if (!pkey)
    return soap_set_receiver_error(soap, "SSL error", "Can't find key", SOAP_SSL_ERROR);
  if (!SSL_CTX_use_PrivateKey(soap->ctx, pkey))
    return soap_set_receiver_error(soap, "SSL error", "Can't read key", SOAP_SSL_ERROR);
#endif
  if ((soap->ssl_flags & SOAP_SSL_RSA))
  {
#if OPENSSL_VERSION_NUMBER >= 0x10002000L
    if (SSL_CTX_need_tmp_RSA(soap->ctx))
    {
      unsigned long e = RSA_F4;
      BIGNUM *bne = BN_new();
      RSA *rsa = RSA_new();
      if (!bne || !rsa || !BN_set_word(bne, e) || !RSA_generate_key_ex(rsa, SOAP_SSL_RSA_BITS, bne, NULL) || !SSL_CTX_set_tmp_rsa(soap->ctx, rsa))
      {
        if (bne)
          BN_free(bne);
        if (rsa)
          RSA_free(rsa);
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't generate RSA key", SOAP_SSL_ERROR);
      }
      BN_free(bne);
      RSA_free(rsa);
    }
#else
    RSA *rsa = RSA_generate_key(SOAP_SSL_RSA_BITS, RSA_F4, NULL, NULL);
    if (!rsa || !SSL_CTX_set_tmp_rsa(soap->ctx, rsa))
    {
      if (rsa)
        RSA_free(rsa);
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't generate RSA key", SOAP_SSL_ERROR);
    }
    RSA_free(rsa);
#endif
  }
  else if (soap->dhfile)
  {
    DH *dh = NULL;
    char *s = NULL;
    int n = (int)soap_strtoul(soap->dhfile, &s, 10);
    /* if dhfile is numeric, treat it as a key length to generate DH params which can take a while */
    if (n >= 512 && s && *s == '\0')
    {
#if OPENSSL_VERSION_NUMBER >= 0x10002000L
      dh = DH_new();
      if (!DH_generate_parameters_ex(dh, n, 2/*or 5*/, NULL))
      {
        DH_free(dh);
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't generate DH parameters", SOAP_SSL_ERROR);
      }
#elif defined(VXWORKS)
      dh = DH_new();
      DH_generate_parameters_ex(dh, n, 2/*or 5*/, NULL);
#else
      dh = DH_generate_parameters(n, 2/*or 5*/, NULL, NULL);
#endif
    }
    else
    {
      BIO *bio;
      bio = BIO_new_file(soap->dhfile, "r");
      if (!bio)
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read DH PEM file", SOAP_SSL_ERROR);
      dh = PEM_read_bio_DHparams(bio, NULL, NULL, NULL);
      BIO_free(bio);
    }
    if (!dh || DH_check(dh, &n) != 1 || SSL_CTX_set_tmp_dh(soap->ctx, dh) < 0)
    {
      if (dh)
        DH_free(dh);
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't set DH parameters", SOAP_SSL_ERROR);
    }
    DH_free(dh);
  }
  /* enable all TSLv1 protocols and disable SSLv3 by default if no SSL/TLS flags are set */
  if ((soap->ssl_flags & SOAP_SSLv3_TLSv1) == 0)
    soap->ssl_flags |= SOAP_TLSv1;
#if OPENSSL_VERSION_NUMBER >= 0x10101000L
  if ((soap->ssl_flags & SOAP_SSLv3))
    minv = SSL3_VERSION;
  else if ((soap->ssl_flags & SOAP_TLSv1_0))
    minv = TLS1_VERSION;
  else if ((soap->ssl_flags & SOAP_TLSv1_1))
    minv = TLS1_1_VERSION;
  else if ((soap->ssl_flags & SOAP_TLSv1_2))
    minv = TLS1_2_VERSION;
  else if ((soap->ssl_flags & SOAP_TLSv1_3))
    minv = TLS1_3_VERSION;
  if ((soap->ssl_flags & SOAP_TLSv1_3) && OpenSSL_version_num() >= 0x10101000L)
    maxv = TLS1_3_VERSION;
  else if ((soap->ssl_flags & SOAP_TLSv1_2))
    maxv = TLS1_2_VERSION;
  else if ((soap->ssl_flags & SOAP_TLSv1_1))
    maxv = TLS1_1_VERSION;
  else if ((soap->ssl_flags & SOAP_TLSv1_0))
    maxv = TLS1_VERSION;
  else
    maxv = SSL3_VERSION;
  if (!SSL_CTX_set_min_proto_version(soap->ctx, minv)
   || !SSL_CTX_set_max_proto_version(soap->ctx, maxv))
    return soap_set_receiver_error(soap, "SSL/TLS error", "Can't set protocol version", SOAP_SSL_ERROR);
#else
  /* disable SSL v2 by default and enable specific protos */
  flags = SSL_OP_NO_SSLv2;
  if (!(soap->ssl_flags & SOAP_SSLv3))
    flags |= SSL_OP_NO_SSLv3;
#if OPENSSL_VERSION_NUMBER >= 0x10001000L
  if (!(soap->ssl_flags & SOAP_TLSv1_0))
    flags |= SSL_OP_NO_TLSv1;
  if (!(soap->ssl_flags & SOAP_TLSv1_1))
    flags |= SSL_OP_NO_TLSv1_1;
  if (!(soap->ssl_flags & SOAP_TLSv1_2))
    flags |= SSL_OP_NO_TLSv1_2;
#endif
#endif
#ifdef SSL_OP_NO_TICKET
  /* TLS extension is enabled by default in OPENSSL v0.9.8k disable it by */
  flags |= SSL_OP_NO_TICKET;
#endif
  SSL_CTX_set_options(soap->ctx, flags);
  if ((soap->ssl_flags & SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION))
    mode = (SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT);
  else if ((soap->ssl_flags & SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION))
    mode = SSL_VERIFY_PEER;
  else
    mode = SSL_VERIFY_NONE;
  SSL_CTX_set_verify(soap->ctx, mode, soap->fsslverify);
#if OPENSSL_VERSION_NUMBER < 0x00905100L
  SSL_CTX_set_verify_depth(soap->ctx, 1);
#else
  SSL_CTX_set_verify_depth(soap->ctx, 9);
#endif
#endif
#ifdef WITH_GNUTLS
  int ret;
  char priority[80];
  soap_strcpy(priority, sizeof(priority), "PERFORMANCE");
  if (!soap_ssl_init_done)
    soap_ssl_init();
  if (!soap->xcred)
  {
    if (gnutls_certificate_allocate_credentials(&soap->xcred) != GNUTLS_E_SUCCESS)
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't allocate credentials or trust", SOAP_SSL_ERROR);
#if GNUTLS_VERSION_NUMBER >= 0x030300
    gnutls_certificate_set_x509_system_trust(soap->xcred);
#endif
    if (soap->cafile)
    {
      if (gnutls_certificate_set_x509_trust_file(soap->xcred, soap->cafile, GNUTLS_X509_FMT_PEM) < 0)
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read CA PEM file", SOAP_SSL_ERROR);
    }
    if (soap->crlfile)
    {
      if (soap_ssl_crl(soap, soap->crlfile))
        return soap->error;
    }
    if (soap->keyfile)
    {
      if (gnutls_certificate_set_x509_key_file2(soap->xcred, soap->keyfile, soap->keyfile, GNUTLS_X509_FMT_PEM, soap->password, GNUTLS_PKCS_PKCS12_3DES | GNUTLS_PKCS_PKCS12_ARCFOUR | GNUTLS_PKCS_PKCS12_RC2_40 | GNUTLS_PKCS_PBES2_AES_128 | GNUTLS_PKCS_PBES2_AES_192 | GNUTLS_PKCS_PBES2_AES_256 | GNUTLS_PKCS_PBES2_DES) < 0) /* Assumes that key and cert(s) are concatenated in the keyfile and the key is encrypted with one of these algorithms */
        return soap_set_receiver_error(soap, "SSL/TLS error", "Can't read private key PEM file", SOAP_SSL_ERROR);
    }
  }
  if ((soap->ssl_flags & SOAP_SSL_CLIENT))
  {
    gnutls_init(&soap->session, GNUTLS_CLIENT);
    if (soap->cafile || soap->crlfile || soap->keyfile)
    {
      ret = gnutls_priority_set_direct(soap->session, "PERFORMANCE", NULL);
      if (ret != GNUTLS_E_SUCCESS)
        return soap_set_receiver_error(soap, soap_ssl_error(soap, ret, 0), "SSL/TLS set priority error", SOAP_SSL_ERROR);
      gnutls_credentials_set(soap->session, GNUTLS_CRD_CERTIFICATE, soap->xcred);
    }
    else
    {
      if (!soap->acred)
        gnutls_anon_allocate_client_credentials(&soap->acred);
      ret = gnutls_priority_set_direct(soap->session, "PERFORMANCE:+ANON-DH:!ARCFOUR-128", NULL);
      if (ret != GNUTLS_E_SUCCESS)
        return soap_set_receiver_error(soap, soap_ssl_error(soap, ret, 0), "SSL/TLS set priority error", SOAP_SSL_ERROR);
      gnutls_credentials_set(soap->session, GNUTLS_CRD_ANON, soap->acred);
    }
  }
  else if (!soap->keyfile)
  {
    return soap_set_receiver_error(soap, "SSL/TLS error", "No key file: anonymous server authentication not supported in this release", SOAP_SSL_ERROR);
  }
  else
  {
#if GNUTLS_VERSION_NUMBER < 0x030300
    int protocol_priority[] = { 0, 0, 0, 0, 0 };
    int *protocol = protocol_priority;
    if ((soap->ssl_flags & SOAP_SSL_RSA) && soap->rsa_params)
      gnutls_certificate_set_rsa_export_params(soap->xcred, soap->rsa_params);
#endif
    if (!(soap->ssl_flags & SOAP_SSL_RSA) && soap->dh_params)
      gnutls_certificate_set_dh_params(soap->xcred, soap->dh_params);
    if (!soap->cache)
      gnutls_priority_init(&soap->cache, "NORMAL", NULL);
    gnutls_init(&soap->session, GNUTLS_SERVER);
    gnutls_priority_set(soap->session, soap->cache);
    gnutls_credentials_set(soap->session, GNUTLS_CRD_CERTIFICATE, soap->xcred);
    if ((soap->ssl_flags & SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION))
      gnutls_certificate_server_set_request(soap->session, GNUTLS_CERT_REQUEST);
    gnutls_session_enable_compatibility_mode(soap->session);
    /* enable all TSLv1 protocols and disable SSLv3 by default if no SSL/TLS flags are set */
    if ((soap->ssl_flags & SOAP_SSLv3_TLSv1) == 0)
      soap->ssl_flags |= SOAP_TLSv1;
#if GNUTLS_VERSION_NUMBER < 0x030300
    if ((soap->ssl_flags & SOAP_SSLv3))
      *protocol++ = GNUTLS_SSL3;
    if ((soap->ssl_flags & SOAP_TLSv1_0))
      *protocol++ = GNUTLS_TLS1_0;
    if ((soap->ssl_flags & SOAP_TLSv1_1))
      *protocol++ = GNUTLS_TLS1_1;
    if ((soap->ssl_flags & SOAP_TLSv1_2))
      *protocol++ = GNUTLS_TLS1_2;
    if (gnutls_protocol_set_priority(soap->session, protocol_priority) != GNUTLS_E_SUCCESS)
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't set protocol", SOAP_SSL_ERROR);
#else
    soap_strcpy(soap->tmpbuf, sizeof(soap->tmpbuf), "NORMAL:+VERS-ALL");
    if (!(soap->ssl_flags & SOAP_TLSv1_0))
      soap_strcat(soap->tmpbuf, sizeof(soap->tmpbuf), ":-VERS-TLS1.0");
    if (!(soap->ssl_flags & SOAP_TLSv1_1))
      soap_strcat(soap->tmpbuf, sizeof(soap->tmpbuf), ":-VERS-TLS1.1");
    if (!(soap->ssl_flags & SOAP_TLSv1_2))
      soap_strcat(soap->tmpbuf, sizeof(soap->tmpbuf), ":-VERS-TLS1.2");
    if (gnutls_priority_set_direct(soap->session, soap->tmpbuf, NULL) != GNUTLS_E_SUCCESS)
      return soap_set_receiver_error(soap, "SSL/TLS error", "Can't set protocol priority", SOAP_SSL_ERROR);
#endif
  }
#endif
#ifdef WITH_SYSTEMSSL
  if (!soap->ctx)
  {
    int err;
    err = gsk_environment_open(&soap->ctx);
    if (err == GSK_OK)
      err = gsk_attribute_set_enum(soap->ctx, GSK_PROTOCOL_SSLV2, GSK_PROTOCOL_SSLV2_OFF); 
    /* enable all TSLv1 protocols and disable SSLv3 by default if no SSL/TLS flags are set */
    if ((soap->ssl_flags & SOAP_SSLv3_TLSv1) == 0)
      soap->ssl_flags |= SOAP_TLSv1;
    if (err == GSK_OK)
    {
      if ((soap->ssl_flags & SOAP_SSLv3))
        err = gsk_attribute_set_enum(soap->ctx, GSK_PROTOCOL_SSLV3, GSK_PROTOCOL_SSLV3_ON);
      else
        err = gsk_attribute_set_enum(soap->ctx, GSK_PROTOCOL_SSLV3, GSK_PROTOCOL_SSLV3_OFF);
    }
    if (err == GSK_OK)
    {
      if ((soap->ssl_flags & SOAP_TLSv1_0))
        err = gsk_attribute_set_enum(soap->ctx, GSK_PROTOCOL_TLSV1, GSK_PROTOCOL_TLSV1_ON);
      else
        err = gsk_attribute_set_enum(soap->ctx, GSK_PROTOCOL_TLSV1, GSK_PROTOCOL_TLSV1_OFF);
    }
    if (err == GSK_OK)
    {
      if ((soap->ssl_flags & SOAP_TLSv1_1))
        err = gsk_attribute_set_enum(soap->ctx, GSK_PROTOCOL_TLSV1, GSK_PROTOCOL_TLSV1_1_ON);
      else
        err = gsk_attribute_set_enum(soap->ctx, GSK_PROTOCOL_TLSV1, GSK_PROTOCOL_TLSV1_1_OFF);
    }
    if (err == GSK_OK)
    {
      if ((soap->ssl_flags & SOAP_TLSv1_2))
        err = gsk_attribute_set_enum(soap->ctx, GSK_PROTOCOL_TLSV1, GSK_PROTOCOL_TLSV1_2_ON);
      else
        err = gsk_attribute_set_enum(soap->ctx, GSK_PROTOCOL_TLSV1, GSK_PROTOCOL_TLSV1_2_OFF);
    }
    if (err == GSK_OK)
      err = gsk_attribute_set_buffer(soap->ctx, GSK_KEYRING_FILE, soap->keyfile, 0); /* keyfile is a keyring .kdb file */
    if (err == GSK_OK)
      err = gsk_attribute_set_buffer(soap->ctx, GSK_KEYRING_PW, soap->password, 0); /* locked by password */
    if (err == GSK_OK)
      err = gsk_environment_init(soap->ctx);
    if (err != GSK_OK)
      return soap_set_receiver_error(soap, gsk_strerror(err), "SYSTEM SSL error in ssl_auth_init()", SOAP_SSL_ERROR);
  }
#endif
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifdef WITH_OPENSSL
static int
ssl_password(char *buf, int num, int rwflag, void *userdata)
{
  (void)rwflag;
  if (!buf || !userdata)
    return 0;
  soap_strcpy(buf, (size_t)num, (char*)userdata);
  return (int)strlen(buf);
}
#endif

/******************************************************************************/

#ifdef WITH_OPENSSL
static int
ssl_verify_callback(int ok, X509_STORE_CTX *store)
{
  (void)store;
#ifdef SOAP_DEBUG
  if (!ok)
  {
    char buf[1024];
    int err = X509_STORE_CTX_get_error(store);
    X509 *cert = X509_STORE_CTX_get_current_cert(store);
    fprintf(stderr, "\nDEBUG mode TLS/SSL warnings:\nSSL verify error %d or warning with certificate at depth %d: %s\n", err, X509_STORE_CTX_get_error_depth(store), X509_verify_cert_error_string(err));
    X509_NAME_oneline(X509_get_issuer_name(cert), buf, sizeof(buf)-1);
    fprintf(stderr, "  certificate issuer:  %s\n", buf);
    X509_NAME_oneline(X509_get_subject_name(cert), buf, sizeof(buf)-1);
    fprintf(stderr, "  certificate subject: %s\n", buf);
    /* accept self-signed certificates and certificates out of date */
    switch (err)
    {
      case X509_V_ERR_CERT_NOT_YET_VALID:
      case X509_V_ERR_CERT_HAS_EXPIRED:
      case X509_V_ERR_DEPTH_ZERO_SELF_SIGNED_CERT:
      case X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN:
      case X509_V_ERR_UNABLE_TO_GET_CRL:
      case X509_V_ERR_CRL_NOT_YET_VALID:
      case X509_V_ERR_CRL_HAS_EXPIRED:
        X509_STORE_CTX_set_error(store, X509_V_OK);
        ok = 1;
        fprintf(stderr, "Initialize soap_ssl_client_context with SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE to allow this verification error to pass without DEBUG mode enabled\n");
    }
  }
#endif
  /* Note: return 1 to try to continue, but unsafe progress will be terminated by OpenSSL */
  return ok;
}
#endif

/******************************************************************************/

#ifdef WITH_OPENSSL
static int
ssl_verify_callback_allow_expired_certificate(int ok, X509_STORE_CTX *store)
{
  ok = ssl_verify_callback(ok, store);
  if (!ok)
  {
    /* accept self signed certificates, expired certificates, and certficiates w/o CRL */
    switch (X509_STORE_CTX_get_error(store))
    {
      case X509_V_ERR_CERT_NOT_YET_VALID:
      case X509_V_ERR_CERT_HAS_EXPIRED:
      case X509_V_ERR_DEPTH_ZERO_SELF_SIGNED_CERT:
      case X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN:
      case X509_V_ERR_UNABLE_TO_GET_CRL:
      case X509_V_ERR_CRL_NOT_YET_VALID:
      case X509_V_ERR_CRL_HAS_EXPIRED:
        X509_STORE_CTX_set_error(store, X509_V_OK);
        ok = 1;
    }
  }
  /* Note: return 1 to continue, but unsafe progress will be terminated by SSL */
  return ok;
}
#endif

/******************************************************************************/

#ifdef WITH_GNUTLS
static const char *
ssl_verify(struct soap *soap, const char *host)
{
  unsigned int status;
  const char *err = NULL;
  int r = gnutls_certificate_verify_peers2(soap->session, &status);
  if (r < 0)
    err = "Certificate verify error";
  else if ((status & GNUTLS_CERT_INVALID))
    err = "The certificate is not trusted";
  else if ((status & GNUTLS_CERT_SIGNER_NOT_FOUND))
    err = "The certificate hasn't got a known issuer";
  else if ((status & GNUTLS_CERT_REVOKED))
    err = "The certificate has been revoked";
  else if (gnutls_certificate_type_get(soap->session) == GNUTLS_CRT_X509)
  {
    gnutls_x509_crt_t cert;
    const gnutls_datum_t *cert_list;
    unsigned int cert_list_size;
    if (gnutls_x509_crt_init(&cert) < 0)
      err = "Could not get X509 certificates";
    else if ((cert_list = gnutls_certificate_get_peers(soap->session, &cert_list_size)) == NULL)
      err = "Could not get X509 certificates";
    else if (gnutls_x509_crt_import(cert, &cert_list[0], GNUTLS_X509_FMT_DER) < 0)
      err = "Error parsing X509 certificate";
    else if (!(soap->ssl_flags & SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE) && gnutls_x509_crt_get_expiration_time(cert) < time(NULL))
      err = "The certificate has expired";
    else if (!(soap->ssl_flags & SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE) && gnutls_x509_crt_get_activation_time(cert) > time(NULL))
      err = "The certificate is not yet activated";
    else if (host && !(soap->ssl_flags & SOAP_SSL_SKIP_HOST_CHECK))
    {
      if (!gnutls_x509_crt_check_hostname(cert, host))
        err = "Certificate host name mismatch";
    }
    gnutls_x509_crt_deinit(cert);
  }
  return err;
}
#endif

/******************************************************************************/

#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)
#ifndef WITH_NOIO
SOAP_FMAC1
int
SOAP_FMAC2
soap_ssl_accept(struct soap *soap)
{
  SOAP_SOCKET sk = soap->socket;
#ifdef WITH_OPENSSL
  BIO *bio;
  int err = SSL_ERROR_NONE;
  int retries, r, s;
  ERR_clear_error();
  if (!soap_valid_socket(sk))
    return soap_set_receiver_error(soap, "SSL/TLS error", "No socket in soap_ssl_accept()", SOAP_SSL_ERROR);
  soap->ssl_flags &= ~SOAP_SSL_CLIENT;
  if (!soap->ctx && (soap->error = soap->fsslauth(soap)) != SOAP_OK)
    return soap_closesock(soap);
  if (!soap->ssl)
  {
    soap->ssl = SSL_new(soap->ctx);
    if (!soap->ssl)
    {
      (void)soap_closesock(soap);
      return soap_set_receiver_error(soap, "SSL/TLS error", "SSL_new() failed in soap_ssl_accept()", SOAP_SSL_ERROR);
    }
  }
  else
  {
    SSL_clear(soap->ssl);
  }
  bio = BIO_new_socket((int)sk, BIO_NOCLOSE);
  SSL_set_bio(soap->ssl, bio, bio);
  /* Default timeout: 10 sec retries, 100 times 0.1 sec */
  retries = 100;
  if (soap->recv_timeout || soap->send_timeout)
  {
    int t = soap->recv_timeout > soap->send_timeout ? soap->recv_timeout : soap->send_timeout;
    if (t > 0) 
      retries = 10 * t;
    else if (t > -100000)
      retries = 1;
    else
      retries = t/-100000;
  }
  SOAP_SOCKNONBLOCK(sk)
  while ((r = SSL_accept(soap->ssl)) <= 0)
  {
    err = SSL_get_error(soap->ssl, r);
    if (err == SSL_ERROR_WANT_ACCEPT || err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE)
    {
      if (err == SSL_ERROR_WANT_READ)
        s = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, -100000);
      else
        s = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, -100000);
      if (s < 0)
        break;
    }
    else
    {
      soap->errnum = soap_socket_errno;
      break;
    }
    if (retries-- <= 0)
      break;
  }
  if (!soap->recv_timeout && !soap->send_timeout)
    SOAP_SOCKBLOCK(sk)
  if (r <= 0)
  {
    (void)soap_set_receiver_error(soap, soap_ssl_error(soap, r, err), "SSL_accept() failed in soap_ssl_accept()", SOAP_SSL_ERROR);
    return soap_closesock(soap);
  }
  if ((soap->ssl_flags & SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION))
  {
    X509 *peer;
    int err;
    if ((err = SSL_get_verify_result(soap->ssl)) != X509_V_OK)
    {
      (void)soap_closesock(soap);
      return soap_set_sender_error(soap, X509_verify_cert_error_string(err), "SSL certificate presented by peer cannot be verified in soap_ssl_accept()", SOAP_SSL_ERROR);
    }
    peer = SSL_get_peer_certificate(soap->ssl);
    if (!peer)
    {
      (void)soap_closesock(soap);
      return soap_set_sender_error(soap, "SSL/TLS error", "No SSL certificate was presented by the peer in soap_ssl_accept()", SOAP_SSL_ERROR);
    }
    X509_free(peer);
  }
#endif
#ifdef WITH_GNUTLS
  int retries, r, s;
  if (!soap_valid_socket(sk))
    return soap_set_receiver_error(soap, "SSL/TLS error", "No socket in soap_ssl_accept()", SOAP_SSL_ERROR);
  soap->ssl_flags &= ~SOAP_SSL_CLIENT;
  if (!soap->session && (soap->error = soap->fsslauth(soap)) != SOAP_OK)
    return soap_closesock(soap);
  gnutls_transport_set_ptr(soap->session, (gnutls_transport_ptr_t)(long)sk);
  /* default timeout: 10 sec retries, 100 times 0.1 sec */
  retries = 100;
  if (soap->recv_timeout || soap->send_timeout)
  {
    int t = soap->recv_timeout > soap->send_timeout ? soap->recv_timeout : soap->send_timeout;
    if (t > 0) 
      retries = 10 * t;
    else if (t > -100000)
      retries = 1;
    else
      retries = t/-100000;
  }
  SOAP_SOCKNONBLOCK(sk)
  while ((r = gnutls_handshake(soap->session)))
  {
    /* GNUTLS repeat handhake when GNUTLS_E_AGAIN */
    if (r == GNUTLS_E_AGAIN || r == GNUTLS_E_INTERRUPTED)
    {
      if (!gnutls_record_get_direction(soap->session))
        s = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, -100000);
      else
        s = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, -100000);
      if (s < 0)
        break;
    }
    else
    {
      soap->errnum = soap_socket_errno;
      break;
    }
    if (retries-- <= 0)
      break;
  }
  if (!soap->recv_timeout && !soap->send_timeout)
    SOAP_SOCKBLOCK(sk)
  if (r)
  {
    (void)soap_set_receiver_error(soap, soap_ssl_error(soap, r, 0), "SSL/TLS handshake failed", SOAP_SSL_ERROR);
    return soap_closesock(soap);
  }
  if ((soap->ssl_flags & SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION))
  {
    const char *err = ssl_verify(soap, NULL);
    if (err)
    {
      (void)soap_closesock(soap);
      return soap_set_receiver_error(soap, "SSL/TLS error", err, SOAP_SSL_ERROR);
    }
  }
#endif
#ifdef WITH_SYSTEMSSL
  gsk_iocallback local_io = { ssl_recv, ssl_send, NULL, NULL, NULL, NULL };
  int retries, r, s;
  if (!soap_valid_socket(sk))
    return soap_set_receiver_error(soap, "SSL/TLS error", "No socket in soap_ssl_accept()", SOAP_SSL_ERROR);
  soap->ssl_flags &= ~SOAP_SSL_CLIENT;
  /* default timeout: 10 sec retries, 100 times 0.1 sec */
  retries = 100;
  if (soap->recv_timeout || soap->send_timeout)
  {
    int t = soap->recv_timeout > soap->send_timeout ? soap->recv_timeout : soap->send_timeout;
    if (t > 0) 
      retries = 10 * t;
    else if (t > -100000)
      retries = 1;
    else
      retries = t/-100000;
  }
  SOAP_SOCKNONBLOCK(sk)
  r = gsk_secure_socket_open(soap->ctx, &soap->ssl);
  if (r == GSK_OK)
    r = gsk_attribute_set_numeric_value(soap->ssl, GSK_FD, sk);
  if (r == GSK_OK)
    r = gsk_attribute_set_buffer(soap->ssl, GSK_KEYRING_LABEL, soap->cafile, 0);
  if (r == GSK_OK)
    r = gsk_attribute_set_enum(soap->ssl, GSK_SESSION_TYPE, GSK_SERVER_SESSION);
  if (r == GSK_OK)
    r = gsk_attribute_set_buffer(soap->ssl, GSK_V3_CIPHER_SPECS_EXPANDED, "0035002F000A", 0);
  if (r == GSK_OK)
    r = gsk_attribute_set_enum(soap->ssl, GSK_V3_CIPHERS, GSK_V3_CIPHERS_CHAR4);
  if (r == GSK_OK)
    r = gsk_attribute_set_callback(soap->ssl, GSK_IO_CALLBACK, &local_io);
  if (r != GSK_OK)
    return soap_set_receiver_error(soap, gsk_strerror(r), "SYSTEM SSL error in soap_ssl_accept()", SOAP_SSL_ERROR);
  while ((r = gsk_secure_socket_init(soap->ssl)) != GSK_OK)
  {
    if (r == GSK_WOULD_BLOCK_READ || r == GSK_WOULD_BLOCK_WRITE)
    {
      if (r == GSK_WOULD_BLOCK_READ)
        s = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, -100000);
      else
        s = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, -100000);
      if (s < 0)
        break;
    }
    else
    {
      soap->errnum = soap_socket_errno;
      break;
    }
    if (retries-- <= 0)
      break;
  }
  if (!soap->recv_timeout && !soap->send_timeout)
    SOAP_SOCKBLOCK(sk)
  if (r != GSK_OK)
  {
    (void)soap_set_receiver_error(soap, gsk_strerror(r), "gsk_secure_socket_init() failed in soap_ssl_accept()", SOAP_SSL_ERROR);
    return soap_closesock(soap);
  }
#endif
  soap->imode |= SOAP_ENC_SSL;
  soap->omode |= SOAP_ENC_SSL;
  return SOAP_OK;
}
#endif
#endif

/******************************************************************************\
 *
 *      TCP/UDP [SSL/TLS] IPv4 and IPv6
 *
\******************************************************************************/

#ifndef WITH_NOIO
static int
tcp_init(struct soap *soap)
{
  soap->errmode = 1;
#ifdef WIN32
  if (tcp_done)
    return 0;
  else
  {
    WSADATA w;
    if (WSAStartup(MAKEWORD(1, 1), &w))
      return -1;
    tcp_done = 1;
  }
#endif
  return 0;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static const char*
tcp_error(struct soap *soap)
{
  const char *msg = NULL;
  switch (soap->errmode)
  {
    case 0:
      msg = soap_strerror(soap);
      break;
    case 1:
      msg = "WSAStartup failed";
      break;
    case 2:
    {
#ifndef WITH_LEAN
      msg = soap_code_str(h_error_codes, soap->errnum);
      if (!msg)
#endif
      {
        (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), 37), "TCP/UDP IP error %d", soap->errnum);
        msg = soap->msgbuf;
      }
    }
  }
  return msg;
}
#endif

/******************************************************************************/

#if !defined(WITH_IPV6) || defined(WITH_COOKIES)
#ifndef WITH_NOIO
static int
tcp_gethostbyname(struct soap *soap, const char *addr, struct hostent *hostent, struct in_addr *inaddr)
{
#if (defined(_AIX43) || defined(TRU64) || defined(HP_UX)) && defined(HAVE_GETHOSTBYNAME_R)
  struct hostent_data ht_data;
#elif (!defined(__GLIBC__) || !defined(_GNU_SOURCE) || (!(~_GNU_SOURCE+1) && !defined(_POSIX_C_SOURCE) && !defined(_XOPEN_SOURCE)) || _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600 || defined(__ANDROID__) || defined(FREEBSD) || defined(__FreeBSD__)) && defined(HAVE_GETHOSTBYNAME_R)
  int r;
  char *tmpbuf = soap->tmpbuf;
  size_t tmplen = sizeof(soap->tmpbuf);
#elif defined(HAVE_GETHOSTBYNAME_R)
  char *tmpbuf = soap->tmpbuf;
  size_t tmplen = sizeof(soap->tmpbuf);
#endif
#ifdef VXWORKS
  int hostint; /* vxWorks compatible */
#endif
  if (inaddr)
  {
    soap_int32 iadd = -1;
#ifdef AS400
    iadd = inet_addr((void*)addr);
#else
    iadd = inet_addr((char*)addr);
#endif
    if (iadd != -1)
    {
      if (soap_memcpy((void*)inaddr, sizeof(struct in_addr), (const void*)&iadd, sizeof(iadd)))
        return soap->error = SOAP_EOM;
      return SOAP_OK;
    }
  }
#if (defined(_AIX43) || defined(TRU64) || defined(HP_UX)) && defined(HAVE_GETHOSTBYNAME_R)
  memset((void*)&ht_data, 0, sizeof(ht_data));
  if (gethostbyname_r(addr, hostent, &ht_data) < 0)
  {
    hostent = NULL;
    soap->errnum = h_errno;
  }
#elif (!defined(__GLIBC__) || !defined(_GNU_SOURCE) || (!(~_GNU_SOURCE+1) && !defined(_POSIX_C_SOURCE) && !defined(_XOPEN_SOURCE)) || _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600 || defined(__ANDROID__) || defined(FREEBSD) || defined(__FreeBSD__)) && defined(HAVE_GETHOSTBYNAME_R)
  while ((r = gethostbyname_r(addr, hostent, tmpbuf, tmplen, &hostent, &soap->errnum)) < 0)
  {
    if (tmpbuf != soap->tmpbuf)
      SOAP_FREE(soap, tmpbuf);
    if (r != SOAP_ERANGE)
    {
      hostent = NULL;
      break;
    }
    tmplen *= 2;
    tmpbuf = (char*)SOAP_MALLOC(soap, tmplen);
    if (!tmpbuf)
      break;
  }
#elif defined(HAVE_GETHOSTBYNAME_R)
  hostent = gethostbyname_r(addr, hostent, tmpbuf, tmplen, &soap->errnum);
#elif defined(VXWORKS)
  /* vxWorks compatible */
  /* If the DNS resolver library resolvLib has been configured in the vxWorks
   * image, a query for the host IP address is sent to the DNS server, if the
   * name was not found in the local host table. */
  hostint = hostGetByName((char*)addr);
  if (hostint == ERROR)
  {
    hostent = NULL;
    soap->errnum = soap_errno;
  }
#else
#ifdef AS400
  hostent = gethostbyname((void*)addr);
#else
  hostent = gethostbyname((char*)addr);
#endif
  if (!hostent)
    soap->errnum = h_errno;
#endif
  if (!hostent)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Host name not found\n"));
    return SOAP_ERR;
  }
  if (inaddr)
  {
#ifdef VXWORKS
    inaddr->s_addr = hostint; /* vxWorks compatible */
#else
    if (soap_memcpy((void*)inaddr, sizeof(struct in_addr), (const void*)hostent->h_addr, (size_t)hostent->h_length))
    {
#if (!defined(_AIX43) && !defined(TRU64) && !defined(HP_UX)) || !defined(HAVE_GETHOSTBYNAME_R)
#if (!defined(__GLIBC__) || !defined(_GNU_SOURCE) || (!(~_GNU_SOURCE+1) && !defined(_POSIX_C_SOURCE) && !defined(_XOPEN_SOURCE)) || _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600 || defined(__ANDROID__) || defined(FREEBSD) || defined(__FreeBSD__)) && defined(HAVE_GETHOSTBYNAME_R)
      if (tmpbuf && tmpbuf != soap->tmpbuf)
        SOAP_FREE(soap, tmpbuf);
#endif
#endif
      return soap->error = SOAP_EOM;
    }
#endif
  }
#if (!defined(__GLIBC__) || !defined(_GNU_SOURCE) || (!(~_GNU_SOURCE+1) && !defined(_POSIX_C_SOURCE) && !defined(_XOPEN_SOURCE)) || _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600 || defined(__ANDROID__) || defined(FREEBSD) || defined(__FreeBSD__)) && defined(HAVE_GETHOSTBYNAME_R)
  if (tmpbuf && tmpbuf != soap->tmpbuf)
    SOAP_FREE(soap, tmpbuf);
#endif
  return SOAP_OK;
}
#endif
#endif

/******************************************************************************/

#if !defined(WITH_IPV6)
#ifndef WITH_NOIO
static int
tcp_gethost(struct soap *soap, const char *addr, struct in_addr *inaddr)
{
  struct hostent hostent;
  return tcp_gethostbyname(soap, addr, &hostent, inaddr);
}
#endif
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static SOAP_SOCKET
tcp_connect(struct soap *soap, const char *endpoint, const char *host, int port)
{
#ifdef WITH_IPV6
  struct addrinfo hints, *res, *ressave;
#endif
  SOAP_SOCKET sk;
  int err = 0;
#ifndef WITH_LEAN
  int set = 1;
#endif
#if !defined(WITH_LEAN) || defined(WITH_OPENSSL) || defined(WITH_GNUTLS) || defined(WITH_SYSTEMSSL)
  int retries;
#endif
  soap->errnum = 0;
  soap->errmode = 0;
  if (soap_valid_socket(soap->socket))
  {
    if ((soap->omode & SOAP_IO_UDP) && soap->socket == soap->master)
    {
#ifdef IP_MULTICAST_TTL
#ifndef WITH_IPV6
      soap->errmode = 2;
      if (soap->fresolve(soap, host, &soap->peer.in.sin_addr))
      {
        (void)soap_set_receiver_error(soap, tcp_error(soap), "get host by name failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, soap->socket);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
      soap->peer.in.sin_port = htons((short)port);
      soap->errmode = 0;
#else
      memset((void*)&hints, 0, sizeof(hints));
      err = getaddrinfo(host, soap_int2s(soap, port), &hints, &res);
      if (err || !res)
      {
        (void)soap_set_receiver_error(soap, SOAP_GAI_STRERROR(err), "getaddrinfo failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, soap->socket);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
      if (soap_memcpy((void*)&soap->peer.storage, sizeof(soap->peer.storage), (const void*)res->ai_addr, res->ai_addrlen))
      {
        soap->error = SOAP_EOM;
        (void)soap->fclosesocket(soap, soap->socket);
        freeaddrinfo(res);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
      soap->peerlen = res->ai_addrlen;
      freeaddrinfo(res);
#endif
      if (soap->ipv4_multicast_ttl)
      {
        unsigned char ttl = soap->ipv4_multicast_ttl;
        if (setsockopt(soap->socket, IPPROTO_IP, IP_MULTICAST_TTL, (char*)&ttl, sizeof(ttl)))
        {
          soap->errnum = soap_socket_errno;
          (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt IP_MULTICAST_TTL failed in tcp_connect()", SOAP_TCP_ERROR);
          (void)soap->fclosesocket(soap, soap->socket);
          return soap->socket = SOAP_INVALID_SOCKET;
        }
      }
      if (soap->ipv4_multicast_if && !soap->ipv6_multicast_if)
      {
        if (setsockopt(soap->socket, IPPROTO_IP, IP_MULTICAST_IF, (char*)soap->ipv4_multicast_if, sizeof(struct in_addr)))
#ifndef WINDOWS
        {
          soap->errnum = soap_socket_errno;
          (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt IP_MULTICAST_IF failed in tcp_connect()", SOAP_TCP_ERROR);
          (void)soap->fclosesocket(soap, soap->socket);
          return soap->socket = SOAP_INVALID_SOCKET;
        }
#else
#ifndef IP_MULTICAST_IF
#define IP_MULTICAST_IF 2
#endif
        if (setsockopt(soap->socket, IPPROTO_IP, IP_MULTICAST_IF, (char*)soap->ipv4_multicast_if, sizeof(struct in_addr)))
        {
          soap->errnum = soap_socket_errno;
          (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt IP_MULTICAST_IF failed in tcp_connect()", SOAP_TCP_ERROR);
          (void)soap->fclosesocket(soap, soap->socket);
          return soap->socket = SOAP_INVALID_SOCKET;
        }
#endif
      }
#endif
      return soap->socket;
    }
    (void)soap->fclosesocket(soap, soap->socket);
    soap->socket = SOAP_INVALID_SOCKET;
  }
  if (tcp_init(soap))
  {
    (void)soap_set_receiver_error(soap, tcp_error(soap), "TCP init failed in tcp_connect()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
  soap->errmode = 0;
#ifdef WITH_IPV6
  memset((void*)&hints, 0, sizeof(hints));
  hints.ai_family = PF_UNSPEC;
#ifndef WITH_LEAN
  if ((soap->omode & SOAP_IO_UDP))
    hints.ai_socktype = SOCK_DGRAM;
  else
#endif
    hints.ai_socktype = SOCK_STREAM;
  soap->errmode = 2;
  if (soap->proxy_host)
    err = getaddrinfo(soap->proxy_host, soap_int2s(soap, soap->proxy_port), &hints, &res);
  else
    err = getaddrinfo(host, soap_int2s(soap, port), &hints, &res);
  if (err || !res)
  {
    (void)soap_set_receiver_error(soap, SOAP_GAI_STRERROR(err), "getaddrinfo failed in tcp_connect()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
  ressave = res;
again:
  sk = soap->socket = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  soap->error = SOAP_OK;
  soap->errmode = 0;
#else
#ifndef WITH_LEAN
again:
#endif
#ifndef WITH_LEAN
  if ((soap->omode & SOAP_IO_UDP))
    sk = soap->socket = socket(AF_INET, SOCK_DGRAM, 0);
  else
#endif
    sk = soap->socket = socket(AF_INET, SOCK_STREAM, 0);
#endif
  if (!soap_valid_socket(sk))
  {
#ifdef WITH_IPV6
    if (res->ai_next)
    {
      res = res->ai_next;
      goto again;
    }
#endif
    soap->errnum = soap_socket_errno;
    (void)soap_set_receiver_error(soap, tcp_error(soap), "socket failed in tcp_connect()", SOAP_TCP_ERROR);
#ifdef WITH_IPV6
    freeaddrinfo(ressave);
#endif
    return SOAP_INVALID_SOCKET;
  }
#ifdef WITH_SOCKET_CLOSE_ON_EXIT
#ifdef WIN32
#ifndef UNDER_CE
  SetHandleInformation((HANDLE)sk, HANDLE_FLAG_INHERIT, 0);
#endif
#else
  fcntl(sk, F_SETFD, 1);
#endif
#endif
#ifndef WITH_LEAN
  if ((soap->connect_flags & SO_LINGER))
  {
    struct linger linger;
    memset((void*)&linger, 0, sizeof(linger));
    linger.l_onoff = 1;
    linger.l_linger = soap->linger_time;
    if (setsockopt(sk, SOL_SOCKET, SO_LINGER, (char*)&linger, sizeof(struct linger)))
    {
      soap->errnum = soap_socket_errno;
      (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_LINGER failed in tcp_connect()", SOAP_TCP_ERROR);
      (void)soap->fclosesocket(soap, sk);
#ifdef WITH_IPV6
      freeaddrinfo(ressave);
#endif
      return soap->socket = SOAP_INVALID_SOCKET;
    }
  }
  if ((soap->connect_flags & ~SO_LINGER) && setsockopt(sk, SOL_SOCKET, soap->connect_flags & ~SO_LINGER, (char*)&set, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
    freeaddrinfo(ressave);
#endif
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt failed in tcp_connect()", SOAP_TCP_ERROR);
    (void)soap->fclosesocket(soap, sk);
    return soap->socket = SOAP_INVALID_SOCKET;
  }
#ifndef UNDER_CE
  if ((soap->keep_alive || soap->tcp_keep_alive) && setsockopt(sk, SOL_SOCKET, SO_KEEPALIVE, (char*)&set, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
    freeaddrinfo(ressave);
#endif
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_KEEPALIVE failed in tcp_connect()", SOAP_TCP_ERROR);
    (void)soap->fclosesocket(soap, sk);
    return soap->socket = SOAP_INVALID_SOCKET;
  }
  if (soap->sndbuf > 0 && setsockopt(sk, SOL_SOCKET, SO_SNDBUF, (char*)&soap->sndbuf, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
    freeaddrinfo(ressave);
#endif
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_SNDBUF failed in tcp_connect()", SOAP_TCP_ERROR);
    (void)soap->fclosesocket(soap, sk);
    return soap->socket = SOAP_INVALID_SOCKET;
  }
  if (soap->rcvbuf > 0 && setsockopt(sk, SOL_SOCKET, SO_RCVBUF, (char*)&soap->rcvbuf, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
    freeaddrinfo(ressave);
#endif
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_RCVBUF failed in tcp_connect()", SOAP_TCP_ERROR);
    (void)soap->fclosesocket(soap, sk);
    return soap->socket = SOAP_INVALID_SOCKET;
  }
#ifdef TCP_KEEPIDLE
  if (soap->tcp_keep_idle && setsockopt(sk, IPPROTO_TCP, TCP_KEEPIDLE, (char*)&(soap->tcp_keep_idle), sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
    freeaddrinfo(ressave);
#endif
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt TCP_KEEPIDLE failed in tcp_connect()", SOAP_TCP_ERROR);
    (void)soap->fclosesocket(soap, sk);
    return soap->socket = SOAP_INVALID_SOCKET;
  }
#endif
#ifdef TCP_KEEPINTVL
  if (soap->tcp_keep_intvl && setsockopt(sk, IPPROTO_TCP, TCP_KEEPINTVL, (char*)&(soap->tcp_keep_intvl), sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
    freeaddrinfo(ressave);
#endif
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt TCP_KEEPINTVL failed in tcp_connect()", SOAP_TCP_ERROR);
    (void)soap->fclosesocket(soap, sk);
    return soap->socket = SOAP_INVALID_SOCKET;
  }
#endif
#ifdef TCP_KEEPCNT
  if (soap->tcp_keep_cnt && setsockopt(sk, IPPROTO_TCP, TCP_KEEPCNT, (char*)&(soap->tcp_keep_cnt), sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
    freeaddrinfo(ressave);
#endif
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt TCP_KEEPCNT failed in tcp_connect()", SOAP_TCP_ERROR);
    (void)soap->fclosesocket(soap, sk);
    return soap->socket = SOAP_INVALID_SOCKET;
  }
#endif
#ifdef TCP_NODELAY
  if (!(soap->omode & SOAP_IO_UDP) && setsockopt(sk, IPPROTO_TCP, TCP_NODELAY, (char*)&set, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
    freeaddrinfo(ressave);
#endif
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt TCP_NODELAY failed in tcp_connect()", SOAP_TCP_ERROR);
    (void)soap->fclosesocket(soap, sk);
    return soap->socket = SOAP_INVALID_SOCKET;
  }
#endif
#ifdef WITH_IPV6
  if ((soap->omode & SOAP_IO_UDP) && soap->ipv6_multicast_if)
  {
    struct sockaddr_in6 *in6addr = (struct sockaddr_in6*)res->ai_addr;
    in6addr->sin6_scope_id = soap->ipv6_multicast_if;
  }
#endif
#endif
#ifdef IP_MULTICAST_TTL
  if ((soap->omode & SOAP_IO_UDP))
  {
    if (soap->ipv4_multicast_ttl)
    {
      unsigned char ttl = soap->ipv4_multicast_ttl;
      if (setsockopt(sk, IPPROTO_IP, IP_MULTICAST_TTL, (char*)&ttl, sizeof(ttl)))
      {
        soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
        freeaddrinfo(ressave);
#endif
        (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt IP_MULTICAST_TTL failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
    }
    if ((soap->omode & SOAP_IO_UDP) && soap->ipv4_multicast_if && !soap->ipv6_multicast_if)
    {
      if (setsockopt(sk, IPPROTO_IP, IP_MULTICAST_IF, (char*)soap->ipv4_multicast_if, sizeof(struct in_addr)))
#ifndef WINDOWS
      {
        soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
        freeaddrinfo(ressave);
#endif
        (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt IP_MULTICAST_IF failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
#else
#ifndef IP_MULTICAST_IF
#define IP_MULTICAST_IF 2
#endif
      if (setsockopt(sk, IPPROTO_IP, IP_MULTICAST_IF, (char*)soap->ipv4_multicast_if, sizeof(struct in_addr)))
      {
        soap->errnum = soap_socket_errno;
#ifdef WITH_IPV6
        freeaddrinfo(ressave);
#endif
        (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt IP_MULTICAST_IF failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
#endif
    }
  }
#endif
#endif
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Opening socket=%d to host='%s' port=%d\n", (int)sk, host, port));
#ifndef WITH_IPV6
  soap->peerlen = sizeof(soap->peer.in);
  memset((void*)&soap->peer.in, 0, sizeof(soap->peer.in));
  soap->peer.in.sin_family = AF_INET;
#ifndef WIN32
  if (soap->client_addr)
  {
    struct sockaddr_in addr;
    memset((void*)&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    if (soap->client_port >= 0)
      addr.sin_port = htons(soap->client_port);
    if (inet_pton(AF_INET, soap->client_addr, (void*)&addr.sin_addr) != 1 || bind(sk, (struct sockaddr*)&addr, sizeof(addr)))
    {
      soap->errnum = soap_socket_errno;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not bind before connect\n"));
      (void)soap_set_receiver_error(soap, tcp_error(soap), "bind failed in tcp_connect()", SOAP_TCP_ERROR);
      (void)soap->fclosesocket(soap, sk);
      soap->client_addr = NULL;
      soap->client_port = -1;
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    soap->client_addr = NULL; /* disable bind before connect, so need to set it again before the next connect */
    soap->client_port = -1; /* disable bind before connect, so need to set it again before the next connect */
  }
  else
#endif
  if (soap->client_port >= 0)
  {
    struct sockaddr_in addr;
    memset((void*)&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(soap->client_port);
    if (bind(sk, (struct sockaddr*)&addr, sizeof(addr)))
    {
      soap->errnum = soap_socket_errno;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not bind before connect\n"));
      (void)soap_set_receiver_error(soap, tcp_error(soap), "bind failed in tcp_connect()", SOAP_TCP_ERROR);
      (void)soap->fclosesocket(soap, sk);
      soap->client_port = -1;
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    soap->client_port = -1; /* disable bind before connect, so need to set it again before the next connect */
  }
#ifndef WIN32
  if (soap->client_interface)
  {
    if (inet_pton(AF_INET, soap->client_interface, &soap->peer.in.sin_addr) != 1)
    {
      soap->errnum = soap_socket_errno;
      (void)soap_set_receiver_error(soap, tcp_error(soap), "inet_pton() failed in tcp_connect()", SOAP_TCP_ERROR);
      (void)soap->fclosesocket(soap, sk);
      soap->client_interface = NULL;
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    soap->client_interface = NULL; /* disable client interface, so need to set it again before the next connect */
  }
#endif
  soap->errmode = 2;
  if (soap->proxy_host)
  {
    if (soap->fresolve(soap, soap->proxy_host, &soap->peer.in.sin_addr))
    {
      (void)soap_set_receiver_error(soap, tcp_error(soap), "get proxy host by name failed in tcp_connect()", SOAP_TCP_ERROR);
      (void)soap->fclosesocket(soap, sk);
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    soap->peer.in.sin_port = htons((short)soap->proxy_port);
  }
  else
  {
    if (soap->fresolve(soap, host, &soap->peer.in.sin_addr))
    {
      (void)soap_set_receiver_error(soap, tcp_error(soap), "get host by name failed in tcp_connect()", SOAP_TCP_ERROR);
      (void)soap->fclosesocket(soap, sk);
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    soap->peer.in.sin_port = htons((short)port);
  }
  soap->errmode = 0;
#ifndef WITH_LEAN
  if ((soap->omode & SOAP_IO_UDP))
    return sk;
#endif
#else
#ifndef WIN32
  if (soap->client_addr)
  {
    struct sockaddr_in6 addr;
    memset((void*)&addr, 0, sizeof(addr));
    addr.sin6_family = AF_INET6;
    if ((soap->client_addr_ipv6 && res->ai_family == AF_INET6 && inet_pton(AF_INET6, soap->client_addr_ipv6, (void*)&addr.sin6_addr.s6_addr) == 1)
     || (!soap->client_addr_ipv6 && inet_pton(AF_INET6, soap->client_addr, (void*)&addr.sin6_addr.s6_addr) == 1)
     )
    {
      if (soap->client_port >= 0)
        addr.sin6_port = htons(soap->client_port);
      if (bind(sk, (struct sockaddr*)&addr, sizeof(addr)))
      {
        soap->errnum = soap_socket_errno;
        freeaddrinfo(ressave);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not bind before connect\n"));
        (void)soap_set_receiver_error(soap, tcp_error(soap), "bind failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, sk);
        soap->client_addr = NULL;
        soap->client_addr_ipv6 = NULL;
        soap->client_port = -1;
        return soap->socket = SOAP_INVALID_SOCKET;
      }
    }
    else /* not an IPv6 address, must be IPv4 */
    {
      struct sockaddr_in addr;
      memset((void*)&addr, 0, sizeof(addr));
      addr.sin_family = AF_INET;
      if (soap->client_port >= 0)
        addr.sin_port = htons(soap->client_port);
      if (inet_pton(AF_INET, soap->client_addr, (void*)&addr.sin_addr) != 1 || bind(sk, (struct sockaddr*)&addr, sizeof(addr)))
      {
        soap->errnum = soap_socket_errno;
        freeaddrinfo(ressave);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not bind before connect\n"));
        (void)soap_set_receiver_error(soap, tcp_error(soap), "bind failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, sk);
        soap->client_addr = NULL;
        soap->client_addr_ipv6 = NULL;
        soap->client_port = -1;
        return soap->socket = SOAP_INVALID_SOCKET;
      }
    }
    soap->client_addr = NULL; /* disable bind before connect, so need to set it again before the next connect */
    soap->client_addr_ipv6 = NULL; /* disable bind before connect, so need to set it again before the next connect */
    soap->client_port = -1; /* disable bind before connect, so need to set it again before the next connect */
  }
  else
#endif
  if (soap->client_port >= 0)
  {
    struct sockaddr_in6 addr;
    memset((void*)&addr, 0, sizeof(addr));
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(soap->client_port);
    if (bind(sk, (struct sockaddr*)&addr, sizeof(addr)))
    {
      soap->errnum = soap_socket_errno;
      freeaddrinfo(ressave);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not bind before connect\n"));
      (void)soap_set_receiver_error(soap, tcp_error(soap), "bind failed in tcp_connect()", SOAP_TCP_ERROR);
      (void)soap->fclosesocket(soap, sk);
      soap->client_port = -1;
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    soap->client_port = -1; /* disable bind before connect, so need to set it again before the next connect */
  }
#ifndef WIN32
  if (soap->client_interface)
  {
    if (inet_pton(AF_INET6, soap->client_interface, res->ai_addr) != 1)
    {
      if (inet_pton(AF_INET, soap->client_interface, res->ai_addr) != 1)
      {
        soap->errnum = soap_socket_errno;
        freeaddrinfo(ressave);
        (void)soap_set_receiver_error(soap, tcp_error(soap), "inet_pton() failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, sk);
        soap->client_interface = NULL;
        return soap->socket = SOAP_INVALID_SOCKET;
      }
    }
    soap->client_interface = NULL; /* disable client interface, so need to set it again before the next connect */
  }
#endif
#ifndef WITH_LEAN
  if ((soap->omode & SOAP_IO_UDP))
  {
    if (soap_memcpy((void*)&soap->peer.storage, sizeof(soap->peer.storage), (const void*)res->ai_addr, res->ai_addrlen))
    {
      soap->error = SOAP_EOM;
      (void)soap->fclosesocket(soap, sk);
      soap->socket = sk = SOAP_INVALID_SOCKET;
    }
    soap->peerlen = res->ai_addrlen;
    freeaddrinfo(ressave);
    return sk;
  }
#endif
#endif
#ifndef WITH_LEAN
  if (soap->connect_timeout)
    SOAP_SOCKNONBLOCK(sk)
  else
    SOAP_SOCKBLOCK(sk)
  retries = 10;
#endif
  for (;;)
  {
#ifdef WITH_IPV6
    if (connect(sk, res->ai_addr, (int)res->ai_addrlen))
#else
    if (connect(sk, &soap->peer.addr, sizeof(soap->peer.in)))
#endif
    {
      err = soap_socket_errno;
#ifdef WITH_IPV6
      if (err == SOAP_ECONNREFUSED && res->ai_next)
      {
        (void)soap->fclosesocket(soap, sk);
        res = res->ai_next;
        goto again;
      }
#endif
#ifndef WITH_LEAN
      if (err == SOAP_EADDRINUSE)
      {
        (void)soap->fclosesocket(soap, sk);
        if (retries-- > 0)
          goto again;
      }
      else if (soap->connect_timeout && (err == SOAP_EINPROGRESS || err == SOAP_EAGAIN || err == SOAP_EWOULDBLOCK))
      {
        SOAP_SOCKLEN_T k;
        for (;;)
        {
          int r;
#ifdef WITH_SELF_PIPE
          r = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_PIP, soap->connect_timeout);
          if ((r & SOAP_TCP_SELECT_PIP))
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Connection closed by self pipe\n"));
            (void)soap->fclosesocket(soap, sk);
            return soap->socket = SOAP_INVALID_SOCKET;
          }
#else
          r = tcp_select(soap, sk, SOAP_TCP_SELECT_SND, soap->connect_timeout);
#endif
          if (r > 0)
            break;
          if (!r)
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Connect timeout\n"));
            (void)soap_set_receiver_error(soap, "Timeout", "connect failed in tcp_connect()", SOAP_TCP_ERROR);
            (void)soap->fclosesocket(soap, sk);
#ifdef WITH_IPV6
            if (res->ai_next)
            {
              res = res->ai_next;
              goto again;
            }
            freeaddrinfo(ressave);
#endif
            return soap->socket = SOAP_INVALID_SOCKET;
          }
          r = soap->errnum = soap_socket_errno;
          if (r != SOAP_EINTR)
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not connect to host\n"));
            (void)soap_set_receiver_error(soap, tcp_error(soap), "connect failed in tcp_connect()", SOAP_TCP_ERROR);
            (void)soap->fclosesocket(soap, sk);
#ifdef WITH_IPV6
            if (res->ai_next)
            {
              res = res->ai_next;
              goto again;
            }
            freeaddrinfo(ressave);
#endif
            return soap->socket = SOAP_INVALID_SOCKET;
          }
        }
        k = (SOAP_SOCKLEN_T)sizeof(soap->errnum);
        if (!getsockopt(sk, SOL_SOCKET, SO_ERROR, (char*)&soap->errnum, &k) && !soap->errnum)   /* portability note: see SOAP_SOCKLEN_T definition in stdsoap2.h */
          break;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not connect to host\n"));
        if (!soap->errnum)
          soap->errnum = soap_socket_errno;
        (void)soap_set_receiver_error(soap, tcp_error(soap), "connect failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, sk);
#ifdef WITH_IPV6
        if (res->ai_next)
        {
          res = res->ai_next;
          goto again;
        }
        freeaddrinfo(ressave);
#endif
        return soap->socket = SOAP_INVALID_SOCKET;
      }
#endif
#ifdef WITH_IPV6
      if (res->ai_next)
      {
        res = res->ai_next;
        (void)soap->fclosesocket(soap, sk);
        goto again;
      }
#endif
      if (err && err != SOAP_EINTR)
      {
        soap->errnum = err;
#ifdef WITH_IPV6
        freeaddrinfo(ressave);
#endif
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not connect to host\n"));
        (void)soap_set_receiver_error(soap, tcp_error(soap), "connect failed in tcp_connect()", SOAP_TCP_ERROR);
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
    }
    else
    {
      break;
    }
  }
#ifdef WITH_IPV6
  soap->peerlen = 0; /* IPv6: already connected so use send() */
  freeaddrinfo(ressave);
#endif
  soap->imode &= ~SOAP_ENC_SSL;
  soap->omode &= ~SOAP_ENC_SSL;
  if (endpoint && !soap_tag_cmp(endpoint, "https:*"))
  {
#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS) || defined(WITH_SYSTEMSSL)
#ifdef WITH_OPENSSL
    BIO *bio;
#endif
#ifdef WITH_SYSTEMSSL
    gsk_iocallback local_io = { ssl_recv, ssl_send, NULL, NULL, NULL, NULL };
#endif
    int r;
    if (soap->proxy_host)
    {
      soap_mode m = soap->mode; /* preserve settings */
      soap_mode om = soap->omode; /* make sure we only parse HTTP */
      ULONG64 count = soap->count; /* save the content length */
      const char *http_content = soap->http_content; /* save http_content when set */
      const char *http_extra_header = soap->http_extra_header; /* save http_extra_header when set */
      const char *bearer = soap->bearer; /* save bearer when set */
      int status = soap->status; /* save the current status/command */
      int keep_alive = soap->keep_alive; /* save the KA status */
      const char *userid, *passwd;
      soap->omode &= ~SOAP_ENC; /* mask IO and ENC */
      soap->omode |= SOAP_IO_BUFFER;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Connecting to %s proxy server %s for destination endpoint %s\n", soap->proxy_http_version, soap->proxy_host, endpoint));
#ifdef WITH_NTLM
      if (soap->ntlm_challenge && soap_ntlm_handshake(soap, SOAP_CONNECT, endpoint, host, port))
      {
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
#endif
      if (soap_init_send(soap))
      {
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
      soap->status = SOAP_CONNECT;
      if (!soap->keep_alive)
        soap->keep_alive = -1; /* must keep alive */
      soap->error = soap->fpost(soap, endpoint, host, port, NULL, NULL, 0);
      if (soap->error || soap_end_send_flush(soap))
      {
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
      soap->keep_alive = keep_alive;
      soap->omode = om;
      om = soap->imode; /* preserve */
      soap->imode &= ~SOAP_ENC; /* mask IO and ENC */
      userid = soap->userid; /* preserve */
      passwd = soap->passwd; /* preserve */
      soap->error = soap->fparse(soap);
      if (soap->error)
      {
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
      soap->status = status; /* restore */
      soap->userid = userid; /* restore */
      soap->passwd = passwd; /* restore */
      soap->imode = om; /* restore */
      soap->count = count; /* restore */
      soap->http_content = http_content; /* restore */
      soap->http_extra_header = http_extra_header; /* restore */
      soap->bearer = bearer; /* restore */
      if (soap_init_send(soap))
      {
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
      if (endpoint)
        soap_strcpy(soap->endpoint, sizeof(soap->endpoint), endpoint); /* restore */
      soap->mode = m;
    }
#ifdef WITH_OPENSSL
    ERR_clear_error();
    soap->ssl_flags |= SOAP_SSL_CLIENT;
    if (!soap->ctx && (soap->error = soap->fsslauth(soap)) != SOAP_OK)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL required, but no ctx set\n"));
      (void)soap->fclosesocket(soap, sk);
      soap->error = SOAP_SSL_ERROR;
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    if (!soap->ssl)
    {
      soap->ssl = SSL_new(soap->ctx);
      if (!soap->ssl)
      {
        (void)soap->fclosesocket(soap, sk);
        soap->error = SOAP_SSL_ERROR;
        return soap->socket = SOAP_INVALID_SOCKET;
      }
    }
    else
    {
      SSL_clear(soap->ssl);
    }
    if (soap->session)
    {
      if (!strcmp(soap->session_host, host) && soap->session_port == port)
        SSL_set_session(soap->ssl, soap->session);
      SSL_SESSION_free(soap->session);
      soap->session = NULL;
    }
#if OPENSSL_VERSION_NUMBER >= 0x1000000aL
    if (!(soap->ssl_flags & SOAP_SSLv3) && !SSL_set_tlsext_host_name(soap->ssl, host))
    {
      (void)soap_set_receiver_error(soap, "SSL/TLS error", "SNI failed", SOAP_SSL_ERROR);
      (void)soap->fclosesocket(soap, sk);
      return soap->socket = SOAP_INVALID_SOCKET;
    }
#elif (OPENSSL_VERSION_NUMBER >= 0x0090800fL) && defined(SSL_CTRL_SET_TLSEXT_HOSTNAME)
    if (!SSL_ctrl(soap->ssl, SSL_CTRL_SET_TLSEXT_HOSTNAME, TLSEXT_NAMETYPE_host_name, (void*)host))
    {
      (void)soap_set_receiver_error(soap, "SSL/TLS error", "SNI failed", SOAP_SSL_ERROR);
      (void)soap->fclosesocket(soap, sk);
      return soap->socket = SOAP_INVALID_SOCKET;
    }
#endif
    bio = BIO_new_socket((int)sk, BIO_NOCLOSE);
    SSL_set_bio(soap->ssl, bio, bio);
    if (soap->connect_timeout || soap->recv_timeout || soap->send_timeout)
    {
      /* Set SSL connect timeout and set SSL sockets to non-blocking */
      int t = soap->recv_timeout > soap->send_timeout ? soap->recv_timeout : soap->send_timeout;
      if (soap->connect_timeout > 0 && t < soap->connect_timeout)
        t = soap->connect_timeout;
      if (t > 0) 
        retries = 10 * t;
      else if (t > -100000)
        retries = 1;
      else
        retries = t/-100000;
      SOAP_SOCKNONBLOCK(sk)
    }
    else
    {
      /* Set sockets to blocking */
      retries = 1;
      SOAP_SOCKBLOCK(sk)
    }
    err = SSL_ERROR_NONE;
    /* Try connecting until success or timeout */
    do
    {
      if ((r = SSL_connect(soap->ssl)) <= 0)
      {
        err = SSL_get_error(soap->ssl, r);
        if (err == SSL_ERROR_WANT_CONNECT || err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE)
        {
          int s;
          if (err == SSL_ERROR_WANT_READ)
            s = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, -100000);
          else
            s = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, -100000);
          if (s < 0)
            break;
          if (s == 0 && retries-- <= 0)
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL/TLS connect timeout\n"));
            (void)soap_set_receiver_error(soap, "Timeout", "SSL_connect() failed in tcp_connect()", SOAP_TCP_ERROR);
            (void)soap->fclosesocket(soap, sk);
            return soap->socket = SOAP_INVALID_SOCKET;
          }
        }
        else
        {
          soap->errnum = soap_socket_errno;
          break;
        }
      }
    } while (!SSL_is_init_finished(soap->ssl));
    if (r <= 0)
    {
      (void)soap_set_sender_error(soap, soap_ssl_error(soap, r, err), "SSL/TLS handshake failed", SOAP_SSL_ERROR);
      (void)soap->fclosesocket(soap, sk);
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    /* Check server credentials when required */
    if ((soap->ssl_flags & SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION))
    {
      if ((err = SSL_get_verify_result(soap->ssl)) != X509_V_OK)
      {
        (void)soap_set_sender_error(soap, X509_verify_cert_error_string(err), "SSL/TLS certificate presented by peer cannot be verified in tcp_connect()", SOAP_SSL_ERROR);
        (void)soap->fclosesocket(soap, sk);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
      if (!(soap->ssl_flags & SOAP_SSL_SKIP_HOST_CHECK))
      {
        X509_NAME *subj;
        STACK_OF(CONF_VALUE) *val = NULL;
#if OPENSSL_VERSION_NUMBER >= 0x0090800fL
        GENERAL_NAMES *names = NULL;
#else
        int ext_count;
#endif
        int ok = 0;
        X509 *peer = SSL_get_peer_certificate(soap->ssl);
        if (!peer)
        {
          (void)soap_set_sender_error(soap, "SSL/TLS error", "No SSL/TLS certificate was presented by the peer in tcp_connect()", SOAP_SSL_ERROR);
          (void)soap->fclosesocket(soap, sk);
          return soap->socket = SOAP_INVALID_SOCKET;
        }
#if OPENSSL_VERSION_NUMBER < 0x0090800fL
        ext_count = X509_get_ext_count(peer);
        if (ext_count > 0)
        {
          int i;
          for (i = 0; i < ext_count; i++)
          {
            X509_EXTENSION *ext = X509_get_ext(peer, i);
            const char *ext_str = OBJ_nid2sn(OBJ_obj2nid(X509_EXTENSION_get_object(ext)));
            if (ext_str && !strcmp(ext_str, "subjectAltName"))
            {
              X509V3_EXT_METHOD *meth = (X509V3_EXT_METHOD*)X509V3_EXT_get(ext);
              unsigned char *data;
              if (!meth)
                break;
              data = ext->value->data;
              if (data)
              {
#if OPENSSL_VERSION_NUMBER > 0x00907000L
                void *ext_data;
                if (meth->it)
                  ext_data = ASN1_item_d2i(NULL, &data, ext->value->length, ASN1_ITEM_ptr(meth->it));
                else
                {
#if OPENSSL_VERSION_NUMBER > 0x0090800fL
                  ext_data = meth->d2i(NULL, (const unsigned char **)&data, ext->value->length);
#else
                  ext_data = meth->d2i(NULL, &data, ext->value->length);
#endif
                }
                if (ext_data)
                  val = meth->i2v(meth, ext_data, NULL);
                else
                  val = NULL;
                if (meth->it)
                  ASN1_item_free((ASN1_VALUE*)ext_data, ASN1_ITEM_ptr(meth->it));
                else
                  meth->ext_free(ext_data);
#else
                void *ext_data = meth->d2i(NULL, &data, ext->value->length);
                if (ext_data)
                  val = meth->i2v(meth, ext_data, NULL);
                meth->ext_free(ext_data);
#endif
                if (val)
                {
                  int j;
                  for (j = 0; j < sk_CONF_VALUE_num(val); j++)
                  {
                    CONF_VALUE *nval = sk_CONF_VALUE_value(val, j);
                    if (nval && nval->name && && (!strcmp(nval->name, "DNS") || !strcmp(nval->name, "IP Address")) && !soap_tag_cmp(host, nval->value))
                    {
                      ok = 1;
                      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL: host name %s match with certificate %s %s\n", host, nval->name, nval->value));
                      break;
                    }
                    else
                    {
                      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL: host name %s mismatch with certificate %s %s\n", host, nval->name, nval->value));
                    }
                  }
                  sk_CONF_VALUE_pop_free(val, X509V3_conf_free);
                }
              }
            }
            if (ok)
              break;
          }
        }
#else
        names = (GENERAL_NAMES*)X509_get_ext_d2i(peer, NID_subject_alt_name, NULL, NULL);
        if (names)
        {
          val = i2v_GENERAL_NAMES(NULL, names, val);
          sk_GENERAL_NAME_pop_free(names, GENERAL_NAME_free);
        }
        if (val)
        {
          int j;
          for (j = 0; j < sk_CONF_VALUE_num(val); j++)
          {
            CONF_VALUE *nval = sk_CONF_VALUE_value(val, j);
            if (nval && nval->name && (!strcmp(nval->name, "DNS") || !strcmp(nval->name, "IP Address")) && !soap_tag_cmp(host, nval->value))
            {
              ok = 1;
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL: host name %s match with certificate %s %s\n", host, nval->name, nval->value));
              break;
            }
            else
            {
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL: host name %s mismatch with certificate %s %s\n", host, nval->name, nval->value));
            }
          }
          sk_CONF_VALUE_pop_free(val, X509V3_conf_free);
        }
#endif
        if (!ok && (subj = X509_get_subject_name(peer)) != 0)
        {
          int i = -1;
          do
          {
            ASN1_STRING *name;
            i = X509_NAME_get_index_by_NID(subj, NID_commonName, i);
            if (i == -1)
              break;
            name = X509_NAME_ENTRY_get_data(X509_NAME_get_entry(subj, i));
            if (name)
            {
#if OPENSSL_VERSION_NUMBER < 0x10100000L
              const char *tmp = (const char*)ASN1_STRING_data(name);
#else
              const char *tmp = (const char*)ASN1_STRING_get0_data(name);
#endif
              if (!soap_tag_cmp(host, tmp))
              {
                ok = 1;
                DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL: host name %s match with certificate subject %s\n", host, tmp));
              }
              else
              {
                unsigned char *tmp = NULL;
                ASN1_STRING_to_UTF8(&tmp, name);
                if (tmp)
                {
                  if (!soap_tag_cmp(host, (const char*)tmp))
                  {
                    ok = 1;
                    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL: host name %s match with certificate subject %s\n", host, tmp));
                  }
                  else if (tmp[0] == '*') /* wildcard domain */
                  {
                    const char *t = strchr(host, '.');
                    if (t && !soap_tag_cmp(t, (const char*)tmp + 1))
                    {
                      ok = 1;
                      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL: host name %s match with certificate subject %s\n", host, tmp));
                    }
                  }
                  else
                  {
                    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL: host name %s mismatch with certificate %s\n", host, tmp));
                  }
                  OPENSSL_free(tmp);
                }
              }
            }
          } while (!ok);
        }
        X509_free(peer);
        if (!ok)
        {
          (void)soap_set_sender_error(soap, "SSL/TLS error", "SSL/TLS certificate host name mismatch in tcp_connect()", SOAP_SSL_ERROR);
          (void)soap->fclosesocket(soap, sk);
          return soap->socket = SOAP_INVALID_SOCKET;
        }
      }
    }
#endif
#ifdef WITH_GNUTLS
    soap->ssl_flags |= SOAP_SSL_CLIENT;
    if (!soap->session && (soap->error = soap->fsslauth(soap)) != SOAP_OK)
    {
      (void)soap->fclosesocket(soap, sk);
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    gnutls_transport_set_ptr(soap->session, (gnutls_transport_ptr_t)(long)sk);
    if (soap->connect_timeout || soap->recv_timeout || soap->send_timeout)
    {
      /* Set SSL connect timeout and set SSL sockets to non-blocking */
      int t = soap->recv_timeout > soap->send_timeout ? soap->recv_timeout : soap->send_timeout;
      if (soap->connect_timeout > 0 && t < soap->connect_timeout)
        t = soap->connect_timeout;
      if (t > 0) 
        retries = 10 * t;
      else if (t > -100000)
        retries = 1;
      else
        retries = t/-100000;
      SOAP_SOCKNONBLOCK(sk)
    }
    else
    {
      /* Set sockets to blocking */
      retries = 1;
      SOAP_SOCKBLOCK(sk)
    }
    /* Try connecting until success or timeout */
    while ((r = gnutls_handshake(soap->session)))
    {
      /* GNUTLS repeat handhake when GNUTLS_E_AGAIN */
      if (r == GNUTLS_E_AGAIN || r == GNUTLS_E_INTERRUPTED)
      {
        int s;
        if (!gnutls_record_get_direction(soap->session))
          s = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, -100000);
        else
          s = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, -100000);
        if (s < 0)
          break;
        if (s == 0 && retries-- <= 0)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL/TLS connect timeout\n"));
          (void)soap_set_receiver_error(soap, "Timeout", "SSL_connect() failed in tcp_connect()", SOAP_TCP_ERROR);
          (void)soap->fclosesocket(soap, sk);
          return soap->socket = SOAP_INVALID_SOCKET;
        }
      }
      else
      {
        soap->errnum = soap_socket_errno;
        break;
      }
    }
    if (r)
    {
      (void)soap_set_sender_error(soap, soap_ssl_error(soap, r, 0), "SSL/TLS handshake failed", SOAP_SSL_ERROR);
      (void)soap->fclosesocket(soap, sk);
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    if ((soap->ssl_flags & SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION))
    {
      const char *s = ssl_verify(soap, host);
      if (s)
      {
        (void)soap->fclosesocket(soap, sk);
        soap->error = soap_set_sender_error(soap, "SSL/TLS verify error", s, SOAP_SSL_ERROR);
        return soap->socket = SOAP_INVALID_SOCKET;
      }
    }
#endif
#ifdef WITH_SYSTEMSSL
    soap->ssl_flags |= SOAP_SSL_CLIENT;
    if (!soap->ctx && (soap->error = soap->fsslauth(soap)) != SOAP_OK)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL required, but no ctx set\n"));
      (void)soap->fclosesocket(soap, sk);
      soap->error = SOAP_SSL_ERROR;
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    if (soap->connect_timeout || soap->recv_timeout || soap->send_timeout)
    {
      /* Set SSL connect timeout and set SSL sockets to non-blocking */
      int t = soap->recv_timeout > soap->send_timeout ? soap->recv_timeout : soap->send_timeout;
      if (soap->connect_timeout > 0 && t < soap->connect_timeout)
        t = soap->connect_timeout;
      if (t > 0) 
        retries = 10 * t;
      else if (t > -100000)
        retries = 1;
      else
        retries = t/-100000;
      SOAP_SOCKNONBLOCK(sk)
    }
    else
    {
      /* Set sockets to blocking */
      retries = 1;
      SOAP_SOCKBLOCK(sk)
    }
    r = gsk_secure_socket_open(soap->ctx, &soap->ssl);
    if (r == GSK_OK)
      r = gsk_attribute_set_numeric_value(soap->ssl, GSK_FD, sk);
    if (r == GSK_OK)
      r = gsk_attribute_set_buffer(soap->ssl, GSK_KEYRING_LABEL, soap->cafile, 0); /* Certificate label */
    if (r == GSK_OK)
      r = gsk_attribute_set_enum(soap->ssl, GSK_SESSION_TYPE, GSK_CLIENT_SESSION);
    if (r == GSK_OK)
      r = gsk_attribute_set_buffer(soap->ssl, GSK_V3_CIPHER_SPECS_EXPANDED, "0035002F000A", 0);
    if (r == GSK_OK)
      r = gsk_attribute_set_enum(soap->ssl, GSK_V3_CIPHERS, GSK_V3_CIPHERS_CHAR4);
    if (r == GSK_OK)
      r = gsk_attribute_set_callback(soap->ssl, GSK_IO_CALLBACK, &local_io);
    if (r != GSK_OK)
    {
      (void)soap_set_receiver_error(soap, gsk_strerror(r), "SYSTEM SSL error in tcp_connect()", SOAP_SSL_ERROR);
      return soap->socket = SOAP_INVALID_SOCKET;
    }
    /* Try connecting until success or timeout */
    while ((r = gsk_secure_socket_init(soap->ssl)) != GSK_OK)
    {
      if (r == GSK_WOULD_BLOCK_READ || r == GSK_WOULD_BLOCK_WRITE)
      {
        int s;
        if (r == GSK_WOULD_BLOCK_READ)
          s = tcp_select(soap, sk, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, -100000);
        else
          s = tcp_select(soap, sk, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, -100000);
        if (s < 0)
          break;
        if (s == 0 && retries-- <= 0)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL/TLS connect timeout\n"));
          (void)soap_set_receiver_error(soap, "Timeout", "SSL_connect() failed in tcp_connect()", SOAP_TCP_ERROR);
          (void)soap->fclosesocket(soap, sk);
          return soap->socket = SOAP_INVALID_SOCKET;
        }
      }
      else
      {
        soap->errnum = soap_socket_errno;
        break;
      }
    }
    if (r != GSK_OK)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SSL_connect/select error in tcp_connect\n"));
      (void)soap_set_receiver_error(soap, gsk_strerror(r), "SSL/TLS handshake failed", SOAP_SSL_ERROR);
      (void)soap->fclosesocket(soap, sk);
      return soap->socket = SOAP_INVALID_SOCKET;
    }
#endif
    soap->imode |= SOAP_ENC_SSL;
    soap->omode |= SOAP_ENC_SSL;
#else
    (void)soap->fclosesocket(soap, sk);
    soap->error = SOAP_SSL_ERROR;
    return soap->socket = SOAP_INVALID_SOCKET;
#endif
  }
  if (soap->recv_timeout || soap->send_timeout)
    SOAP_SOCKNONBLOCK(sk)
  else
    SOAP_SOCKBLOCK(sk)
  return sk;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static int
tcp_select(struct soap *soap, SOAP_SOCKET sk, int flags, int timeout)
{
  int r;
  struct timeval tv;
  fd_set fd[3], *rfd, *sfd, *efd;
  int retries = 0;
  int eintr = SOAP_MAXEINTR;
  soap->errnum = 0;
  if (!soap_valid_socket(sk))
  {
    soap->error = SOAP_EOF;
    return -1;
  }
#ifndef WIN32
#if !defined(FD_SETSIZE) || defined(__QNX__) || defined(QNX)
  /* no FD_SETSIZE or select() is not MT safe on some QNX: always poll */
  if (1)
#else
  /* if fd max set size exceeded, use poll() when available */
  if ((int)sk >= (int)FD_SETSIZE)
#endif
#ifdef HAVE_POLL
  {
#ifdef WITH_SELF_PIPE
    struct pollfd pollfd[2];
    pollfd[1].fd = soap->pipe_fd[0];
    pollfd[1].events = POLLIN;
#else
    struct pollfd pollfd[1];
#endif
    pollfd[0].fd = (int)sk;
    pollfd[0].events = 0;
    if ((flags & SOAP_TCP_SELECT_RCV))
      pollfd[0].events |= POLLIN;
    if ((flags & SOAP_TCP_SELECT_SND))
      pollfd[0].events |= POLLOUT;
    if ((flags & SOAP_TCP_SELECT_ERR))
      pollfd[0].events |= POLLERR;
    if (timeout <= 0)
      timeout /= -1000; /* -usec -> ms */
    else
    {
      retries = timeout - 1;
      timeout = 1000;
    }
    do
    {
#ifdef WITH_SELF_PIPE
      r = poll(pollfd, 2, timeout);
#else
      r = poll(pollfd, 1, timeout);
#endif
      if (r < 0 && (soap->errnum = soap_socket_errno) == SOAP_EINTR && eintr > 0)
      {
        eintr--;
        r = 0;
      }
      else if (retries-- <= 0)
      {
        break;
      }
    } while (r == 0);
    if (r > 0)
    {
      r = 0;
      if ((flags & SOAP_TCP_SELECT_RCV) && (pollfd[0].revents & POLLIN))
        r |= SOAP_TCP_SELECT_RCV;
      if ((flags & SOAP_TCP_SELECT_SND) && (pollfd[0].revents & POLLOUT))
        r |= SOAP_TCP_SELECT_SND;
      if ((flags & SOAP_TCP_SELECT_ERR) && (pollfd[0].revents & POLLERR))
        r |= SOAP_TCP_SELECT_ERR;
#ifdef WITH_SELF_PIPE
      if ((flags & SOAP_TCP_SELECT_PIP) && (pollfd[1].revents & POLLIN))
      {
        char ch;
        for (;;)
        {
          if (read(soap->pipe_fd[0], &ch, 1) == -1)
          {
            if (soap_socket_errno == SOAP_EAGAIN)
              break;
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Self pipe read error\n"));
            return -1;
          }
        }
        r |= SOAP_TCP_SELECT_PIP;
      }
#endif
    }
    else if (r == 0)
    {
      soap->errnum = 0;
    }
    return r;
  }
#else
  {
    soap->error = SOAP_FD_EXCEEDED;
    return -1;
  }
#endif
#endif
  if (timeout > 0)
    retries = timeout - 1;
  do
  {
    rfd = sfd = efd = NULL;
#ifdef WITH_SELF_PIPE
    if ((flags & SOAP_TCP_SELECT_PIP) || (flags & SOAP_TCP_SELECT_RCV))
    {
      rfd = &fd[0];
      FD_ZERO(rfd);
      if ((flags & SOAP_TCP_SELECT_PIP))
        FD_SET(soap->pipe_fd[0], rfd);
      if ((flags & SOAP_TCP_SELECT_RCV))
        FD_SET(sk, rfd);
    }
#else
    if ((flags & SOAP_TCP_SELECT_RCV))
    {
      rfd = &fd[0];
      FD_ZERO(rfd);
      FD_SET(sk, rfd);
    }
#endif
    if ((flags & SOAP_TCP_SELECT_SND))
    {
      sfd = &fd[1];
      FD_ZERO(sfd);
      FD_SET(sk, sfd);
    }
    if ((flags & SOAP_TCP_SELECT_ERR))
    {
      efd = &fd[2];
      FD_ZERO(efd);
      FD_SET(sk, efd);
    }
    if (timeout <= 0)
    {
      tv.tv_sec = -timeout / 1000000;
      tv.tv_usec = -timeout % 1000000;
    }
    else
    {
      tv.tv_sec = 1;
      tv.tv_usec = 0;
    }
#ifdef WITH_SELF_PIPE
    r = select((int)(sk > soap->pipe_fd[0] ? sk : soap->pipe_fd[0]) + 1, rfd, sfd, efd, &tv);
#else
    r = select((int)sk + 1, rfd, sfd, efd, &tv);
#endif
    if (r < 0 && (soap->errnum = soap_socket_errno) == SOAP_EINTR && eintr > 0)
    {
      eintr--;
      r = 0;
    }
    else if (retries-- <= 0)
    {
      break;
    }
  } while (r == 0);
  if (r > 0)
  {
    r = 0;
    if ((flags & SOAP_TCP_SELECT_RCV) && FD_ISSET(sk, rfd))
      r |= SOAP_TCP_SELECT_RCV;
    if ((flags & SOAP_TCP_SELECT_SND) && FD_ISSET(sk, sfd))
      r |= SOAP_TCP_SELECT_SND;
    if ((flags & SOAP_TCP_SELECT_ERR) && FD_ISSET(sk, efd))
      r |= SOAP_TCP_SELECT_ERR;
#ifdef WITH_SELF_PIPE
    if ((flags & SOAP_TCP_SELECT_PIP) && FD_ISSET(soap->pipe_fd[0], rfd))
    {
      char ch;
      for (;;)
      {
        if (read(soap->pipe_fd[0], &ch, 1) == -1)
        {
          if (soap_socket_errno == SOAP_EAGAIN)
            break;
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Self pipe read error\n"));
          return -1;
        }
      }
      r |= SOAP_TCP_SELECT_PIP;
    }
#endif
  }
  else if (r == 0)
  {
    soap->errnum = 0;
  }
  return r;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static SOAP_SOCKET
tcp_accept(struct soap *soap, SOAP_SOCKET sk, struct sockaddr *addr, int *len)
{
  SOAP_SOCKET s;
  (void)soap;
  s = accept(sk, addr, (SOAP_SOCKLEN_T*)len); /* portability note: see SOAP_SOCKLEN_T definition in stdsoap2.h */
#ifdef WITH_SOCKET_CLOSE_ON_EXIT
#ifdef WIN32
#ifndef UNDER_CE
  SetHandleInformation((HANDLE)s, HANDLE_FLAG_INHERIT, 0);
#endif
#else
  fcntl(s, F_SETFD, FD_CLOEXEC);
#endif
#endif
  return s;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static int
tcp_disconnect(struct soap *soap)
{
#ifdef WITH_OPENSSL
  if (soap->ssl)
  {
    int r;
    if (soap->session)
    {
      SSL_SESSION_free(soap->session);
      soap->session = NULL;
    }
    if (*soap->host)
    {
      soap->session = SSL_get1_session(soap->ssl);
      if (soap->session)
      {
        soap_strcpy(soap->session_host, sizeof(soap->session_host), soap->host);
        soap->session_port = soap->port;
      }
    }
    if (soap_valid_socket(soap->socket))
    {
      r = SSL_shutdown(soap->ssl);
      /* SSL shutdown does not work when reads are pending, non-blocking */
      if (r == 0)
      {
        while (SSL_want_read(soap->ssl))
        {
          if (SSL_read(soap->ssl, NULL, 0)
              || soap_socket_errno != SOAP_EAGAIN)
          {
            r = SSL_shutdown(soap->ssl);
            break;
          }
        }
      }
      if (r == 0 && !soap->fshutdownsocket(soap, soap->socket, SOAP_SHUT_WR))
      {
#if !defined(WITH_LEAN) && !defined(WIN32)
        /*
           wait up to 5 seconds for close_notify to be sent by peer (if peer not
           present, this avoids calling SSL_shutdown() which has a lengthy return
           timeout)
         */
        r = tcp_select(soap, soap->socket, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, 5);
        if (r <= 0)
        {
          soap->errnum = 0;
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Connection lost...\n"));
          (void)soap->fclosesocket(soap, soap->socket);
          soap->socket = SOAP_INVALID_SOCKET;
          ERR_clear_error();
          SSL_free(soap->ssl);
          soap->ssl = NULL;
          return SOAP_OK;
        }
#else
        r = SSL_shutdown(soap->ssl);
        if (r <= 0)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Shutdown failed: %d\n", SSL_get_error(soap->ssl, r)));
          if (soap_valid_socket(soap->socket) && !(soap->omode & SOAP_IO_UDP))
          {
            (void)soap->fclosesocket(soap, soap->socket);
            soap->socket = SOAP_INVALID_SOCKET;
          }
        }
#endif
      }
    }
    SSL_free(soap->ssl);
    soap->ssl = NULL;
    ERR_clear_error();
  }
#endif
#ifdef WITH_GNUTLS
  if (soap->session)
  {
    gnutls_bye(soap->session, GNUTLS_SHUT_RDWR);
    gnutls_deinit(soap->session);
    soap->session = NULL;
  }
#endif
#ifdef WITH_SYSTEMSSL
  if (soap->ssl)
  {
    gsk_secure_socket_shutdown(soap->ssl);
    gsk_secure_socket_close(&soap->ssl);
  }
#endif
  if (soap_valid_socket(soap->socket) && !(soap->omode & SOAP_IO_UDP))
  {
    (void)soap->fshutdownsocket(soap, soap->socket, SOAP_SHUT_RDWR);
    (void)soap->fclosesocket(soap, soap->socket);
    soap->socket = SOAP_INVALID_SOCKET;
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static int
tcp_closesocket(struct soap *soap, SOAP_SOCKET sk)
{
  (void)soap;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Close socket=%d\n", (int)sk));
  return soap_closesocket(sk);
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static int
tcp_shutdownsocket(struct soap *soap, SOAP_SOCKET sk, int how)
{
  (void)soap;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Shutdown socket=%d how=%d\n", (int)sk, how));
  return shutdown(sk, how);
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
SOAP_FMAC1
SOAP_SOCKET
SOAP_FMAC2
soap_bind(struct soap *soap, const char *host, int port, int backlog)
{
#if defined(WITH_IPV6)
  struct addrinfo *addrinfo = NULL;
  struct addrinfo hints;
  struct addrinfo res;
  int err;
  int set = 1;
  int unset = 0;
#elif !defined(WITH_LEAN)
  int set = 1;
#endif
  if (soap_valid_socket(soap->master))
  {
    (void)soap->fclosesocket(soap, soap->master);
    soap->master = SOAP_INVALID_SOCKET;
  }
  soap->socket = SOAP_INVALID_SOCKET;
  soap->errnum = 0;
  soap->errmode = 1;
  if (tcp_init(soap))
  {
    (void)soap_set_receiver_error(soap, tcp_error(soap), "TCP init failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
#ifdef WITH_IPV6
  memset((void*)&hints, 0, sizeof(hints));
  hints.ai_family = soap->bind_inet6 ? AF_INET6 : PF_UNSPEC;
#ifndef WITH_LEAN
  if ((soap->omode & SOAP_IO_UDP))
    hints.ai_socktype = SOCK_DGRAM;
  else
#endif
    hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;
  soap->errmode = 2;
  err = getaddrinfo(host, soap_int2s(soap, port), &hints, &addrinfo);
  if (err || !addrinfo)
  {
    (void)soap_set_receiver_error(soap, SOAP_GAI_STRERROR(err), "getaddrinfo failed in soap_bind()", SOAP_TCP_ERROR);
    if (addrinfo)
      freeaddrinfo(addrinfo);
    return SOAP_INVALID_SOCKET;
  }
  res = *addrinfo;
  if (soap_memcpy((void*)&soap->peer.storage, sizeof(soap->peer.storage), (const void*)addrinfo->ai_addr, addrinfo->ai_addrlen))
  {
    freeaddrinfo(addrinfo);
    soap->error = SOAP_EOM;
    return SOAP_INVALID_SOCKET;
  }
  soap->peerlen = addrinfo->ai_addrlen;
  res.ai_addr = &soap->peer.addr;
  res.ai_addrlen = soap->peerlen;
  freeaddrinfo(addrinfo);
  soap->master = (int)socket(res.ai_family, res.ai_socktype, res.ai_protocol);
#else
#ifndef WITH_LEAN
  if ((soap->omode & SOAP_IO_UDP))
    soap->master = (int)socket(AF_INET, SOCK_DGRAM, 0);
  else
#endif
    soap->master = (int)socket(AF_INET, SOCK_STREAM, 0);
#endif
  soap->errmode = 0;
  if (!soap_valid_socket(soap->master))
  {
    soap->errnum = soap_socket_errno;
    (void)soap_set_receiver_error(soap, tcp_error(soap), "socket failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
  soap->port = port;
#ifndef WITH_LEAN
  if ((soap->omode & SOAP_IO_UDP))
    soap->socket = soap->master;
#endif
#ifdef WITH_SOCKET_CLOSE_ON_EXIT
#ifdef WIN32
#ifndef UNDER_CE
  SetHandleInformation((HANDLE)soap->master, HANDLE_FLAG_INHERIT, 0);
#endif
#else
  fcntl(soap->master, F_SETFD, 1);
#endif
#endif
#ifndef WITH_LEAN
  if (soap->bind_flags && setsockopt(soap->master, SOL_SOCKET, soap->bind_flags, (char*)&set, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
#ifndef UNDER_CE
  if (((soap->imode | soap->omode) & SOAP_IO_KEEPALIVE) && (!((soap->imode | soap->omode) & SOAP_IO_UDP)) && setsockopt(soap->master, SOL_SOCKET, SO_KEEPALIVE, (char*)&set, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_KEEPALIVE failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
  if (soap->sndbuf > 0 && setsockopt(soap->master, SOL_SOCKET, SO_SNDBUF, (char*)&soap->sndbuf, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_SNDBUF failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
  if (soap->rcvbuf > 0 && setsockopt(soap->master, SOL_SOCKET, SO_RCVBUF, (char*)&soap->rcvbuf, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_RCVBUF failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
#ifdef TCP_NODELAY
  if (!(soap->omode & SOAP_IO_UDP) && setsockopt(soap->master, IPPROTO_TCP, TCP_NODELAY, (char*)&set, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt TCP_NODELAY failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
#endif
#ifdef TCP_FASTOPEN
  if (!(soap->omode & SOAP_IO_UDP) && setsockopt(soap->master, IPPROTO_TCP, TCP_FASTOPEN, (char*)&set, sizeof(int)))
  {
    /* silently ignore */
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "setsockopt TCP_FASTOPEN failed in soap_bind()\n"));
  }
#endif
#endif
#endif
#ifdef WITH_IPV6
  if (res.ai_family == AF_INET6 && setsockopt(soap->master, IPPROTO_IPV6, IPV6_V6ONLY, soap->bind_v6only ? (char*)&set : (char*)&unset, sizeof(int)))
  {
    soap->errnum = soap_socket_errno;
    (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt IPV6_V6ONLY failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
  soap->errmode = 0;
  if (bind(soap->master, res.ai_addr, (int)res.ai_addrlen))
  {
    soap->errnum = soap_socket_errno;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not bind to host, bind failed\n"));
    (void)soap_closesock(soap);
    (void)soap_set_receiver_error(soap, tcp_error(soap), "bind failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
#else
  soap->peerlen = sizeof(soap->peer.in);
  memset((void*)&soap->peer.in, 0, sizeof(soap->peer.in));
  soap->peer.in.sin_family = AF_INET;
  soap->errmode = 2;
  if (host)
  {
    if (soap->fresolve(soap, host, &soap->peer.in.sin_addr))
    {
      (void)soap_set_receiver_error(soap, tcp_error(soap), "get host by name failed in soap_bind()", SOAP_TCP_ERROR);
      return SOAP_INVALID_SOCKET;
    }
  }
  else
  {
    soap->peer.in.sin_addr.s_addr = htonl(INADDR_ANY);
  }
  soap->peer.in.sin_port = htons((short)port);
  soap->errmode = 0;
  if (bind(soap->master, &soap->peer.addr, (int)soap->peerlen))
  {
    soap->errnum = soap_socket_errno;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not bind to host, bind failed\n"));
    (void)soap_closesock(soap);
    (void)soap_set_receiver_error(soap, tcp_error(soap), "bind failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
#endif
  if (!(soap->omode & SOAP_IO_UDP) && listen(soap->master, backlog))
  {
    soap->errnum = soap_socket_errno;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not bind to host, listen failed\n"));
    (void)soap_closesock(soap);
    (void)soap_set_receiver_error(soap, tcp_error(soap), "listen failed in soap_bind()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
  return soap->master;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
SOAP_FMAC1
int
SOAP_FMAC2
soap_poll(struct soap *soap)
{
#ifndef WITH_LEAN
  int r;
  if (soap_valid_socket(soap->socket))
  {
    r = tcp_select(soap, soap->socket, SOAP_TCP_SELECT_ALL, 0);
    if (r > 0 && (r & SOAP_TCP_SELECT_ERR))
      r = -1;
  }
  else if (soap_valid_socket(soap->master))
  {
    r = tcp_select(soap, soap->master, SOAP_TCP_SELECT_SND | SOAP_TCP_SELECT_ERR, 0);
  }
  else
  {
    return SOAP_OK; /* OK when no socket! */
  }
  if (r > 0)
  {
    int t;
#ifdef WITH_OPENSSL
    if ((soap->imode & SOAP_ENC_SSL) && soap->ssl)
    {
      if (soap_valid_socket(soap->socket)
       && (r & SOAP_TCP_SELECT_SND)
       && (!(r & SOAP_TCP_SELECT_RCV)
        || SSL_peek(soap->ssl, (char*)&t, 1) > 0))
        return SOAP_OK;
    }
    else
#endif
    {
      if (soap_valid_socket(soap->socket)
       && (r & SOAP_TCP_SELECT_SND)
       && (!(r & SOAP_TCP_SELECT_RCV)
        || recv(soap->socket, (char*)&t, 1, MSG_PEEK) > 0))
        return SOAP_OK;
    }
  }
  else if (r < 0)
  {
    if ((soap_valid_socket(soap->master) && soap_socket_errno != SOAP_EINTR)
     || (soap_valid_socket(soap->socket) && soap_socket_errno != SOAP_EINTR))
      return soap_set_receiver_error(soap, tcp_error(soap), "select failed in soap_poll()", SOAP_TCP_ERROR);
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "soap_poll: other end down on socket=%d select=%d\n", (int)soap->socket, r));
  return SOAP_EOF;
#else
  (void)soap;
  return SOAP_OK;
#endif
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
SOAP_FMAC1
int
SOAP_FMAC2
soap_ready(struct soap *soap)
{
#ifndef WITH_LEAN
  int r;
  if (!soap_valid_socket(soap->socket))
    return SOAP_OK; /* OK when no socket! */
  r = tcp_select(soap, soap->socket, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_ERR, 0);
  if (r > 0 && (r & SOAP_TCP_SELECT_ERR))
    r = -1;
  if (r < 0 && soap_socket_errno != SOAP_EINTR)
    return soap_set_receiver_error(soap, tcp_error(soap), "select failed in soap_ready()", SOAP_TCP_ERROR);
  if (r > 0)
  {
    char t;
#ifdef WITH_OPENSSL
    if ((soap->imode & SOAP_ENC_SSL) && soap->ssl)
    {
      if (SSL_peek(soap->ssl, &t, 1) > 0)
        return SOAP_OK;
    }
    else
#endif
    {
      if (recv(soap->socket, &t, 1, MSG_PEEK) > 0)
        return SOAP_OK;
    }
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "soap_ready: other end not ready to send on socket=%d select=%d\n", (int)soap->socket, r));
  return SOAP_EOF;
#else
  (void)soap;
  return SOAP_OK;
#endif
}
#endif
/******************************************************************************/

#ifndef WITH_NOIO
SOAP_FMAC1
SOAP_SOCKET
SOAP_FMAC2
soap_accept(struct soap *soap)
{
  int n = (int)sizeof(soap->peer);
  int err;
#ifndef WITH_LEAN
  int set = 1;
#endif
  soap->error = SOAP_OK;
  memset((void*)&soap->peer, 0, sizeof(soap->peer));
  soap->socket = SOAP_INVALID_SOCKET;
  soap->errmode = 0;
  soap_reset_errno;
  soap->errnum = 0;
  soap->keep_alive = 0;
  if (!soap_valid_socket(soap->master))
  {
    (void)soap_set_receiver_error(soap, tcp_error(soap), "no master socket in soap_accept()", SOAP_TCP_ERROR);
    return SOAP_INVALID_SOCKET;
  }
#ifndef WITH_LEAN
  if ((soap->omode & SOAP_IO_UDP))
    return soap->socket = soap->master;
#endif
  for (;;)
  {
    if (soap->accept_timeout)
    {
      for (;;)
      {
        int r;
        r = tcp_select(soap, soap->master, SOAP_TCP_SELECT_ALL, soap->accept_timeout);
        if (r > 0)
          break;
        if (!r)
        {
          (void)soap_set_receiver_error(soap, "Timeout", "accept failed in soap_accept()", SOAP_TCP_ERROR);
          return SOAP_INVALID_SOCKET;
        }
        if (r < 0)
        {
          r = soap->errnum;
          if (r != SOAP_EINTR)
          {
            (void)soap_closesock(soap);
            (void)soap_set_receiver_error(soap, tcp_error(soap), "accept failed in soap_accept()", SOAP_TCP_ERROR);
            return SOAP_INVALID_SOCKET;
          }
        }
      }
    }
    n = (int)sizeof(soap->peer);
    soap->socket = soap->faccept(soap, soap->master, &soap->peer.addr, &n);
    soap->peerlen = (size_t)n;
    if (soap_valid_socket(soap->socket))
    {
#ifdef WITH_IPV6
      char port[16];
      struct addrinfo *res = NULL;
      struct addrinfo hints;
      memset(&hints, 0, sizeof(struct addrinfo));
      hints.ai_family = PF_UNSPEC;
      hints.ai_socktype = SOCK_STREAM;
      hints.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV;
      getnameinfo(&soap->peer.addr, n, soap->host, sizeof(soap->host), port, sizeof(port), NI_NUMERICHOST | NI_NUMERICSERV);
      soap->ip = 0;
      soap->ip6[0] = 0;
      soap->ip6[1] = 0;
      soap->ip6[2] = 0;
      soap->ip6[3] = 0;
      if (getaddrinfo(soap->host, NULL, &hints, &res) == 0 && res)
      {
        struct sockaddr_storage result;
        memset((void*)&result, 0, sizeof(result));
        (void)soap_memcpy(&result, sizeof(result), res->ai_addr, res->ai_addrlen);
        freeaddrinfo(res);
        if (result.ss_family == AF_INET6)
        {
          struct sockaddr_in6 *addr = (struct sockaddr_in6*)&result;
          struct in6_addr *inaddr = &addr->sin6_addr;
          int i;
          for (i = 0; i < 16; i++)
            soap->ip6[i/4] = (soap->ip6[i/4] << 8) + inaddr->s6_addr[i];
        }
        else if (result.ss_family == AF_INET)
        {
          struct sockaddr_in *addr = (struct sockaddr_in*)&result;
          soap->ip = ntohl(addr->sin_addr.s_addr);
          soap->ip6[2] = 0xFFFF;
          soap->ip6[3] = soap->ip;
        }
      }
      soap->port = soap_strtol(port, NULL, 10);
#else
      soap->ip = ntohl(soap->peer.in.sin_addr.s_addr);
      soap->ip6[0] = 0;
      soap->ip6[1] = 0;
      soap->ip6[2] = 0xFFFF;
      soap->ip6[3] = soap->ip;
      (SOAP_SNPRINTF(soap->host, sizeof(soap->host), 80), "%u.%u.%u.%u", (int)(soap->ip>>24)&0xFF, (int)(soap->ip>>16)&0xFF, (int)(soap->ip>>8)&0xFF, (int)soap->ip&0xFF);
      soap->port = (int)ntohs(soap->peer.in.sin_port); /* does not return port number on some systems */
#endif
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Accept socket=%d at port=%d from IP='%s'\n", (int)soap->socket, soap->port, soap->host));
#ifndef WITH_LEAN
      if ((soap->accept_flags & SO_LINGER))
      {
        struct linger linger;
        memset((void*)&linger, 0, sizeof(linger));
        linger.l_onoff = 1;
        linger.l_linger = soap->linger_time;
        if (setsockopt(soap->socket, SOL_SOCKET, SO_LINGER, (char*)&linger, sizeof(struct linger)))
        {
          soap->errnum = soap_socket_errno;
          (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_LINGER failed in soap_accept()", SOAP_TCP_ERROR);
          (void)soap_closesock(soap);
          return SOAP_INVALID_SOCKET;
        }
      }
      if ((soap->accept_flags & ~SO_LINGER) && setsockopt(soap->socket, SOL_SOCKET, soap->accept_flags & ~SO_LINGER, (char*)&set, sizeof(int)))
      {
        soap->errnum = soap_socket_errno;
        (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt failed in soap_accept()", SOAP_TCP_ERROR);
        (void)soap_closesock(soap);
        return SOAP_INVALID_SOCKET;
      }
#ifndef UNDER_CE
      if (((soap->imode | soap->omode) & SOAP_IO_KEEPALIVE) && setsockopt(soap->socket, SOL_SOCKET, SO_KEEPALIVE, (char*)&set, sizeof(int)))
      {
        soap->errnum = soap_socket_errno;
        (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_KEEPALIVE failed in soap_accept()", SOAP_TCP_ERROR);
        (void)soap_closesock(soap);
        return SOAP_INVALID_SOCKET;
      }
      if (soap->sndbuf > 0 && setsockopt(soap->socket, SOL_SOCKET, SO_SNDBUF, (char*)&soap->sndbuf, sizeof(int)))
      {
        soap->errnum = soap_socket_errno;
        (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_SNDBUF failed in soap_accept()", SOAP_TCP_ERROR);
        (void)soap_closesock(soap);
        return SOAP_INVALID_SOCKET;
      }
      if (soap->rcvbuf > 0 && setsockopt(soap->socket, SOL_SOCKET, SO_RCVBUF, (char*)&soap->rcvbuf, sizeof(int)))
      {
        soap->errnum = soap_socket_errno;
        (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt SO_RCVBUF failed in soap_accept()", SOAP_TCP_ERROR);
        (void)soap_closesock(soap);
        return SOAP_INVALID_SOCKET;
      }
#ifdef TCP_NODELAY
      if (setsockopt(soap->socket, IPPROTO_TCP, TCP_NODELAY, (char*)&set, sizeof(int)))
      {
        soap->errnum = soap_socket_errno;
        (void)soap_set_receiver_error(soap, tcp_error(soap), "setsockopt TCP_NODELAY failed in soap_accept()", SOAP_TCP_ERROR);
        (void)soap_closesock(soap);
        return SOAP_INVALID_SOCKET;
      }
#endif
#endif
#endif
      soap->keep_alive = -(((soap->imode | soap->omode) & SOAP_IO_KEEPALIVE) != 0);
      if (soap->send_timeout || soap->recv_timeout)
        SOAP_SOCKNONBLOCK(soap->socket)
      else
        SOAP_SOCKBLOCK(soap->socket)
      return soap->socket;
    }
    err = soap_socket_errno;
    if (err != 0 && err != SOAP_EINTR && err != SOAP_EAGAIN && err != SOAP_EWOULDBLOCK)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Accept failed from %s\n", soap->host));
      soap->errnum = err;
      (void)soap_set_receiver_error(soap, tcp_error(soap), "accept failed in soap_accept()", SOAP_TCP_ERROR);
      (void)soap_closesock(soap);
      return SOAP_INVALID_SOCKET;
    }
  }
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_closesock(struct soap *soap)
{
  int status = soap->error;
  int err = SOAP_OK;
  soap->part = SOAP_END;
#ifndef WITH_LEANER
  if (status && status < 200) /* attachment state is not to be trusted */
  {
    soap->mime.first = NULL;
    soap->mime.last = NULL;
    soap->dime.first = NULL;
    soap->dime.last = NULL;
  }
#endif
  if (soap->fdisconnect)
    err = soap->fdisconnect(soap);
  if (err || status == SOAP_EOF || status == SOAP_TCP_ERROR || status == SOAP_SSL_ERROR || !soap->keep_alive)
  {
    soap->keep_alive = 0;
    if (soap->fclose && (soap->error = soap->fclose(soap)) != SOAP_OK)
      return soap->error;
    if (err)
      return soap->error = err;
  }
#ifdef WITH_ZLIB
  if (!(soap->mode & SOAP_MIME_POSTCHECK))
  {
    if (soap->zlib_state == SOAP_ZLIB_DEFLATE)
      deflateEnd(soap->d_stream);
    else if (soap->zlib_state == SOAP_ZLIB_INFLATE)
      inflateEnd(soap->d_stream);
    soap->zlib_state = SOAP_ZLIB_NONE;
  }
#endif
  return soap->error = status;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_force_closesock(struct soap *soap)
{
  soap->keep_alive = 0;
  if (soap_valid_socket(soap->socket) && soap->fclosesocket)
  {
    (void)soap->fclosesocket(soap, soap->socket);
    soap->socket = SOAP_INVALID_SOCKET;
  }
  return soap->error;
}

/******************************************************************************/

#ifdef WITH_SELF_PIPE
SOAP_FMAC1
void
SOAP_FMAC2
soap_close_connection(struct soap *soap)
{
  if (soap_valid_socket(soap->socket))
    write(soap->pipe_fd[1], "1", 1);
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
SOAP_FMAC1
void
SOAP_FMAC2
soap_cleanup(struct soap *soap)
{
  soap_done(soap);
#ifdef WIN32
  if (!tcp_done)
    return;
  tcp_done = 0;
  WSACleanup();
#endif
}
#endif

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_done(struct soap *soap)
{
#ifdef SOAP_DEBUG
  int i;
#endif
  if (soap_check_state(soap))
    return;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Done with context%s\n", soap->state == SOAP_COPY ? " copy" : ""));
  soap_free_temp(soap);
#ifdef SOAP_DEBUG
  if (soap->clist)
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Warning: managed C++ data was not deallocated with soap_destroy() from the heap managed by context %p\n", (void*)soap));
  if (soap->alist)
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Warning: managed C data was not deallocated with soap_end() from the heap managed by context %p\n", (void*)soap));
#endif
  soap->alist = NULL;
  while (soap->clist)
  {
    struct soap_clist *p = soap->clist->next;
    SOAP_FREE(soap, soap->clist);
    soap->clist = p;
  }
  if (soap->state == SOAP_INIT)
    soap->omode &= ~SOAP_IO_UDP; /* to force close the socket */
  soap->keep_alive = 0; /* to force close the socket */
  if (soap->master == soap->socket) /* do not close twice */
    soap->master = SOAP_INVALID_SOCKET;
  (void)soap_closesock(soap);
#ifdef WITH_COOKIES
  soap_free_cookies(soap);
#endif
  while (soap->plugins)
  {
    struct soap_plugin *p = soap->plugins->next;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Removing plugin '%s'\n", soap->plugins->id));
    if (soap->plugins->fcopy || soap->state == SOAP_INIT)
      soap->plugins->fdelete(soap, soap->plugins);
    SOAP_FREE(soap, soap->plugins);
    soap->plugins = p;
  }
  soap->fplugin = fplugin;
#ifndef WITH_NOHTTP
  soap->fpost = http_post;
  soap->fget = http_get;
  soap->fput = http_put;
  soap->fpatch = http_patch;
  soap->fdel = http_del;
  soap->fopt = http_200;
  soap->fhead = http_200;
  soap->fform = NULL;
  soap->fposthdr = http_post_header;
  soap->fresponse = http_response;
  soap->fparse = http_parse;
  soap->fparsehdr = http_parse_header;
#endif
  soap->fheader = NULL;
#ifndef WITH_NOIO
#ifndef WITH_IPV6
  soap->fresolve = tcp_gethost;
#else
  soap->fresolve = NULL;
#endif
  soap->faccept = tcp_accept;
  soap->fopen = tcp_connect;
  soap->fclose = tcp_disconnect;
  soap->fclosesocket = tcp_closesocket;
  soap->fshutdownsocket = tcp_shutdownsocket;
  soap->fsend = fsend;
  soap->frecv = frecv;
  soap->fpoll = soap_poll;
#else
  soap->fopen = NULL;
  soap->fclose = NULL;
  soap->fpoll = NULL;
#endif
#ifndef WITH_LEANER
  soap->fsvalidate = NULL;
  soap->fwvalidate = NULL;
  soap->feltbegin = NULL;
  soap->feltendin = NULL;
  soap->feltbegout = NULL;
  soap->feltendout = NULL;
  soap->fprepareinitsend = NULL;
  soap->fprepareinitrecv = NULL;
  soap->fpreparesend = NULL;
  soap->fpreparerecv = NULL;
  soap->fpreparefinalsend = NULL;
  soap->fpreparefinalrecv = NULL;
  soap->ffiltersend = NULL;
  soap->ffilterrecv = NULL;
#endif
  soap->fseterror = NULL;
  soap->fignore = NULL;
  soap->fserveloop = NULL;
#ifdef WITH_OPENSSL
  if (soap->session)
  {
    SSL_SESSION_free(soap->session);
    soap->session = NULL;
  }
#endif
  if (soap->state == SOAP_INIT)
  {
    if (soap_valid_socket(soap->master))
    {
      (void)soap->fclosesocket(soap, soap->master);
      soap->master = SOAP_INVALID_SOCKET;
    }
  }
#ifdef WITH_OPENSSL
  if (soap->ssl)
  {
    SSL_free(soap->ssl);
    soap->ssl = NULL;
  }
  if (soap->state == SOAP_INIT)
  {
    if (soap->ctx)
    {
      SSL_CTX_free(soap->ctx);
      soap->ctx = NULL;
    }
  }
  ERR_clear_error();
# if OPENSSL_VERSION_NUMBER >= 0x10100000L && !defined(LIBRESSL_VERSION_NUMBER)
  /* OpenSSL libraries handle thread init and deinit */
# elif OPENSSL_VERSION_NUMBER >= 0x10000000L
  ERR_remove_thread_state(NULL);
# else
  ERR_remove_state(0);
# endif
#endif
#ifdef WITH_GNUTLS
  if (soap->state == SOAP_INIT)
  {
    if (soap->xcred)
    {
      gnutls_certificate_free_credentials(soap->xcred);
      soap->xcred = NULL;
    }
    if (soap->acred)
    {
      gnutls_anon_free_client_credentials(soap->acred);
      soap->acred = NULL;
    }
    if (soap->cache)
    {
      gnutls_priority_deinit(soap->cache);
      soap->cache = NULL;
    }
    if (soap->dh_params)
    {
      gnutls_dh_params_deinit(soap->dh_params);
      soap->dh_params = NULL;
    }
# if GNUTLS_VERSION_NUMBER < 0x030300
    if (soap->rsa_params)
    {
      gnutls_rsa_params_deinit(soap->rsa_params);
      soap->rsa_params = NULL;
    }
#endif
  }
  if (soap->session)
  {
    gnutls_deinit(soap->session);
    soap->session = NULL;
  }
#endif
#ifdef WITH_SYSTEMSSL
  if (soap->ssl)
    gsk_secure_socket_close(&soap->ssl);
  if (soap->state == SOAP_INIT)
    if (soap->ctx)
      gsk_environment_close(&soap->ctx);
#endif  
#ifdef WITH_C_LOCALE
  SOAP_FREELOCALE(soap);
#endif
#ifdef WITH_ZLIB
  if (soap->d_stream)
  {
    SOAP_FREE(soap, soap->d_stream);
    soap->d_stream = NULL;
  }
  if (soap->z_buf)
  {
    SOAP_FREE(soap, soap->z_buf);
    soap->z_buf = NULL;
  }
#endif
#ifdef SOAP_DEBUG
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Free logfiles\n"));
  for (i = 0; i < SOAP_MAXLOGS; i++)
  {
    soap_close_logfile(soap, i);
    if (soap->logfile[i])
    {
      SOAP_FREE_UNMANAGED(soap->logfile[i]);
      soap->logfile[i] = NULL;
    }
  }
#endif
#ifdef WITH_SELF_PIPE
  close(soap->pipe_fd[0]);
  close(soap->pipe_fd[1]);
#endif
#ifdef SOAP_MEM_DEBUG
  soap_free_mht(soap);
#endif
  soap->state = SOAP_NONE;
}

/******************************************************************************\
 *
 *      HTTP
 *
\******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_parse(struct soap *soap)
{
  char header[SOAP_HDRLEN], *s;
  int err = SOAP_OK;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Waiting for HTTP request/response...\n"));
  soap->fform = NULL;
  *soap->endpoint = '\0';
  soap->bearer = NULL;
#ifdef WITH_NTLM
  if (!soap->ntlm_challenge)
#endif
  {
    soap->userid = NULL;
    soap->passwd = NULL;
    soap->authrealm = NULL;
  }
#ifdef WITH_NTLM
  soap->ntlm_challenge = NULL;
#endif
  soap->proxy_from = NULL;
  soap->cors_origin = NULL;
  soap->cors_method = NULL;
  soap->cors_header = NULL;
  do
  {
    soap->length = 0;
    soap->http_content = NULL;
    soap->action = NULL;
    soap->status = 0;
    soap->body = 1;
    if (soap_getline(soap, soap->msgbuf, sizeof(soap->msgbuf)))
    {
      if (soap->error == SOAP_EOF)
        return SOAP_EOF;
      return soap->error = 414;
    }
    s = strchr(soap->msgbuf, ' ');
    if (s)
    {
      soap->status = (unsigned short)soap_strtoul(s, &s, 10);
      if (!soap_coblank((soap_wchar)*s))
        soap->status = 0;
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "HTTP status: %s\n", soap->msgbuf));
    for (;;)
    {
      if (soap_getline(soap, header, SOAP_HDRLEN))
      {
        if (soap->error == SOAP_EOF)
        {
          soap->error = SOAP_OK;
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EOF in HTTP header, try to continue anyway\n"));
          break;
        }
        return soap->error;
      }
      if (!*header)
        break;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "HTTP header: %s\n", header));
      s = strchr(header, ':');
      if (s)
      {
        char *t;
        *s = '\0';
        do
        {
          s++;
        } while (*s && *s <= 32);
        if (*s == '"')
          s++;
        t = s + strlen(s) - 1;
        while (t > s && *t <= 32)
          t--;
        if (t >= s && *t == '"')
          t--;
        t[1] = '\0';
        soap->error = soap->fparsehdr(soap, header, s);
        if (soap->error)
        {
          if (soap->error < SOAP_STOP)
            return soap->error;
          err = soap->error;
          soap->error = SOAP_OK;
        }
      }
    }
  } while (soap->status == 100);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Finished HTTP header parsing, status = %d\n", soap->status));
  s = strstr(soap->msgbuf, "HTTP/");
  if (s && s[5] == '1' && s[6] == '.' && s[7] == '0')
  {
    soap->keep_alive = 0; /* HTTP 1.0 does not support keep-alive */
    if (soap->status == 0 && (soap->omode & SOAP_IO) == SOAP_IO_CHUNK) /* soap->status == 0 for HTTP request */
      soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_STORE; /* HTTP 1.0 does not support chunked transfers */
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Keep alive connection = %d\n", soap->keep_alive));
  if (soap->status == 0)
  {
    size_t l = 0;
    if (s)
    {
      if (!strncmp(soap->msgbuf, "POST ", l = 5))
        soap->status = SOAP_POST;
      else if (!strncmp(soap->msgbuf, "GET ", l = 4))
        soap->status = SOAP_GET;
      else if (!strncmp(soap->msgbuf, "PUT ", l = 4))
        soap->status = SOAP_PUT;
      else if (!strncmp(soap->msgbuf, "PATCH ", l = 4))
        soap->status = SOAP_PATCH;
      else if (!strncmp(soap->msgbuf, "DELETE ", l = 7))
        soap->status = SOAP_DEL;
      else if (!strncmp(soap->msgbuf, "HEAD ", l = 5))
        soap->status = SOAP_HEAD;
      else if (!strncmp(soap->msgbuf, "OPTIONS ", l = 8))
        soap->status = SOAP_OPTIONS;
    }
    if (s && soap->status)
    {
      size_t m, n, k;
      int r;
      while (soap->msgbuf[l] && soap_coblank((soap_wchar)soap->msgbuf[l]))
        l++;
      m = strlen(soap->endpoint);
      n = m + (s - soap->msgbuf) - l - 1;
      if (n >= sizeof(soap->endpoint))
        n = sizeof(soap->endpoint) - 1;
      if (m > n)
        m = n;
      k = n - m + 1;
      if (k >= sizeof(soap->path))
        k = sizeof(soap->path) - 1;
      while (k > 0 && soap_coblank((soap_wchar)soap->msgbuf[l + k - 1]))
        k--;
      if (soap_strncpy(soap->path, sizeof(soap->path), soap->msgbuf + l, k))
        return soap->error = 414;
      if (*soap->path && *soap->path != '/')
        r = soap_strncpy(soap->endpoint, sizeof(soap->endpoint), soap->path, k);
      else
        r = soap_strncat(soap->endpoint, sizeof(soap->endpoint), soap->path, k);
      if (r)
        return soap->error = 414;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Target endpoint='%s' path='%s'\n", soap->endpoint, soap->path));
      if (err)
        return soap->error = err;
    }
    else if (err)
    {
      return soap->error = err;
    }
    else if (s)
    {
      return soap->error = 405;
    }
    return SOAP_OK;
  }
  if ((soap->status >= 200 && soap->status <= 299) /* OK, Accepted, etc */
   || soap->status == 400                          /* Bad Request */
   || soap->status == 500)                         /* Internal Server Error */
    return soap->error = SOAP_OK;
  return soap->error = soap->status;
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_parse_header(struct soap *soap, const char *key, const char *val)
{
  if (!soap_tag_cmp(key, "Host"))
  {
#if defined(WITH_OPENSSL) || defined(WITH_GNUTLS)
    if ((soap->imode & SOAP_ENC_SSL))
      soap_strcpy(soap->endpoint, sizeof(soap->endpoint), "https://");
    else
#endif
      soap_strcpy(soap->endpoint, sizeof(soap->endpoint), "http://");
    if (soap_strncat(soap->endpoint, sizeof(soap->endpoint), val, sizeof(soap->endpoint) - 9))
      return soap->error = SOAP_HDR;
  }
#ifndef WITH_LEANER
  else if (!soap_tag_cmp(key, "Content-Type"))
  {
    const char *action;
    soap->http_content = soap_strdup(soap, val);
    if (soap_http_header_attribute(soap, val, "application/dime"))
      soap->imode |= SOAP_ENC_DIME;
    else if (soap_http_header_attribute(soap, val, "multipart/related")
          || soap_http_header_attribute(soap, val, "multipart/form-data"))
    {
      const char *type;
      soap->mime.boundary = soap_strdup(soap, soap_http_header_attribute(soap, val, "boundary"));
      soap->mime.start = soap_strdup(soap, soap_http_header_attribute(soap, val, "start"));
      soap->imode |= SOAP_ENC_MIME;
      type = soap_http_header_attribute(soap, val, "type");
      if (type && !strcmp(type, "application/xop+xml"))
        soap->imode |= SOAP_ENC_MTOM;
    }
    action = soap_http_header_attribute(soap, val, "action");
    if (action)
    {
      if (*action == '"')
      {
        soap->action = soap_strdup(soap, action + 1);
        if (soap->action && *soap->action)
          soap->action[strlen(soap->action) - 1] = '\0';
      }
      else
        soap->action = soap_strdup(soap, action);
    }
  }
#endif
  else if (!soap_tag_cmp(key, "Content-Length"))
  {
    soap->length = soap_strtoull(val, NULL, 10);
    if (soap->length == 0)
      soap->body = 0;
  }
  else if (!soap_tag_cmp(key, "Content-Encoding"))
  {
    if (!soap_tag_cmp(val, "deflate"))
#ifdef WITH_ZLIB
      soap->zlib_in = SOAP_ZLIB_DEFLATE;
#else
      return SOAP_ZLIB_ERROR;
#endif
    else if (!soap_tag_cmp(val, "gzip"))
#ifdef WITH_GZIP
      soap->zlib_in = SOAP_ZLIB_GZIP;
#else
      return SOAP_ZLIB_ERROR;
#endif
  }
#ifdef WITH_ZLIB
  else if (!soap_tag_cmp(key, "Accept-Encoding"))
  {
#ifdef WITH_GZIP
    if (strchr(val, '*') || soap_http_header_attribute(soap, val, "gzip"))
      soap->zlib_out = SOAP_ZLIB_GZIP;
    else
#endif
    if (strchr(val, '*') || soap_http_header_attribute(soap, val, "deflate"))
      soap->zlib_out = SOAP_ZLIB_DEFLATE;
    else
      soap->zlib_out = SOAP_ZLIB_NONE;
  }
#endif
  else if (!soap_tag_cmp(key, "Transfer-Encoding"))
  {
    soap->imode &= ~SOAP_IO;
    if (!soap_tag_cmp(val, "chunked"))
      soap->imode |= SOAP_IO_CHUNK;
  }
  else if (!soap_tag_cmp(key, "Connection"))
  {
    if (!soap_tag_cmp(val, "close"))
      soap->keep_alive = 0;
  }
#if !defined(WITH_LEAN) || defined(WITH_NTLM)
  else if (!soap_tag_cmp(key, "Authorization") || !soap_tag_cmp(key, "Proxy-Authorization"))
  {
#ifdef WITH_NTLM
    if (!soap_tag_cmp(val, "NTLM*"))
    {
      soap->ntlm_challenge = soap_strdup(soap, val + 4);
    }
    else
#endif
    if (!soap_tag_cmp(val, "Bearer *"))
    {
      soap->bearer = soap_strdup(soap, val + 7);
    }
    else if (!soap_tag_cmp(val, "Basic *"))
    {
      int n;
      char *s;
      soap_base642s(soap, val + 6, soap->tmpbuf, sizeof(soap->tmpbuf) - 1, &n);
      soap->tmpbuf[n] = '\0';
      s = strchr(soap->tmpbuf, ':');
      if (s)
      {
        *s = '\0';
        soap->userid = soap_strdup(soap, soap->tmpbuf);
        soap->passwd = soap_strdup(soap, s + 1);
      }
    }
  }
  else if (!soap_tag_cmp(key, "WWW-Authenticate") || !soap_tag_cmp(key, "Proxy-Authenticate"))
  {
#ifdef WITH_NTLM
    if (!soap_tag_cmp(val, "NTLM*"))
      soap->ntlm_challenge = soap_strdup(soap, val + 4);
    else
#endif
      soap->authrealm = soap_strdup(soap, soap_http_header_attribute(soap, val + 6, "realm"));
  }
  else if (!soap_tag_cmp(key, "Expect"))
  {
    if (!soap_tag_cmp(val, "100-continue"))
    {
      if ((soap->error = soap->fposthdr(soap, "HTTP/1.1 100 Continue", NULL)) != SOAP_OK
       || (soap->error = soap->fposthdr(soap, NULL, NULL)) != SOAP_OK)
        return soap->error;
    }
  }
#endif
  else if (!soap_tag_cmp(key, "SOAPAction"))
  {
    if (*val == '"')
    {
      soap->action = soap_strdup(soap, val + 1);
      if (*soap->action)
        soap->action[strlen(soap->action) - 1] = '\0';
    }
    else
      soap->action = soap_strdup(soap, val);
  }
  else if (!soap_tag_cmp(key, "Location"))
  {
    soap_strcpy(soap->endpoint, sizeof(soap->endpoint), val);
  }
  else if (!soap_tag_cmp(key, "X-Forwarded-For"))
  {
    soap->proxy_from = soap_strdup(soap, val);
  }
  else if (!soap_tag_cmp(key, "Origin"))
  {
    soap->origin = soap_strdup(soap, val);
    soap->cors_origin = soap->cors_allow;
  }
  else if (!soap_tag_cmp(key, "Access-Control-Request-Method"))
  {
    soap->cors_method = soap_strdup(soap, val);
  }
  else if (!soap_tag_cmp(key, "Access-Control-Request-Headers"))
  {
    soap->cors_header = soap_strdup(soap, val);
  }
#ifdef WITH_COOKIES
  else if (!soap_tag_cmp(key, "Cookie")
        || !soap_tag_cmp(key, "Cookie2")
        || !soap_tag_cmp(key, "Set-Cookie")
        || !soap_tag_cmp(key, "Set-Cookie2"))
  {
    soap_getcookies(soap, val);
  }
#endif
  return SOAP_OK;
}
#endif

/******************************************************************************/

#if !defined(WITH_NOHTTP) || !defined(WITH_LEANER)
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_http_header_attribute(struct soap *soap, const char *line, const char *key)
{
  const char *s = line;
  if (s)
  {
    while (*s)
    {
      short flag;
      s = soap_decode_key(soap->tmpbuf, sizeof(soap->tmpbuf), s);
      flag = soap_tag_cmp(soap->tmpbuf, key);
      s = soap_decode_val(soap->tmpbuf, sizeof(soap->tmpbuf), s);
      if (!flag)
        return soap->tmpbuf;
    }
  }
  return NULL;
}
#endif

/******************************************************************************/

#if !defined(WITH_NOHTTP) || !defined(WITH_LEANER)
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_decode_key(char *buf, size_t len, const char *val)
{
  return soap_decode(buf, len, val, "=,;");
}
#endif

/******************************************************************************/

#if !defined(WITH_NOHTTP) || !defined(WITH_LEANER)
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_decode_val(char *buf, size_t len, const char *val)
{
  if (*val != '=')
  {
    *buf = '\0';
    return val;
  }
  return soap_decode(buf, len, val + 1, ",;");
}
#endif

/******************************************************************************/

#if !defined(WITH_NOHTTP) || !defined(WITH_LEANER)
static const char *
soap_decode(char *buf, size_t len, const char *val, const char *sep)
{
  const char *s;
  char *t = buf;
  size_t i = len;
  if (!buf || !val || !sep || len == 0)
    return val;
  for (s = val; *s; s++)
    if (*s != ' ' && *s != '\t' && !strchr(sep, *s))
      break;
  if (*s == '"')
  {
    s++;
    while (*s && *s != '"' && i-- > 1)
      *t++ = *s++;
  }
  else
  {
    while (*s && !strchr(sep, *s) && i-- > 1)
    {
      if (*s == '%' && s[1] && s[2])
      {
        *t++ = ((s[1] >= 'A' ? (s[1] & 0x7) + 9 : s[1] - '0') << 4)
          + (s[2] >= 'A' ? (s[2] & 0x7) + 9 : s[2] - '0');
        s += 3;
      }
      else
        *t++ = *s++;
    }
  }
  buf[len - 1] = '\0'; /* appease static checkers that get confused */
  *t = '\0';
  while (*s && !strchr(sep, *s))
    s++;
  return s;
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static const char*
http_error(struct soap *soap, int status)
{
  const char *msg = SOAP_STR_EOS;
  (void)soap;
  (void)status;
#ifndef WITH_LEAN
  msg = soap_code_str(h_http_error_codes, status);
  if (!msg)
    msg = SOAP_STR_EOS;
#endif
  return msg;
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_get(struct soap *soap)
{
  (void)soap;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "HTTP GET request\n"));
  return SOAP_GET_METHOD;
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_put(struct soap *soap)
{
  (void)soap;
  return SOAP_PUT_METHOD;
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_patch(struct soap *soap)
{
  (void)soap;
  return SOAP_PATCH_METHOD;
}
#endif
/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_del(struct soap *soap)
{
  (void)soap;
  return SOAP_DEL_METHOD;
}
#endif
/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_200(struct soap *soap)
{
  if (soap->origin && soap->cors_method) /* CORS Origin and Access-Control-Request-Method headers */
  {
    soap->cors_origin = soap->cors_allow; /* modify this code or hook your own soap->fopt() callback with logic */
    soap->cors_methods = "GET, PUT, PATCH, POST, HEAD, OPTIONS";
    soap->cors_headers = soap->cors_header;
  }
  return soap_send_empty_response(soap, 200);
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_post(struct soap *soap, const char *endpoint, const char *host, int port, const char *path, const char *action, ULONG64 count)
{
  const char *s;
  int err;
  size_t l;
  switch (soap->status)
  {
    case SOAP_GET: 
      s = "GET";
      break;
    case SOAP_PUT: 
      s = "PUT";
      break;
    case SOAP_PATCH: 
      s = "PATCH";
      break;
    case SOAP_DEL: 
      s = "DELETE";
      break;
    case SOAP_CONNECT:
      s = "CONNECT";
      break;
    case SOAP_HEAD: 
      s = "HEAD";
      break;
    case SOAP_OPTIONS: 
      s = "OPTIONS";
      break;
    default:
      s = "POST";
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "HTTP %s to %s\n", s, endpoint ? endpoint : "(null)"));
  if (!endpoint || (soap_tag_cmp(endpoint, "http:*") && soap_tag_cmp(endpoint, "https:*") && soap_tag_cmp(endpoint, "httpg:*")))
    return SOAP_OK;
  /* set l to prevent overruns ('host' and 'soap->host' are substrings of 'endpoint') */
  l = strlen(endpoint) + strlen(soap->http_version) + 80;
  if (l > sizeof(soap->tmpbuf))
    return soap->error = SOAP_EOM;
  if (soap->status == SOAP_CONNECT)
    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), l), "%s %s:%d HTTP/%s", s, soap->host, soap->port, soap->http_version);
  else if (soap->proxy_host && endpoint)
    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), l), "%s %s HTTP/%s", s, endpoint, soap->http_version);
  else
    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), l), "%s /%s HTTP/%s", s, (*path == '/' ? path + 1 : path), soap->http_version);
  err = soap->fposthdr(soap, soap->tmpbuf, NULL);
  if (err)
    return err;
#ifdef WITH_OPENSSL
  if ((soap->ssl && port != 443) || (!soap->ssl && port != 80))
#else
  if (port != 80)
#endif
  {
#ifdef WITH_IPV6
    if (*host != '[' && strchr(host, ':'))
      (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), l), "[%s]:%d", host, port); /* RFC 2732 */
    else
#endif
      (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), l), "%s:%d", host, port);
  }
  else
  {
#ifdef WITH_IPV6
    if (*host != '[' && strchr(host, ':'))
      (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), l), "[%s]", host); /* RFC 2732 */
    else
#endif
      soap_strcpy(soap->tmpbuf, sizeof(soap->tmpbuf), host);
  }
  err = soap->fposthdr(soap, "Host", soap->tmpbuf);
  if (err)
    return err;
  err = soap->fposthdr(soap, "User-Agent", "gSOAP/2.8");
  if (err)
    return err;
  if (soap->origin)
  {
    err = soap->fposthdr(soap, "Origin", soap->origin);
    if (err)
      return err;
    if (soap->status == SOAP_OPTIONS)
    {
      err = soap->fposthdr(soap, "Access-Control-Request-Method", soap->cors_method ? soap->cors_method : "POST");
      if (err)
        return err;
      if (soap->cors_header)
      {
        err = soap->fposthdr(soap, "Access-Control-Request-Headers", soap->cors_header);
        if (err)
          return err;
      }
    }
  }
  err = soap_puthttphdr(soap, SOAP_OK, count);
  if (err)
    return err;
#ifndef WITH_LEANER
  if ((soap->imode & SOAP_ENC_MTOM))
  {
    err = soap->fposthdr(soap, "Accept", "multipart/related,application/xop+xml,*/*;q=0.8");
    if (err)
      return err;
  }
#endif
#ifdef WITH_ZLIB
#ifdef WITH_GZIP
  err = soap->fposthdr(soap, "Accept-Encoding", "gzip,deflate");
#else
  err = soap->fposthdr(soap, "Accept-Encoding", "deflate");
#endif
  if (err)
    return err;
#endif
#if !defined(WITH_LEAN) || defined(WITH_NTLM)
  if (soap->bearer)
  {
    l = strlen(soap->bearer);
    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), l + 1), "Bearer %s", soap->bearer);
    err = soap->fposthdr(soap, "Authorization", soap->tmpbuf);
    if (err)
      return err;
  }
#ifdef WITH_NTLM
  if (soap->ntlm_challenge)
  {
    l = strlen(soap->ntlm_challenge);
    if (l)
    {
      (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), l + 5), "NTLM %s", soap->ntlm_challenge);
      if (soap->proxy_host)
      {
        err = soap->fposthdr(soap, "Proxy-Authorization", soap->tmpbuf);
        if (err)
          return err;
      }
      else
      {
        err = soap->fposthdr(soap, "Authorization", soap->tmpbuf);
        if (err)
          return err;
      }
    }
  }
  else
  {
#endif
  if (soap->userid && soap->passwd)
  {
    soap_strcpy(soap->tmpbuf, sizeof(soap->tmpbuf), "Basic ");
    (SOAP_SNPRINTF(soap->tmpbuf + 262, sizeof(soap->tmpbuf) - 262, strlen(soap->userid) + strlen(soap->passwd) + 1), "%s:%s", soap->userid, soap->passwd);
    soap_s2base64(soap, (const unsigned char*)(soap->tmpbuf + 262), soap->tmpbuf + 6, (int)strlen(soap->tmpbuf + 262));
    err = soap->fposthdr(soap, "Authorization", soap->tmpbuf);
    if (err)
      return err;
  }
  if (soap->proxy_userid && soap->proxy_passwd)
  {
    soap_strcpy(soap->tmpbuf, sizeof(soap->tmpbuf), "Basic ");
    (SOAP_SNPRINTF(soap->tmpbuf + 262, sizeof(soap->tmpbuf) - 262, strlen(soap->proxy_userid) + strlen(soap->proxy_passwd) + 1), "%s:%s", soap->proxy_userid, soap->proxy_passwd);
    soap_s2base64(soap, (const unsigned char*)(soap->tmpbuf + 262), soap->tmpbuf + 6, (int)strlen(soap->tmpbuf + 262));
    err = soap->fposthdr(soap, "Proxy-Authorization", soap->tmpbuf);
    if (err)
      return err;
  }
#ifdef WITH_NTLM
  }
#endif
#endif
#ifdef WITH_COOKIES
#ifdef WITH_OPENSSL
  if (soap_putcookies(soap, host, path, soap->ssl != NULL))
    return soap->error;
#else
  if (soap_putcookies(soap, host, path, 0))
    return soap->error;
#endif
#endif
  if (action && soap->status != SOAP_GET && soap->status != SOAP_DEL)
  {
    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(action) + 2), "\"%s\"", action);
    err = soap->fposthdr(soap, "SOAPAction", soap->tmpbuf);
    if (err)
      return err;
  }
  return soap->fposthdr(soap, NULL, NULL);
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_send_header(struct soap *soap, const char *s)
{
  const char *t;
  do
  {
    t = strchr(s, '\n'); /* disallow \n in HTTP headers */
    if (!t)
      t = s + strlen(s);
    if (soap_send_raw(soap, s, t - s))
      return soap->error;
    s = t + 1;
  } while (*t);
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_post_header(struct soap *soap, const char *key, const char *val)
{
  if (key)
  {
    if (http_send_header(soap, key))
      return soap->error;
    if (val && (soap_send_raw(soap, ": ", 2) || http_send_header(soap, val)))
      return soap->error;
  }
  return soap_send_raw(soap, "\r\n", 2);
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
static int
http_response(struct soap *soap, int status, ULONG64 count)
{
  int err;
  char http[32];
  int code = status;
  const char *line;
#ifdef WMW_RPM_IO
  if (soap->rpmreqid)
    httpOutputEnable(soap->rpmreqid);
  if (soap->rpmreqid
   || soap_valid_socket(soap->master)
   || soap_valid_socket(soap->socket)
   || soap->recvfd != 0
   || soap->sendfd != 1
   || soap->os) /* RPM behaves as if standalone */
#else
  if (soap_valid_socket(soap->master)
   || soap_valid_socket(soap->socket)
#ifndef UNDER_CE
   || soap->recvfd != 0
   || soap->sendfd != 1
#else
   || soap->recvfd != stdin
   || soap->sendfd != stdout
#endif
   || soap->os) /* standalone server application (over sockets), not CGI (over stdin/out) */
#endif
    (SOAP_SNPRINTF(http, sizeof(http), strlen(soap->http_version) + 5), "HTTP/%s", soap->http_version);
  else
    soap_strcpy(http, sizeof(http), "Status:");
  if (status >= SOAP_FILE && status < SOAP_FILE + 600)
  {
    code = status - SOAP_FILE;
    if (code == 0)
      code = 200;
  }
  else if (!status || status == SOAP_HTML)
  {
    if (count || ((soap->omode & SOAP_IO) == SOAP_IO_CHUNK))
      code = 200;
    else
      code = 202;
  }
  else if (status < 200 || status >= 600)
  {
    const char *s = *soap_faultcode(soap);
    if (status >= SOAP_GET_METHOD && status <= SOAP_HTTP_METHOD)
      code = 405;
    else if (soap->version == 2 && (!s || !strcmp(s, "SOAP-ENV:Sender")))
      code = 400;
    else
      code = 500;
  }
  line = http_error(soap, code);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "HTTP Status = %d %s\n", code, line));
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), sizeof(http) + 22 + strlen(line)), "%s %d %s", http, code, line);
  err = soap->fposthdr(soap, soap->tmpbuf, NULL);
  if (err)
    return err;
#ifndef WITH_LEAN
  if (status == 401)
  {
    (SOAP_SNPRINTF_SAFE(soap->tmpbuf, sizeof(soap->tmpbuf)), "Basic realm=\"%s\"", (soap->authrealm && strlen(soap->authrealm) + 14 < sizeof(soap->tmpbuf)) ? soap->authrealm : "gSOAP Web Service");
    err = soap->fposthdr(soap, "WWW-Authenticate", soap->tmpbuf);
    if (err)
      return err;
  }
  else if ((status >= 301 && status <= 303) || status == 307)
  {
    err = soap->fposthdr(soap, "Location", soap->endpoint);
    if (err)
      return err;
  }
#endif
  err = soap->fposthdr(soap, "Server", "gSOAP/2.8");
  if (err)
    return err;
  if (soap->cors_origin)
  {
    err = soap->fposthdr(soap, "Access-Control-Allow-Origin", soap->cors_origin);
    if (err)
      return err;
    err = soap->fposthdr(soap, "Access-Control-Allow-Credentials", "true");
    if (err)
      return err;
    if (soap->cors_methods)
    {
      err = soap->fposthdr(soap, "Access-Control-Allow-Methods", soap->cors_methods);
      if (err)
        return err;
      if (soap->cors_headers)
      {
        err = soap->fposthdr(soap, "Access-Control-Allow-Headers", soap->cors_headers);
        if (err)
          return err;
      }
    }
  }
  if (soap->x_frame_options)
  {
    err = soap->fposthdr(soap, "X-Frame-Options", soap->x_frame_options);
    if (err)
      return err;
  }
  soap->cors_origin = NULL;
  soap->cors_methods = NULL;
  soap->cors_headers = NULL;
  err = soap_puthttphdr(soap, status, count);
  if (err)
    return err;
#ifdef WITH_COOKIES
  if (soap_putsetcookies(soap))
    return soap->error;
  soap_free_cookies(soap);
#endif
  return soap->fposthdr(soap, NULL, NULL);
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_response(struct soap *soap, int status)
{
  ULONG64 count;
  if (!(soap->omode & (SOAP_ENC_PLAIN | SOAP_IO_STORE /* this tests for chunking too */))
   && (status == SOAP_HTML || (status >= SOAP_FILE && status < SOAP_FILE + 600)))
    soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_STORE;
  soap->status = status;
  count = soap_count_attachments(soap);
  if (soap_init_send(soap))
    return soap->error;
#ifndef WITH_NOHTTP
  if ((soap->mode & SOAP_IO) != SOAP_IO_STORE && !(soap->mode & SOAP_ENC_PLAIN))
  {
    int k = soap->mode;
    soap->mode &= ~(SOAP_IO | SOAP_ENC_ZLIB);
    if ((k & SOAP_IO) != SOAP_IO_FLUSH)
      soap->mode |= SOAP_IO_BUFFER;
    soap->error = soap->fresponse(soap, status, count);
    if (soap->error)
      return soap->error;
#ifndef WITH_LEANER
    if ((k & SOAP_IO) == SOAP_IO_CHUNK)
    {
      if (soap_flush(soap))
        return soap->error;
    }
#endif
    soap->mode = k;
  }
#endif
#ifndef WITH_LEANER
  if (soap_begin_attachments(soap))
    return soap->error;
#endif
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_extend_url(struct soap *soap, const char *s, const char *t)
{
  if (s)
    soap_strcpy(soap->msgbuf, sizeof(soap->msgbuf), s);
  else
    *soap->msgbuf = '\0';
  if (t && (*t == '/' || *t == '?'))
  {
    char *r = strchr(soap->msgbuf, '?');
    if (r)
    {
      if (*t == '?')
      {
        soap_strcat(soap->msgbuf, sizeof(soap->msgbuf), "&");
        soap_strcat(soap->msgbuf, sizeof(soap->msgbuf), t + 1);
      }
      else /* *t == '/' */
      {
        size_t l = r - soap->msgbuf;
        *r = '\0';
        soap_strcat(soap->msgbuf, sizeof(soap->msgbuf), t);
        if (s)
          soap_strcat(soap->msgbuf, sizeof(soap->msgbuf), s + l);
      }
    }
    else
    {
      soap_strcat(soap->msgbuf, sizeof(soap->msgbuf), t);
    }
  }
  return soap->msgbuf;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_extend_url_query(struct soap *soap, const char *s, const char *t)
{
  (void)soap_extend_url(soap, s, t); /* fills and returns soap->msgbuf */
  if (strchr(soap->msgbuf, '?'))
    soap_strcat(soap->msgbuf, sizeof(soap->msgbuf), "&");
  else
    soap_strcat(soap->msgbuf, sizeof(soap->msgbuf), "?");
  return soap->msgbuf;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_url_query(struct soap *soap, const char *s, const char *t)
{
  size_t n = strlen(s);
  if (n)
  {
    char *r = soap->msgbuf;
    size_t k = n - (s[n-1] == '=');
    while ((r = strchr(r, '{')) != NULL)
      if (!strncmp(++r, s, k) && r[k] == '}')
        break;
    if (r)
    {
      size_t m = t ? strlen(t) : 0;
      (void)soap_memmove(r + m - 1, soap->msgbuf + sizeof(soap->msgbuf) - (r + n + 1), r + k + 1, strlen(r + k + 1) + 1);
      if (m)
        (void)soap_memmove(r - 1, soap->msgbuf + sizeof(soap->msgbuf) - (r - 1), t, m);
    }
    else
    {
      soap_strcat(soap->msgbuf, sizeof(soap->msgbuf), s);
      if (t)
      {
        int m = (int)strlen(soap->msgbuf); /* msgbuf length is max SOAP_TMPLEN or just 1024 bytes */
        (void)soap_encode_url(t, soap->msgbuf + m, (int)sizeof(soap->msgbuf) - m);
      }
      soap_strcat(soap->msgbuf, sizeof(soap->msgbuf), "&");
    }
  }
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_encode_url(const char *s, char *t, int len)
{
  int c;
  int n = len;
  if (s && n > 0)
  {
    while ((c = *s++) && n-- > 1)
    {
      if (c == '-'
       || c == '.'
       || (c >= '0' && c <= '9')
       || (c >= 'A' && c <= 'Z')
       || c == '_'
       || (c >= 'a' && c <= 'z')
       || c == '~')
      {
        *t++ = c;
      }
      else if (n > 2)
      {
        *t++ = '%';
        *t++ = (c >> 4) + (c > 159 ? '7' : '0');
        c &= 0xF;
        *t++ = c + (c > 9 ? '7' : '0');
        n -= 2;
      }
      else
      {
        break;
      }
    }
    *t = '\0';
  }
  return len - n;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_encode_url_string(struct soap *soap, const char *s)
{
  if (s)
  {
    int n = 3 * (int)strlen(s) + 1;
    char *t = (char*)soap_malloc(soap, n);
    if (t)
    {
      (void)soap_encode_url(s, t, n);
      return t;
    }
  }
  return SOAP_STR_EOS;
}

/******************************************************************************\
 *
 *      HTTP Cookies RFC 6265
 *
\******************************************************************************/

#ifdef WITH_COOKIES

SOAP_FMAC1
struct soap_cookie*
SOAP_FMAC2
soap_cookie(struct soap *soap, const char *name, const char *domain, const char *path)
{
  return soap_cookie_env(soap, name, domain, path, 0);
}

/******************************************************************************/

SOAP_FMAC1
struct soap_cookie*
SOAP_FMAC2
soap_cookie_env(struct soap *soap, const char *name, const char *domain, const char *path, short env)
{
  struct soap_cookie *p;
  if (!domain && !env)
    domain = soap->cookie_domain;
  if (!path)
    path = soap->cookie_path;
  if (!path)
    path = SOAP_STR_EOS;
  else if (*path == '/')
    path++;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Search cookie='%s' domain='%s' path='%s' env=%hd\n", name, domain ? domain : "(null)", path ? path : "(null)", env));
  for (p = soap->cookies; p; p = p->next)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Cookie in database: %s='%s' domain='%s' path='%s' env=%hd\n", p->name, p->value ? p->value : "(null)", p->domain ? p->domain : "(null)", p->path ? p->path : "(null)", p->env));
    if ((!env || p->env)
     && !strcmp(p->name, name)
     && (!domain || (domain && p->domain && !strcmp(p->domain, domain)))
     && (!path || (path && p->path && !strncmp(p->path, path, strlen(p->path)))))
      break;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
struct soap_cookie*
SOAP_FMAC2
soap_set_cookie(struct soap *soap, const char *name, const char *value, const char *domain, const char *path)
{
  struct soap_cookie **p, *q;
  int n;
  if (!domain)
    domain = soap->cookie_domain;
  if (!path)
    path = soap->cookie_path;
  if (!path)
    path = SOAP_STR_EOS;
  else if (*path == '/')
    path++;
  q = soap_cookie(soap, name, domain, path);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Set %scookie: %s='%s' domain='%s' path='%s'\n", q ? SOAP_STR_EOS : "new ", name, value ? value : "(null)", domain ? domain : "(null)", path ? path : "(null)"));
  if (!q)
  {
    q = (struct soap_cookie*)SOAP_MALLOC(soap, sizeof(struct soap_cookie));
    if (q)
    {
      size_t l = strlen(name) + 1;
      q->name = NULL;
      if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
        q->name = (char*)SOAP_MALLOC(soap, l);
      if (q->name)
        (void)soap_memcpy(q->name, l, name, l);
      q->value = NULL;
      q->domain = NULL;
      q->path = NULL;
      q->expire = 0;
      q->maxage = -1;
      q->version = 1;
      q->secure = 0;
      q->modified = 0;
      for (p = &soap->cookies, n = soap->cookie_max; *p && n; p = &(*p)->next, n--)
        if ((*p)->name && !strcmp((*p)->name, name) && (*p)->path && path && strcmp((*p)->path, path) < 0)
          break;
      if (n)
      {
        q->next = *p;
        *p = q;
      }
      else
      {
        if (q->name)
          SOAP_FREE(soap, q->name);
        SOAP_FREE(soap, q);
        q = NULL;
      }
    }
  }
  else
  {
    q->modified = 1;
  }
  if (q)
  {
    if (q->value)
    {
      if (!value || strcmp(value, q->value))
      {
        SOAP_FREE(soap, q->value);
        q->value = NULL;
      }
    }
    if (value && *value && !q->value)
    {
      size_t l = strlen(value) + 1;
      q->value = NULL;
      if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
        q->value = (char*)SOAP_MALLOC(soap, l);
      if (q->value)
        soap_strcpy(q->value, l, value);
    }
    if (q->domain)
    {
      if (!domain || strcmp(domain, q->domain))
      {
        SOAP_FREE(soap, q->domain);
        q->domain = NULL;
      }
    }
    if (domain && !q->domain)
    {
      size_t l = strlen(domain) + 1;
      q->domain = NULL;
      if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
        q->domain = (char*)SOAP_MALLOC(soap, l);
      if (q->domain)
        soap_strcpy(q->domain, l, domain);
    }
    if (q->path)
    {
      if (!path || strncmp(path, q->path, strlen(q->path)))
      {
        SOAP_FREE(soap, q->path);
        q->path = NULL;
      }
    }
    if (path && !q->path)
    {
      size_t l = strlen(path) + 1;
      q->path  = NULL;
      if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
        q->path = (char*)SOAP_MALLOC(soap, l);
      if (q->path)
        soap_strcpy(q->path, l, path);
    }
    q->session = 1;
    q->env = 0;
  }
  return q;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_clr_cookie(struct soap *soap, const char *name, const char *domain, const char *path)
{
  struct soap_cookie **p, *q;
  if (!domain)
    domain = soap->cookie_domain;
  if (!domain)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Error in clear cookie='%s': cookie domain not set\n", name ? name : "(null)"));
    return;
  }
  if (!path)
    path = soap->cookie_path;
  if (!path)
    path = SOAP_STR_EOS;
  else if (*path == '/')
    path++;
  for (p = &soap->cookies, q = *p; q; q = *p)
  {
    if (q->name && !strcmp(q->name, name) && (!q->domain || !strcmp(q->domain, domain)) && (!q->path || !strncmp(q->path, path, strlen(q->path))))
    {
      SOAP_FREE(soap, q->name);
      if (q->value)
        SOAP_FREE(soap, q->value);
      if (q->domain)
        SOAP_FREE(soap, q->domain);
      if (q->path)
        SOAP_FREE(soap, q->path);
      *p = q->next;
      SOAP_FREE(soap, q);
    }
    else
    {
      p = &q->next;
    }
  }
}

/******************************************************************************/

SOAP_FMAC1
const char *
SOAP_FMAC2
soap_cookie_value(struct soap *soap, const char *name, const char *domain, const char *path)
{
  struct soap_cookie *p;
  p = soap_cookie(soap, name, domain, path);
  if (p)
    return p->value;
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
const char *
SOAP_FMAC2
soap_env_cookie_value(struct soap *soap, const char *name, const char *domain, const char *path)
{
  struct soap_cookie *p;
  p = soap_cookie(soap, name, domain, path);
  if (p && p->env)
    return p->value;
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
time_t
SOAP_FMAC2
soap_cookie_expire(struct soap *soap, const char *name, const char *domain, const char *path)
{
  struct soap_cookie *p;
  p = soap_cookie(soap, name, domain, path);
  if (p)
    return (time_t)p->expire;
  return -1;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_set_cookie_expire(struct soap *soap, const char *name, long maxage, const char *domain, const char *path)
{
  struct soap_cookie *p;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Set cookie expiration max-age=%ld: cookie='%s' domain='%s' path='%s'\n", maxage, name, domain ? domain : "(null)", path ? path : "(null)"));
  p = soap_cookie(soap, name, domain, path);
  if (p)
  {
    p->maxage = maxage;
    p->modified = 1;
    return SOAP_OK;
  }
  return SOAP_ERR;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_set_cookie_secure(struct soap *soap, const char *name, const char *domain, const char *path)
{
  struct soap_cookie *p;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Set cookie secure: cookie='%s' domain='%s' path='%s'\n", name, domain ? domain : "(null)", path ? path : "(null)"));
  p = soap_cookie(soap, name, domain, path);
  if (p)
  {
    p->secure = 1;
    p->modified = 1;
    return SOAP_OK;
  }
  return SOAP_ERR;
}
/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_set_cookie_session(struct soap *soap, const char *name, const char *domain, const char *path)
{
  struct soap_cookie *p;
  p = soap_cookie(soap, name, domain, path);
  if (p)
  {
    p->session = 1;
    p->modified = 1;
    return SOAP_OK;
  }
  return SOAP_ERR;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_clr_cookie_session(struct soap *soap, const char *name, const char *domain, const char *path)
{
  struct soap_cookie *p;
  p = soap_cookie(soap, name, domain, path);
  if (p)
  {
    p->session = 0;
    p->modified = 1;
    return SOAP_OK;
  }
  return SOAP_ERR;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_putsetcookies(struct soap *soap)
{
  char tmp[4096];
  struct soap_cookie *p;
  for (p = soap->cookies; p; p = p->next)
  {
    if ((p->modified
#ifdef WITH_OPENSSL
     || (!p->env && !soap->ssl == !p->secure)
#endif
     ) && p->name && p->value && *p->name && *p->value)
    {
      char *s = tmp;
      const char *t;
      s += soap_encode_url(p->name, s, 3967);
      *s++ = '=';
      s += soap_encode_url(p->value, s, 3968 - (int)(s-tmp));
      t = p->domain ? p->domain : soap->cookie_domain;
      if (t && (int)strlen(t) < 3968 - (int)(s-tmp))
      {
        soap_strcpy(s, 4096 - (s-tmp), ";Domain=");
        s += 8;
        soap_strcpy(s, 4096 - (s-tmp), t);
        s += strlen(s);
      }
      t = p->path ? p->path : soap->cookie_path;
      if (t && (int)strlen(t) < 3976 - (int)(s-tmp))
      {
        soap_strcpy(s, 4096 - (s-tmp), ";Path=/");
        s += 7;
        if (*t == '/')
          t++;
        if (strchr(t, '%')) /* already URL encoded? */
        {
          soap_strcpy(s, 4096 - (s-tmp), t);
          s += strlen(s);
        }
        else
        {
          s += soap_encode_url(t, s, 4096 - (int)(s-tmp));
        }
      }
      if (p->version > 0 && s-tmp < 3983)
      {
        (SOAP_SNPRINTF(s, 4096 - (s-tmp), 29), ";Version=%u", p->version);
        s += strlen(s);
      }
      if (p->maxage >= 0 && s-tmp < 4012)
      {
        (SOAP_SNPRINTF(s, 4096 - (s-tmp), 29), ";Max-Age=%ld", p->maxage);
        s += strlen(s);
      }
#if !defined(WITH_LEAN)
#if defined(HAVE_GMTIME_R) || defined(HAVE_GMTIME)
      if (p->maxage >= 0 && s-tmp < 4041)
      {
        time_t n = time(NULL) + p->maxage;
        struct tm T, *pT = &T;
        size_t l = 0;
        /* format is Wed, 09 Jun 2021 10:18:14 GMT */
#if defined(HAVE_GMTIME_R)
        if (gmtime_r(&n, pT) != SOAP_FUNC_R_ERR)
          l = strftime(s, 4096 - (s-tmp), ";Expires=%a, %d %b %Y %H:%M:%S GMT", pT);
#else
        pT = gmtime(&n);
        if (pT)
          l = strftime(s, 4096 - (s-tmp), ";Expires=%a, %d %b %Y %H:%M:%S GMT", pT);
#endif
        s += l;
      }
#endif
#endif
      if (s-tmp < 4079
       && (p->secure
#ifdef WITH_OPENSSL
       || soap->ssl
#endif
         ))
      {
        soap_strcpy(s, 4096 - (s-tmp), ";Secure");
        s += strlen(s);
      }
      if (s-tmp < 4086)
        soap_strcpy(s, 4096 - (s-tmp), ";HttpOnly");
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Set-Cookie: %s\n", tmp));
      soap->error = soap->fposthdr(soap, "Set-Cookie", tmp);
      if (soap->error)
        return soap->error;
    }
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_putcookies(struct soap *soap, const char *domain, const char *path, int secure)
{
  struct soap_cookie **p, *q;
  char tmp[4096];
  unsigned int version = 0;
  time_t now = time(NULL);
  char *s = tmp;
  if (!domain || !path)
    return SOAP_OK;
  p = &soap->cookies;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Sending cookies for domain='%s' path='%s'\n", domain, path));
  if (*path == '/')
    path++;
  while ((q = *p))
  {
    if (q->expire && now >= (time_t)q->expire)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Cookie %s expired\n", q->name));
      SOAP_FREE(soap, q->name);
      if (q->value)
        SOAP_FREE(soap, q->value);
      if (q->domain)
        SOAP_FREE(soap, q->domain);
      if (q->path)
        SOAP_FREE(soap, q->path);
      *p = q->next;
      SOAP_FREE(soap, q);
    }
    else
    {
      int flag;
      char *t = q->domain;
      size_t n = 0;
      if (!t)
      {
        flag = 1;
      }
      else
      {
        const char *r = strchr(t, ':');
        if (r)
          n = r - t;
        else
          n = strlen(t);
        flag = !strncmp(t, domain, n);
      }
      /* domain-level cookies, cannot compile when WITH_NOIO set */
#ifndef WITH_NOIO
      if (!flag)
      {
        struct hostent hostent;
        if (!tcp_gethostbyname(soap, (char*)domain, &hostent, NULL))
        {
          const char *r = hostent.h_name;
          if (*t == '.')
          {
            size_t k = strlen(hostent.h_name);
            if (k >= n)
              r = hostent.h_name + k - n;
          }
          flag = !strncmp(t, r, n);
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Domain cookie %s host %s (match=%d)\n", t, r, flag));
        }
      }
#endif
      if (flag
          && (!q->path || !strncmp(q->path, path, strlen(q->path)))
#ifndef WITH_INSECURE_COOKIES
          && (!q->secure || secure)
#endif
         )
      {
        size_t n = 12;
        if (q->name)
          n += 3*strlen(q->name);
        if (q->value && *q->value)
          n += 3*strlen(q->value) + 1;
        if (q->path && *q->path)
          n += strlen(q->path) + 9;
        if (q->domain)
          n += strlen(q->domain) + 11;
        if (s + n >= tmp + sizeof(tmp))
        {
          if (s == tmp)
            return SOAP_OK; /* header too big, cannot split */
          /* split up HTTP header */
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Cookie: %s\n", tmp));
          soap->error = soap->fposthdr(soap, "Cookie", tmp);
          if (soap->error)
            return soap->error;
          s = tmp;
        }
        else if (s != tmp)
        {
          *s++ = ';';
        }
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Sending cookie %s=%s path=\"/%s\" domain=\"%s\"\n", q->name ? q->name : "(null)", q->value ? q->value : "(null)", q->path ? q->path : "(null)", q->domain ? q->domain : "(null)"));
        if (q->version != version && (s-tmp) + (size_t)36 < sizeof(tmp))
        {
          (SOAP_SNPRINTF_SAFE(s, sizeof(tmp) - (s-tmp)), "$Version=%u;", q->version);
          version = q->version;
          s += strlen(s);
        }
        if (q->name && (s-tmp) + strlen(q->name) + (size_t)15 < sizeof(tmp))
        {
          s += soap_encode_url(q->name, s, (int)(tmp+sizeof(tmp)-s)-15);
        }
        if (q->value && *q->value && (s-tmp) + strlen(q->value) + (size_t)16 < sizeof(tmp))
        {
          *s++ = '=';
          s += soap_encode_url(q->value, s, (int)(tmp+sizeof(tmp)-s)-16);
        }
        if (q->path && (s-tmp) + strlen(q->path) + (size_t)36 < sizeof(tmp))
        {
          (SOAP_SNPRINTF_SAFE(s, sizeof(tmp) - (s-tmp)), ";$Path=\"/%s\"", (*q->path == '/' ? q->path + 1 : q->path));
          s += strlen(s);
        }
        if (q->domain && (s-tmp) + strlen(q->domain) + (size_t)36 < sizeof(tmp))
        {
          (SOAP_SNPRINTF_SAFE(s, sizeof(tmp) - (s-tmp)), ";$Domain=\"%s\"", q->domain);
          s += strlen(s);
        }
      }
      p = &q->next;
    }
  }
  if (s != tmp)
  {
    soap->error = soap->fposthdr(soap, "Cookie", tmp);
    if (soap->error)
      return soap->error;
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_getcookies(struct soap *soap, const char *val)
{
  struct soap_cookie *p = NULL, *q = NULL;
  const char *s = val;
  char tmp[4096]; /* cookie size is up to 4096 bytes [RFC2109] */
  char *domain = NULL;
  char *path = NULL;
  unsigned int version = 0;
  time_t now = time(NULL);
  if (!s)
    return;
  while (*s)
  {
    s = soap_decode_key(tmp, sizeof(tmp), s);
    if (!soap_tag_cmp(tmp, "$Version"))
    {
      s = soap_decode_val(tmp, sizeof(tmp), s);
      if (s)
      {
        if (p)
          p->version = (int)soap_strtol(tmp, NULL, 10);
        else
          version = (int)soap_strtol(tmp, NULL, 10);
      }
    }
    else if (!soap_tag_cmp(tmp, "$Path"))
    {
      char *t = NULL;
      s = soap_decode_val(tmp, sizeof(tmp), s);
      if (*tmp)
      {
        size_t l = strlen(tmp) + 1;
        if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
          t = (char*)SOAP_MALLOC(soap, l);
        if (t)
          (void)soap_memcpy((void*)t, l, (const void*)tmp, l);
      }
      if (p)
      {
        if (p->path)
          SOAP_FREE(soap, p->path);
        p->path = t;
      }
      else
      {
        if (path)
          SOAP_FREE(soap, path);
        path = t;
      }
    }
    else if (!soap_tag_cmp(tmp, "$Domain"))
    {
      char *t = NULL;
      s = soap_decode_val(tmp, sizeof(tmp), s);
      if (*tmp)
      {
        size_t l = strlen(tmp) + 1;
        if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
          t = (char*)SOAP_MALLOC(soap, l);
        if (t)
          (void)soap_memcpy((void*)t, l, (const void*)tmp, l);
      }
      if (p)
      {
        if (p->domain)
          SOAP_FREE(soap, p->domain);
        p->domain = t;
      }
      else
      {
        if (domain)
          SOAP_FREE(soap, domain);
        domain = t;
      }
    }
    else if (p && !soap_tag_cmp(tmp, "Path"))
    {
      if (p->path)
        SOAP_FREE(soap, p->path);
      p->path = NULL;
      s = soap_decode_val(tmp, sizeof(tmp), s);
      if (*tmp)
      {
        size_t l = strlen(tmp) + 1;
        if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
          p->path = (char*)SOAP_MALLOC(soap, l);
        if (p->path)
          (void)soap_memcpy((void*)p->path, l, (const void*)tmp, l);
      }
    }
    else if (p && !soap_tag_cmp(tmp, "Domain"))
    {
      if (p->domain)
        SOAP_FREE(soap, p->domain);
      p->domain = NULL;
      s = soap_decode_val(tmp, sizeof(tmp), s);
      if (*tmp)
      {
        size_t l = strlen(tmp) + 1;
        if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
          p->domain = (char*)SOAP_MALLOC(soap, l);
        if (p->domain)
          (void)soap_memcpy((void*)p->domain, l, (const void*)tmp, l);
      }
    }
    else if (p && !soap_tag_cmp(tmp, "Version"))
    {
      s = soap_decode_val(tmp, sizeof(tmp), s);
      p->version = (unsigned int)soap_strtoul(tmp, NULL, 10);
    }
    else if (p && !soap_tag_cmp(tmp, "Max-Age"))
    {
      s = soap_decode_val(tmp, sizeof(tmp), s);
      p->expire = (ULONG64)(now + soap_strtol(tmp, NULL, 10));
    }
    else if (p && !soap_tag_cmp(tmp, "Expires"))
    {
      if (*s == '=')
      {
        s = soap_decode(tmp, sizeof(tmp), s + 1, ";");
        if (!p->expire && strlen(tmp) >= 23)
        {
          char a[3];
          struct tm T;
          static const char mns[] = "anebarprayunulugepctovec";
          const char *t = strchr(tmp, ' ');
          if (t)
          {
            a[2] = '\0';
            memset((void*)&T, 0, sizeof(T));
            if (t[1] >= 'A')
            {
              /* format is Sun Nov  6 08:49:37 94 */
              a[0] = t[2];
              a[1] = t[3];
              T.tm_mon = (int)(strstr(mns, a) - mns) / 2;
              a[0] = t[5];
              a[1] = t[6];
              T.tm_mday = (int)soap_strtol(a, NULL, 10);
              if (t[17] && t[18] && t[19] != ' ')
                t += 2; /* format is Sun Nov  6 08:49:37 2017 - ANSI-C */
              a[0] = t[17];
              a[1] = t[18];
              T.tm_year = 100 + (int)soap_strtol(a, NULL, 10);
              t += 6;
            }
            else
            {
              /* format is Sunday, 06-Nov-17 08:49:37 GMT - RFC 850 */
              a[0] = t[1];
              a[1] = t[2];
              T.tm_mday = (int)soap_strtol(a, NULL, 10);
              a[0] = t[5];
              a[1] = t[6];
              T.tm_mon = (int)(strstr(mns, a) - mns) / 2;
              if (t[10] != ' ')
                t += 2; /* format is Wed, 09 Jun 2021 10:18:14 GMT - RFC 822 */
              a[0] = t[8];
              a[1] = t[9];
              T.tm_year = 100 + (int)soap_strtol(a, NULL, 10);
              t += 11;
            }
            a[0] = t[0];
            a[1] = t[1];
            T.tm_hour = (int)soap_strtol(a, NULL, 10);
            a[0] = t[3];
            a[1] = t[4];
            T.tm_min = (int)soap_strtol(a, NULL, 10);
            a[0] = t[6];
            a[1] = t[7];
            T.tm_sec = (int)soap_strtol(a, NULL, 10);
            p->expire = (ULONG64)soap_timegm(&T);
          }
        }
      }
    }
    else if (p && !soap_tag_cmp(tmp, "Secure"))
    {
      p->secure = 1;
      s = soap_decode_val(tmp, sizeof(tmp), s);
    }
    else if (p && !soap_tag_cmp(tmp, "HttpOnly"))
    {
      s = soap_decode_val(tmp, sizeof(tmp), s);
    }
    else if (p && !soap_tag_cmp(tmp, "Comment"))
    {
      s = soap_decode_val(tmp, sizeof(tmp), s);
    }
    else if (*tmp)
    {
      if (p)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Got environment cookie='%s' value='%s' domain='%s' path='%s' expire=" SOAP_ULONG_FORMAT " secure=%d\n", p->name, p->value ? p->value : "(null)", p->domain ? p->domain : "(null)", p->path ? p->path : "(null)", p->expire, p->secure));
        q = soap_set_cookie(soap, p->name, p->value, p->domain, p->path);
        if (q)
        {
          q->version = p->version;
          q->expire = p->expire;
          q->secure = p->secure;
          q->env = 1;
        }
        if (p->name)
          SOAP_FREE(soap, p->name);
        if (p->value)
          SOAP_FREE(soap, p->value);
        if (p->domain)
          SOAP_FREE(soap, p->domain);
        if (p->path)
          SOAP_FREE(soap, p->path);
        SOAP_FREE(soap, p);
      }
      p = (struct soap_cookie*)SOAP_MALLOC(soap, sizeof(struct soap_cookie));
      if (p)
      {
        size_t l = strlen(tmp) + 1;
        p->name = NULL;
        if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
          p->name = (char*)SOAP_MALLOC(soap, l);
        if (p->name)
          (void)soap_memcpy(p->name, l, tmp, l);
        s = soap_decode_val(tmp, sizeof(tmp), s);
        p->value = NULL;
        if (*tmp)
        {
          l = strlen(tmp) + 1;
          if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
            p->value = (char*)SOAP_MALLOC(soap, l);
          if (p->value)
            (void)soap_memcpy((void*)p->value, l, (const void*)tmp, l);
        }
        if (domain)
        {
          p->domain = domain;
          domain = NULL;
        }
        else
        {
          p->domain = NULL;
        }
        if (path)
        {
          p->path = path;
          path = NULL;
        }
        else if (*soap->path)
        {
          l = strlen(soap->path) + 1;
          p->path = NULL;
          if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
            p->path = (char*)SOAP_MALLOC(soap, l);
          if (p->path)
            (void)soap_memcpy((void*)p->path, l, (const void*)soap->path, l);
        }
        else
        {
          p->path = (char*)SOAP_MALLOC(soap, 2);
          if (p->path)
            (void)soap_memcpy((void*)p->path, 2, (const void*)"/", 2);
        }
        p->expire = 0;
        p->secure = 0;
        p->version = version;
      }
    }
  }
  if (p)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Got environment cookie='%s' value='%s' domain='%s' path='%s' expire=" SOAP_ULONG_FORMAT " secure=%d\n", p->name, p->value ? p->value : "(null)", p->domain ? p->domain : "(null)", p->path ? p->path : "(null)", p->expire, p->secure));
    q = soap_set_cookie(soap, p->name, p->value, p->domain, p->path);
    if (q)
    {
      q->version = p->version;
      q->expire = p->expire;
      q->secure = p->secure;
      q->env = 1;
    }
    if (p->name)
      SOAP_FREE(soap, p->name);
    if (p->value)
      SOAP_FREE(soap, p->value);
    if (p->domain)
      SOAP_FREE(soap, p->domain);
    if (p->path)
      SOAP_FREE(soap, p->path);
    SOAP_FREE(soap, p);
  }
  if (domain)
    SOAP_FREE(soap, domain);
  if (path)
    SOAP_FREE(soap, path);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_getenv_cookies(struct soap *soap)
{
  struct soap_cookie *p;
  const char *s;
  char key[4096], val[4096]; /* cookie size is up to 4096 bytes [RFC2109] */
  s = getenv("HTTP_COOKIE");
  if (!s)
    return SOAP_ERR;
  do
  {
    s = soap_decode_key(key, sizeof(key), s);
    s = soap_decode_val(val, sizeof(val), s);
    p = soap_set_cookie(soap, key, val, NULL, NULL);
    if (p)
      p->env = 1;
  } while (*s);
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
struct soap_cookie*
SOAP_FMAC2
soap_copy_cookies(struct soap *copy, const struct soap *soap)
{
  struct soap_cookie *p, **q, *r;
  (void)copy;
  q = &r;
  for (p = soap->cookies; p; p = p->next)
  {
    *q = (struct soap_cookie*)SOAP_MALLOC(copy, sizeof(struct soap_cookie));
    if (!*q)
      return r;
    **q = *p;
    if (p->name)
    {
      size_t l = strlen(p->name) + 1;
      (*q)->name = NULL;
      if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
        (*q)->name = (char*)SOAP_MALLOC(copy, l);
      if ((*q)->name)
        (void)soap_memcpy((*q)->name, l, p->name, l);
    }
    if (p->value)
    {
      size_t l = strlen(p->value) + 1;
      (*q)->value = NULL;
      if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
        (*q)->value = (char*)SOAP_MALLOC(copy, l);
      if ((*q)->value)
        (void)soap_memcpy((*q)->value, l, p->value, l);
    }
    if (p->domain)
    {
      size_t l = strlen(p->domain) + 1;
      (*q)->domain = NULL;
      if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
        (*q)->domain = (char*)SOAP_MALLOC(copy, l);
      if ((*q)->domain)
        (void)soap_memcpy((*q)->domain, l, p->domain, l);
    }
    if (p->path)
    {
      size_t l = strlen(p->path) + 1;
      (*q)->path = NULL;
      if (SOAP_MAXALLOCSIZE <= 0 || l <= SOAP_MAXALLOCSIZE)
        (*q)->path = (char*)SOAP_MALLOC(copy, l);
      if ((*q)->path)
        (void)soap_memcpy((*q)->path, l, p->path, l);
    }
    q = &(*q)->next;
  }
  *q = NULL;
  return r;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_free_cookies(struct soap *soap)
{
  struct soap_cookie *p;
  for (p = soap->cookies; p; p = soap->cookies)
  {
    soap->cookies = p->next;
    SOAP_FREE(soap, p->name);
    if (p->value)
      SOAP_FREE(soap, p->value);
    if (p->domain)
      SOAP_FREE(soap, p->domain);
    if (p->path)
      SOAP_FREE(soap, p->path);
    SOAP_FREE(soap, p);
  }
}

/******************************************************************************/

#endif /* WITH_COOKIES */

/******************************************************************************/

SOAP_FMAC1
size_t
SOAP_FMAC2
soap_hash(const char *s)
{
  size_t h = 0;
  while (*s)
    h = *s++ + (h << 6) + (h << 16) - h; /* Red Dragon book h = 65599*h + c */
  return h % SOAP_IDHASH;
}

/******************************************************************************/

static void
soap_init_pht(struct soap *soap)
{
  int i;
  soap->pblk = NULL;
  soap->pidx = 0;
  for (i = 0; i < (int)SOAP_PTRHASH; i++)
    soap->pht[i] = NULL;
}

/******************************************************************************/

SOAP_FMAC1
struct soap*
SOAP_FMAC2
soap_versioning(soap_new)(soap_mode imode, soap_mode omode)
{
  struct soap *soap;
#ifdef __cplusplus
  soap = SOAP_NEW_UNMANAGED(struct soap);
#else
  soap = (struct soap*)SOAP_MALLOC_UNMANAGED(sizeof(struct soap));
#endif
  if (soap)
    soap_versioning(soap_init)(soap, imode, omode);
  return soap;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_free(struct soap *soap)
{
  soap_done(soap);
#ifdef __cplusplus
  SOAP_DELETE_UNMANAGED(soap);
#else
  SOAP_FREE_UNMANAGED(soap);
#endif
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_del(struct soap *soap)
{
  free(soap);
}

/******************************************************************************/

static void
soap_free_pht(struct soap *soap)
{
  struct soap_pblk *pb, *next;
  int i;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Free pointer hashtable\n"));
  for (pb = soap->pblk; pb; pb = next)
  {
    next = pb->next;
    SOAP_FREE(soap, pb);
  }
  soap->pblk = NULL;
  soap->pidx = 0;
  for (i = 0; i < (int)SOAP_PTRHASH; i++)
    soap->pht[i] = NULL;
}

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_embed(struct soap *soap, const void *p, const void *a, int n, int t)
{
  int id;
  struct soap_plist *pp;
  if (soap->version == 2)
    soap->encoding = 1;
  if (!p || (!soap->encodingStyle && !(soap->mode & SOAP_XML_GRAPH)) || (soap->mode & SOAP_XML_TREE))
    return 0;
  if (a)
    id = soap_array_pointer_lookup(soap, p, a, n, t, &pp);
  else
    id = soap_pointer_lookup(soap, p, t, &pp);
  if (id)
  {
    if (soap_is_embedded(soap, pp) || soap_is_single(soap, pp))
      return 0;
    soap_set_embedded(soap, pp);
  }
  return id;
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_pointer_lookup(struct soap *soap, const void *p, int type, struct soap_plist **ppp)
{
  struct soap_plist *pp;
  *ppp = NULL;
  if (p)
  {
    for (pp = soap->pht[soap_hash_ptr(p)]; pp; pp = pp->next)
    {
      if (pp->ptr == p && pp->type == type)
      {
        *ppp = pp;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Lookup location=%p type=%d id=%d\n", p, type, pp->id));
        return pp->id;
      }
    }
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Lookup location=%p type=%d: not found\n", p, type));
  return 0;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_pointer_enter(struct soap *soap, const void *p, const void *a, int n, int type, struct soap_plist **ppp)
{
  size_t h;
  struct soap_plist *pp;
  (void)n;
  if (!soap->pblk || soap->pidx >= SOAP_PTRBLK)
  {
    struct soap_pblk *pb = (struct soap_pblk*)SOAP_MALLOC(soap, sizeof(struct soap_pblk));
    if (!pb)
    {
      soap->error = SOAP_EOM;
      return 0;
    }
    pb->next = soap->pblk;
    soap->pblk = pb;
    soap->pidx = 0;
  }
  *ppp = pp = &soap->pblk->plist[soap->pidx++];
  if (a)
    h = soap_hash_ptr(a);
  else
    h = soap_hash_ptr(p);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Pointer enter location=%p array=%p size=%lu type=%d id=%d\n", p, a, (unsigned long)n, type, soap->idnum+1));
  pp->next = soap->pht[h];
  pp->type = type;
  pp->mark1 = 0;
  pp->mark2 = 0;
  pp->ptr = p;
  pp->dup = NULL;
  pp->array = a;
  pp->size = n;
  soap->pht[h] = pp;
  pp->id = ++soap->idnum;
  return pp->id;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_array_pointer_lookup(struct soap *soap, const void *p, const void *a, int n, int type, struct soap_plist **ppp)
{
  struct soap_plist *pp;
  *ppp = NULL;
  if (!p || !a)
    return 0;
  for (pp = soap->pht[soap_hash_ptr(a)]; pp; pp = pp->next)
  {
    if (pp->type == type && pp->array == a && pp->size == n)
    {
      *ppp = pp;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Array lookup location=%p type=%d id=%d\n", a, type, pp->id));
      return pp->id;
    }
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Array lookup location=%p type=%d: not found\n", a, type));
  return 0;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_begin_count(struct soap *soap)
{
  soap_free_ns(soap);
  soap->error = SOAP_OK;
#ifndef WITH_LEANER
  if ((soap->mode & SOAP_ENC_DIME) || (soap->omode & SOAP_ENC_DIME))
  {
    soap->mode = soap->omode | SOAP_IO_LENGTH | SOAP_ENC_DIME;
  }
  else
#endif
  {
    soap->mode = soap->omode;
    if ((soap->mode & SOAP_IO_UDP))
    {
      soap->mode &= SOAP_IO;
      soap->mode |= SOAP_IO_BUFFER | SOAP_ENC_PLAIN;
    }
    if ((soap->mode & SOAP_IO) == SOAP_IO_STORE
     || (((soap->mode & SOAP_IO) == SOAP_IO_CHUNK || (soap->mode & SOAP_ENC_PLAIN))
#ifndef WITH_LEANER
      && !soap->fpreparesend
#endif
      ))
      soap->mode &= ~SOAP_IO_LENGTH;
    else
      soap->mode |= SOAP_IO_LENGTH;
  }
#ifdef WITH_ZLIB
  if ((soap->mode & SOAP_ENC_ZLIB) && (soap->mode & SOAP_IO) == SOAP_IO_FLUSH)
  {
    if (!(soap->mode & SOAP_ENC_DIME))
      soap->mode &= ~SOAP_IO_LENGTH;
    if ((soap->mode & SOAP_ENC_PLAIN))
      soap->mode |= SOAP_IO_BUFFER;
    else
      soap->mode |= SOAP_IO_STORE;
  }
#endif
#ifndef WITH_LEANER
  if ((soap->mode & SOAP_ENC_MTOM) && (soap->mode & SOAP_ENC_DIME))
    soap->mode |= SOAP_ENC_MIME;
  else if (!(soap->mode & SOAP_ENC_MIME))
    soap->mode &= ~SOAP_ENC_MTOM;
  if ((soap->mode & SOAP_ENC_MIME))
    soap_select_mime_boundary(soap);
  soap->dime.list = soap->dime.last;    /* keep track of last DIME attachment */
#endif
  soap->count = 0;
  soap->ns = 0;
  soap->null = 0;
  soap->position = 0;
  soap->mustUnderstand = 0;
  soap->encoding = 0;
  soap->part = SOAP_BEGIN_SEND;
  soap->event = 0;
  soap->evlev = 0;
  soap->idnum = 0;
  soap->body = 1;
  soap->level = 0;
  soap_clr_attr(soap);
  soap_set_local_namespaces(soap);
#ifndef WITH_LEANER
  soap->dime.size = 0; /* accumulate total size of attachments */
  if (soap->fprepareinitsend && (soap->mode & SOAP_IO) != SOAP_IO_STORE && (soap->error = soap->fprepareinitsend(soap)) != SOAP_OK)
    return soap->error;
#endif
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Begin count phase (socket=%d mode=0x%x count=" SOAP_ULONG_FORMAT ")\n", (int)soap->socket, (unsigned int)soap->mode, soap->count));
#ifndef WITH_LEANER
  if ((soap->mode & SOAP_IO_LENGTH))
    return soap_begin_attachments(soap);
#endif
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_end_count(struct soap *soap)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "End of count phase\n"));
#ifndef WITH_LEANER
  if ((soap->mode & SOAP_IO_LENGTH))
  {
    if (soap_end_attachments(soap))
      return soap->error;
    if (soap->fpreparefinalsend && (soap->error = soap->fpreparefinalsend(soap)) != SOAP_OK)
      return soap->error;
  }
#else
  (void)soap;
#endif
  return SOAP_OK;
}

/******************************************************************************/

static int
soap_init_send(struct soap *soap)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Initializing for output to socket=%d/fd=%d\n", (int)soap->socket, soap->sendfd));
  *soap->tag = '\0';
  soap_free_ns(soap);
  soap->error = SOAP_OK;
  soap->mode = soap->omode | (soap->mode & (SOAP_IO_LENGTH | SOAP_ENC_DIME));
#ifndef WITH_LEAN
  if ((soap->mode & SOAP_IO_UDP))
  {
    soap->mode &= ~SOAP_IO;
    soap->mode |= SOAP_IO_BUFFER | SOAP_ENC_PLAIN;
    if ((soap->mode & SOAP_IO_LENGTH) && soap->count > sizeof(soap->buf))
      return soap->error = SOAP_UDP_ERROR;
  }
#endif
#ifdef WITH_ZLIB
  if ((soap->mode & SOAP_ENC_ZLIB) && (soap->mode & SOAP_IO) == SOAP_IO_FLUSH)
  {
    if ((soap->mode & SOAP_ENC_PLAIN))
      soap->mode |= SOAP_IO_BUFFER;
    else
      soap->mode |= SOAP_IO_STORE;
  }
#endif
#if !defined(__cplusplus) || defined(WITH_COMPAT)
  if (soap->os)
  {
    *soap->os = NULL;
    soap->mode = (soap->mode & ~SOAP_IO) | SOAP_IO_STORE;
  }
  else
#endif
  if ((soap->mode & SOAP_IO) == SOAP_IO_FLUSH && soap_valid_socket(soap->socket))
  {
    if ((soap->mode & SOAP_IO_LENGTH) || (soap->mode & SOAP_ENC_PLAIN))
      soap->mode |= SOAP_IO_BUFFER;
    else
      soap->mode |= SOAP_IO_STORE;
  }
  soap->mode &= ~SOAP_IO_LENGTH;
  if ((soap->mode & SOAP_IO) == SOAP_IO_STORE && soap_alloc_block(soap) == NULL)
    return soap->error;
  if (!(soap->mode & SOAP_IO_KEEPALIVE))
    soap->keep_alive = 0;
#ifndef WITH_LEANER
  if ((soap->mode & SOAP_ENC_MTOM) && (soap->mode & SOAP_ENC_DIME))
  {
    soap->mode |= SOAP_ENC_MIME;
    soap->mode &= ~SOAP_ENC_DIME;
  }
  else if (!(soap->mode & SOAP_ENC_MIME))
  {
    soap->mode &= ~SOAP_ENC_MTOM;
  }
  if ((soap->mode & SOAP_ENC_MIME))
    soap_select_mime_boundary(soap);
#ifdef WIN32
#ifndef UNDER_CE
#ifndef WITH_FASTCGI
  if (!soap_valid_socket(soap->socket) && !soap->os && soap->sendfd >= 0) /* Set win32 stdout or soap->sendfd to BINARY, e.g. to support DIME */
#ifdef __BORLANDC__
    setmode(soap->sendfd, _O_BINARY);
#else
    _setmode(soap->sendfd, _O_BINARY);
#endif
#endif
#endif
#endif
#endif
  if ((soap->mode & SOAP_IO))
    soap->buflen = soap->bufidx = 0;
  soap->chunksize = 0;
  soap->ns = 0;
  soap->null = 0;
  soap->position = 0;
  soap->mustUnderstand = 0;
  soap->encoding = 0;
  soap->event = 0;
  soap->evlev = 0;
  soap->idnum = 0;
  soap->body = 1;
  soap->level = 0;
  soap_clr_attr(soap);
  soap_set_local_namespaces(soap);
#ifdef WITH_ZLIB
  soap->z_ratio_out = 1.0;
  if ((soap->mode & SOAP_ENC_ZLIB) && soap->zlib_state != SOAP_ZLIB_DEFLATE)
  {
    if (!soap->d_stream)
    {
      soap->d_stream = (z_stream*)SOAP_MALLOC(soap, sizeof(z_stream));
      if (!soap->d_stream)
        return soap->error = SOAP_EOM;
      soap->d_stream->zalloc = Z_NULL;
      soap->d_stream->zfree = Z_NULL;
      soap->d_stream->opaque = Z_NULL;
      soap->d_stream->next_in = Z_NULL;
    }
    if (!soap->z_buf)
      soap->z_buf = (char*)SOAP_MALLOC(soap, sizeof(soap->buf));
    if (!soap->z_buf)
      return soap->error = SOAP_EOM;
    soap->d_stream->next_out = (Byte*)soap->z_buf;
    soap->d_stream->avail_out = sizeof(soap->buf);
#ifdef WITH_GZIP
    if (soap->zlib_out != SOAP_ZLIB_DEFLATE)
    {
      (void)soap_memcpy((void*)soap->z_buf, sizeof(soap->buf), (const void*)"\37\213\10\0\0\0\0\0\0\377", 10);
      soap->d_stream->next_out = (Byte*)soap->z_buf + 10;
      soap->d_stream->avail_out = sizeof(soap->buf) - 10;
      soap->z_crc = crc32(0L, NULL, 0);
      soap->zlib_out = SOAP_ZLIB_GZIP;
      if (soap->z_dict)
        *((Byte*)soap->z_buf + 2) = 0xff;
      if (deflateInit2(soap->d_stream, soap->z_level, Z_DEFLATED, -MAX_WBITS, 8, Z_DEFAULT_STRATEGY) != Z_OK)
        return soap->error = SOAP_ZLIB_ERROR;
    }
    else
#endif
    if (deflateInit(soap->d_stream, soap->z_level) != Z_OK)
      return soap->error = SOAP_ZLIB_ERROR;
    if (soap->z_dict)
    {
      if (deflateSetDictionary(soap->d_stream, (const Bytef*)soap->z_dict, soap->z_dict_len) != Z_OK)
        return soap->error = SOAP_ZLIB_ERROR;
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Deflate initialized\n"));
    soap->zlib_state = SOAP_ZLIB_DEFLATE;
  }
#endif
#ifdef WITH_OPENSSL
  if (soap->ssl)
    ERR_clear_error();
#endif
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Begin send phase (socket=%d mode=0x%x count=" SOAP_ULONG_FORMAT ")\n", (int)soap->socket, soap->mode, soap->count));
  soap->part = SOAP_BEGIN_SEND;
#ifndef WITH_LEANER
  if (soap->fprepareinitsend && (soap->mode & SOAP_IO) == SOAP_IO_STORE && (soap->error = soap->fprepareinitsend(soap)) != SOAP_OK)
    return soap->error;
#endif
#ifndef WITH_LEAN
  soap->start = (ULONG64)time(NULL);
#endif
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_begin_send(struct soap *soap)
{
#ifndef WITH_LEANER
  if (soap_init_send(soap))
    return soap->error;
  return soap_begin_attachments(soap);
#else
  return soap_init_send(soap);
#endif
}

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
void
SOAP_FMAC2
soap_embedded(struct soap *soap, const void *p, int t)
{
  struct soap_plist *pp;
  if (soap_pointer_lookup(soap, p, t, &pp))
  {
    pp->mark1 = 1;
    pp->mark2 = 1;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Embedded %p type=%d mark set to 1\n", p, t));
  }
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_reference(struct soap *soap, const void *p, int t)
{
  struct soap_plist *pp;
  if (!p || (!soap->encodingStyle && !(soap->omode & (SOAP_ENC_DIME | SOAP_ENC_MIME | SOAP_ENC_MTOM | SOAP_XML_GRAPH))) || (soap->omode & SOAP_XML_TREE))
    return 1;
  if (soap_pointer_lookup(soap, p, t, &pp))
  {
    if (pp->mark1 == 0)
    {
      pp->mark1 = 2;
      pp->mark2 = 2;
    }
  }
  else if (!soap_pointer_enter(soap, p, NULL, 0, t, &pp))
  {
    return 1;
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Reference %p type=%d (%d %d)\n", p, t, (int)pp->mark1, (int)pp->mark2));
  return pp->mark1;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_array_reference(struct soap *soap, const void *p, const void *a, int n, int t)
{
  struct soap_plist *pp;
  if (!p || !a || (!soap->encodingStyle && !(soap->omode & SOAP_XML_GRAPH)) || (soap->omode & SOAP_XML_TREE))
    return 1;
  if (soap_array_pointer_lookup(soap, p, a, n, t, &pp))
  {
    if (pp->mark1 == 0)
    {
      pp->mark1 = 2;
      pp->mark2 = 2;
    }
  }
  else if (!soap_pointer_enter(soap, p, a, n, t, &pp))
  {
    return 1;
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Array reference %p ptr=%p n=%lu t=%d (%d %d)\n", p, a, (unsigned long)n, t, (int)pp->mark1, (int)pp->mark2));
  return pp->mark1;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_attachment_reference(struct soap *soap, const void *p, const void *a, int n, int t, const char *id, const char *type)
{
  struct soap_plist *pp;
  if (!p || !a || (!soap->encodingStyle && !(soap->omode & SOAP_XML_GRAPH) && !id && !type) || (soap->omode & SOAP_XML_TREE))
    return 1;
  if (soap_array_pointer_lookup(soap, p, a, n, t, &pp))
  {
    if (pp->mark1 == 0)
    {
      pp->mark1 = 2;
      pp->mark2 = 2;
    }
  }
  else if (!soap_pointer_enter(soap, p, a, n, t, &pp))
  {
    return 1;
  }
  if (id || type)
    soap->mode |= SOAP_ENC_DIME;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Attachment reference %p ptr=%p n=%lu t=%d (%d %d)\n", p, a, (unsigned long)n, t, (int)pp->mark1, (int)pp->mark2));
  return pp->mark1;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_embedded_id(struct soap *soap, int id, const void *p, int t)
{
  struct soap_plist *pp = NULL;
  if (id >= 0 || (!soap->encodingStyle && !(soap->omode & SOAP_XML_GRAPH)) || (soap->omode & SOAP_XML_TREE))
    return id;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Embedded_id %p type=%d id=%d\n", p, t, id));
  if (id < -1)
    return soap_embed(soap, p, NULL, 0, t);
  if (id < 0)
  {
    id = soap_pointer_lookup(soap, p, t, &pp);
    if (soap->version == 1 && soap->part != SOAP_IN_HEADER)
    {
      if (id)
      {
        if ((soap->mode & SOAP_IO_LENGTH))
          pp->mark1 = 2;
        else
          pp->mark2 = 2;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Embedded_id multiref id=%d %p type=%d = (%d %d)\n", id, p, t, (int)pp->mark1, (int)pp->mark2));
      }
      return -1;
    }
    else if (id)
    {
      if ((soap->mode & SOAP_IO_LENGTH))
        pp->mark1 = 1;
      else
        pp->mark2 = 1;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Embedded_id embedded ref id=%d %p type=%d = (%d %d)\n", id, p, t, (int)pp->mark1, (int)pp->mark2));
    }
  }
  return id;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_is_embedded(struct soap *soap, struct soap_plist *pp)
{
  if (!pp)
    return 0;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Is embedded? %d %d\n", (int)pp->mark1, (int)pp->mark2));
  if (soap->version == 1 && soap->encodingStyle && !(soap->omode & SOAP_XML_GRAPH) && soap->part != SOAP_IN_HEADER)
  {
    if ((soap->mode & SOAP_IO_LENGTH))
      return pp->mark1 != 0;
    return pp->mark2 != 0;
  }
  if ((soap->mode & SOAP_IO_LENGTH))
    return pp->mark1 == 1;
  return pp->mark2 == 1;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_is_single(struct soap *soap, struct soap_plist *pp)
{
  if (soap->part == SOAP_IN_HEADER)
    return 1;
  if (!pp)
    return 0;
  if ((soap->mode & SOAP_IO_LENGTH))
    return pp->mark1 == 0;
  return pp->mark2 == 0;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
void
SOAP_FMAC2
soap_set_embedded(struct soap *soap, struct soap_plist *pp)
{
  if (!pp)
    return;
  if ((soap->mode & SOAP_IO_LENGTH))
    pp->mark1 = 1;
  else
    pp->mark2 = 1;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_attachment(struct soap *soap, const char *tag, int id, const void *p, const void *a, int n, const char *aid, const char *atype, const char *aoptions, const char *type, int t)
{
  struct soap_plist *pp;
  int i;
  if (!p || !a || (!aid && !atype) || (!soap->encodingStyle && !(soap->omode & (SOAP_ENC_DIME | SOAP_ENC_MIME | SOAP_ENC_MTOM | SOAP_XML_GRAPH))) || (soap->omode & SOAP_XML_TREE))
    return soap_element_id(soap, tag, id, p, a, n, type, t, NULL);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Attachment tag='%s' id='%s' (%d) type='%s'\n", tag, aid ? aid : SOAP_STR_EOS, id, atype ? atype : SOAP_STR_EOS));
  i = soap_array_pointer_lookup(soap, p, a, n, t, &pp);
  if (!i)
  {
    i = soap_pointer_enter(soap, p, a, n, t, &pp);
    if (!i)
    {
      soap->error = SOAP_EOM;
      return -1;
    }
  }
  if (id <= 0)
    id = i;
  if (!aid)
  {
    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(soap->dime_id_format) + 20), soap->dime_id_format, id);
    aid = soap_strdup(soap, soap->tmpbuf);
    if (!aid)
      return -1;
  }
  /* Add MTOM xop:Include element when necessary */
  /* TODO: this code to be obsoleted with new import/xop.h conventions */
  if ((soap->omode & SOAP_ENC_MTOM) && strcmp(tag, "xop:Include"))
  {
    if (soap_element_begin_out(soap, tag, 0, type)
     || soap_element_href(soap, "xop:Include", 0, "xmlns:xop=\"http://www.w3.org/2004/08/xop/include\" href", aid)
     || soap_element_end_out(soap, tag))
      return soap->error;
  }
  else if (soap_element_href(soap, tag, 0, "href", aid))
  {
    return soap->error;
  }
  if ((soap->mode & SOAP_IO_LENGTH))
  {
    if (pp->mark1 != 3)
    {
      struct soap_multipart *content;
      if ((soap->omode & SOAP_ENC_MTOM))
        content = soap_alloc_multipart(soap, &soap->mime.first, &soap->mime.last, (const char*)a, n);
      else
        content = soap_alloc_multipart(soap, &soap->dime.first, &soap->dime.last, (const char*)a, n);
      if (!content)
      {
        soap->error = SOAP_EOM;
        return -1;
      }
      if (!strncmp(aid, "cid:", 4)) /* RFC 2111 */
      {
        if ((soap->omode & SOAP_ENC_MTOM))
        {
          size_t l = strlen(aid) - 1;
          char *s = (char*)soap_malloc(soap, l);
          if (s)
          {
            s[0] = '<';
            (void)soap_strncpy(s + 1, l - 1, aid + 4, l - 3);
            s[l - 2] = '>';
            s[l - 1] = '\0';
            content->id = s;
          }
        }
        else
        {
          content->id = aid + 4;
        }
      }
      else
      {
        content->id = aid;
      }
      content->type = atype;
      content->options = aoptions;
      content->encoding = SOAP_MIME_BINARY;
      pp->mark1 = 3;
    }
  }
  else
  {
    pp->mark2 = 3;
  }
  return -1;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
static void
soap_init_iht(struct soap *soap)
{
  int i;
  for (i = 0; i < SOAP_IDHASH; i++)
    soap->iht[i] = NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
static void
soap_free_iht(struct soap *soap)
{
  int i;
  struct soap_ilist *ip = NULL, *p = NULL;
  struct soap_flist *fp = NULL, *fq = NULL;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Free ID hashtable\n"));
  for (i = 0; i < SOAP_IDHASH; i++)
  {
    for (ip = soap->iht[i]; ip; ip = p)
    {
      for (fp = ip->flist; fp; fp = fq)
      {
        fq = fp->next;
        SOAP_FREE(soap, fp);
      }
      p = ip->next;
      SOAP_FREE(soap, ip);
    }
    soap->iht[i] = NULL;
  }
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
struct soap_ilist *
SOAP_FMAC2
soap_lookup(struct soap *soap, const char *id)
{
  struct soap_ilist *ip = NULL;
  for (ip = soap->iht[soap_hash(id)]; ip; ip = ip->next)
    if (!strcmp(ip->id, id))
      return ip;
  return NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
struct soap_ilist *
SOAP_FMAC2
soap_enter(struct soap *soap, const char *id, int t, size_t n)
{
  size_t h;
  struct soap_ilist *ip = NULL;
  size_t l = strlen(id);
  if (sizeof(struct soap_ilist) + l > l && (SOAP_MAXALLOCSIZE <= 0 || sizeof(struct soap_ilist) + l <= SOAP_MAXALLOCSIZE))
    ip = (struct soap_ilist*)SOAP_MALLOC(soap, sizeof(struct soap_ilist) + l);
  if (ip)
  {
    ip->type = t;
    ip->size = n;
    ip->ptr = NULL;
    ip->spine = NULL;
    ip->link = NULL;
    ip->copy = NULL;
    ip->flist = NULL;
    ip->smart = NULL;
    ip->shaky = 0;
    (void)soap_memcpy((char*)ip->id, l + 1, id, l + 1);
    h = soap_hash(id); /* h = (HASH(id) % SOAP_IDHASH) so soap->iht[h] is safe */
    ip->next = soap->iht[h];
    soap->iht[h] = ip;
  }
  return ip;
}
#endif

/******************************************************************************/

SOAP_FMAC1
void*
SOAP_FMAC2
soap_malloc(struct soap *soap, size_t n)
{
  char *p;
  size_t k = n;
  if (SOAP_MAXALLOCSIZE > 0 && n > SOAP_MAXALLOCSIZE)
  {
    soap->error = SOAP_EOM;
    return NULL;
  }
  if (!soap)
    return SOAP_MALLOC(soap, n);
  n += sizeof(short);
  n += (~n+1) & (sizeof(void*)-1); /* align at 4-, 8- or 16-byte boundary by rounding up */
  if (n + sizeof(void*) + sizeof(size_t) < k)
  {
    soap->error = SOAP_EOM;
    return NULL;
  }
  p = (char*)SOAP_MALLOC(soap, n + sizeof(void*) + sizeof(size_t));
  if (!p)
  {
    soap->error = SOAP_EOM;
    return NULL;
  }
  /* set a canary word to detect memory overruns and data corruption */
  *(unsigned short*)(p + n - sizeof(unsigned short)) = (unsigned short)SOAP_CANARY;
  /* keep chain of alloced cells for destruction */
  *(void**)(p + n) = soap->alist;
  *(size_t*)(p + n + sizeof(void*)) = n;
  soap->alist = p + n;
  return p;
}

/******************************************************************************/

#ifdef SOAP_MEM_DEBUG
static void
soap_init_mht(struct soap *soap)
{
  int i;
  for (i = 0; i < (int)SOAP_PTRHASH; i++)
    soap->mht[i] = NULL;
}
#endif

/******************************************************************************/

#ifdef SOAP_MEM_DEBUG
static void
soap_free_mht(struct soap *soap)
{
  int i;
  struct soap_mlist *mp, *mq;
  for (i = 0; i < (int)SOAP_PTRHASH; i++)
  {
    for (mp = soap->mht[i]; mp; mp = mq)
    {
      mq = mp->next;
      if (mp->live)
        fprintf(stderr, "%s(%d): malloc() = %p not freed (memory leak or forgot to call soap_end()?)\n", mp->file, mp->line, mp->ptr);
      free(mp);
    }
    soap->mht[i] = NULL;
  }
}
#endif

/******************************************************************************/

#ifdef SOAP_MEM_DEBUG
SOAP_FMAC1
void*
SOAP_FMAC2
soap_track_malloc(struct soap *soap, const char *file, int line, size_t size)
{
  void *p = malloc(size);
  if (soap)
  {
    size_t h = soap_hash_ptr(p);
    struct soap_mlist *mp = (struct soap_mlist*)malloc(sizeof(struct soap_mlist));
    if (soap->fdebug[SOAP_INDEX_TEST])
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "%s(%d): malloc(%lu) = %p\n", file, line, (unsigned long)size, p));
    }
    mp->next = soap->mht[h];
    mp->ptr = p;
    mp->file = file;
    mp->line = line;
    mp->live = 1;
    soap->mht[h] = mp;
  }
  return p;
}
#endif

/******************************************************************************/

#ifdef SOAP_MEM_DEBUG
SOAP_FMAC1
void
SOAP_FMAC2
soap_track_free(struct soap *soap, const char *file, int line, void *p)
{
  if (!soap)
  {
    free(p);
  }
  else
  {
    size_t h = soap_hash_ptr(p);
    struct soap_mlist *mp;
    for (mp = soap->mht[h]; mp; mp = mp->next)
      if (mp->ptr == p)
        break;
    if (mp)
    {
      if (mp->live)
      {
        if (soap->fdebug[SOAP_INDEX_TEST])
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "%s(%d): free(%p)\n", file, line, p));
        }
        free(p);
        mp->live = 0;
      }
      else
      {
        fprintf(stderr, "%s(%d): free(%p) double free of pointer malloced at %s(%d)\n", file, line, p, mp->file, mp->line);
      }
    }
    else
    {
      fprintf(stderr, "%s(%d): free(%p) pointer not malloced\n", file, line, p);
    }
  }
}
#endif

/******************************************************************************/

#ifdef SOAP_MEM_DEBUG
static void
soap_track_unlink(struct soap *soap, const void *p)
{
  size_t h = soap_hash_ptr(p);
  struct soap_mlist *mp;
  for (mp = soap->mht[h]; mp; mp = mp->next)
    if (mp->ptr == p)
      break;
  if (mp)
    mp->live = 0;
}
#endif

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_dealloc(struct soap *soap, void *p)
{
  if (soap_check_state(soap))
    return;
  if (p)
  {
    char **q;
    for (q = (char**)(void*)&soap->alist; *q; q = *(char***)q)
    {
      if (*(unsigned short*)(char*)(*q - sizeof(unsigned short)) != (unsigned short)SOAP_CANARY)
      {
#ifdef SOAP_MEM_DEBUG
        fprintf(stderr, "Data corruption in dynamic allocation (see logs)\n");
#endif
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Data corruption:\n"));
        DBGHEX(TEST, *q - 200, 200);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n"));
        soap->error = SOAP_MOE;
        return;
      }
      if (p == (void*)(*q - *(size_t*)(*q + sizeof(void*))))
      {
        *q = **(char***)q;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Freed data at %p\n", p));
        SOAP_FREE(soap, p);
        return;
      }
    }
    soap_delete(soap, p);
  }
  else
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Free all soap_malloc() data\n"));
    while (soap->alist)
    {
      char *q = (char*)soap->alist;
      if (*(unsigned short*)(char*)(q - sizeof(unsigned short)) != (unsigned short)SOAP_CANARY)
      {
#ifdef SOAP_MEM_DEBUG
        fprintf(stderr, "Data corruption in dynamic allocation (see logs)\n");
#endif
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Data corruption:\n"));
        DBGHEX(TEST, q - 200, 200);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n"));
        soap->error = SOAP_MOE;
        return;
      }
      soap->alist = *(void**)q;
      q -= *(size_t*)(q + sizeof(void*));
      SOAP_FREE(soap, q);
    }
    /* assume these were deallocated: */
    soap->http_content = NULL;
    soap->action = NULL;
    soap->fault = NULL;
    soap->header = NULL;
    soap->bearer = NULL;
    soap->userid = NULL;
    soap->passwd = NULL;
    soap->authrealm = NULL;
#ifdef WITH_NTLM
    soap->ntlm_challenge = NULL;
#endif
#ifndef WITH_LEANER
    soap_clr_mime(soap);
#endif
  }
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_delete(struct soap *soap, void *p)
{
  struct soap_clist **cp;
  if (soap_check_state(soap))
    return;
  cp = &soap->clist;
  if (p)
  {
    while (*cp)
    {
      if (p == (*cp)->ptr)
      {
        struct soap_clist *q = *cp;
        *cp = q->next;
        if (q->fdelete(soap, q))
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not dealloc data %p: deletion callback failed for object type=%d\n", q->ptr, q->type));
#ifdef SOAP_MEM_DEBUG
          fprintf(stderr, "new(object type=%d) = %p not freed: deletion callback failed\n", q->type, q->ptr);
#endif
        }
        SOAP_FREE(soap, q);
        return;
      }
      cp = &(*cp)->next;
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not dealloc data %p: address not in list\n", p));
  }
  else
  {
    while (*cp)
    {
      struct soap_clist *q = *cp;
      *cp = q->next;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Delete %p type=%d (cp=%p)\n", q->ptr, q->type, (void*)q));
      if (q->fdelete(soap, q))
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not dealloc data %p: deletion callback failed for object type=%d\n", q->ptr, q->type));
#ifdef SOAP_MEM_DEBUG
        fprintf(stderr, "new(object type=%d) = %p not freed: deletion callback failed\n", q->type, q->ptr);
#endif
      }
      SOAP_FREE(soap, q);
    }
  }
  soap->fault = NULL; /* assume this was deallocated */
  soap->header = NULL; /* assume this was deallocated */
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_delegate_deletion(struct soap *soap, struct soap *soap_to)
{
  struct soap_clist *cp;
  char **q;
#ifdef SOAP_MEM_DEBUG
  void *p;
  struct soap_mlist **mp, *mq;
  size_t h;
#endif
  for (q = (char**)(void*)&soap->alist; *q; q = *(char***)q)
  {
    if (*(unsigned short*)(char*)(*q - sizeof(unsigned short)) != (unsigned short)SOAP_CANARY)
    {
#ifdef SOAP_MEM_DEBUG
      fprintf(stderr, "Data corruption in dynamic allocation (see logs)\n");
#endif
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Data corruption:\n"));
      DBGHEX(TEST, *q - 200, 200);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n"));
      soap->error = SOAP_MOE;
      return;
    }
#ifdef SOAP_MEM_DEBUG
    p = (void*)(*q - *(size_t*)(*q + sizeof(void*)));
    h = soap_hash_ptr(p);
    for (mp = &soap->mht[h]; *mp; mp = &(*mp)->next)
    {
      if ((*mp)->ptr == p)
      {
        mq = *mp;
        *mp = mq->next;
        mq->next = soap_to->mht[h];
        soap_to->mht[h] = mq;
        break;
      }
    }
#endif
  }
  *q = (char*)soap_to->alist;
  soap_to->alist = soap->alist;
  soap->alist = NULL;
#ifdef SOAP_MEM_DEBUG
  cp = soap->clist;
  while (cp)
  {
    h = soap_hash_ptr(cp);
    for (mp = &soap->mht[h]; *mp; mp = &(*mp)->next)
    {
      if ((*mp)->ptr == cp)
      {
        mq = *mp;
        *mp = mq->next;
        mq->next = soap_to->mht[h];
        soap_to->mht[h] = mq;
        break;
      }
    }
    cp = cp->next;
  }
#endif
  cp = soap_to->clist;
  if (cp)
  {
    while (cp->next)
      cp = cp->next;
    cp->next = soap->clist;
  }
  else
  {
    soap_to->clist = soap->clist;
  }
  soap->clist = NULL;
}

/******************************************************************************/

SOAP_FMAC1
struct soap_clist *
SOAP_FMAC2
soap_link(struct soap *soap, int t, int n, int (*fdelete)(struct soap*, struct soap_clist*))
{
  struct soap_clist *cp = NULL;
  if (soap)
  {
    if (n != SOAP_NO_LINK_TO_DELETE)
    {
      cp = (struct soap_clist*)SOAP_MALLOC(soap, sizeof(struct soap_clist));
      if (!cp)
      {
        soap->error = SOAP_EOM;
      }
      else
      {
        cp->next = soap->clist;
        cp->type = t;
        cp->size = n;
        cp->ptr = NULL;
        cp->fdelete = fdelete;
        soap->clist = cp;
      }
    }
    soap->alloced = t;
  }
  return cp;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_unlink(struct soap *soap, const void *p)
{
  if (soap && p)
  {
    char **q;
    struct soap_clist **cp;
    for (q = (char**)(void*)&soap->alist; *q; q = *(char***)q)
    {
      if (p == (void*)(*q - *(size_t*)(*q + sizeof(void*))))
      {
        *q = **(char***)q;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Unlinked data %p\n", p));
#ifdef SOAP_MEM_DEBUG
        soap_track_unlink(soap, p);
#endif
        return SOAP_OK;         /* found and removed from dealloc chain */
      }
    }
    for (cp = &soap->clist; *cp; cp = &(*cp)->next)
    {
      if (p == (*cp)->ptr)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Unlinked class instance %p\n", p));
        q = (char**)*cp;
        *cp = (*cp)->next;
        SOAP_FREE(soap, q);
        return SOAP_OK;         /* found and removed from dealloc chain */
      }
    }
  }
  return SOAP_ERR;
}

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_lookup_type(struct soap *soap, const char *id)
{
  struct soap_ilist *ip;
  if (id && *id)
  {
    ip = soap_lookup(soap, id);
    if (ip)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Lookup id='%s' type=%d\n", id, ip->type));
      return ip->type;
    }
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "lookup type id='%s' NOT FOUND! Need to get it from xsi:type\n", id));
  return 0;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
short
SOAP_FMAC2
soap_begin_shaky(struct soap *soap)
{
  short f = soap->shaky;
  soap->shaky = 1;
  return f;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
void
SOAP_FMAC2
soap_end_shaky(struct soap *soap, short f)
{
  soap->shaky = f;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
static int
soap_is_shaky(struct soap *soap, void *p)
{
  (void)p;
  if (!soap->blist && !soap->shaky)
    return 0;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Shaky %p\n", p));
  return 1;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
void*
SOAP_FMAC2
soap_id_lookup(struct soap *soap, const char *id, void **p, int t, size_t n, unsigned int k, int (*fbase)(int, int))
{
  struct soap_ilist *ip;
  if (!p || !id || !*id)
    return p;
  ip = soap_lookup(soap, id); /* lookup pointer to hash table entry for string id */
  if (!ip)
  {
    ip = soap_enter(soap, id, t, n); /* new hash table entry for string id */
    if (!ip)
      return NULL;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Forwarding first href='%s' type=%d location=%p (%u bytes) level=%u\n", id, t, (void*)p, (unsigned int)n, k));
    *p = NULL;
    if (k)
    {
      int i;
      if (k > SOAP_MAXPTRS)
        return NULL;
      ip->spine = (void**)soap_malloc(soap, SOAP_MAXPTRS * sizeof(void*));
      if (!ip->spine)
        return NULL;
      ip->spine[0] = NULL;
      for (i = 1; i < SOAP_MAXPTRS; i++)
        ip->spine[i] = &ip->spine[i - 1];
      *p = &ip->spine[k - 1];
    }
    else
    {
      ip->link = p;
      ip->shaky = soap_is_shaky(soap, (void*)p);
    }
  }
  else if (ip->type != t && (!fbase || !fbase(ip->type, t)) && (!fbase || !fbase(t, ip->type) || soap_type_punned(soap, ip)))
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Lookup type incompatibility: ref='%s' id-type=%d ref-type=%d\n", id, ip->type, t));
    (void)soap_id_nullify(soap, id);
    return NULL;
  }
  else if (k == 0 && ip->ptr && !ip->shaky) /* when block lists are in use, ip->ptr will change */
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Resolved href='%s' type=%d location=%p (%u bytes) level=%u\n", id, t, ip->ptr, (unsigned int)n, k));
    *p = ip->ptr;
  }
  else
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Forwarded href='%s' type=%d location=%p (%u bytes) level=%u\n", id, t, (void*)p, (unsigned int)n, k));
    if (fbase && fbase(t, ip->type) && !soap_type_punned(soap, ip))
    {
      ip->type = t;
      ip->size = n;
    }
    *p = NULL;
    if (k)
    {
      if (!ip->spine)
      {
        int i;
        if (k > SOAP_MAXPTRS)
          return NULL;
        ip->spine = (void**)soap_malloc(soap, SOAP_MAXPTRS * sizeof(void*));
        if (!ip->spine)
          return NULL;
        ip->spine[0] = NULL;
        for (i = 1; i < SOAP_MAXPTRS; i++)
          ip->spine[i] = &ip->spine[i - 1];
      }
      *p = &ip->spine[k - 1];
      if (ip->ptr && !ip->shaky)
        ip->spine[0] = ip->ptr;
    }
    else
    {
      void *q = ip->link;
      ip->link = p;
      ip->shaky = soap_is_shaky(soap, (void*)p);
      *p = q;
    }
  }
  return p;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
void*
SOAP_FMAC2
soap_id_forward(struct soap *soap, const char *href, void *p, size_t i, int t, int tt, size_t n, unsigned int k, void (*finsert)(struct soap*, int, int, void*, size_t, const void*, void**), int (*fbase)(int, int))
{
  struct soap_ilist *ip;
  if (!p || !href || !*href)
    return p;
  ip = soap_lookup(soap, href); /* lookup pointer to hash table entry for string id */
  if (!ip)
  {
    ip = soap_enter(soap, href, t, n); /* new hash table entry for string id */
    if (!ip)
      return NULL;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "New entry href='%s' type=%d size=%lu level=%d location=%p\n", href, t, (unsigned long)n, k, p));
  }
  else if ((ip->type != t || ip->size != n) && k == 0)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Forward type incompatibility id='%s' expect type=%d size=%lu level=%u got type=%d size=%lu\n", href, ip->type, (unsigned long)ip->size, k, t, (unsigned long)n));
    (void)soap_id_nullify(soap, href);
    return NULL;
  }
  if (finsert || n < sizeof(void*))
  {
    struct soap_flist *fp = (struct soap_flist*)SOAP_MALLOC(soap, sizeof(struct soap_flist));
    if (!fp)
    {
      soap->error = SOAP_EOM;
      return NULL;
    }
    if (fbase && fbase(t, ip->type) && !soap_type_punned(soap, ip))
    {
      ip->type = t;
      ip->size = n;
    }
    if ((ip->type != t || ip->size != n) && (!fbase || !fbase(ip->type, t)))
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Forward type incompatibility id='%s' expect type=%d size=%lu level=%u got type=%d size=%lu\n", href, ip->type, (unsigned long)ip->size, k, t, (unsigned long)n));
      SOAP_FREE(soap, fp);
      (void)soap_id_nullify(soap, href);
      return NULL;
    }
    fp->next = ip->flist;
    fp->type = tt;
    fp->ptr = p;
    fp->level = k;
    fp->index = i;
    fp->finsert = finsert;
    ip->flist = fp;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Forwarding type=%d (target type=%d) size=%lu location=%p level=%u index=%lu href='%s'\n", t, tt, (unsigned long)n, p, k, (unsigned long)i, href));
  }
  else
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Forwarding copying address %p for type=%d href='%s'\n", p, t, href));
    *(void**)p = ip->copy;
    ip->copy = p;
  }
  ip->shaky = soap_is_shaky(soap, p);
  return p;
}
#endif

/******************************************************************************/

SOAP_FMAC1
void*
SOAP_FMAC2
soap_id_enter(struct soap *soap, const char *id, void *p, int t, size_t n, const char *type, const char *arrayType, void *(*finstantiate)(struct soap*, int, const char*, const char*, size_t*), int (*fbase)(int, int))
{
#ifndef WITH_NOIDREF
  struct soap_ilist *ip;
#endif
  (void)id; (void)fbase;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Enter id='%s' type=%d location=%p size=%lu\n", id, t, p, (unsigned long)n));
  soap->alloced = 0;
  if (!p)
  {
    if (finstantiate)
    {
      p = finstantiate(soap, t, type, arrayType, &n); /* soap->alloced is set in soap_link() */
      t = soap->alloced;
    }
    else
    {
      p = soap_malloc(soap, n);
      soap->alloced = t;
    }
  }
#ifndef WITH_NOIDREF
  if (!id || !*id)
#endif
    return p;
#ifndef WITH_NOIDREF
  ip = soap_lookup(soap, id); /* lookup pointer to hash table entry for string id */
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Lookup entry id='%s' for location=%p type=%d\n", id, p, t));
  if (!ip)
  {
    ip = soap_enter(soap, id, t, n); /* new hash table entry for string id */
    if (!ip)
      return NULL;
    ip->ptr = p;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "New entry id='%s' type=%d size=%lu location=%p\n", id, t, (unsigned long)n, p));
    if (!soap->alloced)
      ip->shaky = soap_is_shaky(soap, p);
  }
  else if (ip->ptr)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Multiply defined id='%s'\n", id));
    soap_strcpy(soap->id, sizeof(soap->id), id);
    soap->error = SOAP_DUPLICATE_ID;
    return NULL;
  }
  else if ((ip->type != t && (!fbase || !fbase(t, ip->type) || soap_type_punned(soap, ip)))
        || (ip->type == t && ip->size != n && soap_type_punned(soap, ip)))
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Enter type incompatibility id='%s' expect type=%d size=%lu got type=%d size=%lu\n", id, ip->type, (unsigned long)ip->size, t, (unsigned long)n));
    (void)soap_id_nullify(soap, id);
    return NULL;
  }
  else
  {
    ip->type = t;
    ip->size = n;
    ip->ptr = p;
    if (!soap->alloced)
      ip->shaky = soap_is_shaky(soap, p);
    if (soap->alloced || !ip->shaky)
    {
      void **q; /* ptr will not change later, so resolve links now */
      if (ip->spine)
        ip->spine[0] = p;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Traversing link chain to resolve id='%s' type=%d\n", ip->id, ip->type));
      q = (void**)ip->link;
      while (q)
      {
        void *r = *q;
        *q = p;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "... link %p -> %p\n", (void*)q, p));
        q = (void**)r;
      }
      ip->link = NULL;
    }
  }
  return ip->ptr;
#endif
}

/******************************************************************************/

SOAP_FMAC1
void**
SOAP_FMAC2
soap_id_smart(struct soap *soap, const char *id, int t, size_t n)
{
  (void)soap; (void)id; (void)t; (void)n;
#ifndef WITH_NOIDREF
  if (id && *id)
  {
    struct soap_ilist *ip = soap_lookup(soap, id); /* lookup pointer to hash table entry for string id */
    if (!ip)
    {
      ip = soap_enter(soap, id, t, n); /* new hash table entry for string id */
      if (!ip)
        return NULL;
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "New smart shared pointer entry id='%s' type=%d size=%lu smart=%p\n", id, t, (unsigned long)n, ip->smart));
    return &ip->smart;
  }
#endif
  return NULL;
}

/******************************************************************************/

#ifndef WITH_NOIDREF
static int
soap_type_punned(struct soap *soap, const struct soap_ilist *ip)
{
  const struct soap_flist *fp;
  (void)soap;
  if (ip->ptr || ip->copy)
    return 1;
  for (fp = ip->flist; fp; fp = fp->next)
    if (fp->level == 0)
      return 1;
  return 0;
}
#endif

/******************************************************************************/

#ifndef WITH_NOIDREF
SOAP_FMAC1
int
SOAP_FMAC2
soap_id_nullify(struct soap *soap, const char *id)
{
  int i;
  for (i = 0; i < SOAP_IDHASH; i++)
  {
    struct soap_ilist *ip;
    for (ip = soap->iht[i]; ip; ip = ip->next)
    {
      void *p, *q;
      for (p = ip->link; p; p = q)
      {
        q = *(void**)p;
        *(void**)p = NULL;
      }
      ip->link = NULL;
    }
  }
  soap_strcpy(soap->id, sizeof(soap->id), id);
  return soap->error = SOAP_HREF;
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_end_send(struct soap *soap)
{
#ifndef WITH_LEANER
  int err = soap_end_attachments(soap);
  if (soap->dime.list)
  {
    /* SOAP body referenced attachments must appear first */
    soap->dime.last->next = soap->dime.first;
    soap->dime.first = soap->dime.list->next;
    soap->dime.list->next = NULL;
    soap->dime.last = soap->dime.list;
  }
  if (!err)
    err = soap_putdime(soap);
  if (!err)
    err = soap_putmime(soap);
  soap->mime.list = NULL;
  soap->mime.first = NULL;
  soap->mime.last = NULL;
  soap->dime.list = NULL;
  soap->dime.first = NULL;
  soap->dime.last = NULL;
  if (err)
    return err;
#endif
  return soap_end_send_flush(soap);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_end_send_flush(struct soap *soap)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "End send mode=0x%x\n", soap->mode));
  if ((soap->mode & SOAP_IO)) /* need to flush the remaining data in buffer */
  {
    if (soap_flush(soap))
#ifdef WITH_ZLIB
    {
      if ((soap->mode & SOAP_ENC_ZLIB) && soap->zlib_state == SOAP_ZLIB_DEFLATE)
      {
        soap->zlib_state = SOAP_ZLIB_NONE;
        deflateEnd(soap->d_stream);
      }
      return soap->error;
    }
#else
      return soap->error;
#endif
#ifdef WITH_ZLIB
    if ((soap->mode & SOAP_ENC_ZLIB) && soap->d_stream)
    {
      int r;
      soap->d_stream->avail_in = 0;
      do
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Deflating remainder\n"));
        r = deflate(soap->d_stream, Z_FINISH);
        if (soap->d_stream->avail_out != sizeof(soap->buf))
        {
          if (soap_flush_raw(soap, soap->z_buf, sizeof(soap->buf) - soap->d_stream->avail_out))
          {
            soap->zlib_state = SOAP_ZLIB_NONE;
            deflateEnd(soap->d_stream);
            return soap->error;
          }
          soap->d_stream->next_out = (Byte*)soap->z_buf;
          soap->d_stream->avail_out = sizeof(soap->buf);
        }
      } while (r == Z_OK);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Deflated total %lu->%lu bytes\n", soap->d_stream->total_in, soap->d_stream->total_out));
      soap->z_ratio_out = (float)soap->d_stream->total_out / (float)soap->d_stream->total_in;
      soap->mode &= ~SOAP_ENC_ZLIB;
      soap->zlib_state = SOAP_ZLIB_NONE;
      if (deflateEnd(soap->d_stream) != Z_OK || r != Z_STREAM_END)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Unable to end deflate: %s\n", soap->d_stream->msg ? soap->d_stream->msg : SOAP_STR_EOS));
        return soap->error = SOAP_ZLIB_ERROR;
      }
#ifdef WITH_GZIP
      if (soap->zlib_out != SOAP_ZLIB_DEFLATE)
      {
        soap->z_buf[0] = soap->z_crc & 0xFF;
        soap->z_buf[1] = (soap->z_crc >> 8) & 0xFF;
        soap->z_buf[2] = (soap->z_crc >> 16) & 0xFF;
        soap->z_buf[3] = (soap->z_crc >> 24) & 0xFF;
        soap->z_buf[4] = soap->d_stream->total_in & 0xFF;
        soap->z_buf[5] = (soap->d_stream->total_in >> 8) & 0xFF;
        soap->z_buf[6] = (soap->d_stream->total_in >> 16) & 0xFF;
        soap->z_buf[7] = (soap->d_stream->total_in >> 24) & 0xFF;
        if (soap_flush_raw(soap, soap->z_buf, 8))
          return soap->error;
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "gzip crc32=%lu\n", (unsigned long)soap->z_crc));
      }
#endif
    }
#endif
    if ((soap->mode & SOAP_IO) == SOAP_IO_STORE)
    {
#if !defined(__cplusplus) || defined(WITH_COMPAT)
      if (soap->os)
      {
        char *b = (char*)soap_push_block(soap, NULL, 1);
        if (b)
        {
          *b = '\0';
          *soap->os = (char*)soap_save_block(soap, NULL, NULL, 0);
        }
      }
      else
#endif
      {
        char *p;
#ifndef WITH_NOHTTP
        if (!(soap->mode & SOAP_ENC_PLAIN))
        {
          soap->mode--;
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Sending buffered message of length %u\n", (unsigned int)soap->blist->size));
          if (soap->status >= SOAP_POST)
            soap->error = soap->fpost(soap, soap->endpoint, soap->host, soap->port, soap->path, soap->action, soap->blist->size);
          else if (soap->status != SOAP_STOP)
            soap->error = soap->fresponse(soap, soap->status, soap->blist->size);
          if (soap->error || soap_flush(soap))
            return soap->error;
          soap->mode++;
        }
#endif
        for (p = soap_first_block(soap, NULL); p; p = soap_next_block(soap, NULL))
        {
          DBGMSG(SENT, p, soap_block_size(soap, NULL));
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Send %u bytes to socket=%d/fd=%d\n", (unsigned int)soap_block_size(soap, NULL), (int)soap->socket, soap->sendfd));
          soap->error = soap->fsend(soap, p, soap_block_size(soap, NULL));
          if (soap->error)
          {
            soap_end_block(soap, NULL);
            return soap->error;
          }
        }
        soap_end_block(soap, NULL);
      }
#ifndef WITH_LEANER
      if (soap->fpreparefinalsend && (soap->error = soap->fpreparefinalsend(soap)) != SOAP_OK)
        return soap->error;
#endif
      if ((soap->omode & SOAP_IO) == SOAP_IO_STORE && (soap->imode & SOAP_IO) != SOAP_IO_STORE)
        soap->omode = (soap->omode & ~SOAP_IO) | (soap->imode & SOAP_IO);
    }
#ifndef WITH_LEANER
    else if ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK)
    {
      DBGMSG(SENT, "\r\n0\r\n\r\n", 7);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Send 7 bytes to socket=%d/fd=%d\n", (int)soap->socket, soap->sendfd));
      soap->error = soap->fsend(soap, "\r\n0\r\n\r\n", 7);
      if (soap->error)
        return soap->error;
    }
#endif
  }
#ifdef WITH_TCPFIN
#if defined(WITH_OPENSSL) || defined(WITH_SYSTEMSSL)
  if (!soap->ssl)
#endif
    if (soap_valid_socket(soap->socket) && !soap->keep_alive && !(soap->omode & SOAP_IO_UDP))
      (void)soap->fshutdownsocket(soap, soap->socket, SOAP_SHUT_WR); /* Send TCP FIN */
#endif
#if defined(__cplusplus) && !defined(WITH_COMPAT)
  if (soap->os)
    soap->os->flush();
#endif
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "End of send phase\n"));
  soap->omode &= ~SOAP_SEC_WSUID;
  soap->count = 0;
  soap->part = SOAP_END;
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_end_recv(struct soap *soap)
{
  soap->part = SOAP_END;
#ifndef WITH_LEAN
  soap->wsuid = NULL;           /* reset before next send */
  soap->c14nexclude = NULL;     /* reset before next send */
  soap->c14ninclude = NULL;     /* reset before next send */
#endif
#ifndef WITH_LEANER
  soap->ffilterrecv = NULL;
  if ((soap->mode & SOAP_ENC_DIME) && soap_getdime(soap))
  {
    soap->dime.first = NULL;
    soap->dime.last = NULL;
    return soap->error;
  }
  soap->dime.list = soap->dime.first;
  soap->dime.first = NULL;
  soap->dime.last = NULL;
  /* Check if MIME attachments and mime-post-check flag is set, if so call soap_resolve() and return */
  if ((soap->mode & SOAP_ENC_MIME))
  {
    if ((soap->mode & SOAP_MIME_POSTCHECK))
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Post checking MIME attachments\n"));
      if (!soap->keep_alive)
        soap->keep_alive = -2; /* special case to keep alive */
#ifndef WITH_NOIDREF
      soap_resolve(soap);
#endif
      return SOAP_OK;
    }
    if (soap_getmime(soap))
      return soap->error;
  }
  soap->mime.list = soap->mime.first;
  soap->mime.first = NULL;
  soap->mime.last = NULL;
  soap->mime.boundary = NULL;
  if (soap->xlist)
  {
    struct soap_multipart *content;
    for (content = soap->mime.list; content; content = content->next)
      soap_resolve_attachment(soap, content);
  }
#endif
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "End of receive message ok\n"));
#ifdef WITH_ZLIB
  if ((soap->mode & SOAP_ENC_ZLIB) && soap->d_stream)
  {
    /* Make sure end of compressed content is reached */
    while (soap->d_stream->next_out != Z_NULL)
      if ((int)soap_get1(soap) == EOF)
        break;
    soap->mode &= ~SOAP_ENC_ZLIB;
    (void)soap_memcpy((void*)soap->buf, sizeof(soap->buf), (const void*)soap->z_buf, sizeof(soap->buf));
    soap->bufidx = (char*)soap->d_stream->next_in - soap->z_buf;
    soap->buflen = soap->z_buflen;
    soap->zlib_state = SOAP_ZLIB_NONE;
    if (inflateEnd(soap->d_stream) != Z_OK)
      return soap->error = SOAP_ZLIB_ERROR;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Inflate end ok\n"));
#ifdef WITH_GZIP
    if (soap->zlib_in == SOAP_ZLIB_GZIP)
    {
      soap_wchar c;
      short i;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Inflate gzip crc check\n"));
      for (i = 0; i < 8; i++)
      {
        if ((int)(c = soap_get1(soap)) == EOF)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Gzip error: unable to read crc value\n"));
          return soap->error = SOAP_ZLIB_ERROR;
        }
        soap->z_buf[i] = (char)c;
      }
      if (soap->z_crc != ((uLong)(unsigned char)soap->z_buf[0] | ((uLong)(unsigned char)soap->z_buf[1] << 8) | ((uLong)(unsigned char)soap->z_buf[2] << 16) | ((uLong)(unsigned char)soap->z_buf[3] << 24)))
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Gzip inflate error: crc check failed, message corrupted? (crc32=%lu)\n", (unsigned long)soap->z_crc));
        return soap->error = SOAP_ZLIB_ERROR;
      }
      if (soap->d_stream->total_out != ((uLong)(unsigned char)soap->z_buf[4] | ((uLong)(unsigned char)soap->z_buf[5] << 8) | ((uLong)(unsigned char)soap->z_buf[6] << 16) | ((uLong)(unsigned char)soap->z_buf[7] << 24)))
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Gzip inflate error: incorrect message length\n"));
        return soap->error = SOAP_ZLIB_ERROR;
      }
    }
    soap->zlib_in = SOAP_ZLIB_NONE;
#endif
  }
#endif
  if ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK)
    while ((int)soap->ahead != EOF && !soap_recv_raw(soap))
      continue;
#ifndef WITH_NOIDREF
  if (soap_resolve(soap))
    return soap->error;
#endif
#ifndef WITH_LEANER
  if (soap->xlist)
  {
    if ((soap->mode & SOAP_ENC_MTOM))
      return soap->error = SOAP_MIME_HREF;
    return soap->error = SOAP_DIME_HREF;
  }
#endif
  soap_free_ns(soap);
#ifndef WITH_LEANER
  if (soap->fpreparefinalrecv)
    return soap->error = soap->fpreparefinalrecv(soap);
#endif
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_free_temp(struct soap *soap)
{
  struct soap_attribute *tp, *tq;
  struct Namespace *ns;
  soap_free_ns(soap);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Free any remaining temp blocks\n"));
  while (soap->blist)
    soap_end_block(soap, NULL);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Free attribute storage\n"));
  for (tp = soap->attributes; tp; tp = tq)
  {
    tq = tp->next;
    if (tp->value)
      SOAP_FREE(soap, tp->value);
    SOAP_FREE(soap, tp);
  }
  soap->attributes = NULL;
#ifdef WITH_FAST
  if (soap->labbuf)
    SOAP_FREE(soap, soap->labbuf);
  soap->labbuf = NULL;
  soap->lablen = 0;
  soap->labidx = 0;
#endif
  ns = soap->local_namespaces;
  if (ns)
  {
    for (; ns->id; ns++)
    {
      if (ns->out)
      {
        SOAP_FREE(soap, ns->out);
        ns->out = NULL;
      }
    }
    SOAP_FREE(soap, soap->local_namespaces);
    soap->local_namespaces = NULL;
  }
#ifndef WITH_LEANER
  while (soap->xlist)
  {
    struct soap_xlist *xp = soap->xlist->next;
    SOAP_FREE(soap, soap->xlist);
    soap->xlist = xp;
  }
#endif
#ifndef WITH_NOIDREF
  soap_free_iht(soap);
#endif
  soap_free_pht(soap);
}

/******************************************************************************/

static void
soap_free_ns(struct soap *soap)
{
  struct soap_nlist *np, *nq;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Free namespace stack\n"));
  for (np = soap->nlist; np; np = nq)
  {
    nq = np->next;
    SOAP_FREE(soap, np);
  }
  soap->nlist = NULL;
}

/******************************************************************************/

#ifdef SOAP_DEBUG
static void
soap_init_logs(struct soap *soap)
{
  int i;
  for (i = 0; i < SOAP_MAXLOGS; i++)
  {
    soap->logfile[i] = NULL;
    soap->fdebug[i] = NULL;
  }
}
#endif

/******************************************************************************/

#ifdef SOAP_DEBUG
SOAP_FMAC1
void
SOAP_FMAC2
soap_open_logfile(struct soap *soap, int i)
{
  if (soap->logfile[i])
    soap->fdebug[i] = fopen(soap->logfile[i], i < 2 ? "ab" : "a");
}
#endif

/******************************************************************************/

#ifdef SOAP_DEBUG
static void
soap_close_logfile(struct soap *soap, int i)
{
  if (soap->fdebug[i])
  {
    fclose(soap->fdebug[i]);
    soap->fdebug[i] = NULL;
  }
}
#endif

/******************************************************************************/

#ifdef SOAP_DEBUG
SOAP_FMAC1
void
SOAP_FMAC2
soap_close_logfiles(struct soap *soap)
{
  int i;
  for (i = 0; i < SOAP_MAXLOGS; i++)
    soap_close_logfile(soap, i);
}
#endif

/******************************************************************************/

#ifdef SOAP_DEBUG
static void
soap_set_logfile(struct soap *soap, int i, const char *logfile)
{
  const char *s;
  char *t = NULL;
  soap_close_logfile(soap, i);
  s = soap->logfile[i];
  if (s)
    SOAP_FREE_UNMANAGED(s);
  if (logfile)
  {
    size_t l = strlen(logfile) + 1;
    t = (char*)SOAP_MALLOC_UNMANAGED(l);
    if (t)
      (void)soap_memcpy((void*)t, l, (const void*)logfile, l);
  }
  soap->logfile[i] = t;
}
#endif

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_set_recv_logfile(struct soap *soap, const char *logfile)
{
  (void)soap; (void)logfile;
#ifdef SOAP_DEBUG
  soap_set_logfile(soap, SOAP_INDEX_RECV, logfile);
#endif
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_set_sent_logfile(struct soap *soap, const char *logfile)
{
  (void)soap; (void)logfile;
#ifdef SOAP_DEBUG
  soap_set_logfile(soap, SOAP_INDEX_SENT, logfile);
#endif
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_set_test_logfile(struct soap *soap, const char *logfile)
{
  (void)soap; (void)logfile;
#ifdef SOAP_DEBUG
  soap_set_logfile(soap, SOAP_INDEX_TEST, logfile);
#endif
}

/******************************************************************************/

SOAP_FMAC1
struct soap*
SOAP_FMAC2
soap_copy(const struct soap *soap)
{
  struct soap *copy = soap_versioning(soap_new)(SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);
  soap_set_test_logfile(copy, NULL);
  soap_set_sent_logfile(copy, NULL);
  soap_set_recv_logfile(copy, NULL);
  soap_done(copy);
  if (soap_copy_context(copy, soap) != NULL)
    return copy;
  soap_free(copy);
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
struct soap*
SOAP_FMAC2
soap_copy_context(struct soap *copy, const struct soap *soap)
{
  if (copy == soap)
    return copy;
  if (soap_check_state(soap))
    return NULL;
  if (copy)
  {
    struct soap_plugin *p = NULL;
    (void)soap_memcpy((void*)copy, sizeof(struct soap), (const void*)soap, sizeof(struct soap));
    copy->state = SOAP_COPY;
#ifdef SOAP_MEM_DEBUG
    soap_init_mht(copy);
#endif
#ifdef SOAP_DEBUG
    soap_init_logs(copy);
    soap_set_test_logfile(copy, soap->logfile[SOAP_INDEX_TEST]);
    soap_set_sent_logfile(copy, soap->logfile[SOAP_INDEX_SENT]);
    soap_set_recv_logfile(copy, soap->logfile[SOAP_INDEX_RECV]);
#endif
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Copying context\n"));
    copy->error = SOAP_OK;
    copy->bearer = NULL;
    copy->userid = NULL;
    copy->passwd = NULL;
#ifdef WITH_NTLM
    copy->ntlm_challenge = NULL;
#endif
    copy->nlist = NULL;
    copy->blist = NULL;
    copy->clist = NULL;
    copy->alist = NULL;
    copy->attributes = NULL;
    copy->labbuf = NULL;
    copy->lablen = 0;
    copy->labidx = 0;
    copy->namespaces = soap->local_namespaces;
    copy->local_namespaces = NULL;
    soap_set_local_namespaces(copy); /* copy content of soap->local_namespaces */
    copy->namespaces = soap->namespaces; /* point to shared read-only namespaces table */
    copy->c_locale = NULL;
#ifdef WITH_OPENSSL
    copy->bio = NULL;
    copy->ssl = NULL;
    copy->session = NULL;
    copy->session_host[0] = '\0';
    copy->session_port = 443;
#endif
#ifdef WITH_GNUTLS
    copy->session = NULL;
#endif
#ifdef WITH_ZLIB
    copy->d_stream = NULL;
    copy->z_buf = NULL;
#endif
#ifndef WITH_NOIDREF
    soap_init_iht(copy);
#endif
    soap_init_pht(copy);
    copy->header = NULL;
    copy->fault = NULL;
    copy->action = NULL;
#ifndef WITH_LEAN
#ifdef WITH_COOKIES
    copy->cookies = soap_copy_cookies(copy, soap);
#else
    copy->cookies = NULL;
#endif
#endif
    copy->plugins = NULL;
    for (p = soap->plugins; p; p = p->next)
    {
      struct soap_plugin *q = (struct soap_plugin*)SOAP_MALLOC(copy, sizeof(struct soap_plugin));
      if (!q)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not allocate plugin '%s'\n", p->id));
        soap_end(copy);
        soap_done(copy);
        return NULL;
      }
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Copying plugin '%s'\n", p->id));
      *q = *p;
      if (p->fcopy && (copy->error = p->fcopy(copy, q, p)) != SOAP_OK)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not copy plugin '%s' error = %d\n", p->id, copy->error));
        SOAP_FREE(copy, q);
        soap_end(copy);
        soap_done(copy);
        return NULL;
      }
      q->next = copy->plugins;
      copy->plugins = q;
    }
  }
#ifdef WITH_SELF_PIPE
  pipe(copy->pipe_fd);
  SOAP_SOCKNONBLOCK(copy->pipe_fd[0])
  SOAP_SOCKNONBLOCK(copy->pipe_fd[1])
#endif
  return copy;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_copy_stream(struct soap *copy, struct soap *soap)
{
  struct soap_attribute *tp = NULL, *tq;
  if (copy == soap)
    return;
  copy->header = soap->header;
  copy->mode = soap->mode;
  copy->imode = soap->imode;
  copy->omode = soap->omode;
  copy->socket = soap->socket;
  copy->sendsk = soap->sendsk;
  copy->recvsk = soap->recvsk;
  copy->transfer_timeout = soap->transfer_timeout;
  copy->recv_maxlength = soap->recv_maxlength;
  copy->recv_timeout = soap->recv_timeout;
  copy->send_timeout = soap->send_timeout;
  copy->connect_timeout = soap->connect_timeout;
  copy->accept_timeout = soap->accept_timeout;
  copy->socket_flags = soap->socket_flags;
  copy->connect_flags = soap->connect_flags;
  copy->connect_retry = soap->connect_retry;
  copy->bind_flags = soap->bind_flags;
  copy->bind_inet6 = soap->bind_inet6;
  copy->bind_v6only = soap->bind_v6only;
  copy->accept_flags = soap->accept_flags;
  copy->sndbuf = soap->sndbuf;
  copy->rcvbuf = soap->rcvbuf;
  copy->linger_time = soap->linger_time;
  copy->maxlevel = soap->maxlevel;
  copy->maxlength = soap->maxlength;
  copy->maxoccurs = soap->maxoccurs;
  copy->os = soap->os;
  copy->is = soap->is;
  copy->sendfd = soap->sendfd;
  copy->recvfd = soap->recvfd;
  copy->bufidx = soap->bufidx;
  copy->buflen = soap->buflen;
  copy->ahead = soap->ahead;
  copy->cdata = soap->cdata;
  copy->chunksize = soap->chunksize;
  copy->chunkbuflen = soap->chunkbuflen;
  copy->keep_alive = soap->keep_alive;
  copy->tcp_keep_alive = soap->tcp_keep_alive;
  copy->tcp_keep_idle = soap->tcp_keep_idle;
  copy->tcp_keep_intvl = soap->tcp_keep_intvl;
  copy->tcp_keep_cnt = soap->tcp_keep_cnt;
  copy->max_keep_alive = soap->max_keep_alive;
#ifndef WITH_NOIO
  copy->peer = soap->peer;
  copy->peerlen = soap->peerlen;
  copy->ip = soap->ip;
  copy->ip6[0] = soap->ip6[0];
  copy->ip6[1] = soap->ip6[1];
  copy->ip6[2] = soap->ip6[2];
  copy->ip6[3] = soap->ip6[3];
  copy->port = soap->port;
  (void)soap_memcpy((void*)copy->host, sizeof(copy->host), (const void*)soap->host, sizeof(soap->host));
  (void)soap_memcpy((void*)copy->endpoint, sizeof(copy->endpoint), (const void*)soap->endpoint, sizeof(soap->endpoint));
#endif
#ifdef WITH_OPENSSL
  copy->bio = soap->bio;
  copy->ctx = soap->ctx;
  copy->ssl = soap->ssl;
#endif
#ifdef WITH_GNUTLS
  copy->session = soap->session;
#endif
#ifdef WITH_SYSTEMSSL
  copy->ctx = soap->ctx;
  copy->ssl = soap->ssl;
#endif
#ifdef WITH_ZLIB
  copy->zlib_state = soap->zlib_state;
  copy->zlib_in = soap->zlib_in;
  copy->zlib_out = soap->zlib_out;
  if (soap->d_stream && soap->zlib_state != SOAP_ZLIB_NONE)
  {
    if (!copy->d_stream)
      copy->d_stream = (z_stream*)SOAP_MALLOC(copy, sizeof(z_stream));
    if (copy->d_stream)
      (void)soap_memcpy((void*)copy->d_stream, sizeof(z_stream), (const void*)soap->d_stream, sizeof(z_stream));
  }
  copy->z_crc = soap->z_crc;
  copy->z_ratio_in = soap->z_ratio_in;
  copy->z_ratio_out = soap->z_ratio_out;
  copy->z_level = soap->z_level;
  if (soap->z_buf && soap->zlib_state != SOAP_ZLIB_NONE)
  {
    if (!copy->z_buf)
      copy->z_buf = (char*)SOAP_MALLOC(copy, sizeof(soap->buf));
    if (copy->z_buf)
      (void)soap_memcpy((void*)copy->z_buf, sizeof(soap->buf), (const void*)soap->z_buf, sizeof(soap->buf));
    else
      copy->z_buflen = 0;
  }
  else
  {
    copy->z_buf = NULL;
    copy->z_buflen = 0;
  }
  copy->z_dict = soap->z_dict;
  copy->z_dict_len = soap->z_dict_len;
#endif
  (void)soap_memcpy((void*)copy->buf, sizeof(copy->buf), (const void*)soap->buf, sizeof(soap->buf));
  /* copy XML parser state */
  soap_free_ns(copy);
  soap_set_local_namespaces(copy);
  copy->version = soap->version;
  if (soap->nlist && soap->local_namespaces)
  {
    struct soap_nlist *np = NULL, *nq;
    /* copy reversed nlist */
    for (nq = soap->nlist; nq; nq = nq->next)
    {
      struct soap_nlist *nr = np;
      size_t n = sizeof(struct soap_nlist) + strlen(nq->id);
      np = (struct soap_nlist*)SOAP_MALLOC(copy, n);
      if (!np)
      {
        np = nr;
        break;
      }
      (void)soap_memcpy((void*)np, n, (const void*)nq, n);
      np->next = nr;
    }
    while (np)
    {
      const char *s = np->ns;
      copy->level = np->level; /* preserve element nesting level */
      if (!s && np->index >= 0)
      {
        s = soap->local_namespaces[np->index].out;
        if (!s)
          s = soap->local_namespaces[np->index].ns;
      }
      if (s)
        (void)soap_push_namespace(copy, np->id, s);
      nq = np;
      np = np->next;
      SOAP_FREE(copy, nq);
    }
  }
  (void)soap_memcpy((void*)copy->tag, sizeof(copy->tag), (const void*)soap->tag, sizeof(soap->tag));
  (void)soap_memcpy((void*)copy->id, sizeof(copy->id), (const void*)soap->id, sizeof(soap->id));
  (void)soap_memcpy((void*)copy->href, sizeof(copy->href), (const void*)soap->href, sizeof(soap->href));
  (void)soap_memcpy((void*)copy->type, sizeof(copy->type), (const void*)soap->type, sizeof(soap->type));
  copy->other = soap->other;
  copy->root = soap->root;
  copy->null = soap->null;
  copy->body = soap->body;
  copy->part = soap->part;
  copy->mustUnderstand = soap->mustUnderstand;
  copy->level = soap->level;
  copy->peeked = soap->peeked;
  /* copy attributes */
  for (tq = soap->attributes; tq; tq = tq->next)
  {
    struct soap_attribute *tr = tp;
    size_t n = sizeof(struct soap_attribute) + strlen(tq->name);
    tp = (struct soap_attribute*)SOAP_MALLOC(copy, n);
    (void)soap_memcpy((void*)tp, n, (const void*)tq, n);
    if (tp->size)
    {
      tp->value = (char*)SOAP_MALLOC(copy, tp->size);
      if (tp->value)
        (void)soap_memcpy((void*)tp->value, tp->size, (const void*)tq->value, tp->size);
    }
    tp->ns = NULL;
    tp->next = tr;
  }
  copy->attributes = tp;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_free_stream(struct soap *soap)
{
  soap->socket = SOAP_INVALID_SOCKET;
  soap->sendsk = SOAP_INVALID_SOCKET;
  soap->recvsk = SOAP_INVALID_SOCKET;
#ifdef WITH_OPENSSL
  soap->bio = NULL;
  soap->ctx = NULL;
  soap->ssl = NULL;
#endif
#ifdef WITH_GNUTLS
  soap->xcred = NULL;
  soap->acred = NULL;
  soap->cache = NULL;
  soap->session = NULL;
  soap->dh_params = NULL;
  soap->rsa_params = NULL;
#endif
#ifdef WITH_SYSTEMSSL
  soap->ctx = (gsk_handle)NULL;
  soap->ssl = (gsk_handle)NULL;
#endif
#ifdef WITH_ZLIB
  if (soap->z_buf)
    SOAP_FREE(soap, soap->z_buf);
  soap->z_buf = NULL;
#endif
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_initialize(struct soap *soap)
{
  soap_versioning(soap_init)(soap, SOAP_IO_DEFAULT, SOAP_IO_DEFAULT);
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_versioning(soap_init)(struct soap *soap, soap_mode imode, soap_mode omode)
{
  size_t i;
  soap->state = SOAP_INIT;
#ifdef SOAP_MEM_DEBUG
  soap_init_mht(soap);
#endif
#ifdef SOAP_DEBUG
  soap_init_logs(soap);
#endif
#ifdef TANDEM_NONSTOP
  soap_set_test_logfile(soap, "TESTLOG");
  soap_set_sent_logfile(soap, "SENTLOG");
  soap_set_recv_logfile(soap, "RECVLOG");
#else
  soap_set_test_logfile(soap, "TEST.log");
  soap_set_sent_logfile(soap, "SENT.log");
  soap_set_recv_logfile(soap, "RECV.log");
#endif
#ifdef WITH_SELF_PIPE
  pipe(soap->pipe_fd);
  SOAP_SOCKNONBLOCK(soap->pipe_fd[0])
  SOAP_SOCKNONBLOCK(soap->pipe_fd[1])
#endif
  soap->version = 0;
  soap->imode = imode;
  soap->omode = omode;
  soap->mode = imode;
  soap->plugins = NULL;
  soap->user = NULL;
  for (i = 0; i < sizeof(soap->data)/sizeof(*soap->data); i++)
    soap->data[i] = NULL;
  soap->bearer = NULL;
  soap->userid = NULL;
  soap->passwd = NULL;
  soap->authrealm = NULL;
#ifdef WITH_NTLM
  soap->ntlm_challenge = NULL;
#endif
#ifndef WITH_NOHTTP
  soap->fpost = http_post;
  soap->fget = http_get;
  soap->fput = http_put;
  soap->fpatch = http_patch;
  soap->fdel = http_del;
  soap->fopt = http_200;
  soap->fhead = http_200;
  soap->fform = NULL;
  soap->fposthdr = http_post_header;
  soap->fresponse = http_response;
  soap->fparse = http_parse;
  soap->fparsehdr = http_parse_header;
#endif
  soap->fheader = NULL;
  soap->fconnect = NULL;
  soap->fdisconnect = NULL;
#ifndef WITH_NOIO
  soap->ipv6_multicast_if = 0; /* in_addr_t value */
  soap->ipv4_multicast_if = NULL; /* points to struct in_addr or in_addr_t */
  soap->ipv4_multicast_ttl = 0; /* 0: use default */
  soap->client_addr = NULL; /* client address (IPv4 or IPv6 or host name) to bind before connect, NULL for none */
  soap->client_addr_ipv6 = NULL; /* client address IPv6 or host name to bind before connect, NULL for none */
  soap->client_port = -1; /* client port to bind before connect, -1 for none */
  soap->client_interface = NULL; /* client interface address, NULL for none */
#ifndef WITH_IPV6
  soap->fresolve = tcp_gethost;
#else
  soap->fresolve = NULL;
#endif
  soap->faccept = tcp_accept;
  soap->fopen = tcp_connect;
  soap->fclose = tcp_disconnect;
  soap->fclosesocket = tcp_closesocket;
  soap->fshutdownsocket = tcp_shutdownsocket;
  soap->fsend = fsend;
  soap->frecv = frecv;
  soap->fpoll = soap_poll;
#else
  soap->fopen = NULL;
  soap->fclose = NULL;
  soap->fpoll = NULL;
#endif
  soap->fseterror = NULL;
  soap->fignore = NULL;
  soap->fserveloop = NULL;
  soap->fplugin = fplugin;
#ifndef WITH_LEANER
  soap->fsvalidate = NULL;
  soap->fwvalidate = NULL;
  soap->feltbegin = NULL;
  soap->feltendin = NULL;
  soap->feltbegout = NULL;
  soap->feltendout = NULL;
  soap->fprepareinitsend = NULL;
  soap->fprepareinitrecv = NULL;
  soap->fpreparesend = NULL;
  soap->fpreparerecv = NULL;
  soap->fpreparefinalsend = NULL;
  soap->fpreparefinalrecv = NULL;
  soap->ffiltersend = NULL;
  soap->ffilterrecv = NULL;
  soap->fdimereadopen = NULL;
  soap->fdimewriteopen = NULL;
  soap->fdimereadclose = NULL;
  soap->fdimewriteclose = NULL;
  soap->fdimeread = NULL;
  soap->fdimewrite = NULL;
  soap->fmimereadopen = NULL;
  soap->fmimewriteopen = NULL;
  soap->fmimereadclose = NULL;
  soap->fmimewriteclose = NULL;
  soap->fmimeread = NULL;
  soap->fmimewrite = NULL;
#endif
  soap->float_format = "%.9G"; /* Alternative: use "%G" */
  soap->double_format = "%.17lG"; /* Alternative: use "%lG" */
  soap->long_double_format = NULL; /* Defined in custom serializer custom/long_double.c */
  soap->dime_id_format = "cid:id%d"; /* default DIME id format for int id index */
  soap->recv_maxlength = 0x7FFFFFFF; /* default max length of messages received (2GB) */
  soap->recv_timeout = 0;
  soap->send_timeout = 0;
  soap->transfer_timeout = 0;
  soap->connect_timeout = 0;
  soap->accept_timeout = 0;
  soap->socket_flags = 0;
  soap->connect_flags = 0;
  soap->connect_retry = 0;
  soap->bind_flags = 0;
#ifdef WITH_IPV6_V6ONLY
  soap->bind_inet6 = 1;
  soap->bind_v6only = 1;
#else
  soap->bind_inet6 = 0;
  soap->bind_v6only = 0;
#endif
  soap->accept_flags = 0;
#ifdef WIN32
  soap->sndbuf = SOAP_BUFLEN + 1; /* this size speeds up windows xfer */
  soap->rcvbuf = SOAP_BUFLEN + 1;
#else
  soap->sndbuf = SOAP_BUFLEN;
  soap->rcvbuf = SOAP_BUFLEN;
#endif
  soap->linger_time = 0;
  soap->maxlevel = SOAP_MAXLEVEL;
  soap->maxlength = SOAP_MAXLENGTH;
  soap->maxoccurs = SOAP_MAXOCCURS;
  soap->http_version = "1.1";
  soap->proxy_http_version = "1.0";
  soap->http_content = NULL;
  soap->http_extra_header = NULL;
  soap->actor = NULL;
  soap->lang = "en";
  soap->keep_alive = 0;
  soap->tcp_keep_alive = 0;
  soap->tcp_keep_idle = 0;
  soap->tcp_keep_intvl = 0;
  soap->tcp_keep_cnt = 0;
  soap->max_keep_alive = SOAP_MAXKEEPALIVE;
  soap->ip = 0;
  soap->ip6[0] = 0;
  soap->ip6[1] = 0;
  soap->ip6[2] = 0;
  soap->ip6[3] = 0;
  soap->labbuf = NULL;
  soap->lablen = 0;
  soap->labidx = 0;
  soap->encodingStyle = NULL;
#ifndef WITH_NONAMESPACES
  soap->namespaces = namespaces;
#else
  soap->namespaces = NULL;
#endif
  soap->local_namespaces = NULL;
  soap->nlist = NULL;
  soap->blist = NULL;
  soap->clist = NULL;
  soap->alist = NULL;
  soap->shaky = 0;
  soap->attributes = NULL;
  soap->header = NULL;
  soap->fault = NULL;
  soap->master = SOAP_INVALID_SOCKET;
  soap->socket = SOAP_INVALID_SOCKET;
  soap->sendsk = SOAP_INVALID_SOCKET;
  soap->recvsk = SOAP_INVALID_SOCKET;
  soap->os = NULL;
  soap->is = NULL;
#ifndef WITH_LEANER
  soap->dom = NULL;
  soap->dime.list = NULL;
  soap->dime.first = NULL;
  soap->dime.last = NULL;
  soap->mime.list = NULL;
  soap->mime.first = NULL;
  soap->mime.last = NULL;
  soap->mime.boundary = NULL;
  soap->mime.start = NULL;
  soap->xlist = NULL;
#endif
#ifndef UNDER_CE
  soap->recvfd = 0;
  soap->sendfd = 1;
#else
  soap->recvfd = stdin;
  soap->sendfd = stdout;
#endif
  soap->tag[0] = '\0';
  soap->id[0] = '\0';
  soap->href[0] = '\0';
  soap->type[0] = '\0';
  soap->arrayType[0] = '\0';
  soap->arraySize[0] = '\0';
  soap->arrayOffset[0] = '\0';
  soap->endpoint[0] = '\0';
  soap->host[0] = '\0';
  soap->path[0] = '\0';
  soap->port = 0;
  soap->override_host = NULL;
  soap->override_port = 0;
  soap->action = NULL;
  soap->proxy_host = NULL;
  soap->proxy_port = 8080;
  soap->proxy_userid = NULL;
  soap->proxy_passwd = NULL;
  soap->proxy_from = NULL;
  soap->origin = NULL;
  soap->cors_origin = NULL;
  soap->cors_allow = "*";
  soap->cors_method = NULL;
  soap->cors_header = NULL;
  soap->cors_methods = NULL;
  soap->cors_headers = NULL;
  soap->x_frame_options = "SAMEORIGIN";
  soap->prolog = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  soap->zlib_state = SOAP_ZLIB_NONE;
  soap->zlib_in = SOAP_ZLIB_NONE;
  soap->zlib_out = SOAP_ZLIB_NONE;
  soap->d_stream = NULL;
  soap->z_buf = NULL;
  soap->z_level = 6;
  soap->z_dict = NULL;
  soap->z_dict_len = 0;
#ifndef WITH_LEAN
  soap->wsuid = NULL;
  soap->c14nexclude = NULL;
  soap->c14ninclude = NULL;
  soap->cookies = NULL;
  soap->cookie_domain = NULL;
  soap->cookie_path = NULL;
  soap->cookie_max = 32;
#endif
#ifdef WMW_RPM_IO
  soap->rpmreqid = NULL;
#endif
#ifndef WITH_NOIDREF
  soap_init_iht(soap);
#endif
  soap_init_pht(soap);
#ifdef WITH_OPENSSL
  if (!soap_ssl_init_done)
    soap_ssl_init();
  soap->fsslauth = ssl_auth_init;
  soap->fsslverify = NULL;
  soap->bio = NULL;
  soap->ssl = NULL;
  soap->ctx = NULL;
  soap->session = NULL;
  soap->session_host[0] = '\0';
  soap->session_port = 443;
  soap->ssl_flags = SOAP_SSL_DEFAULT;
  soap->keyfile = NULL;
  soap->keyid = NULL;
  soap->password = NULL;
  soap->cafile = NULL;
  soap->capath = NULL;
  soap->crlfile = NULL;
  soap->dhfile = NULL;
  soap->randfile = NULL;
#endif
#ifdef WITH_GNUTLS
  if (!soap_ssl_init_done)
    soap_ssl_init();
  soap->fsslauth = ssl_auth_init;
  soap->fsslverify = NULL;
  soap->xcred = NULL;
  soap->acred = NULL;
  soap->cache = NULL;
  soap->session = NULL;
  soap->ssl_flags = SOAP_SSL_DEFAULT;
  soap->keyfile = NULL;
  soap->keyid = NULL;
  soap->password = NULL;
  soap->cafile = NULL;
  soap->capath = NULL;
  soap->crlfile = NULL;
  soap->dh_params = NULL;
  soap->rsa_params = NULL;
#endif
#ifdef WITH_SYSTEMSSL
  soap->fsslauth = ssl_auth_init;
  soap->fsslverify = NULL;
  soap->bio = NULL;
  soap->ssl = (gsk_handle)NULL;
  soap->ctx = (gsk_handle)NULL;
  soap->session = NULL;
  soap->ssl_flags = SOAP_SSL_DEFAULT;
  soap->keyfile = NULL;
  soap->keyid = NULL;
  soap->password = NULL;
  soap->cafile = NULL;
  soap->capath = NULL;
  soap->crlfile = NULL;
  soap->dhfile = NULL;
  soap->randfile = NULL;
#endif
  soap->c_locale = NULL;
  soap->buflen = 0;
  soap->bufidx = 0;
#ifndef WITH_LEANER
  soap->dime.chunksize = 0;
  soap->dime.buflen = 0;
#endif
  soap->other = 0;
  soap->root = -1;
  soap->null = 0;
  soap->position = 0;
  soap->encoding = 0;
  soap->mustUnderstand = 0;
  soap->ns = 0;
  soap->part = SOAP_END;
  soap->event = 0;
  soap->evlev = 0;
  soap->alloced = 0;
  soap->count = 0;
  soap->length = 0;
  soap->cdata = 0;
  soap->peeked = 0;
  soap->ahead = 0;
  soap->idnum = 0;
  soap->level = 0;
  soap->status = 0;
  soap->error = SOAP_OK;
  soap->errmode = 0;
  soap->errnum = 0;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_begin(struct soap *soap)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Clean up for input/output\n"));
  soap->error = SOAP_OK;
  if (!soap->keep_alive)
  {
    soap->buflen = 0;
    soap->bufidx = 0;
  }
  soap->encoding = 0;
  soap->mode = 0;
  soap->part = SOAP_END;
  soap->peeked = 0;
  soap->ahead = 0;
  soap->level = 0;
  *soap->endpoint = '\0';
  soap->encodingStyle = SOAP_STR_EOS;
  soap_free_temp(soap);
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_end(struct soap *soap)
{
  if (soap_check_state(soap))
    return;
  soap_free_temp(soap);
  soap_dealloc(soap, NULL);
  while (soap->clist)
  {
    struct soap_clist *cp = soap->clist->next;
    SOAP_FREE(soap, soap->clist);
    soap->clist = cp;
  }
  (void)soap_closesock(soap);
#ifdef SOAP_DEBUG
  soap_close_logfiles(soap);
#endif
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_set_version(struct soap *soap, short version)
{
  soap_set_local_namespaces(soap);
  if (soap->version != version && soap->local_namespaces && soap->local_namespaces[0].id && soap->local_namespaces[1].id)
  {
    if (version == 1)
    {
      soap->local_namespaces[0].ns = soap_env1;
      soap->local_namespaces[1].ns = soap_enc1;
    }
    else if (version == 2)
    {
      soap->local_namespaces[0].ns = soap_env2;
      soap->local_namespaces[1].ns = soap_enc2;
    }
    soap->version = version;
  }
  if (version == 0)
    soap->encodingStyle = SOAP_STR_EOS;
  else
    soap->encodingStyle = NULL;
}

/******************************************************************************/

static void
soap_version(struct soap *soap)
{
  struct Namespace *p = soap->local_namespaces;
  if (p)
  {
    const char *ns = p[0].out;
    if (!ns)
      ns = p[0].ns;
    if (ns)
    {
      if (!strcmp(ns, soap_env1))
      {
        soap->version = 1; /* make sure we use SOAP 1.1 */
        if (p[1].out)
          SOAP_FREE(soap, p[1].out);
        p[1].out = (char*)SOAP_MALLOC(soap, sizeof(soap_enc1));
        if (p[1].out)
          (void)soap_memcpy(p[1].out, sizeof(soap_enc1), soap_enc1, sizeof(soap_enc1));
      }
      else if (!strcmp(ns, soap_env2))
      {
        soap->version = 2; /* make sure we use SOAP 1.2 */
        if (p[1].out)
          SOAP_FREE(soap, p[1].out);
        p[1].out = (char*)SOAP_MALLOC(soap, sizeof(soap_enc2));
        if (p[1].out)
          (void)soap_memcpy(p[1].out, sizeof(soap_enc2), soap_enc2, sizeof(soap_enc2));
      }
    }
  }
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_set_namespaces(struct soap *soap, const struct Namespace *p)
{
  struct Namespace *ns = soap->local_namespaces;
  struct soap_nlist *np, *nq, *nr;
  unsigned int level = soap->level;
  soap->namespaces = p;
  soap->local_namespaces = NULL;
  soap_set_local_namespaces(soap);
  /* reverse the namespace list */
  np = soap->nlist;
  soap->nlist = NULL;
  if (np)
  {
    nq = np->next;
    np->next = NULL;
    while (nq)
    {
      nr = nq->next;
      nq->next = np;
      np = nq;
      nq = nr;
    }
  }
  /* then push on new stack */
  while (np)
  {
    const char *s;
    soap->level = np->level; /* preserve element nesting level */
    s = np->ns;
    if (!s && np->index >= 0 && ns)
    {
      s = ns[np->index].out;
      if (!s)
        s = ns[np->index].ns;
    }
    if (s)
      (void)soap_push_namespace(soap, np->id, s);
    nq = np;
    np = np->next;
    SOAP_FREE(soap, nq);
  }
  if (ns)
  {
    int i;
    for (i = 0; ns[i].id; i++)
    {
      if (ns[i].out)
      {
        SOAP_FREE(soap, ns[i].out);
        ns[i].out = NULL;
      }
    }
    SOAP_FREE(soap, ns);
  }
  soap->level = level; /* restore level */
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_set_local_namespaces(struct soap *soap)
{
  if (soap->namespaces && !soap->local_namespaces)
  {
    const struct Namespace *ns1;
    struct Namespace *ns2;
    size_t n = 1;
    for (ns1 = soap->namespaces; ns1->id; ns1++)
      n++;
    n *= sizeof(struct Namespace);
    ns2 = (struct Namespace*)SOAP_MALLOC(soap, n);
    if (ns2)
    {
      (void)soap_memcpy((void*)ns2, n, (const void*)soap->namespaces, n);
      if (ns2[0].ns)
      {
        if (!strcmp(ns2[0].ns, soap_env1))
          soap->version = 1;
        else if (!strcmp(ns2[0].ns, soap_env2))
          soap->version = 2;
      }
      soap->local_namespaces = ns2;
      for (; ns2->id; ns2++)
        ns2->out = NULL;
    }
  }
}

/******************************************************************************/

#ifndef WITH_LEAN
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_tagsearch(const char *big, const char *little)
{
  if (big && little)
  {
    size_t n = strlen(little);
    const char *s = big;
    while (s)
    {
      const char *t = s;
      size_t i;
      for (i = 0; i < n; i++, t++)
      {
        if (*t != little[i])
          break;
      }
      if (*t == '\0' || *t == ' ')
      {
        if (i == n || (i > 0 && little[i-1] == ':'))
          return s;
      }
      s = strchr(t, ' ');
      if (s)
        s++;
    }
  }
  return NULL;
}
#endif

/******************************************************************************/

SOAP_FMAC1
struct soap_nlist *
SOAP_FMAC2
soap_lookup_ns(struct soap *soap, const char *tag, size_t n)
{
  struct soap_nlist *np;
  for (np = soap->nlist; np; np = np->next)
    if (!strncmp(np->id, tag, n) && !np->id[n])
      return np;
  return NULL;
}

/******************************************************************************/

#ifndef WITH_LEAN
static struct soap_nlist *
soap_push_ns(struct soap *soap, const char *id, const char *ns, short utilized, short isearly)
{
  struct soap_nlist *np = NULL;
  size_t n, k;
  unsigned int level = soap->level + isearly;
  if (soap_tagsearch(soap->c14nexclude, id))
    return NULL;
  if (!utilized)
  {
    for (np = soap->nlist; np; np = np->next)
    {
      if (!strcmp(np->id, id) && ((!np->ns && *id) || (ns && np->ns && !strcmp(np->ns, ns))))
        break;
    }
    if (np)
    {
      if ((np->level < level || (!np->ns && *id)) && np->index == 1)
        utilized = 1;
      else
        return NULL;
    }
  }
  else if (!*id)
  {
    for (np = soap->nlist; np; np = np->next)
    {
      if (!*np->id && np->level == level && np->index != 1)
        return NULL;
    }
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Adding namespace binding (level=%u) '%s' '%s' utilized=%d\n", level, id, ns ? ns : "(null)", utilized));
  n = strlen(id);
  if (ns)
    k = strlen(ns) + 1;
  else
    k = 0;
  if (sizeof(struct soap_nlist) + n + k > n && (SOAP_MAXALLOCSIZE <= 0 || sizeof(struct soap_nlist) + n + k <= SOAP_MAXALLOCSIZE))
    np = (struct soap_nlist*)SOAP_MALLOC(soap, sizeof(struct soap_nlist) + n + k);
  if (!np)
  {
    soap->error = SOAP_EOM;
    return NULL;
  }
  np->next = soap->nlist;
  soap->nlist = np;
  soap_strcpy((char*)np->id, n + 1, id);
  if (ns)
  {
    np->ns = np->id + n + 1;
    soap_strcpy((char*)np->ns, k, ns);
  }
  else
  {
    np->ns = NULL;
  }
  np->level = level;
  np->index = utilized;
  return np;
}
#endif

/******************************************************************************/

#ifndef WITH_LEAN
static void
soap_utilize_ns(struct soap *soap, const char *tag, short isearly)
{
  struct soap_nlist *np;
  size_t n = 0;
  if (!strncmp(tag, "xmlns:", 6))
  {
    tag += 6;
    n = strlen(tag);
  }
  else
  {
    const char *t = strchr(tag, ':');
    if (t)
      n = t - tag;
  }
  np = soap_lookup_ns(soap, tag, n);
  if (np)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Utilizing namespace '%s' of '%s' at level %u utilized=%d at level=%u\n", np->ns ? np->ns : "", tag, soap->level + isearly, np->index, np->level));
    if (np->index <= 0)
    {
      if (np->level == soap->level + isearly)
        np->index = 1;
      else
        (void)soap_push_ns(soap, np->id, np->ns, 1, isearly);
    }
  }
  else if (strncmp(tag, "xml", 3))
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Utilizing default namespace of '%s' at level %u\n", tag, soap->level + isearly));
    (void)soap_strncpy(soap->tag, sizeof(soap->tag), tag, n);
    (void)soap_push_ns(soap, soap->tag, NULL, 1, isearly);
  }
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element(struct soap *soap, const char *tag, int id, const char *type)
{
#ifndef WITH_LEAN
  const char *s;
#endif
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Element begin tag='%s' level='%u' id='%d' type='%s'\n", tag, soap->level, id, type ? type : SOAP_STR_EOS));
#ifdef WITH_DOM
#ifndef WITH_LEAN
  if (soap_tagsearch(soap->wsuid, tag))
  {
    size_t i;
    for (s = tag, i = 0; *s && i < sizeof(soap->href) - 1; s++, i++)
      soap->href[i] = *s == ':' ? '-' : *s;
    soap->href[i] = '\0';
    if (soap_set_attr(soap, "wsu:Id", soap->href, 1))
      return soap->error;
  }
#endif
#endif
  soap->level++;
  if (soap->level > soap->maxlevel)
    return soap->error = SOAP_LEVEL;
#ifdef WITH_DOM
#ifndef WITH_LEAN
  if ((soap->mode & SOAP_XML_CANONICAL) && !(soap->mode & SOAP_DOM_ASIS))
  {
    if (soap->evlev >= soap->level)
      soap->evlev = 0;
    if (soap->event == SOAP_SEC_BEGIN && !soap->evlev)
    {
      struct soap_nlist *np;
      /* non-nested wsu:Id found: clear xmlns, re-emit them for exc-c14n */
      for (np = soap->nlist; np; np = np->next)
      {
        int p = soap->c14ninclude ? *soap->c14ninclude == '+' || soap_tagsearch(soap->c14ninclude, np->id) != NULL : 0;
        if (np->index == 2 || p)
        {
          struct soap_nlist *np1 = soap_push_ns(soap, np->id, np->ns, 1, 0);
          if (np1 && !p)
            np1->index = 0;
        }
      }
      soap->evlev = soap->level;
    }
  }
#endif
  if ((soap->mode & SOAP_XML_DOM))
  {
    struct soap_dom_element *elt = (struct soap_dom_element*)soap_malloc(soap, sizeof(struct soap_dom_element));
    if (!elt)
      return soap->error = SOAP_EOM;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Adding DOM element tag='%s' %p (parent='%s' %p)\n", tag, elt, soap->dom ? soap->dom->name : "(null)", soap->dom));
    elt->soap = soap;
    elt->next = NULL;
    elt->prnt = soap->dom;
    elt->elts = NULL;
    elt->atts = NULL;
    elt->nstr = NULL;
    elt->name = soap_strdup(soap, tag);
    elt->lead = NULL;
    elt->text = NULL;
    elt->code = NULL;
    elt->tail = NULL;
    elt->node = NULL;
    elt->type = 0;
    if (soap->dom)
    {
      struct soap_dom_element *p = soap->dom->elts;
      if (p)
      {
        while (p->next)
          p = p->next;
        p->next = elt;
      }
      else
      {
        soap->dom->elts = elt;
      }
    }
    soap->dom = elt;
    if (!elt->name)
      return soap->error = SOAP_EOM;
  }
  else
  {
#endif
#ifndef WITH_LEAN
    if (!soap->ns)
    {
      if (!(soap->mode & SOAP_XML_CANONICAL) && soap_send(soap, soap->prolog))
        return soap->error;
    }
    else if ((soap->mode & SOAP_XML_INDENT))
    {
      if (soap->ns == 1 && soap_send_raw(soap, soap_indent, soap->level < sizeof(soap_indent) ? soap->level : sizeof(soap_indent) - 1))
        return soap->error;
      soap->body = 1;
    }
    if ((soap->mode & SOAP_XML_DEFAULTNS))
    {
      size_t n = 0;
      s = strchr(tag, ':');
      if (s)
        n = s++ - tag;
      else
        s = tag;
      if (soap_send_raw(soap, "<", 1)
       || soap_send(soap, s))
        return soap->error;
      if (n)
      {
        struct Namespace *ns = soap->local_namespaces;
        for (; ns && ns->id; ns++)
        {
          if (*ns->id && ns->ns && !strncmp(ns->id, tag, n) && !ns->id[n])
          {
            if (!soap->nlist || *soap->nlist->id || (soap->nlist->ns && strcmp(soap->nlist->ns, ns->ns)))
            {
              (void)soap_push_ns(soap, SOAP_STR_EOS, ns->out ? ns->out : ns->ns, 0, 0);
              if (soap_attribute(soap, "xmlns", ns->out ? ns->out : ns->ns))
                return soap->error;
            }
            break;
          }
        }
      }
#ifndef WITH_NOEMPTYNAMESPACES
      else if (!soap->nlist || *soap->nlist->id || (soap->nlist->ns && *soap->nlist->ns))
      {
        (void)soap_push_ns(soap, SOAP_STR_EOS, SOAP_STR_EOS, 0, 0);
        if (soap_attribute(soap, "xmlns", SOAP_STR_EOS))
          return soap->error;
      }
#endif
    }
    else
#endif
    if (soap_send_raw(soap, "<", 1)
     || soap_send(soap, tag))
      return soap->error;
#ifdef WITH_DOM
  }
#endif
  if (!soap->ns)
  {
    struct Namespace *ns = soap->local_namespaces;
    for (; ns && ns->id; ns++)
    {
      const char *t = ns->out;
      if (!t)
        t = ns->ns;
      if (*ns->id && t && *t)
      {
        (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(ns->id) + 6), "xmlns:%s", ns->id);
        if (soap_attribute(soap, soap->tmpbuf, t))
          return soap->error;
      }
    }
  }
  soap->ns = 1; /* namespace table control: ns = 0 or 2 to start, then 1 to stop dumping the table  */
#ifndef WITH_LEAN
  if ((soap->mode & SOAP_XML_CANONICAL))
  {
    if ((soap->mode & SOAP_XML_DEFAULTNS))
      soap_utilize_ns(soap, SOAP_STR_EOS, 0);
    else
      soap_utilize_ns(soap, tag, 0);
  }
#endif
  if (id > 0)
  {
    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), sizeof(SOAP_BASEREFNAME) + 20), SOAP_BASEREFNAME "%d", id);
    if (soap->version == 2)
    {
      if (soap_attribute(soap, "SOAP-ENC:id", soap->tmpbuf))
        return soap->error;
    }
    else if (soap_attribute(soap, "id", soap->tmpbuf))
    {
      return soap->error;
    }
  }
  if (type && *type && !(soap->mode & SOAP_XML_NOTYPE))
  {
#ifndef WITH_LEAN
    if ((soap->mode & SOAP_XML_CANONICAL) && !(soap->mode & SOAP_XML_CANONICAL_NA))
      soap_utilize_ns(soap, type, 0);
#endif
    if (soap_attribute(soap, "xsi:type", type))
      return soap->error;
  }
  if (soap->null && soap->position > 0 && soap->version == 1)
  {
    int i;
    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf) - 1, 20), "[%d", soap->positions[0]);
    for (i = 1; i < soap->position; i++)
    {
      size_t l = strlen(soap->tmpbuf);
      (SOAP_SNPRINTF(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l - 1, 20), ",%d", soap->positions[i]);
    }
    soap_strcat(soap->tmpbuf, sizeof(soap->tmpbuf), "]");
    if (soap_attribute(soap, "SOAP-ENC:position", soap->tmpbuf))
      return soap->error;
  }
  if (soap->mustUnderstand)
  {
    if (soap->actor && *soap->actor)
    {
      if (soap_attribute(soap, soap->version == 2 ? "SOAP-ENV:role" : "SOAP-ENV:actor", soap->actor))
        return soap->error;
    }
    if (soap_attribute(soap, "SOAP-ENV:mustUnderstand", soap->version == 2 ? "true" : "1"))
      return soap->error;
    soap->mustUnderstand = 0;
  }
  if (soap->encoding)
  {
    if (soap->encodingStyle && soap->local_namespaces && soap->local_namespaces[0].id && soap->local_namespaces[1].id)
    {
      if (!*soap->encodingStyle)
      {
        if (soap->local_namespaces[1].out)
          soap->encodingStyle = soap->local_namespaces[1].out;
        else
          soap->encodingStyle = soap->local_namespaces[1].ns;
      }
      if (soap->encodingStyle && soap_attribute(soap, "SOAP-ENV:encodingStyle", soap->encodingStyle))
        return soap->error;
    }
    else
    {
      soap->encodingStyle = NULL;
    }
    soap->encoding = 0;
  }
  soap->null = 0;
  soap->position = 0;
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_begin_out(struct soap *soap, const char *tag, int id, const char *type)
{
  if (*tag == '-')
    return SOAP_OK;
#ifdef WITH_DOM
  if (soap->feltbegout)
    return soap->error = soap->feltbegout(soap, tag, id, type);
#endif
  if (soap_element(soap, tag, id, type))
    return soap->error;
  return soap_element_start_end_out(soap, NULL);
}

/******************************************************************************/

#if _MSC_VER < 1400 && !defined(HAVE_STRLCAT)
/* concat string (truncating the result, strings must not be NULL) */
SOAP_FMAC1
void
SOAP_FMAC2
soap_strcat(char *t, size_t n, const char *s)
{
  size_t k = strlen(t);
  if (k < n)
  {
    t += k;
    n -= k;
    while (n-- > 1 && *s)
      *t++ = *s++;
    *t = '\0';
  }
}
#endif

/******************************************************************************/

#if _MSC_VER < 1400
/* concat string up to m chars (leaves destination intact on overrun and returns nonzero, zero if OK) */
SOAP_FMAC1
int
SOAP_FMAC2
soap_strncat(char *t, size_t n, const char *s, size_t m)
{
  size_t k;
  if (!t || !s)
    return 1;
  k = strlen(t);
  if (n <= k + m)
    return 1;
  t += k;
  n -= k;
  while (n-- > 1 && *s)
    *t++ = *s++;
  *t = '\0';
  return 0;
}
#endif

/******************************************************************************/

#ifndef HAVE_STRRCHR
SOAP_FMAC1
char*
SOAP_FMAC2
soap_strrchr(const char *s, int t)
{
  char *r = NULL;
  while (*s)
    if (*s++ == t)
      r = (char*)s - 1;
  return r;
}
#endif

/******************************************************************************/

#ifndef HAVE_STRTOL
SOAP_FMAC1
long
SOAP_FMAC2
soap_strtol(const char *s, char **t, int b)
{
  long n = 0;
  int c;
  while (*s > 0 && *s <= 32)
    s++;
  if (b == 10)
  {
    short neg = 0;
    if (*s == '-')
    {
      s++;
      neg = 1;
    }
    else if (*s == '+')
    {
      s++;
    }
    while ((c = *s) && c >= '0' && c <= '9')
    {
      if (n >= 214748364 && (n > 214748364 || c >= '8'))
      {
        if (neg && n == 214748364 && c == '8')
        {
          if (t)
            *t = (char*)(s + 1);
          return -2147483648;
        }
        break;
      }
      n *= 10;
      n += c - '0';
      s++;
    }
    if (neg)
      n = -n;
  }
  else /* assume b == 16 and value is always positive */
  {
    while ((c = *s))
    {
      if (c >= '0' && c <= '9')
        c -= '0';
      else if (c >= 'A' && c <= 'F')
        c -= 'A' - 10;
      else if (c >= 'a' && c <= 'f')
        c -= 'a' - 10;
      if (n > 0x07FFFFFF)
        break;
      n <<= 4;
      n += c;
      s++;
    }
  }
  if (t)
    *t = (char*)s;
  return n;
}
#endif

/******************************************************************************/

#ifndef HAVE_STRTOUL
SOAP_FMAC1
unsigned long
SOAP_FMAC2
soap_strtoul(const char *s, char **t, int b)
{
  unsigned long n = 0;
  int c;
  while (*s > 0 && *s <= 32)
    s++;
  if (b == 10)
  {
    short neg = 0;
    if (*s == '-')
    {
      s++;
      neg = 1;
    }
    else if (*s == '+')
    {
      s++;
    }
    while ((c = *s) && c >= '0' && c <= '9')
    {
      if (n >= 429496729 && (n > 429496729 || c >= '6'))
        break;
      n *= 10;
      n += c - '0';
      s++;
    }
    if (neg && n > 0)
      s--;
  }
  else /* b == 16 */
  {
    while ((c = *s))
    {
      if (c >= '0' && c <= '9')
        c -= '0';
      else if (c >= 'A' && c <= 'F')
        c -= 'A' - 10;
      else if (c >= 'a' && c <= 'f')
        c -= 'a' - 10;
      if (n > 0x0FFFFFFF)
        break;
      n <<= 4;
      n += c;
      s++;
    }
  }
  if (t)
    *t = (char*)s;
  return n;
}
#endif

/******************************************************************************/

#ifndef soap_strtoll
SOAP_FMAC1
LONG64
SOAP_FMAC2
soap_strtoll(const char *s, char **t, int b)
{
  LONG64 n = 0LL;
  int c;
  while (*s > 0 && *s <= 32)
    s++;
  if (b == 10)
  {
    short neg = 0;
    if (*s == '-')
    {
      s++;
      neg = 1;
    }
    else if (*s == '+')
    {
      s++;
    }
    while ((c = *s) && c >= '0' && c <= '9')
    {
      if (n >= 922337203685477580LL && (n > 922337203685477580LL || c >= '8'))
      {
        if (neg && n == 922337203685477580LL && c == '8')
        {
          if (t)
            *t = (char*)(s + 1);
          return -9223372036854775807LL - 1LL; /* appease compilers that complain */
        }
        break;
      }
      n *= 10LL;
      n += c - '0';
      s++;
    }
    if (neg)
      n = -n;
  }
  else /* assume b == 16 and value is always positive */
  {
    while ((c = *s))
    {
      if (c >= '0' && c <= '9')
        c -= '0';
      else if (c >= 'A' && c <= 'F')
        c -= 'A' - 10;
      else if (c >= 'a' && c <= 'f')
        c -= 'a' - 10;
      if (n > 0x07FFFFFFFFFFFFFFLL)
        break;
      n <<= 4;
      n += c;
      s++;
    }
  }
  if (t)
    *t = (char*)s;
  return n;
}
#endif

/******************************************************************************/

#ifndef soap_strtoull
SOAP_FMAC1
ULONG64
SOAP_FMAC2
soap_strtoull(const char *s, char **t, int b)
{
  ULONG64 n = 0UL;
  int c;
  while (*s > 0 && *s <= 32)
    s++;
  if (b == 10)
  {
    short neg = 0;
    if (*s == '-')
    {
      s++;
      neg = 1;
    }
    else if (*s == '+')
    {
      s++;
    }
    while ((c = *s) && c >= '0' && c <= '9')
    {
      if (n >= 1844674407370955161ULL && (n > 1844674407370955161ULL || c >= '6'))
        break;
      n *= 10UL;
      n += c - '0';
      s++;
    }
    if (neg && n > 0UL)
      s--;
  }
  else /* b == 16 */
  {
    while ((c = *s))
    {
      if (c >= '0' && c <= '9')
        c -= '0';
      else if (c >= 'A' && c <= 'F')
        c -= 'A' - 10;
      else if (c >= 'a' && c <= 'f')
        c -= 'a' - 10;
      if (n > 0x0FFFFFFFFFFFFFFFULL)
        break;
      n <<= 4;
      n += c;
      s++;
    }
  }
  if (t)
    *t = (char*)s;
  return n;
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_array_begin_out(struct soap *soap, const char *tag, int id, const char *type, const char *offset)
{
  if (!type || !*type || soap->version == 0)
    return soap_element_begin_out(soap, tag, id, NULL);
  if (soap_element(soap, tag, id, NULL))
    return soap->error;
  if (soap->version == 1)
  {
    if (offset && soap_attribute(soap, "SOAP-ENC:offset", offset))
      return soap->error;
    if (soap_attribute(soap, "SOAP-ENC:arrayType", type))
      return soap->error;
  }
  else
  {
    const char *s;
    s = strchr(type, '[');
    if (s && (size_t)(s - type) < sizeof(soap->tmpbuf))
    {
      (void)soap_strncpy(soap->tmpbuf, sizeof(soap->tmpbuf), type, s - type);
      if (soap_attribute(soap, "SOAP-ENC:itemType", soap->tmpbuf))
        return soap->error;
      s++;
      if (*s && *s != ']')
      {
        soap_strcpy(soap->tmpbuf, sizeof(soap->tmpbuf), s);
        soap->tmpbuf[strlen(soap->tmpbuf) - 1] = '\0';
        if (soap_attribute(soap, "SOAP-ENC:arraySize", soap->tmpbuf))
          return soap->error;
      }
    }
  }
#ifndef WITH_LEAN
  if ((soap->mode & SOAP_XML_CANONICAL) && !(soap->mode & SOAP_XML_CANONICAL_NA))
    soap_utilize_ns(soap, type, 0);
#endif
  return soap_element_start_end_out(soap, NULL);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_start_end_out(struct soap *soap, const char *tag)
{
  struct soap_attribute *tp;
#ifndef WITH_LEAN
  if ((soap->mode & SOAP_XML_CANONICAL))
  {
    struct soap_nlist *np;
    for (tp = soap->attributes; tp; tp = tp->next)
    {
      if (tp->visible && *tp->name && strchr(tp->name, ':'))
        soap_utilize_ns(soap, tp->name, 0);
    }
    if (soap->event == SOAP_SEC_BEGIN)
    {
      for (np = soap->nlist; np; np = np->next)
        if (soap_tagsearch(soap->c14ninclude, np->id))
          (void)soap_push_ns(soap, np->id, np->ns, 1, 0);
      soap->event = 0;
      soap->evlev = 0;
    }
    for (np = soap->nlist; np; np = np->next)
    {
      if (np->ns && np->index == 1)
      {
        if (*np->id)
          (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(np->id) + 6), "xmlns:%s", np->id);
        else
          soap_strcpy(soap->tmpbuf, sizeof(soap->tmpbuf), "xmlns");
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Enabling utilized binding (level=%u) %s='%s' SEC-BEGIN=%d c14ninclude='%s'\n", np->level, soap->tmpbuf, np->ns, soap->event == SOAP_SEC_BEGIN, soap->c14ninclude ? soap->c14ninclude : "(null)"));
        np->index = 2;
        soap->level--;
        if (*np->id || *np->ns || soap->level > 1)
          if (soap_set_attr(soap, soap->tmpbuf, np->ns, 1))
            return soap->error;
        soap->level++;
      }
      else
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Binding (level=%u) %s='%s' utilized=%d\n", np->level, np->id, np->ns, np->index));
      }
    }
  }
#endif
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    struct soap_dom_attribute **att;
    att = &soap->dom->atts;
    for (tp = soap->attributes; tp; tp = tp->next)
    {
      if (tp->visible)
      {
        *att = (struct soap_dom_attribute*)soap_malloc(soap, sizeof(struct soap_dom_attribute));
        if (!*att)
          return soap->error;
        (*att)->next = NULL;
        (*att)->nstr = NULL;
        (*att)->name = soap_strdup(soap, tp->name);
        (*att)->text = soap_strdup(soap, tp->value);
        (*att)->soap = soap;
        if (!(*att)->name || (tp->value && !(*att)->text))
          return soap->error = SOAP_EOM;
        att = &(*att)->next;
        tp->visible = 0;
      }
    }
    return SOAP_OK;
  }
#endif
  for (tp = soap->attributes; tp; tp = tp->next)
  {
    if (tp->visible)
    {
      if (soap_send_raw(soap, " ", 1) || soap_send(soap, tp->name))
        return soap->error;
      if (tp->visible == 2 && tp->value)
      {
        if (soap_send_raw(soap, "=\"", 2)
         || soap_string_out(soap, tp->value, tp->flag)
         || soap_send_raw(soap, "\"", 1))
          return soap->error;
      }
      else
      {
        if (soap_send_raw(soap, "=\"\"", 3))
          return soap->error;
      }
      tp->visible = 0;
    }
  }
  if (tag)
  {
#ifndef WITH_LEAN
    if ((soap->mode & SOAP_XML_CANONICAL))
    {
      if (soap_send_raw(soap, ">", 1)
       || soap_element_end_out(soap, tag))
        return soap->error;
      return SOAP_OK;
    }
#endif
    if (soap->nlist)
      soap_pop_namespace(soap);
    soap->level--;      /* decrement level just before /> */
    soap->body = 0;
    return soap_send_raw(soap, "/>", 2);
  }
  return soap_send_raw(soap, ">", 1);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_end_out(struct soap *soap, const char *tag)
{
  if (*tag == '-')
    return SOAP_OK;
#ifdef WITH_DOM
  if (soap->feltendout)
    return soap->error = soap->feltendout(soap, tag);
#endif
  return soap_element_end(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_end(struct soap *soap, const char *tag)
{
#ifndef WITH_LEAN
  const char *s;
#endif
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Element ending tag='%s'\n", tag));
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    if (soap->dom->prnt)
      soap->dom = soap->dom->prnt;
    return SOAP_OK;
  }
#endif
#ifndef WITH_LEAN
  if (soap->nlist)
    soap_pop_namespace(soap);
  if ((soap->mode & SOAP_XML_INDENT))
  {
    if (!soap->body)
    {
      if (soap_send_raw(soap, soap_indent, soap->level < sizeof(soap_indent) ? soap->level : sizeof(soap_indent) - 1))
        return soap->error;
    }
    soap->body = 0;
  }
  if ((soap->mode & SOAP_XML_DEFAULTNS) && (s = strchr(tag, ':')) != NULL)
    tag = s + 1;
#endif
  if (soap_send_raw(soap, "</", 2)
   || soap_send(soap, tag))
    return soap->error;
  soap->level--;        /* decrement level just before > */
  return soap_send_raw(soap, ">", 1);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_ref(struct soap *soap, const char *tag, int id, int href)
{
  const char *s = "ref";
  int n = 1;
  if (soap->version == 1)
  {
    s = "href";
    n = 0;
  }
  else if (soap->version == 2)
  {
    s = "SOAP-ENC:ref";
  }
  (SOAP_SNPRINTF(soap->href, sizeof(soap->href), sizeof(SOAP_BASEREFNAME) + 21), "#" SOAP_BASEREFNAME "%d", href);
  return soap_element_href(soap, tag, id, s, soap->href + n);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_href(struct soap *soap, const char *tag, int id, const char *ref, const char *val)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Element '%s' reference %s='%s'\n", tag, ref, val));
  if (soap_element(soap, tag, id, NULL)
   || soap_attribute(soap, ref, val)
   || soap_element_start_end_out(soap, tag))
    return soap->error;
  soap->body = 0;
  return SOAP_OK;
}


/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_null(struct soap *soap, const char *tag, int id, const char *type)
{
  struct soap_attribute *tp = NULL;
  for (tp = soap->attributes; tp; tp = tp->next)
    if (tp->visible)
      break;
  if (tp || (soap->version == 2 && soap->position > 0) || id > 0 || (soap->mode & SOAP_XML_NIL))
  {
    if (soap_element(soap, tag, id, type)
     || (!tp && soap_attribute(soap, "xsi:nil", "true"))
     || soap_element_start_end_out(soap, tag))
      return soap->error;
    soap->body = 0;
  }
  else
  {
    soap->null = 1;
    soap->position = 0;
    soap->mustUnderstand = 0;
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_empty(struct soap *soap, const char *tag, int id, const char *type)
{
  if (!tag || *tag == '-')
    return SOAP_OK;
#ifdef WITH_DOM
  if (soap->feltbegout)
    return soap->error = soap->feltbegout(soap, tag, id, type);
#endif
  if (soap_element(soap, tag, id, type))
    return soap->error;
  return soap_element_start_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_nil(struct soap *soap, const char *tag)
{
  if (soap_element(soap, tag, -1, NULL)
   || (soap_attribute(soap, "xsi:nil", "true")))
    return soap->error;
  return soap_element_start_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_id(struct soap *soap, const char *tag, int id, const void *p, const void *a, int n, const char *type, int t, char **mark)
{
  (void)a; (void)n;
  if (!p)
  {
    soap->error = soap_element_null(soap, tag, id, type);
    return -1;
  }
#ifndef WITH_NOIDREF
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Element_id %p type=%d id=%d\n", p, t, id));
  if ((!soap->encodingStyle && !(soap->mode & SOAP_XML_GRAPH)) || (soap->mode & SOAP_XML_TREE))
    return soap_check_and_mark(soap, p, t, mark);
  if (mark)
    *mark = NULL;
  if (id < -1)
    return soap_embed(soap, p, a, n, t);
  else if (id <= 0)
  {
    struct soap_plist *pp;
    if (a)
      id = soap_array_pointer_lookup(soap, p, a, n, t, &pp);
    else
      id = soap_pointer_lookup(soap, p, t, &pp);
    if (id)
    {
      if (soap_is_embedded(soap, pp))
      {
        soap_element_ref(soap, tag, 0, id);
        return -1;
      }
      if (soap_is_single(soap, pp))
        return 0;
      soap_set_embedded(soap, pp);
    }
  }
  return id;
#else
  return soap_check_and_mark(soap, p, t, mark);
#endif
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_check_and_mark(struct soap *soap, const void *p, int t, char **mark)
{
  if (mark)
  {
    struct soap_plist *pp;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Check %p and mark %p\n", p, (void*)mark));
    if (!soap_pointer_lookup(soap, p, t, &pp))
      if (!soap_pointer_enter(soap, p, NULL, 0, t, &pp))
        return -1;
    if ((soap->mode & SOAP_IO_LENGTH))
    {
      if (pp->mark1 > 0)
        return -1;
      pp->mark1 = 1;
      *mark = &pp->mark1;
    }
    else
    {
      if (pp->mark2 > 0)
        return -1;
      pp->mark2 = 1;
      *mark = &pp->mark2;
    }
  }
  return 0;
}

/******************************************************************************/

SOAP_FMAC1
void *
SOAP_FMAC2
soap_mark_lookup(struct soap *soap, const void *p, int t, struct soap_plist **ppp, char **mark)
{
  if (!soap)
    return NULL;
  if (mark || !(soap->mode & SOAP_XML_TREE))
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Mark lookup %p type=%d\n", p, t));
    if (!soap_pointer_lookup(soap, p, t, ppp))
    {
      if (!soap_pointer_enter(soap, p, NULL, 0, t, ppp))
        return NULL;
    }
    else if (!(soap->mode & SOAP_XML_TREE))
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Mark found %p\n", (*ppp)->dup));
      return (*ppp)->dup;
    }
    if (mark)
    {
      if ((*ppp)->mark1 > 0)
        (*ppp)->mark1 = 2; /* cycle */
      else
        (*ppp)->mark1 = 1; /* cycle detection */
      *mark = &(*ppp)->mark1;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Mark cycle %d\n", (*ppp)->mark1));
    }
  }
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_mark_cycle(struct soap *soap, struct soap_plist *pp)
{
  (void)soap;
  return pp && pp->mark1 == 2 && (soap->mode & SOAP_XML_TREE);
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_mark_dup(struct soap *soap, void *a, struct soap_plist *pp)
{
  (void)soap;
  if (pp)
    pp->dup = a;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_unmark(struct soap *soap, char *mark)
{
  (void)soap;
  if (mark)
    *mark = 0; /* release detection */
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_result(struct soap *soap, const char *tag)
{
  if (soap->version == 2 && soap->encodingStyle)
  {
    if (soap_element(soap, "SOAP-RPC:result", 0, NULL)
     || soap_attribute(soap, "xmlns:SOAP-RPC", soap_rpc)
     || soap_element_start_end_out(soap, NULL)
     || soap_string_out(soap, tag, 0)
     || soap_element_end_out(soap, "SOAP-RPC:result"))
      return soap->error;
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_check_result(struct soap *soap, const char *tag)
{
  (void)tag;
  if (soap->version == 2 && soap->encodingStyle)
  {
    soap_instring(soap, ":result", NULL, NULL, 0, 2, -1, -1, NULL);
    /* just ignore content for compliance reasons, but should compare tag to element's QName value? */
  }
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_attribute(struct soap *soap, const char *name, const char *value)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Attribute '%s'='%s'\n", name, value));
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && !(soap->mode & SOAP_XML_CANONICAL) && soap->dom)
  {
    struct soap_dom_attribute *a = (struct soap_dom_attribute*)soap_malloc(soap, sizeof(struct soap_dom_attribute));
    if (!a)
      return soap->error;
    a->next = soap->dom->atts;
    a->nstr = NULL;
    a->name = soap_strdup(soap, name);
    a->text = soap_strdup(soap, value);
    a->soap = soap;
    soap->dom->atts = a;
    if (!a->name || (value && !a->text))
      return soap->error = SOAP_EOM;
    return SOAP_OK;
  }
#endif
#ifndef WITH_LEAN
  if ((soap->mode & SOAP_XML_CANONICAL))
  {
    /* push namespace */
    if (value && !strncmp(name, "xmlns", 5) && ((name[5] == ':') || name[5] == '\0'))
    {
      (void)soap_push_ns(soap, name + 5 + (name[5] == ':'), value, 0, 0);
      if (name[5] == '\0')
        soap_utilize_ns(soap, SOAP_STR_EOS, 0);
      else if (soap->c14ninclude && ((*soap->c14ninclude == '*' || soap_tagsearch(soap->c14ninclude, name + 6))))
        soap_utilize_ns(soap, name, 0);
    }
    else
    {
      soap->level--;
      if (soap_set_attr(soap, name, value, 1))
        return soap->error;
      soap->level++;
    }
  }
  else
#endif
  {
    if (soap_send_raw(soap, " ", 1)
     || soap_send(soap, name))
      return soap->error;
    if (value)
      if (soap_send_raw(soap, "=\"", 2)
       || soap_string_out(soap, value, 1)
       || soap_send_raw(soap, "\"", 1))
        return soap->error;
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_begin_in(struct soap *soap, const char *tag, int nillable, const char *type)
{
  if (!soap_peek_element(soap))
  {
    if (soap->other)
      return soap->error = SOAP_TAG_MISMATCH;
    if (tag && *tag == '-')
      return SOAP_OK;
    soap->error = soap_match_tag(soap, soap->tag, tag);
    if (!soap->error)
    {
      if (type && *soap->type && soap_match_tag(soap, soap->type, type))
        return soap->error = SOAP_TYPE;
      soap->peeked = 0;
      if (!nillable && soap->null && (soap->mode & SOAP_XML_STRICT))
        return soap->error = SOAP_NULL;
      if (soap->body)
      {
        soap->level++;
        if (soap->level > soap->maxlevel)
          return soap->error = SOAP_LEVEL;
      }
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Begin tag found (level=%u) '%s'='%s'\n", soap->level, soap->tag, tag ? tag : SOAP_STR_EOS));
      soap->error = SOAP_OK;
    }
  }
  else if (soap->error == SOAP_NO_TAG && tag && *tag == '-')
  {
    soap->error = SOAP_OK;
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_element_end_in(struct soap *soap, const char *tag)
{
  soap_wchar c;
  char *s = NULL;
  int n = 0;
  if (tag && *tag == '-')
    return SOAP_OK;
  if (soap->error == SOAP_NO_TAG)
    soap->error = SOAP_OK;
#ifdef WITH_DOM
  /* this whitespace or mixed content is significant for DOM "as-is" */
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    const char *t = soap->dom->code; /* save XML code */
    s = soap_string_in(soap, -1, -1, -1, NULL);
    if (!soap->peeked && !s)
      return soap->error;
    if (soap->dom->prnt)
      soap->dom = soap->dom->prnt;
    if (s && (soap->mode & SOAP_XML_STRICT))
    {
      for (; *s; s++)
        if (!soap_coblank((soap_wchar)*s))
          return soap->error = SOAP_END_TAG; /* reject mixed content before ending tag */
    }
    soap->dom->code = t; /* restore XML code */
  }
#endif
  if (soap->peeked)
  {
    if (*soap->tag)
      n++;
    soap->peeked = 0;
  }
  do
  {
    while (((c = soap_get(soap)) != SOAP_TT))
    {
      if ((int)c == EOF)
        return soap->error = SOAP_CHK_EOF;
      if (!soap_coblank(c))
      {
        if ((soap->mode & SOAP_XML_STRICT))
          return soap->error = SOAP_END_TAG; /* reject mixed content before ending tag */
        if (c == SOAP_LT)
          n++;
        else if (c == '/')
        {
          c = soap_get(soap);
          if (c == SOAP_GT && n > 0)
            n--;
          else
            soap_unget(soap, c);
        }
      }
    }
  } while (n-- > 0);
  s = soap->tag;
  n = sizeof(soap->tag);
  while ((c = soap_get(soap)) > 32)
  {
    if (n > 1)
    {
      *s++ = (char)c;
      n--;
    }
  }
  *s = '\0';
  if ((int)c == EOF)
    return soap->error = SOAP_CHK_EOF;
  while (soap_coblank(c))
    c = soap_get(soap);
  if (c != SOAP_GT)
    return soap->error = SOAP_SYNTAX_ERROR;
#ifndef WITH_LEAN
#ifdef WITH_DOM
  if (soap->feltendin)
  {
    int err = soap->error;
    soap->error = soap->feltendin(soap, soap->tag, tag);
    if (soap->error)
      return soap->error;
    if (err)
      return soap->error = err; /* restore error */
  }
#endif
#endif
  if (tag && (soap->mode & SOAP_XML_STRICT))
  {
    soap_pop_namespace(soap);
    if (soap_match_tag(soap, soap->tag, tag))
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "End tag '%s' does not match '%s'\n", soap->tag, tag ? tag : SOAP_STR_EOS));
      return soap->error = SOAP_SYNTAX_ERROR;
    }
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "End tag found (level=%u) '%s'='%s'\n", soap->level, soap->tag, tag ? tag : SOAP_STR_EOS));
  soap->level--;
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
const char *
SOAP_FMAC2
soap_attr_value(struct soap *soap, const char *name, int flag, int occurs)
{
  struct soap_attribute *tp;
  if (*name == '-')
    return SOAP_STR_EOS;
  for (tp = soap->attributes; tp; tp = tp->next)
  {
    if (tp->visible == 2 && !soap_match_att(soap, tp->name, name))
      break;
  }
  if (tp)
  {
    if (occurs == 4 || (occurs == 2 && (soap->mode & SOAP_XML_STRICT)))
      soap->error = SOAP_PROHIBITED;
    else if (flag >= 4)
      return soap_collapse(soap, tp->value, flag, 1);
    else
      return tp->value;
  }
  else if (occurs == 3 || (occurs == 1 && (soap->mode & SOAP_XML_STRICT)))
  {
    soap->error = SOAP_REQUIRED;
  }
  else
  {
    soap->error = SOAP_OK;
  }
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_set_attr(struct soap *soap, const char *name, const char *value, int flag)
{
  struct soap_attribute *tp, *tq;
  if (*name == '-')
    return SOAP_OK;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Set attribute %s='%s'\n", name, value ? value : SOAP_STR_EOS));
  tq = NULL;
  for (tp = soap->attributes; tp; tq = tp, tp = tp->next)
  {
    if (!strcmp(tp->name, name))
      break;
  }
  if (!tp)
  {
    size_t l = strlen(name);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Allocate attribute %s\n", name));
    if (sizeof(struct soap_attribute) + l > l && (SOAP_MAXALLOCSIZE <= 0 || sizeof(struct soap_attribute) + l <= SOAP_MAXALLOCSIZE))
      tp = (struct soap_attribute*)SOAP_MALLOC(soap, sizeof(struct soap_attribute) + l);
    if (!tp)
      return soap->error = SOAP_EOM;
    tp->ns = NULL;
#ifndef WITH_LEAN
    if ((soap->mode & SOAP_XML_CANONICAL))
    {
      struct soap_attribute **tpp = &soap->attributes;
      const char *s = strchr(name, ':');
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Inserting attribute %s for c14n\n", name));
      if (!strncmp(name, "xmlns", 5))
      {
        for (; *tpp; tpp = &(*tpp)->next)
          if (strncmp((*tpp)->name, "xmlns", 5) || strcmp((*tpp)->name + 5, name + 5) > 0)
            break;
      }
      else if (!s)
      {
        for (; *tpp; tpp = &(*tpp)->next)
          if (strncmp((*tpp)->name, "xmlns", 5) && ((*tpp)->ns || strcmp((*tpp)->name, name) > 0))
            break;
      }
      else
      {
        struct soap_nlist *np = soap_lookup_ns(soap, name, s - name);
        if (np)
        {
          tp->ns = np->ns;
        }
        else
        {
          struct soap_attribute *tq;
          for (tq = soap->attributes; tq; tq = tq->next)
          {
            if (!strncmp(tq->name, "xmlns:", 6) && !strncmp(tq->name + 6, name, s - name) && !tq->name[6 + s - name])
            {
              tp->ns = tq->ns;
              break;
            }
          }
        }
        for (; *tpp; tpp = &(*tpp)->next)
        {
          int k;
          if (strncmp((*tpp)->name, "xmlns", 5) && (*tpp)->ns && tp->ns && ((k = strcmp((*tpp)->ns, tp->ns)) > 0 || (!k && strcmp((*tpp)->name, name) > 0)))
            break;
        }
      }
      tp->next = *tpp;
      *tpp = tp;
    }
    else
#endif
    if (tq)
    {
      tq->next = tp;
      tp->next = NULL;
    }
    else
    {
      tp->next = soap->attributes;
      soap->attributes = tp;
    }
    soap_strcpy((char*)tp->name, l + 1, name);
    tp->value = NULL;
  }
  else if (tp->visible)
  {
    return SOAP_OK;
  }
  else if (value && tp->value && tp->size <= strlen(value))
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Free attribute value of %s (free %p)\n", name, (void*)tp->value));
    SOAP_FREE(soap, tp->value);
    tp->value = NULL;
    tp->ns = NULL;
  }
  if (value)
  {
    if (!tp->value)
    {
      tp->size = strlen(value) + 1;
      if (SOAP_MAXALLOCSIZE <= 0 || tp->size <= SOAP_MAXALLOCSIZE)
        tp->value = (char*)SOAP_MALLOC(soap, tp->size);
      if (!tp->value)
        return soap->error = SOAP_EOM;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Allocate attribute value for %s (%p)\n", tp->name, (void*)tp->value));
    }
    soap_strcpy(tp->value, tp->size, value);
    if (!strncmp(tp->name, "xmlns:", 6))
      tp->ns = tp->value;
    tp->visible = 2;
    tp->flag = (short)flag;
#ifndef WITH_LEAN
    if (!strcmp(name, "wsu:Id"))
    {
      soap->event = SOAP_SEC_BEGIN;
      soap_strcpy(soap->id, sizeof(soap->id), value);
    }
    if ((soap->mode & SOAP_XML_CANONICAL) && !(soap->mode & SOAP_XML_CANONICAL_NA))
    {
      const char *s = strchr(name, ':');
      if (s && strchr(value, ':'))
      {
        struct soap_nlist *np = soap_lookup_ns(soap, name, s - name);
        if (np && np->ns && soap->local_namespaces)
        {
          if ((!strcmp(s + 1, "type") && !strcmp(np->ns, soap->local_namespaces[2].ns)) /* xsi:type QName */
           || ((!strcmp(s + 1, "arrayType") || !strcmp(s + 1, "itemType")) && !strcmp(np->ns, soap->local_namespaces[1].ns))) /* SOAP-ENC:arrayType and SOAP-ENC:itemType QName */
            soap_utilize_ns(soap, value, 1);
        }
      }
    }
#endif
  }
  else
  {
    tp->visible = 1;
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_clr_attr(struct soap *soap)
{
  struct soap_attribute *tp;
#ifndef WITH_LEAN
  if ((soap->mode & SOAP_XML_CANONICAL))
  {
    while (soap->attributes)
    {
      tp = soap->attributes->next;
      if (soap->attributes->value)
        SOAP_FREE(soap, soap->attributes->value);
      SOAP_FREE(soap, soap->attributes);
      soap->attributes = tp;
    }
  }
  else
#endif
  {
    for (tp = soap->attributes; tp; tp = tp->next)
      tp->visible = 0;
  }
}

/******************************************************************************/

static int
soap_getattrval(struct soap *soap, char *s, size_t *n, soap_wchar d)
{
  char buf[8];
  size_t i;
  size_t k = *n;
  size_t m = 0;
  char *t = buf;
  for (i = 0; i < k; i++)
  {
    soap_wchar c;
    if (m)
    {
      *s++ = *t++;
      m--;
      continue;
    }
    if ((soap->mode & SOAP_C_UTFSTRING))
    {
      c = soap_get(soap);
      if ((c & 0x80000000) && c >= -0x7FFFFF80 && c < SOAP_AP)
      {
        t = buf;
        c &= 0x7FFFFFFF;
        if (c < 0x0800)
          *t++ = (char)(0xC0 | ((c >> 6) & 0x1F));
        else
        {
#ifdef WITH_REPLACE_ILLEGAL_UTF8
          if (!((c >= 0x80 && c <= 0xD7FF) || (c >= 0xE000 && c <= 0xFFFD) || (c >= 0x10000 && c <= 0x10FFFF)))
            c = SOAP_UNKNOWN_UNICODE_CHAR;
#endif
          if (c < 0x010000)
          {
            *t++ = (char)(0xE0 | ((c >> 12) & 0x0F));
          }
          else
          {
            if (c < 0x200000)
            {
              *t++ = (char)(0xF0 | ((c >> 18) & 0x07));
            }
            else
            {
              if (c < 0x04000000)
              {
                *t++ = (char)(0xF8 | ((c >> 24) & 0x03));
              }
              else
              {
                *t++ = (char)(0xFC | ((c >> 30) & 0x01));
                *t++ = (char)(0x80 | ((c >> 24) & 0x3F));
              }
              *t++ = (char)(0x80 | ((c >> 18) & 0x3F));
            }
            *t++ = (char)(0x80 | ((c >> 12) & 0x3F));
          }
          *t++ = (char)(0x80 | ((c >> 6) & 0x3F));
        }
        *t++ = (char)(0x80 | (c & 0x3F));
        m = t - buf - 1;
        if (i + m >= k)
        {
          soap_unget(soap, c | 0x80000000);
          *n = i;
          return soap->error = SOAP_EOM;
        }
        t = buf;
        *s++ = *t++;
        continue;
      }
    }
    else
    {
      c = soap_getutf8(soap);
    }
    switch (c)
    {
    case SOAP_TT:
      *s++ = '<';
      soap_unget(soap, '/');
      break;
    case SOAP_LT:
      *s++ = '<';
      break;
    case SOAP_GT:
      if (d == ' ')
      {
        soap_unget(soap, c);
        *s = '\0';
        *n = i + 1;
        return SOAP_OK;
      }
      *s++ = '>';
      break;
    case SOAP_QT:
      if (c == d)
      {
        *s = '\0';
        *n = i + 1;
        return SOAP_OK;
      }
      *s++ = '"';
      break;
    case SOAP_AP:
      if (c == d)
      {
        *s = '\0';
        *n = i + 1;
        return SOAP_OK;
      }
      *s++ = '\'';
      break;
    case '\t':
    case '\n':
    case '\r':
    case ' ':
    case '/':
      if (d == ' ')
      {
        soap_unget(soap, c);
        *s = '\0';
        *n = i + 1;
        return SOAP_OK;
      }
      *s++ = (char)c;
      break;
    default:
      if ((int)c == EOF)
      {
        *s = '\0';
        *n = i + 1;
        return soap->error = SOAP_CHK_EOF;
      }
      *s++ = (char)c;
    }
  }
  return soap->error = SOAP_EOM;
}

/******************************************************************************/

#ifdef WITH_FAST
SOAP_FMAC1
int
SOAP_FMAC2
soap_store_lab(struct soap *soap, const char *s, size_t n)
{
  soap->labidx = 0;
  return soap_append_lab(soap, s, n);
}
#endif

/******************************************************************************/

#ifdef WITH_FAST
SOAP_FMAC1
int
SOAP_FMAC2
soap_append_lab(struct soap *soap, const char *s, size_t n)
{
  if (soap->labidx + n < soap->labidx)
    return soap->error = SOAP_EOM;
  if (soap->labidx + n >= soap->lablen)
  {
    char *t = soap->labbuf;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Enlarging look-aside buffer to append data, size=%lu\n", (unsigned long)soap->lablen));
    if (soap->lablen == 0)
      soap->lablen = SOAP_LABLEN;
    while (soap->labidx + n >= soap->lablen)
    {
      if (soap->lablen << 1 < soap->lablen)
        return soap->error = SOAP_EOM;
      soap->lablen <<= 1;
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "New look-aside buffer size=%lu\n", (unsigned long)soap->lablen));
    if (SOAP_MAXALLOCSIZE > 0 && soap->lablen > SOAP_MAXALLOCSIZE)
      return soap->error = SOAP_EOM;
    soap->labbuf = (char*)SOAP_MALLOC(soap, soap->lablen);
    if (!soap->labbuf)
    {
      if (t)
        SOAP_FREE(soap, t);
      return soap->error = SOAP_EOM;
    }
    if (t)
    {
      (void)soap_memcpy((void*)soap->labbuf, soap->lablen, (const void*)t, soap->labidx);
      SOAP_FREE(soap, t);
    }
  }
  if (s)
  {
    (void)soap_memcpy((void*)(soap->labbuf + soap->labidx), soap->lablen - soap->labidx, (const void*)s, n);
    soap->labidx += n;
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_peek_element(struct soap *soap)
{
#ifdef WITH_DOM
  struct soap_dom_attribute **att = NULL;
  char *lead = NULL;
#endif
  struct soap_attribute *tp, *tq = NULL;
  const char *t;
  char *s;
  soap_wchar c;
  int i;
  if (soap->peeked)
  {
    if (!*soap->tag)
      return soap->error = SOAP_NO_TAG;
    return SOAP_OK;
  }
  soap->peeked = 1;
  soap->id[0] = '\0';
  soap->href[0] = '\0';
  soap->type[0] = '\0';
  soap->arrayType[0] = '\0';
  soap->arraySize[0] = '\0';
  soap->arrayOffset[0] = '\0';
  soap->other = 0;
  soap->root = -1;
  soap->position = 0;
  soap->null = 0;
  soap->mustUnderstand = 0;
  /* UTF-8 BOM? */
  c = soap_getchar(soap);
  if (c == 0xEF && soap_get0(soap) == 0xBB)
  {
    soap_get1(soap);
    c = soap_get1(soap);
    if (c == 0xBF)
      soap->mode &= ~SOAP_ENC_LATIN;
    else
      soap_unget(soap, (0x0F << 12) | (0xBB << 6) | (c & 0x3F)); /* UTF-8 */
  }
  else if ((c == 0xFE && soap_get0(soap) == 0xFF)  /* UTF-16 BE */
        || (c == 0xFF && soap_get0(soap) == 0xFE)) /* UTF-16 LE */
  {
    return soap->error = SOAP_UTF_ERROR;
  }
  else
  {
    soap_unget(soap, c);
  }
  c = soap_get(soap);
#ifdef WITH_DOM
  /* whitespace leading up to the start tag is significant for DOM as-is (but comments and PIs are removed from this lead) */
  if (soap_coblank(c))
  {
    soap->labidx = 0;
    do
    {
      if (soap_append_lab(soap, NULL, 0))
        return soap->error;
      s = soap->labbuf + soap->labidx;
      i = soap->lablen - soap->labidx;
      soap->labidx = soap->lablen;
      while (soap_coblank(c))
      {
        if (c != '\r')
        {
          if (i-- <= 0)
            break;
          *s++ = c;
        }
        c = soap_get(soap);
      }
    } while (soap_coblank(c) || i == 0);
    *s = '\0';
    lead = soap->labbuf;
  }
#else
  /* skip space */
  while (soap_coblank(c))
    c = soap_get(soap);
#endif
  if (c != SOAP_LT)
  {
    *soap->tag = '\0';
    if ((int)c == EOF)
      return soap->error = SOAP_CHK_EOF;
    soap_unget(soap, c);
#ifdef WITH_DOM
    /* whitespace leading up to the end tag is significant for DOM as-is */
    if ((soap->mode & SOAP_XML_DOM) && soap->dom)
    {
      if (lead && *lead)
        soap->dom->tail = soap_strdup(soap, lead);
      else
        soap->dom->tail = SOAP_STR_EOS; /* body with closing tag instead of <tag/> */
    }
#endif
    return soap->error = SOAP_NO_TAG;
  }
  do
  {
    c = soap_get1(soap);
  } while (soap_coblank(c));
  s = soap->tag;
  i = sizeof(soap->tag);
  while (c != '>' && c != '/' && c > 32 && (int)c != EOF)
  {
    if (i > 1)
    {
      *s++ = (char)c;
      i--;
    }
    c = soap_get1(soap);
  }
  *s = '\0';
  while (soap_coblank(c))
    c = soap_get1(soap);
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM))
  {
    struct soap_dom_element *elt;
    elt = (struct soap_dom_element*)soap_malloc(soap, sizeof(struct soap_dom_element));
    if (!elt)
      return soap->error;
    elt->next = NULL;
    elt->prnt = soap->dom;
    elt->elts = NULL;
    elt->atts = NULL;
    elt->nstr = NULL;
    elt->name = soap_strdup(soap, soap->tag);
    elt->text = NULL;
    elt->code = NULL;
    elt->tail = NULL;
    elt->node = NULL;
    elt->type = 0;
    if (lead && *lead)
      elt->lead = soap_strdup(soap, lead);
    else
      elt->lead = NULL;
    elt->soap = soap;
    if (soap->dom)
    {
      struct soap_dom_element *p = soap->dom->elts;
      if (p)
      {
        while (p->next)
          p = p->next;
        p->next = elt;
      }
      else
      {
        soap->dom->elts = elt;
      }
    }
    soap->dom = elt;
    att = &elt->atts;
    if (!elt->name)
      return soap->error = SOAP_EOM;
  }
#endif
  soap_pop_namespace(soap);
  for (tp = soap->attributes; tp; tp = tp->next)
    tp->visible = 0;
  while ((int)c != EOF && c != '>' && c != '/')
  {
    s = soap->tmpbuf;
    i = sizeof(soap->tmpbuf);
    while (c != '=' && c != '>' && c != '/' && c > 32 && (int)c != EOF)
    {
      if (i > 1)
      {
        *s++ = (char)c;
        i--;
      }
      c = soap_get1(soap);
    }
    *s = '\0';
    if (i == sizeof(soap->tmpbuf))
      return soap->error = SOAP_SYNTAX_ERROR;
#ifdef WITH_DOM
    /* add attribute name to dom */
    if (att)
    {
      *att = (struct soap_dom_attribute*)soap_malloc(soap, sizeof(struct soap_dom_attribute));
       if (!*att)
         return soap->error;
       (*att)->next = NULL;
       (*att)->nstr = NULL;
       (*att)->name = soap_strdup(soap, soap->tmpbuf);
       (*att)->text = NULL;
       (*att)->soap = soap;
       if (!(*att)->name)
         return soap->error = SOAP_EOM;
    }
#endif
    if (!strncmp(soap->tmpbuf, "xmlns", 5))
    {
      if (soap->tmpbuf[5] == ':')
        t = soap->tmpbuf + 6;
      else if (soap->tmpbuf[5])
        t = NULL;
      else
        t = SOAP_STR_EOS;
    }
    else
    {
      t = NULL;
    }
    tq = NULL;
    for (tp = soap->attributes; tp; tq = tp, tp = tp->next)
    {
      if (!SOAP_STRCMP(tp->name, soap->tmpbuf))
        break;
    }
    if (!tp)
    {
      size_t l = strlen(soap->tmpbuf);
      tp = (struct soap_attribute*)SOAP_MALLOC(soap, sizeof(struct soap_attribute) + l);
      if (!tp)
        return soap->error = SOAP_EOM;
      (void)soap_memcpy((char*)tp->name, l + 1, soap->tmpbuf, l + 1);
      tp->value = NULL;
      tp->size = 0;
      tp->ns = NULL;
      tp->visible = 0;
      /* append attribute to the end of the list */
      if (tq)
      {
        tq->next = tp;
        tp->next = NULL;
      }
      else
      {
        tp->next = soap->attributes;
        soap->attributes = tp;
      }
    }
    else if (tp->visible)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Duplicate attribute in %s\n", soap->tag));
      return soap->error = SOAP_SYNTAX_ERROR; /* redefined (duplicate) attribute */
    }
    while (soap_coblank(c))
      c = soap_get1(soap);
    if (c == '=')
    {
      size_t k;
      do
      {
        c = soap_getutf8(soap);
      } while (soap_coblank(c));
      if (c != SOAP_QT && c != SOAP_AP)
      {
        soap_unget(soap, c);
        c = ' '; /* blank delimiter */
      }
      k = tp->size;
      if (soap_getattrval(soap, tp->value, &k, c))
      {
#ifdef WITH_FAST
        if (soap->error != SOAP_EOM)
          return soap->error;
        soap->error = SOAP_OK;
        if (soap_store_lab(soap, tp->value, k))
          return soap->error;
        if (tp->value)
          SOAP_FREE(soap, tp->value);
        tp->value = NULL;
        for (;;)
        {
          k = soap->lablen - soap->labidx;
          if (soap_getattrval(soap, soap->labbuf + soap->labidx, &k, c))
          {
            if (soap->error != SOAP_EOM)
              return soap->error;
            soap->error = SOAP_OK;
            soap->labidx = soap->lablen;
            if (soap_append_lab(soap, NULL, 0))
              return soap->error;
          }
          else
          {
            break;
          }
        }
        if (soap->labidx)
        {
          tp->size = soap->lablen;
        }
        else
        {
          tp->size = strlen(soap->labbuf) + 1;
          if (tp->size < SOAP_LABLEN)
            tp->size = SOAP_LABLEN;
        }
        tp->value = (char*)SOAP_MALLOC(soap, tp->size);
        if (!tp->value)
          return soap->error = SOAP_EOM;
        soap_strcpy(tp->value, tp->size, soap->labbuf);
#else
        tp->size = k;
        if (soap->error != SOAP_EOM)
          return soap->error;
        soap->error = SOAP_OK;
        if (soap_alloc_block(soap) == NULL)
          return soap->error;
        for (;;)
        {
          s = (char*)soap_push_block(soap, NULL, SOAP_BLKLEN);
          if (!s)
            return soap->error;
          k = SOAP_BLKLEN;
          if (soap_getattrval(soap, s, &k, c))
          {
            if (soap->error != SOAP_EOM)
              return soap->error;
            soap->error = SOAP_OK;
            soap_size_block(soap, NULL, k);
          }
          else
          {
            break;
          }
        }
        k = tp->size + soap->blist->size;
        if (SOAP_MAXALLOCSIZE > 0 && k > SOAP_MAXALLOCSIZE)
          return soap->error = SOAP_EOM;
        s = (char*)SOAP_MALLOC(soap, k);
        if (!s)
          return soap->error = SOAP_EOM;
        if (tp->value)
        {
          (void)soap_memcpy((void*)s, k, (const void*)tp->value, tp->size);
          SOAP_FREE(soap, tp->value);
        }
        (void)soap_save_block(soap, NULL, s + tp->size, 0);
        tp->value = s;
        tp->size = k;
#endif
      }
      tp->visible = 2; /* seen this attribute w/ value */
      do
      {
        c = soap_get1(soap);
      } while (soap_coblank(c));
#ifdef WITH_DOM
      if (att && tp->value)
      {
        (*att)->text = soap_strdup(soap, tp->value);
        if (!(*att)->text)
          return soap->error = SOAP_EOM;
      }
#endif
    }
    else
    {
      tp->visible = 1; /* seen this attribute w/o value */
    }
#ifdef WITH_DOM
    if (att)
      att = &(*att)->next;
#endif
    if (t && tp->value)
    {
      if (soap_push_namespace(soap, t, tp->value) == NULL)
        return soap->error;
    }
  }
#ifdef WITH_DOM
  if (att)
  {
    soap->dom->nstr = soap_current_namespace_tag(soap, soap->tag);
    for (att = &soap->dom->atts; *att; att = &(*att)->next)
      (*att)->nstr = soap_current_namespace_att(soap, (*att)->name);
  }
#endif
  if ((int)c == EOF)
    return soap->error = SOAP_CHK_EOF;
  soap->body = (c != '/');
  if (!soap->body)
  {
    do
    {
      c = soap_get1(soap);
    } while (soap_coblank(c));
  }
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM))
  {
    if (!soap->body && soap->dom->prnt)
      soap->dom = soap->dom->prnt;
  }
#endif
  for (tp = soap->attributes; tp; tp = tp->next)
  {
    if (tp->visible && tp->value)
    {
#ifndef WITH_NOIDREF
      if (!strcmp(tp->name, "id"))
      {
        if ((soap->version > 0 && !(soap->imode & SOAP_XML_TREE))
         || (soap->mode & SOAP_XML_GRAPH))
        {
          *soap->id = '#';
          soap_strcpy(soap->id + 1, sizeof(soap->id) - 1, tp->value);
        }
      }
      else if (!strcmp(tp->name, "href"))
      {
        if ((soap->version == 1 && !(soap->imode & SOAP_XML_TREE))
         || (soap->mode & SOAP_XML_GRAPH)
         || ((soap->mode & (SOAP_ENC_MTOM | SOAP_ENC_DIME)) && *tp->value != '#'))
          soap_strcpy(soap->href, sizeof(soap->href), tp->value);
      }
      else if (!strcmp(tp->name, "ref"))
      {
        if ((soap->version == 2 && !(soap->imode & SOAP_XML_TREE))
         || (soap->mode & SOAP_XML_GRAPH))
        {
          *soap->href = '#';
          soap_strcpy(soap->href + (*tp->value != '#'), sizeof(soap->href) - 1, tp->value);
        }
      }
#else
      if (!strcmp(tp->name, "href"))
      {
        if ((soap->mode & (SOAP_ENC_MTOM | SOAP_ENC_DIME)) && *tp->value != '#')
          soap_strcpy(soap->href, sizeof(soap->href), tp->value);
      }
#endif
      else if (!soap_match_tag(soap, tp->name, "xsi:type"))
      {
        soap_strcpy(soap->type, sizeof(soap->type), tp->value);
      }
      else if ((!soap_match_tag(soap, tp->name, "xsi:null")
             || !soap_match_tag(soap, tp->name, "xsi:nil"))
            && (!strcmp(tp->value, "1")
             || !strcmp(tp->value, "true")))
      {
        soap->null = 1;
      }
      else if (!soap_match_tag(soap, tp->name, "SOAP-ENV:encodingStyle"))
      {
        if (!soap->encodingStyle)
          soap->encodingStyle = SOAP_STR_EOS;
        soap_version(soap);
      }
      else if (soap->version == 1)
      {
        if (!soap_match_tag(soap, tp->name, "SOAP-ENC:arrayType"))
        {
          s = soap_strrchr(tp->value, '[');
          if (s && (size_t)(s - tp->value) < sizeof(soap->arrayType))
          {
            (void)soap_strncpy(soap->arrayType, sizeof(soap->arrayType), tp->value, s - tp->value);
            soap_strcpy(soap->arraySize, sizeof(soap->arraySize), s);
          }
          else
            soap_strcpy(soap->arrayType, sizeof(soap->arrayType), tp->value);
        }
        else if (!soap_match_tag(soap, tp->name, "SOAP-ENC:offset"))
        {
          soap_strcpy(soap->arrayOffset, sizeof(soap->arrayOffset), tp->value);
        }
        else if (!soap_match_tag(soap, tp->name, "SOAP-ENC:position"))
        {
          soap->position = soap_getposition(tp->value, soap->positions);
        }
        else if (!soap_match_tag(soap, tp->name, "SOAP-ENC:root"))
        {
          soap->root = ((!strcmp(tp->value, "1") || !strcmp(tp->value, "true")));
        }
        else if (!soap_match_tag(soap, tp->name, "SOAP-ENV:mustUnderstand")
              && (!strcmp(tp->value, "1") || !strcmp(tp->value, "true")))
        {
          soap->mustUnderstand = 1;
        }
        else if (!soap_match_tag(soap, tp->name, "SOAP-ENV:actor"))
        {
          if ((!soap->actor || strcmp(soap->actor, tp->value))
           && strcmp(tp->value, "http://schemas.xmlsoap.org/soap/actor/next"))
            soap->other = 1;
        }
      }
      else if (soap->version == 2)
      {
#ifndef WITH_NOIDREF
        if (!soap_match_tag(soap, tp->name, "SOAP-ENC:id"))
        {
          *soap->id = '#';
          soap_strcpy(soap->id + 1, sizeof(soap->id) - 1, tp->value);
        }
        else if (!soap_match_tag(soap, tp->name, "SOAP-ENC:ref"))
        {
          *soap->href = '#';
          soap_strcpy(soap->href + (*tp->value != '#'), sizeof(soap->href) - 1, tp->value);
        }
        else
#endif
        if (!soap_match_tag(soap, tp->name, "SOAP-ENC:itemType"))
        {
          soap_strcpy(soap->arrayType, sizeof(soap->arrayType), tp->value);
        }
        else if (!soap_match_tag(soap, tp->name, "SOAP-ENC:arraySize"))
        {
          soap_strcpy(soap->arraySize, sizeof(soap->arraySize), tp->value);
        }
        else if (!soap_match_tag(soap, tp->name, "SOAP-ENV:mustUnderstand")
              && (!strcmp(tp->value, "1") || !strcmp(tp->value, "true")))
        {
          soap->mustUnderstand = 1;
        }
        else if (!soap_match_tag(soap, tp->name, "SOAP-ENV:role"))
        {
          if ((!soap->actor || strcmp(soap->actor, tp->value))
           && strcmp(tp->value, "http://www.w3.org/2003/05/soap-envelope/role/next"))
            soap->other = 1;
        }
      }
      else
      {
        if (!soap_match_tag(soap, tp->name, "wsdl:required") && !strcmp(tp->value, "true"))
          soap->mustUnderstand = 1;
      }
    }
  }
#ifdef WITH_DOM
  if (soap->feltbegin)
    return soap->error = soap->feltbegin(soap, soap->tag);
#endif
  return soap->error = SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_retry(struct soap *soap)
{
  soap->error = SOAP_OK;
  soap_revert(soap);
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_revert(struct soap *soap)
{
  if (!soap->peeked)
  {
    soap->peeked = 1;
    if (soap->body)
      soap->level--;
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Reverting to last element '%s' (level=%u)\n", soap->tag, soap->level));
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_ignore(struct soap *soap)
{
  int n = 0;
  soap_wchar c;
  soap->level++;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Ignoring XML content at level=%u\n", soap->level));
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    if (!soap_string_in(soap, -1, -1, -1, NULL))
      return soap->error;
  }
  else
#endif
  {
    for (;;)
    {
      c = soap_get(soap);
      switch (c)
      {
        case SOAP_TT:
          if (n == 0)
            goto end;
          n--;
          break;
        case SOAP_LT:
          n++;
          break;
        case '/':
          if (n > 0)
          {
            c = soap_get0(soap);
            if (c == '>')
              n--;
          }
          break;
        default:
          if ((int)c == EOF)
            return soap->error = SOAP_EOF;
      }
    }
end:
    soap_unget(soap, c);
  }
  return soap_element_end_in(soap, NULL);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_string_out(struct soap *soap, const char *s, int flag)
{
  const char *t;
  soap_wchar c;
  soap_wchar mask = (soap_wchar)0xFFFFFF80UL;
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    soap->dom->text = soap_strdup(soap, s);
    if (!soap->dom->text)
      return soap->error = SOAP_EOM;
    return SOAP_OK;
  }
#endif
  if (flag == 2 || (soap->mode & SOAP_C_UTFSTRING))
    mask = 0;
  t = s;
  while ((c = *t++))
  {
    switch (c)
    {
    case 0x09:
      if (flag)
      {
        if (soap_send_raw(soap, s, t - s - 1) || soap_send_raw(soap, "&#x9;", 5))
          return soap->error;
        s = t;
      }
      break;
    case 0x0A:
      if (flag)
      {
        if (soap_send_raw(soap, s, t - s - 1) || soap_send_raw(soap, "&#xA;", 5))
          return soap->error;
        s = t;
      }
      break;
#ifdef WITH_CRTOLF
    case 0x0D:
      if (*t == 0x0A)
      {
        if (soap_send_raw(soap, s, t - s - 1))
          return soap->error;
        s = t;
      }
      else if (flag)
      {
        if (soap_send_raw(soap, s, t - s - 1) || soap_send_raw(soap, "&#xA;", 5))
          return soap->error;
        s = t;
      }
      else
      {
        if (soap_send_raw(soap, s, t - s - 1) || soap_send_raw(soap, "\n", 1))
          return soap->error;
        s = t;
      }
      break;
#endif
    case '&':
      if (soap_send_raw(soap, s, t - s - 1) || soap_send_raw(soap, "&amp;", 5))
        return soap->error;
      s = t;
      break;
    case '<':
      if (soap_send_raw(soap, s, t - s - 1) || soap_send_raw(soap, "&lt;", 4))
        return soap->error;
      s = t;
      break;
    case '>':
      if (!flag)
      {
        if (soap_send_raw(soap, s, t - s - 1) || soap_send_raw(soap, "&gt;", 4))
          return soap->error;
        s = t;
      }
      break;
    case '"':
      if (flag)
      {
        if (soap_send_raw(soap, s, t - s - 1) || soap_send_raw(soap, "&quot;", 6))
          return soap->error;
        s = t;
      }
      break;
    case 0x7F:
      if (soap_send_raw(soap, s, t - s - 1) || soap_send_raw(soap, "&#x7F;", 6))
        return soap->error;
      s = t;
      break;
    default:
#ifndef WITH_LEANER
#ifdef HAVE_MBTOWC
      if ((soap->mode & SOAP_C_MBSTRING))
      {
        wchar_t wc;
        int m = mbtowc(&wc, t - 1, MB_CUR_MAX);
        if (m > 0 && !((soap_wchar)wc == c && m == 1 && c < 0x80))
        {
          if (soap_send_raw(soap, s, t - s - 1) || soap_pututf8(soap, (unsigned long)wc))
            return soap->error;
          s = t += m - 1;
          continue;
        }
      }
#endif
#endif
#ifndef WITH_NOSTRINGTOUTF8
      if ((c & mask) || !(c & 0xFFFFFFE0UL))
      {
        if (soap_send_raw(soap, s, t - s - 1) || soap_pututf8(soap, (unsigned char)c))
          return soap->error;
        s = t;
      }
#endif
    }
  }
  return soap_send_raw(soap, s, t - s - 1);
}

/******************************************************************************/

SOAP_FMAC1
char *
SOAP_FMAC2
soap_string_in(struct soap *soap, int flag, long minlen, long maxlen, const char *pattern)
{
  char *s = NULL;
  char *t = NULL;
  size_t i;
  ULONG64 l = 0;
  int n = 0, f = 0, m = 0;
  soap_wchar c;
#if !defined(WITH_LEANER) && defined(HAVE_WCTOMB)
  char buf[MB_LEN_MAX > 8 ? MB_LEN_MAX : 8];
#else
  char buf[8];
#endif
  if (maxlen < 0 && soap->maxlength > 0)
    maxlen = soap->maxlength;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Reading string content, flag=%d\n", flag));
  if (flag <= 0 && soap->peeked && *soap->tag)
  {
#ifndef WITH_LEAN
    struct soap_attribute *tp;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "String content includes tag '%s' and attributes\n", soap->tag));
    t = soap->tmpbuf;
    *t = '<';
    soap_strcpy(t + 1, sizeof(soap->tmpbuf) - 1, soap->tag);
    t += strlen(t);
    for (tp = soap->attributes; tp; tp = tp->next)
    {
      if (tp->visible)
      {
        size_t k = strlen(tp->name);
        if (t + k + 1 >= soap->tmpbuf + sizeof(soap->tmpbuf))
          break; /* too many or attribute values to large */
        *t++ = ' ';
        (void)soap_strncpy(t, sizeof(soap->tmpbuf) - (t - soap->tmpbuf), tp->name, k);
        t += k;
        if (tp->value)
        {
          k = strlen(tp->value);
          if (t + k + 3 >= soap->tmpbuf + sizeof(soap->tmpbuf))
            break; /* too many or attribute values to large */
          *t++ = '=';
          *t++ = '"';
          (void)soap_strncpy(t, sizeof(soap->tmpbuf) - (t - soap->tmpbuf), tp->value, k);
          t += k;
          *t++ = '"';
        }
      }
    }
    if (!soap->body)
      *t++ = '/';
    *t++ = '>';
    *t = '\0';
    t = soap->tmpbuf;
    m = (int)strlen(soap->tmpbuf);
#endif
    if (soap->body)
      n = 1;
    f = 1;
    soap->peeked = 0;
  }
#ifdef WITH_CDATA
  if (flag <= 0)
  {
    int state = 0;
#ifdef WITH_FAST
    soap->labidx = 0;                   /* use look-aside buffer */
#else
    if (soap_alloc_block(soap) == NULL)
      return NULL;
#endif
    for (;;)
    {
#ifdef WITH_FAST
      size_t k;
      if (soap_append_lab(soap, NULL, 0))       /* allocate more space in look-aside buffer if necessary */
        return NULL;
      s = soap->labbuf + soap->labidx;  /* space to populate */
      k = soap->lablen - soap->labidx;  /* number of bytes available */
      soap->labidx = soap->lablen;      /* claim this space */
#else
      size_t k = SOAP_BLKLEN;
      s = (char*)soap_push_block(soap, NULL, k);
      if (!s)
        return NULL;
#endif
      for (i = 0; i < k; i++)
      {
        if (m > 0)
        {
          *s++ = *t++;  /* copy multibyte characters */
          m--;
          continue;
        }
        c = soap_getchar(soap);
        if ((int)c == EOF)
          goto end;
        if ((c >= 0x80 || c < SOAP_AP) && state != 1)
        {
          if ((c & 0x7FFFFFFF) >= 0x80)
          {
            soap_unget(soap, c);
            c = soap_getutf8(soap);
          }
          if ((c & 0x7FFFFFFF) >= 0x80 && (flag <= 0 || (soap->mode & SOAP_C_UTFSTRING)))
          {
            c &= 0x7FFFFFFF;
            t = buf;
            if (c < 0x0800)
              *t++ = (char)(0xC0 | ((c >> 6) & 0x1F));
            else
            {
#ifdef WITH_REPLACE_ILLEGAL_UTF8
              if (!((c >= 0x80 && c <= 0xD7FF) || (c >= 0xE000 && c <= 0xFFFD) || (c >= 0x10000 && c <= 0x10FFFF)))
                c = SOAP_UNKNOWN_UNICODE_CHAR;
#endif
              if (c < 0x010000)
              {
                *t++ = (char)(0xE0 | ((c >> 12) & 0x0F));
              }
              else
              {
                if (c < 0x200000)
                {
                  *t++ = (char)(0xF0 | ((c >> 18) & 0x07));
                }
                else
                {
                  if (c < 0x04000000)
                  {
                    *t++ = (char)(0xF8 | ((c >> 24) & 0x03));
                  }
                  else
                  {
                    *t++ = (char)(0xFC | ((c >> 30) & 0x01));
                    *t++ = (char)(0x80 | ((c >> 24) & 0x3F));
                  }
                  *t++ = (char)(0x80 | ((c >> 18) & 0x3F));
                }
                *t++ = (char)(0x80 | ((c >> 12) & 0x3F));
              }
              *t++ = (char)(0x80 | ((c >> 6) & 0x3F));
            }
            *t++ = (char)(0x80 | (c & 0x3F));
            m = (int)(t - buf) - 1;
            t = buf;
            *s++ = *t++;
            continue;
          }
        }
        switch (state)
        {
          case 1:
            if (c == ']')
              state = 4;
            *s++ = (char)c;
            continue;
          case 2:
            if (c == '-')
              state = 6;
            *s++ = (char)c;
            continue;
          case 3:
            if (c == '?')
              state = 8;
            *s++ = (char)c;
            continue;
          /* CDATA */
          case 4:
            if (c == ']')
              state = 5;
            else
              state = 1;
            *s++ = (char)c;
            continue;
          case 5:
            if (c == '>')
              state = 0;
            else if (c != ']')
              state = 1;
            *s++ = (char)c;
            continue;
          /* comment */
          case 6:
            if (c == '-')
              state = 7;
            else
              state = 2;
            *s++ = (char)c;
            continue;
          case 7:
            if (c == '>')
              state = 0;
            else if (c != '-')
              state = 2;
            *s++ = (char)c;
            continue;
          /* PI */
          case 8:
            if (c == '>')
              state = 0;
            else if (c != '?')
              state = 3;
            *s++ = (char)c;
            continue;
        }
        switch (c)
        {
        case SOAP_TT:
          if (n == 0)
            goto end;
          n--;
          *s++ = '<';
          t = (char*)"/";
          m = 1;
          break;
        case SOAP_LT:
          if (flag == 3 || (f && n == 0))
            goto end;
          n++;
          *s++ = '<';
          break;
        case SOAP_GT:
          *s++ = '>';
          break;
        case SOAP_QT:
          *s++ = '"';
          break;
        case SOAP_AP:
          *s++ = '\'';
          break;
        case '/':
          if (n > 0)
          {
            c = soap_getchar(soap);
            if (c == '>')
              n--;
            soap_unget(soap, c);
          }
          *s++ = '/';
          break;
        case '<':
          c = soap_getchar(soap);
          if (c == '/')
          {
            if (n == 0)
            {
              c = SOAP_TT;
              goto end;
            }
            n--;
          }
          else if (c == '!')
          {
            c = soap_getchar(soap);
            if (c == '[')
            {
              do
              {
                c = soap_getchar(soap);
              } while ((int)c != EOF && c != '[');
              if ((int)c == EOF)
                 goto end;
              t = (char*)"![CDATA[";
              m = 8;
              state = 1;
            }
            else if (c == '-')
            {
              c = soap_getchar(soap);
              if (c == '-')
                state = 2;
              t = (char*)"!-";
              m = 2;
              soap_unget(soap, c);
            }
            else
            {
              t = (char*)"!";
              m = 1;
              soap_unget(soap, c);
            }
            *s++ = '<';
            break;
          }
          else if (c == '?')
          {
            state = 3;
          }
          else if (flag == 3 || (f && n == 0))
          {
            soap_revget1(soap);
            c = '<';
            goto end;
          }
          else
            n++;
          soap_unget(soap, c);
          *s++ = '<';
          break;
        case '>':
          *s++ = '>';
          break;
        case '"':
          *s++ = '"';
          break;
        default:
#ifndef WITH_LEANER
#ifdef HAVE_WCTOMB
          if ((soap->mode & SOAP_C_MBSTRING))
          {
#if defined(WIN32) && !defined(CYGWIN) && !defined(__MINGW32__) && !defined(__MINGW64__) && !defined(__BORLANDC__)
            m = 0;
            wctomb_s(&m, buf, sizeof(buf), (wchar_t)(c & 0x7FFFFFFF));
#else
            m = wctomb(buf, (wchar_t)(c & 0x7FFFFFFF));
#endif
            if (m >= 1 && m <= (int)MB_CUR_MAX)
            {
              t = buf;
              *s++ = *t++;
              m--;
            }
            else
            {
              *s++ = SOAP_UNKNOWN_CHAR;
              m = 0;
            }
          }
          else
#endif
#endif
            *s++ = (char)(c & 0xFF);
        }
        l++;
        if (maxlen >= 0 && l > (size_t)maxlen)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "String too long: maxlen=%ld\n", maxlen));
          soap->error = SOAP_LENGTH;
          return NULL;
        }
      }
    }
  }
#endif
#ifdef WITH_FAST
  soap->labidx = 0;                     /* use look-aside buffer */
#else
  if (soap_alloc_block(soap) == NULL)
    return NULL;
#endif
  for (;;)
  {
#ifdef WITH_FAST
    size_t k;
    if (soap_append_lab(soap, NULL, 0)) /* allocate more space in look-aside buffer if necessary */
      return NULL;
    s = soap->labbuf + soap->labidx;    /* space to populate */
    k = soap->lablen - soap->labidx;    /* number of bytes available */
    soap->labidx = soap->lablen;        /* claim this space */
#else
    size_t k = SOAP_BLKLEN;
    s = (char*)soap_push_block(soap, NULL, k);
    if (!s)
      return NULL;
#endif
    for (i = 0; i < k; i++)
    {
      if (m > 0)
      {
        *s++ = *t++;    /* copy multibyte characters */
        m--;
        continue;
      }
#ifndef WITH_CDATA
      if (flag <= 0)
        c = soap_getchar(soap);
      else
#endif
      {
        c = soap_getutf8(soap);
        if ((soap->mode & SOAP_C_UTFSTRING))
        {
          if (c >= 0x80 || (c < SOAP_AP && c >= -0x7FFFFF80))
          {
            c &= 0x7FFFFFFF;
            t = buf;
            if (c < 0x0800)
            {
              *t++ = (char)(0xC0 | ((c >> 6) & 0x1F));
            }
            else
            {
#ifdef WITH_REPLACE_ILLEGAL_UTF8
              if (!((c >= 0x80 && c <= 0xD7FF) || (c >= 0xE000 && c <= 0xFFFD) || (c >= 0x10000 && c <= 0x10FFFF)))
                c = SOAP_UNKNOWN_UNICODE_CHAR;
#endif
              if (c < 0x010000)
              {
                *t++ = (char)(0xE0 | ((c >> 12) & 0x0F));
              }
              else
              {
                if (c < 0x200000)
                {
                  *t++ = (char)(0xF0 | ((c >> 18) & 0x07));
                }
                else
                {
                  if (c < 0x04000000)
                  {
                    *t++ = (char)(0xF8 | ((c >> 24) & 0x03));
                  }
                  else
                  {
                    *t++ = (char)(0xFC | ((c >> 30) & 0x01));
                    *t++ = (char)(0x80 | ((c >> 24) & 0x3F));
                  }
                  *t++ = (char)(0x80 | ((c >> 18) & 0x3F));
                }
                *t++ = (char)(0x80 | ((c >> 12) & 0x3F));
              }
              *t++ = (char)(0x80 | ((c >> 6) & 0x3F));
            }
            *t++ = (char)(0x80 | (c & 0x3F));
            m = (int)(t - buf) - 1;
            t = buf;
            *s++ = *t++;
            l++;
            if (maxlen >= 0 && l > (size_t)maxlen)
            {
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "String too long: maxlen=%ld\n", maxlen));
              soap->error = SOAP_LENGTH;
              return NULL;
            }
            continue;
          }
        }
      }
      switch (c)
      {
      case SOAP_TT:
        if (n == 0)
          goto end;
        n--;
        *s++ = '<';
        t = (char*)"/";
        m = 1;
        break;
      case SOAP_LT:
        if (flag == 3 || (f && n == 0))
          goto end;
        n++;
        *s++ = '<';
        break;
      case SOAP_GT:
        *s++ = '>';
        break;
      case SOAP_QT:
        *s++ = '"';
        break;
      case SOAP_AP:
        *s++ = '\'';
        break;
      case '/':
        if (n > 0)
        {
          if (flag > 0)
          {
            c = soap_get(soap);
            if (c == SOAP_GT)
              n--;
          }
          else
          {
            c = soap_getchar(soap);
            if (c == '>')
              n--;
          }
          soap_unget(soap, c);
        }
        *s++ = '/';
        break;
      case (soap_wchar)('<' | 0x80000000):
        if (flag > 0)
        {
          *s++ = '<';
        }
        else
        {
          *s++ = '&';
          t = (char*)"lt;";
          m = 3;
        }
        break;
      case (soap_wchar)('>' | 0x80000000):
        if (flag > 0)
        {
          *s++ = '>';
        }
        else
        {
          *s++ = '&';
          t = (char*)"gt;";
          m = 3;
        }
        break;
      case (soap_wchar)('&' | 0x80000000):
        if (flag > 0)
        {
          *s++ = '&';
        }
        else
        {
          *s++ = '&';
          t = (char*)"amp;";
          m = 4;
        }
        break;
      case (soap_wchar)('"' | 0x80000000):
        if (flag > 0)
        {
          *s++ = '"';
        }
        else
        {
          *s++ = '&';
          t = (char*)"quot;";
          m = 5;
        }
        break;
      case (soap_wchar)('\'' | 0x80000000):
        if (flag > 0)
        {
          *s++ = '\'';
        }
        else
        {
          *s++ = '&';
          t = (char*)"apos;";
          m = 5;
        }
        break;
      default:
        if ((int)c == EOF)
          goto end;
#ifndef WITH_CDATA
        if (c == '<')
        {
          c = soap_getchar(soap);
          soap_unget(soap, c);
          if (c == '/')
          {
            c = SOAP_TT;
            if (n == 0)
              goto end;
            n--;
          }
          else
          {
            n++;
          }
          *s++ = '<';
        }
        else
#endif
#ifndef WITH_LEANER
#ifdef HAVE_WCTOMB
        if ((soap->mode & SOAP_C_MBSTRING))
        {
#if defined(WIN32) && !defined(CYGWIN) && !defined(__MINGW32__) && !defined(__MINGW64__) && !defined(__BORLANDC__)
          m = 0;
          wctomb_s(&m, buf, sizeof(buf), (wchar_t)(c & 0x7FFFFFFF));
#else
          m = wctomb(buf, (wchar_t)(c & 0x7FFFFFFF));
#endif
          if (m >= 1 && m <= (int)MB_CUR_MAX)
          {
            t = buf;
            *s++ = *t++;
            m--;
          }
          else
          {
            *s++ = SOAP_UNKNOWN_CHAR;
            m = 0;
          }
        }
        else
#endif
#endif
          *s++ = (char)(c & 0xFF);
      }
      l++;
      if (maxlen >= 0 && l > (size_t)maxlen)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "String too long: maxlen=%ld\n", maxlen));
        soap->error = SOAP_LENGTH;
        return NULL;
      }
    }
  }
end:
  soap_unget(soap, c);
  *s = '\0';
#ifdef WITH_FAST
  t = soap_strdup(soap, soap->labbuf);
  if (!t)
    return NULL;
#else
  soap_size_block(soap, NULL, i + 1);
  t = (char*)soap_save_block(soap, NULL, NULL, 0);
#endif
  if (minlen > 0 && l < (size_t)minlen)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "String too short: %lu chars, minlen=%ld\n", (unsigned long)l, minlen));
    soap->error = SOAP_LENGTH;
    return NULL;
  }
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom && *t)
  {
    if (flag > 0)
      soap->dom->text = t;
    else
      soap->dom->code = t;
  }
#endif
  if (flag == 2)
  {
    if (soap_s2QName(soap, t, &t, minlen, maxlen, pattern))
      return NULL;
  }
  else if (flag >= 4 && t)
  {
    t = soap_collapse(soap, t, flag, 1);
  }
#ifndef WITH_LEANER
  else if (pattern && soap->fsvalidate)
  {
    soap->error = soap->fsvalidate(soap, pattern, t);
    if (soap->error)
      return NULL;
  }
#endif
  return t;
}

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_wstring_out(struct soap *soap, const wchar_t *s, int flag)
{
  const char *t;
  char tmp;
  soap_wchar c;
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    soap->dom->text = soap_wchar2s(soap, s);
    return SOAP_OK;
  }
#endif
  while ((c = *s++))
  {
    switch (c)
    {
    case 0x09:
      if (flag)
        t = "&#x9;";
      else
        t = "\t";
      break;
    case 0x0A:
      if (flag)
        t = "&#xA;";
      else
        t = "\n";
      break;
#ifdef WITH_CR_TO_LF
    case 0x0D:
      if (*s == 0x0A)
        continue;
      if (flag)
        t = "&#xA;";
      else
        t = "\n";
      break;
#endif
    case '&':
      t = "&amp;";
      break;
    case '<':
      t = "&lt;";
      break;
    case '>':
      if (flag)
        t = ">";
      else
        t = "&gt;";
      break;
    case '"':
      if (flag)
        t = "&quot;";
      else
        t = "\"";
      break;
    default:
      if (c >= 0x20 && c < 0x80)
      {
        tmp = (char)c;
        if (soap_send_raw(soap, &tmp, 1))
          return soap->error;
      }
      else
      {
        /* check for UTF16 encoding when wchar_t is too small to hold UCS */
        if (sizeof(wchar_t) < 4 && (c & 0xFC00) == 0xD800)
        {
          soap_wchar d = *s;
          if ((d & 0xFC00) == 0xDC00)
          {
            c = ((c - 0xD800) << 10) + (d - 0xDC00) + 0x10000;
            s++;
          }
#ifdef WITH_REPLACE_ILLEGAL_UTF8
          else
          {
            c = SOAP_UNKNOWN_UNICODE_CHAR; /* Malformed UTF-16 */
          }
#endif
        }
        if (soap_pututf8(soap, (unsigned long)c))
          return soap->error;
      }
      continue;
    }
    if (soap_send(soap, t))
      return soap->error;
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
wchar_t *
SOAP_FMAC2
soap_wstring_in(struct soap *soap, int flag, long minlen, long maxlen, const char *pattern)
{
  wchar_t *s;
  int i, n = 0, f = 0;
  ULONG64 l = 0;
  soap_wchar c;
  char *t = NULL;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Reading wide string content\n"));
  if (maxlen < 0 && soap->maxlength > 0)
    maxlen = soap->maxlength;
  if (flag <= 0 && soap->peeked && *soap->tag)
  {
#ifndef WITH_LEAN
    struct soap_attribute *tp;
    t = soap->tmpbuf;
    *t = '<';
    soap_strcpy(t + 1, sizeof(soap->tmpbuf) - 1, soap->tag);
    t += strlen(t);
    for (tp = soap->attributes; tp; tp = tp->next)
    {
      if (tp->visible)
      {
        size_t k = strlen(tp->name);
        if (t + k + 1 >= soap->tmpbuf + sizeof(soap->tmpbuf))
          break;
        *t++ = ' ';
        (void)soap_strncpy(t, sizeof(soap->tmpbuf) - (t - soap->tmpbuf), tp->name, k);
        t += k;
        if (tp->value)
        {
          k = strlen(tp->value);
          if (t + k + 3 >= soap->tmpbuf + sizeof(soap->tmpbuf))
            break;
          *t++ = '=';
          *t++ = '"';
          (void)soap_strncpy(t, sizeof(soap->tmpbuf) - (t - soap->tmpbuf), tp->value, k);
          t += k;
          *t++ = '"';
        }
      }
    }
    if (!soap->body)
      *t++ = '/';
    *t++ = '>';
    *t = '\0';
    t = soap->tmpbuf;
#endif
    if (soap->body)
      n = 1;
    f = 1;
    soap->peeked = 0;
  }
  if (soap_alloc_block(soap) == NULL)
    return NULL;
  for (;;)
  {
    s = (wchar_t*)soap_push_block(soap, NULL, sizeof(wchar_t)*SOAP_BLKLEN);
    if (!s)
      return NULL;
    for (i = 0; i < SOAP_BLKLEN; i++)
    {
      if (t)
      {
        *s++ = (wchar_t)*t++;
        if (!*t)
          t = NULL;
        continue;
      }
      c = soap_getutf8(soap);
      switch (c)
      {
      case SOAP_TT:
        if (n == 0)
          goto end;
        n--;
        *s++ = L'<';
        soap_unget(soap, '/');
        break;
      case SOAP_LT:
        if (flag == 3 || (f && n == 0))
          goto end;
        n++;
        *s++ = L'<';
        break;
      case SOAP_GT:
        *s++ = L'>';
        break;
      case SOAP_QT:
        *s++ = L'"';
        break;
      case SOAP_AP:
        *s++ = L'\'';
        break;
      case '/':
        if (n > 0)
        {
          c = soap_getutf8(soap);
          if (c == SOAP_GT)
            n--;
          soap_unget(soap, c);
        }
        *s++ = L'/';
        break;
      case '<':
        if (flag > 0)
        {
          *s++ = L'<';
        }
        else
        {
          *s++ = L'&';
          t = (char*)"lt;";
        }
        break;
      case '>':
        if (flag > 0)
        {
          *s++ = L'>';
        }
        else
        {
          *s++ = (wchar_t)'&';
          t = (char*)"gt;";
        }
        break;
      case '"':
        if (flag > 0)
        {
          *s++ = L'"';
        }
        else
        {
          *s++ = L'&';
          t = (char*)"quot;";
        }
        break;
      default:
        if ((int)c == EOF)
          goto end;
        /* use UTF16 encoding when wchar_t is too small to hold UCS */
        if (sizeof(wchar_t) < 4 && c > 0xFFFF)
        {
          soap_wchar c1, c2;
          c1 = 0xD800 - (0x10000 >> 10) + (c >> 10);
          c2 = 0xDC00 + (c & 0x3FF);
          c = c1;
          soap_unget(soap, c2);
        }
        *s++ = (wchar_t)(c & 0x7FFFFFFF);
      }
      l++;
      if (maxlen >= 0 && l > (size_t)maxlen)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "String too long: maxlen=%ld\n", maxlen));
        soap->error = SOAP_LENGTH;
        return NULL;
      }
    }
  }
end:
  soap_unget(soap, c);
  *s = L'\0';
  soap_size_block(soap, NULL, sizeof(wchar_t) * (i + 1));
  if (minlen > 0 && l < (size_t)minlen)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "String too short: %lu chars, minlen=%ld\n", (unsigned long)l, minlen));
    soap->error = SOAP_LENGTH;
    return NULL;
  }
  s = (wchar_t*)soap_save_block(soap, NULL, NULL, 0);
#ifndef WITH_LEAN
  if (flag >= 4 && s)
    s = soap_wcollapse(soap, s, flag, 1);
#endif
#ifndef WITH_LEANER
  if (pattern && soap->fwvalidate)
  {
    soap->error = soap->fwvalidate(soap, pattern, s);
    if (soap->error)
      return NULL;
  }
#endif
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
    soap->dom->text = soap_wchar2s(soap, s);
#endif
  return s;
}
#endif

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_int2s(struct soap *soap, int n)
{
  return soap_long2s(soap, (long)n);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outint(struct soap *soap, const char *tag, int id, const int *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_long2s(soap, (long)*p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2int(struct soap *soap, const char *s, int *p)
{
  if (s)
  {
    long n;
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
#ifndef WITH_NOIO
#ifndef WITH_LEAN
    soap_reset_errno;
#endif
#endif
    n = soap_strtol(s, &r, 10);
    if (s == r || *r
#ifndef WITH_LEAN
        || n != (int)n
#endif
#ifndef WITH_NOIO
#ifndef WITH_LEAN
        || soap_errno == SOAP_ERANGE
#endif
#endif
    )
      soap->error = SOAP_TYPE;
    *p = (int)n;
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
int *
SOAP_FMAC2
soap_inint(struct soap *soap, const char *tag, int *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":int")
   && soap_match_tag(soap, soap->type, ":short")
   && soap_match_tag(soap, soap->type, ":byte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (int*)soap_id_enter(soap, soap->id, p, t, sizeof(int), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2int(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (int*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(int), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_long2s(struct soap *soap, long n)
{
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 20), "%ld", n);
  return soap->tmpbuf;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outlong(struct soap *soap, const char *tag, int id, const long *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_long2s(soap, *p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2long(struct soap *soap, const char *s, long *p)
{
  if (s)
  {
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
#ifndef WITH_NOIO
#ifndef WITH_LEAN
    soap_reset_errno;
#endif
#endif
    *p = soap_strtol(s, &r, 10);
    if (s == r || *r
#ifndef WITH_NOIO
#ifndef WITH_LEAN
        || soap_errno == SOAP_ERANGE
#endif
#endif
    )
      soap->error = SOAP_TYPE;
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
long *
SOAP_FMAC2
soap_inlong(struct soap *soap, const char *tag, long *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":int")
   && soap_match_tag(soap, soap->type, ":short")
   && soap_match_tag(soap, soap->type, ":byte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (long*)soap_id_enter(soap, soap->id, p, t, sizeof(long), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2long(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (long*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(long), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_LONG642s(struct soap *soap, LONG64 n)
{
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 20), SOAP_LONG_FORMAT, n);
  return soap->tmpbuf;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outLONG64(struct soap *soap, const char *tag, int id, const LONG64 *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_LONG642s(soap, *p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2LONG64(struct soap *soap, const char *s, LONG64 *p)
{
  if (s)
  {
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
#ifndef WITH_NOIO
#ifndef WITH_LEAN
    soap_reset_errno;
#endif
#endif
    *p = soap_strtoll(s, &r, 10);
    if (s == r || *r
#ifndef WITH_NOIO
#ifndef WITH_LEAN
        || soap_errno == SOAP_ERANGE
#endif
#endif
      )
      soap->error = SOAP_TYPE;
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
LONG64 *
SOAP_FMAC2
soap_inLONG64(struct soap *soap, const char *tag, LONG64 *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":integer")
   && soap_match_tag(soap, soap->type, ":positiveInteger")
   && soap_match_tag(soap, soap->type, ":negativeInteger")
   && soap_match_tag(soap, soap->type, ":nonPositiveInteger")
   && soap_match_tag(soap, soap->type, ":nonNegativeInteger")
   && soap_match_tag(soap, soap->type, ":long")
   && soap_match_tag(soap, soap->type, ":int")
   && soap_match_tag(soap, soap->type, ":short")
   && soap_match_tag(soap, soap->type, ":byte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (LONG64*)soap_id_enter(soap, soap->id, p, t, sizeof(LONG64), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2LONG64(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (LONG64*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(LONG64), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_byte2s(struct soap *soap, char n)
{
  return soap_long2s(soap, (long)n);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outbyte(struct soap *soap, const char *tag, int id, const char *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_long2s(soap, (long)*p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2byte(struct soap *soap, const char *s, char *p)
{
  if (s)
  {
    long n;
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
    n = soap_strtol(s, &r, 10);
    if (s == r || *r || n < -128 || n > 127)
      soap->error = SOAP_TYPE;
    *p = (char)n;
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
char *
SOAP_FMAC2
soap_inbyte(struct soap *soap, const char *tag, char *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":byte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (char*)soap_id_enter(soap, soap->id, p, t, sizeof(char), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2byte(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (char*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(char), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_short2s(struct soap *soap, short n)
{
  return soap_long2s(soap, (long)n);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outshort(struct soap *soap, const char *tag, int id, const short *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_long2s(soap, (long)*p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2short(struct soap *soap, const char *s, short *p)
{
  if (s)
  {
    long n;
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
    n = soap_strtol(s, &r, 10);
    if (s == r || *r || n < -32768 || n > 32767)
      soap->error = SOAP_TYPE;
    *p = (short)n;
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
short *
SOAP_FMAC2
soap_inshort(struct soap *soap, const char *tag, short *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":short")
   && soap_match_tag(soap, soap->type, ":byte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (short*)soap_id_enter(soap, soap->id, p, t, sizeof(short), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2short(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (short*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(short), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_float2s(struct soap *soap, float n)
{
#if defined(WITH_C_LOCALE)
# if !defined(WIN32)
  SOAP_LOCALE_T locale;
# endif
#else
  char *s;
#endif
  if (soap_isnan((double)n))
    return "NaN";
  if (soap_ispinff(n))
    return "INF";
  if (soap_isninff(n))
    return "-INF";
#if defined(WITH_C_LOCALE)
# ifdef WIN32
  _sprintf_s_l(soap->tmpbuf, _countof(soap->tmpbuf), soap->float_format, SOAP_LOCALE(soap), n);
# else
  locale = uselocale(SOAP_LOCALE(soap));
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 20), soap->float_format, n);
  uselocale(locale);
# endif
#else
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 20), soap->float_format, n);
  s = strchr(soap->tmpbuf, ',');        /* convert decimal comma to DP */
  if (s)
    *s = '.';
#endif
  return soap->tmpbuf;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outfloat(struct soap *soap, const char *tag, int id, const float *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_float2s(soap, *p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2float(struct soap *soap, const char *s, float *p)
{
  if (s)
  {
    if (!*s)
      return soap->error = SOAP_EMPTY;
    if (!soap_tag_cmp(s, "INF"))
    {
      *p = FLT_PINFTY;
    }
    else if (!soap_tag_cmp(s, "+INF"))
    {
      *p = FLT_PINFTY;
    }
    else if (!soap_tag_cmp(s, "-INF"))
    {
      *p = FLT_NINFTY;
    }
    else if (!soap_tag_cmp(s, "NaN"))
    {
      *p = FLT_NAN;
    }
    else
    {
/* On some systems strtof requires -std=c99 or does not even link: so we try strtod first */
#if defined(WITH_C_LOCALE)
# if defined(HAVE_STRTOD_L)
      char *r;
#  ifdef WIN32
      *p = (float)_strtod_l(s, &r, SOAP_LOCALE(soap));
#  else
      *p = (float)strtod_l(s, &r, SOAP_LOCALE(soap));
#  endif
      if (*r)
        soap->error = SOAP_TYPE;
# elif defined(HAVE_STRTOF_L)
      char *r;
      *p = strtof_l((char*)s, &r, SOAP_LOCALE(soap));
      if (*r)
        soap->error = SOAP_TYPE;
# elif defined(HAVE_SSCANF_L)
      double n;
      if (sscanf_l(s, SOAP_LOCALE(soap), "%lf", &n) != 1)
        soap->error = SOAP_TYPE;
      *p = (float)n;
# elif defined(HAVE_STRTOD)
      char *r;
      SOAP_LOCALE_T locale = uselocale(SOAP_LOCALE(soap));
      *p = (float)strtod((char*)s, &r);
      uselocale(locale);
      if (*r)
        soap->error = SOAP_TYPE;
# elif defined(HAVE_STRTOF)
      char *r;
      SOAP_LOCALE_T locale = uselocale(SOAP_LOCALE(soap));
      *p = strtof((char*)s, &r);
      uselocale(locale);
      if (*r)
        soap->error = SOAP_TYPE;
# elif defined(HAVE_SSCANF)
      double n;
      SOAP_LOCALE_T locale = uselocale(SOAP_LOCALE(soap));
      if (sscanf(s, "%lf", &n) != 1)
        soap->error = SOAP_TYPE;
      uselocale(locale);
      *p = (float)n;
# else
      soap->error = SOAP_TYPE;
# endif
#elif defined(HAVE_STRTOD)
      char *r;
      *p = (float)strtod(s, &r);
      if (*r)
        soap->error = SOAP_TYPE;
#elif defined(HAVE_STRTOF)
      char *r;
      *p = strtof((char*)s, &r);
      if (*r)
        soap->error = SOAP_TYPE;
#elif defined(HAVE_SSCANF)
      double n;
      if (sscanf(s, "%lf", &n) != 1)
        soap->error = SOAP_TYPE;
      *p = (float)n;
#else
      soap->error = SOAP_TYPE;
#endif
    }
  }
  return soap->error;
}

/******************************************************************************/

#ifndef WITH_LEAN
static int soap_isnumeric(struct soap *soap, const char *type)
{
  if (soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":float")
   && soap_match_tag(soap, soap->type, ":double")
   && soap_match_tag(soap, soap->type, ":decimal")
   && soap_match_tag(soap, soap->type, ":integer")
   && soap_match_tag(soap, soap->type, ":positiveInteger")
   && soap_match_tag(soap, soap->type, ":negativeInteger")
   && soap_match_tag(soap, soap->type, ":nonPositiveInteger")
   && soap_match_tag(soap, soap->type, ":nonNegativeInteger")
   && soap_match_tag(soap, soap->type, ":long")
   && soap_match_tag(soap, soap->type, ":int")
   && soap_match_tag(soap, soap->type, ":short")
   && soap_match_tag(soap, soap->type, ":byte")
   && soap_match_tag(soap, soap->type, ":unsignedLong")
   && soap_match_tag(soap, soap->type, ":unsignedInt")
   && soap_match_tag(soap, soap->type, ":unsignedShort")
   && soap_match_tag(soap, soap->type, ":unsignedByte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return SOAP_ERR;
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

SOAP_FMAC1
float *
SOAP_FMAC2
soap_infloat(struct soap *soap, const char *tag, float *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type != '\0' && soap_isnumeric(soap, type))
    return NULL;
#else
  (void)type;
#endif
  p = (float*)soap_id_enter(soap, soap->id, p, t, sizeof(float), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2float(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (float*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(float), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_double2s(struct soap *soap, double n)
{
#if defined(WITH_C_LOCALE)
# if !defined(WIN32)
  SOAP_LOCALE_T locale;
# endif
#else
  char *s;
#endif
  if (soap_isnan(n))
    return "NaN";
  if (soap_ispinfd(n))
    return "INF";
  if (soap_isninfd(n))
    return "-INF";
#if defined(WITH_C_LOCALE)
# ifdef WIN32
  _sprintf_s_l(soap->tmpbuf, _countof(soap->tmpbuf), soap->double_format, SOAP_LOCALE(soap), n);
# else
  locale = uselocale(SOAP_LOCALE(soap));
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 40), soap->double_format, n);
  uselocale(locale);
# endif
#else
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 40), soap->double_format, n);
  s = strchr(soap->tmpbuf, ',');        /* convert decimal comma to DP */
  if (s)
    *s = '.';
#endif
  return soap->tmpbuf;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outdouble(struct soap *soap, const char *tag, int id, const double *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_double2s(soap, *p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2double(struct soap *soap, const char *s, double *p)
{
  if (s)
  {
    if (!*s)
      return soap->error = SOAP_EMPTY;
    if (!soap_tag_cmp(s, "INF"))
    {
      *p = DBL_PINFTY;
    }
    else if (!soap_tag_cmp(s, "+INF"))
    {
      *p = DBL_PINFTY;
    }
    else if (!soap_tag_cmp(s, "-INF"))
    {
      *p = DBL_NINFTY;
    }
    else if (!soap_tag_cmp(s, "NaN"))
    {
      *p = DBL_NAN;
    }
    else
    {
#if defined(WITH_C_LOCALE)
# if defined(HAVE_STRTOD_L)
      char *r;
#  ifdef WIN32
      *p = _strtod_l(s, &r, SOAP_LOCALE(soap));
#  else
      *p = strtod_l(s, &r, SOAP_LOCALE(soap));
#  endif
      if (*r)
        soap->error = SOAP_TYPE;
# elif defined(HAVE_STRTOD)
      char *r;
      SOAP_LOCALE_T locale = uselocale(SOAP_LOCALE(soap));
      *p = strtod(s, &r);
      uselocale(locale);
      if (*r)
        soap->error = SOAP_TYPE;
# elif defined(HAVE_SSCANF_L)
      SOAP_LOCALE_T locale = uselocale(SOAP_LOCALE(soap));
      if (sscanf_l(s, SOAP_LOCALE(soap), "%lf", p) != 1)
        soap->error = SOAP_TYPE;
      uselocale(locale);
# else
      soap->error = SOAP_TYPE;
# endif
#elif defined(HAVE_STRTOD)
      char *r;
      *p = strtod(s, &r);
      if (*r)
        soap->error = SOAP_TYPE;
#elif defined(HAVE_SSCANF)
      if (sscanf(s, "%lf", p) != 1)
        soap->error = SOAP_TYPE;
#else
      soap->error = SOAP_TYPE;
#endif
    }
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
double *
SOAP_FMAC2
soap_indouble(struct soap *soap, const char *tag, double *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type != '\0' && soap_isnumeric(soap, type))
    return NULL;
#else
  (void)type;
#endif
  p = (double*)soap_id_enter(soap, soap->id, p, t, sizeof(double), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2double(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (double*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(double), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_unsignedByte2s(struct soap *soap, unsigned char n)
{
  return soap_unsignedLong2s(soap, (unsigned long)n);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outunsignedByte(struct soap *soap, const char *tag, int id, const unsigned char *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_unsignedLong2s(soap, (unsigned long)*p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2unsignedByte(struct soap *soap, const char *s, unsigned char *p)
{
  if (s)
  {
    long n;
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
    n = soap_strtol(s, &r, 10);
    if (s == r || *r || n < 0 || n > 255)
      soap->error = SOAP_TYPE;
    *p = (unsigned char)n;
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
unsigned char *
SOAP_FMAC2
soap_inunsignedByte(struct soap *soap, const char *tag, unsigned char *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":unsignedByte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (unsigned char*)soap_id_enter(soap, soap->id, p, t, sizeof(unsigned char), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2unsignedByte(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (unsigned char*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(unsigned char), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_unsignedShort2s(struct soap *soap, unsigned short n)
{
  return soap_unsignedLong2s(soap, (unsigned long)n);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outunsignedShort(struct soap *soap, const char *tag, int id, const unsigned short *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_unsignedLong2s(soap, (unsigned long)*p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2unsignedShort(struct soap *soap, const char *s, unsigned short *p)
{
  if (s)
  {
    long n;
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
    n = soap_strtol(s, &r, 10);
    if (s == r || *r || n < 0 || n > 65535)
      soap->error = SOAP_TYPE;
    *p = (unsigned short)n;
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
unsigned short *
SOAP_FMAC2
soap_inunsignedShort(struct soap *soap, const char *tag, unsigned short *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":unsignedShort")
   && soap_match_tag(soap, soap->type, ":unsignedByte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (unsigned short*)soap_id_enter(soap, soap->id, p, t, sizeof(unsigned short), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2unsignedShort(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (unsigned short*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(unsigned short), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_unsignedInt2s(struct soap *soap, unsigned int n)
{
  return soap_unsignedLong2s(soap, (unsigned long)n);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outunsignedInt(struct soap *soap, const char *tag, int id, const unsigned int *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_unsignedLong2s(soap, (unsigned long)*p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2unsignedInt(struct soap *soap, const char *s, unsigned int *p)
{
  if (s)
  {
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
#ifndef WITH_NOIO
#ifndef WITH_LEAN
    soap_reset_errno;
#endif
#endif
    *p = (unsigned int)soap_strtoul(s, &r, 10);
    if (s == r || *r
#ifndef WITH_NOIO
#ifndef WITH_LEAN
        || soap_errno == SOAP_ERANGE
#endif
#endif
    )
      soap->error = SOAP_TYPE;
#ifdef HAVE_STRTOUL
    if (*p > 0 && strchr(s, '-'))
      return soap->error = SOAP_TYPE;
#endif
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
unsigned int *
SOAP_FMAC2
soap_inunsignedInt(struct soap *soap, const char *tag, unsigned int *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":unsignedInt")
   && soap_match_tag(soap, soap->type, ":unsignedShort")
   && soap_match_tag(soap, soap->type, ":unsignedByte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (unsigned int*)soap_id_enter(soap, soap->id, p, t, sizeof(unsigned int), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2unsignedInt(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (unsigned int*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(unsigned int), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_unsignedLong2s(struct soap *soap, unsigned long n)
{
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 20), "%lu", n);
  return soap->tmpbuf;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outunsignedLong(struct soap *soap, const char *tag, int id, const unsigned long *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_unsignedLong2s(soap, *p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2unsignedLong(struct soap *soap, const char *s, unsigned long *p)
{
  if (s)
  {
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
#ifndef WITH_NOIO
#ifndef WITH_LEAN
    soap_reset_errno;
#endif
#endif
    *p = soap_strtoul(s, &r, 10);
    if (s == r || *r
#ifndef WITH_NOIO
#ifndef WITH_LEAN
        || soap_errno == SOAP_ERANGE
#endif
#endif
    )
      soap->error = SOAP_TYPE;
#ifdef HAVE_STRTOUL
    if (*p > 0 && strchr(s, '-'))
      return soap->error = SOAP_TYPE;
#endif
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
unsigned long *
SOAP_FMAC2
soap_inunsignedLong(struct soap *soap, const char *tag, unsigned long *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":unsignedInt")
   && soap_match_tag(soap, soap->type, ":unsignedShort")
   && soap_match_tag(soap, soap->type, ":unsignedByte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (unsigned long*)soap_id_enter(soap, soap->id, p, t, sizeof(unsigned long), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2unsignedLong(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (unsigned long*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(unsigned long), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_ULONG642s(struct soap *soap, ULONG64 n)
{
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 20), SOAP_ULONG_FORMAT, n);
  return soap->tmpbuf;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outULONG64(struct soap *soap, const char *tag, int id, const ULONG64 *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_ULONG642s(soap, *p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2ULONG64(struct soap *soap, const char *s, ULONG64 *p)
{
  if (s)
  {
    char *r;
    if (!*s)
      return soap->error = SOAP_EMPTY;
#ifndef WITH_NOIO
#ifndef WITH_LEAN
    soap_reset_errno;
#endif
#endif
    *p = soap_strtoull(s, &r, 10);
    if (s == r || *r
#ifndef WITH_NOIO
#ifndef WITH_LEAN
        || soap_errno == SOAP_ERANGE
#endif
#endif
      )
      soap->error = SOAP_TYPE;
    if (*p > 0 && strchr(s, '-'))
      return soap->error = SOAP_TYPE;
  }
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
ULONG64 *
SOAP_FMAC2
soap_inULONG64(struct soap *soap, const char *tag, ULONG64 *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
#ifndef WITH_LEAN
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":positiveInteger")
   && soap_match_tag(soap, soap->type, ":nonNegativeInteger")
   && soap_match_tag(soap, soap->type, ":unsignedLong")
   && soap_match_tag(soap, soap->type, ":unsignedInt")
   && soap_match_tag(soap, soap->type, ":unsignedShort")
   && soap_match_tag(soap, soap->type, ":unsignedByte"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
#else
  (void)type;
#endif
  p = (ULONG64*)soap_id_enter(soap, soap->id, p, t, sizeof(ULONG64), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2ULONG64(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (ULONG64*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(ULONG64), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2char(struct soap *soap, const char *s, char **t, int flag, long minlen, long maxlen, const char *pattern)
{
  if (s)
  {
    const char *r = soap_string(soap, s, flag, minlen, maxlen, pattern);
    if (r && (*t = soap_strdup(soap, r)) == NULL)
      return soap->error = SOAP_EOM;
  }
  return soap->error;
}

/******************************************************************************/

#ifndef WITH_COMPAT
#ifdef __cplusplus
SOAP_FMAC1
int
SOAP_FMAC2
soap_s2stdchar(struct soap *soap, const char *s, std::string *t, int flag, long minlen, long maxlen, const char *pattern)
{
  if (s)
  {
    const char *r = soap_string(soap, s, flag, minlen, maxlen, pattern);
    if (r)
      t->assign(r);
  }
  return soap->error;
}
#endif
#endif

/******************************************************************************/

static const char*
soap_string(struct soap *soap, const char *s, int flag, long minlen, long maxlen, const char *pattern)
{
  if (s)
  {
    if (maxlen < 0 && soap->maxlength > 0)
      maxlen = soap->maxlength;
    if (minlen > 0 || maxlen >= 0)
    {
      size_t l;
      if ((soap->mode & SOAP_C_UTFSTRING))
        l = soap_utf8len(s);
      else
        l = strlen(s);
      if ((maxlen >= 0 && l > (size_t)maxlen) || (minlen > 0 && l < (size_t)minlen))
      {
        soap->error = SOAP_LENGTH;
        return NULL;
      }
    }
    if (flag >= 4)
      s = soap_collapse(soap, (char*)s, flag, 0);
#ifndef WITH_LEANER
    if (pattern && soap->fsvalidate)
    {
      soap->error = soap->fsvalidate(soap, pattern, s);
      if (soap->error)
        return NULL;
    }
#else
    (void)pattern;
#endif
  }
  return s;
}

/******************************************************************************/

static char*
soap_collapse(struct soap *soap, char *s, int flag, int insitu)
{
  /* flag 4=normalizedString (replace), 5=token (collapse) */
  char *t;
  size_t n;
  if (!s)
    return NULL;
  if (flag == 4)
  {
    for (t = s; *t && (!soap_coblank((soap_wchar)*t) || *t == 32); t++)
      continue;
    if (*t)
    {
      /* replace white space and control chars by blanks */
      if (!insitu)
        s = soap_strdup(soap, s);
      for (t = s; *t; t++)
        if (soap_coblank((soap_wchar)*t))
          *t = ' ';
    }
    return s;
  }
  /* collapse white space */
  for (t = s; *t && soap_coblank((soap_wchar)*t); t++)
    continue;
  n = strlen(t);
  if (insitu && s < t)
    (void)soap_memmove(s, n + 1, t, n + 1);
  else
    s = t;
  if (n > 0)
  {
    if (!soap_coblank((soap_wchar)s[n-1]))
    {
      for (t = s; (*t && !soap_coblank((soap_wchar)*t)) || (*t == 32 && (!t[1] || !soap_coblank((soap_wchar)t[1]))); t++)
        continue;
      if (!*t)
        return s;
    }
    if (!insitu)
      s = soap_strdup(soap, s);
    for (t = s; *t; t++)
    {
      if (soap_coblank((soap_wchar)*t))
      {
        char *r;
        *t = ' ';
        for (r = t + 1; *r && soap_coblank((soap_wchar)*r); r++)
          continue;
        if (r > t + 1)
          (void)soap_memmove(t + 1, n - (t-s), r, n - (r-s) + 1);
      }
    }
    t--;
    if (t >= s && *t == 32)
      *t = '\0';
  }
  return s;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_s2QName(struct soap *soap, const char *s, char **t, long minlen, long maxlen, const char *pattern)
{
  *t = NULL;
  if (s)
  {
    const char *r = soap_QName(soap, s, minlen, maxlen, pattern);
    if (r && (*t = soap_strdup(soap, r)) == NULL)
      return soap->error = SOAP_EOM;
  }
  return soap->error;
}

/******************************************************************************/

#ifndef WITH_COMPAT
#ifdef __cplusplus
SOAP_FMAC1
int
SOAP_FMAC2
soap_s2stdQName(struct soap *soap, const char *s, std::string *t, long minlen, long maxlen, const char *pattern)
{
  t->clear();
  if (s)
  {
    const char *r = soap_QName(soap, s, minlen, maxlen, pattern);
    if (r)
      t->assign(r);
  }
  return soap->error;
}
#endif
#endif

/******************************************************************************/

static const char*
soap_QName(struct soap *soap, const char *s, long minlen, long maxlen, const char *pattern)
{
  if (s)
  {
    char *b;
    if (maxlen < 0 && soap->maxlength > 0)
      maxlen = soap->maxlength;
    if (minlen > 0 || maxlen >= 0)
    {
      size_t l;
      if ((soap->mode & SOAP_C_UTFSTRING))
        l = soap_utf8len(s);
      else
        l = strlen(s);
      if ((maxlen >= 0 && l > (size_t)maxlen) || (minlen > 0 && l < (size_t)minlen))
      {
        soap->error = SOAP_LENGTH;
        return NULL;
      }
    }
#ifdef WITH_FAST
    soap->labidx = 0;
#else
    if (soap_alloc_block(soap) == NULL)
      return NULL;
#endif
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Normalized namespace(s) of QNames '%s'", s));
    /* convert (by prefix normalize prefix) all QNames in s */
    for (;;)
    {
      size_t n;
      struct soap_nlist *np;
      const char *p = NULL;
      short flag = 0;
      const char *r = NULL;
      size_t m = 0;
#ifndef WITH_FAST
      size_t k = 0;
#endif
      /* skip blanks */
      while (*s && soap_coblank((soap_wchar)*s))
        s++;
      if (!*s)
        break;
      /* find next QName */
      n = 1;
      while (s[n] && !soap_coblank((soap_wchar)s[n]))
        n++;
      np = soap->nlist;
      /* if there is no namespace stack, or prefix is "#" or "xml" then copy string */
      if (!np || *s == '#' || !strncmp(s, "xml:", 4))
      {
        r = s;
        m = n;
      }
      else /* we normalize the QName by replacing its prefix */
      {
        const char *q;
        for (p = s; *p && p < s + n; p++)
          if (*p == ':')
            break;
        if (*p == ':')
        {
          size_t k = p - s;
          while (np && (strncmp(np->id, s, k) || np->id[k]))
            np = np->next;
          p++;
        }
        else
        {
          while (np && *np->id)
            np = np->next;
          p = s;
        }
        /* replace prefix */
        if (np)
        {
          if (np->index >= 0 && soap->local_namespaces && (q = soap->local_namespaces[np->index].id) != NULL)
          {
            size_t k = strlen(q);
            if (q[k-1] != '_')
            {
              r = q;
              m = k;
            }
            else
            {
              flag = 1; 
              r = soap->local_namespaces[np->index].ns;
              m = strlen(r);
            }
          }
          else if (np->ns)
          {
            flag = 1;
            r = np->ns;
            m = strlen(r);
          }
          else
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\nNamespace prefix of '%s' not defined (index=%d, URI='%s')\n", s, np->index, np->ns ? np->ns : SOAP_STR_EOS));
            soap->error = SOAP_NAMESPACE;
            return NULL;
          }
        }
        else if (s[n]) /* no namespace, part of string */
        {
          r = s;
          m = n;
        }
        else /* no namespace: assume "" namespace */
        {
          flag = 1;
        }
      }
#ifdef WITH_FAST
      if ((flag && soap_append_lab(soap, "\"", 1))
       || (m && soap_append_lab(soap, r, m))
       || (flag && soap_append_lab(soap, "\"", 1))
       || (p && (soap_append_lab(soap, ":", 1) || soap_append_lab(soap, p, n - (p-s)))))
         return NULL;
#else
      k = 2*flag + m + (p ? n - (p-s) + 1 : 0) + (s[n] != '\0');
      b = (char*)soap_push_block(soap, NULL, k);
      if (!b)
        return NULL;
      if (flag)
        *b++ = '"';
      if (m)
      {
        if (soap_memcpy((void*)b, k, (const void*)r, m))
        {
          soap->error = SOAP_EOM;
          return NULL;
        }
        b += m;
      }
      if (flag)
        *b++ = '"';
      if (p)
      {
        *b++ = ':';
        if (soap_memcpy((void*)b, k - m - flag - 1, (const void*)p, n - (p-s)))
        {
          soap->error = SOAP_EOM;
          return NULL;
        }
        b += n - (p-s);
      }
#endif
      /* advance to next and add spacing */
      s += n;
      while (*s && soap_coblank(*s))
        s++;
      if (*s)
      {
#ifdef WITH_FAST
        if (soap_append_lab(soap, " ", 1))
          return NULL;
#else
        *b = ' ';
#endif
      }
    }
#ifdef WITH_FAST
    if (soap_append_lab(soap, SOAP_STR_EOS, 1))
      return NULL;
    b = soap->labbuf;
#else
    b = (char*)soap_push_block(soap, NULL, 1);
    if (!b)
      return NULL;
    *b = '\0';
    b = (char*)soap_save_block(soap, NULL, NULL, 0);
#endif
#ifndef WITH_LEANER
    if (pattern && soap->fsvalidate)
    {
      soap->error = soap->fsvalidate(soap, pattern, b);
      if (soap->error)
        return NULL;
    }
#else
    (void)pattern;
#endif
    return b;
  }
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_QName2s(struct soap *soap, const char *s)
{
  const char *t = NULL;
  if (s)
  {
#ifdef WITH_FAST
    soap_store_lab(soap, SOAP_STR_EOS, 1);
    soap->labidx = 0;
#else
    char *b = NULL;
    if (soap_alloc_block(soap) == NULL)
      return NULL;
#endif
    for (;;)
    {
      size_t n;
      const char *q = NULL;
      const char *r = NULL;
      size_t m = 0;
#ifndef WITH_FAST
      size_t k = 0;
#endif
      /* skip blanks */
      while (*s && soap_coblank((soap_wchar)*s))
        s++;
      if (!*s)
      {
#ifdef WITH_FAST
        soap->labbuf[soap->labidx > 0 ? soap->labidx - 1 : 0] = '\0';
#else
        if (!b)
          return soap_strdup(soap, SOAP_STR_EOS);
        --b;
        *b = '\0';
#endif
        break;
      }
      /* find next QName */
      n = 0;
      while (s[n] && !soap_coblank((soap_wchar)s[n]))
      {
        if (s[n] == ':')
          r = s;
        n++;
      }
      if (*s != '"') /* non-quoted: pass string as is */
      {
#ifndef WITH_LEAN
        if (r && (soap->mode & SOAP_XML_CANONICAL) && !(soap->mode & SOAP_XML_CANONICAL_NA))
          soap_utilize_ns(soap, s, 1);
#endif
        r = s;
        m = n + 1;
      }
      else /* prefix quoted URI-based string */
      {
        q = strchr(s + 1, '"');
        if (q)
        {
          struct Namespace *p = soap->local_namespaces;
          if (p)
          {
            for (; p->id; p++)
            {
              if (p->ns)
                if (!soap_tag_cmp(s + 1, p->ns))
                  break;
              if (p->in)
                if (!soap_tag_cmp(s + 1, p->in))
                  break;
            }
          }
          q++;
          /* URL is in the namespace table? */
          if (p && p->id)
          {
            r = p->id;
            m = strlen(r);
          }
          else /* not in namespace table: create xmlns binding */
          {
            char *x = soap_strdup(soap, s + 1);
            if (!x)
              return NULL;
            x[q - s - 2] = '\0';
            (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 27), "xmlns:_%d", soap->idnum++);
            soap_set_attr(soap, soap->tmpbuf, x, 1);
            r = soap->tmpbuf + 6;
            m = strlen(r);
          }
        }
      }
      /* copy normalized QName into buffer, including the ending blank or NUL */
#ifdef WITH_FAST
      if ((m && soap_append_lab(soap, r, m))
       || (q && soap_append_lab(soap, q, n - (q - s) + 1)))
        return NULL;
#else
      k = m + (q ? n - (q - s) + 1 : 0);
      b = (char*)soap_push_block(soap, NULL, k);
      if (!b)
      {
        soap->error = SOAP_EOM;
        return NULL;
      }
      if (soap_memcpy((void*)b, k, (const void*)r, m))
      {
        soap->error = SOAP_EOM;
        return NULL;
      }
      b += m;
      if (q)
      {
        if (soap_memcpy((void*)b, k - m, (const void*)q, n - (q - s) + 1))
        {
          soap->error = SOAP_EOM;
          return NULL;
        }
        b += n - (q - s) + 1;
      }
#endif
      /* advance to next */
      s += n;
    }
#ifdef WITH_FAST
    t = soap_strdup(soap, soap->labbuf);
    if (!t)
      soap->error = SOAP_EOM;
#else
    t = (char*)soap_save_block(soap, NULL, NULL, 0);
#endif
  }
  return t;
}

/******************************************************************************/

#ifndef WITH_LEAN
SOAP_FMAC1
int
SOAP_FMAC2
soap_s2wchar(struct soap *soap, const char *s, wchar_t **t, int flag, long minlen, long maxlen, const char *pattern)
{
  if (s)
  {
    const wchar_t *r = soap_wstring(soap, s, flag, minlen, maxlen, pattern);
    if (r && (*t = soap_wstrdup(soap, r)) == NULL)
      return soap->error = SOAP_EOM;
  }
  return soap->error;
}
#endif

/******************************************************************************/

#ifndef WITH_COMPAT
#ifdef __cplusplus
#ifndef WITH_LEAN
SOAP_FMAC1
int
SOAP_FMAC2
soap_s2stdwchar(struct soap *soap, const char *s, std::wstring *t, int flag, long minlen, long maxlen, const char *pattern)
{
  if (s)
  {
    const wchar_t *r = soap_wstring(soap, s, flag, minlen, maxlen, pattern);
    if (r)
      t->assign(r);
  }
  return soap->error;
}
#endif
#endif
#endif

/******************************************************************************/

#ifndef WITH_LEAN
static const wchar_t*
soap_wstring(struct soap *soap, const char *s, int flag, long minlen, long maxlen, const char *pattern)
{
  if (s)
  {
    size_t l;
    soap_wchar c;
    wchar_t *t;
    if (maxlen < 0 && soap->maxlength > 0)
      maxlen = soap->maxlength;
    soap->labidx = 0;
    if ((soap->mode & SOAP_ENC_LATIN))
    {
      wchar_t *r;
      if (soap_append_lab(soap, NULL, sizeof(wchar_t) * (strlen(s) + 1)))
        return NULL;
      r = (wchar_t*)(void*)soap->labbuf;
      while (*s)
        *r++ = (wchar_t)*s++;
    }
    else
    {
      /* Convert UTF8 to wchar_t */
      while (*s)
      {
        c = (unsigned char)*s++;
        if (c >= 0x80)
        {
#ifdef WITH_REPLACE_ILLEGAL_UTF8
          soap_wchar c1, c2, c3;
          c1 = (unsigned char)*s;
          if (c <= 0xC1 || (c1 & 0xC0) != 0x80)
          {
            c = SOAP_UNKNOWN_UNICODE_CHAR;
          }
          else
          {
            ++s;
            c1 &= 0x3F;
            if (c < 0xE0)
            {
              c = (((c & 0x1F) << 6) | c1);
            }
            else
            {
              c2 = (unsigned char)*s;
              if ((c == 0xE0 && c1 < 0x20) || (c2 & 0xC0) != 0x80)
              {
                c = SOAP_UNKNOWN_UNICODE_CHAR;
              }
              else
              {
                ++s;
                c2 &= 0x3F;
                if (c < 0xF0)
                {
                  c = (((c & 0x0F) << 12) | (c1 << 6) | c2);
                }
                else
                {
                  c3 = (unsigned char)*s;
                  if ((c == 0xF0 && c1 < 0x10) || (c == 0xF4 && c1 >= 0x10) || c >= 0xF5 || (c3 & 0xC0) != 0x80)
                  {
                    c = SOAP_UNKNOWN_UNICODE_CHAR;
                  }
                  else
                  {
                    ++s;
                    c = (((c & 0x07) << 18) | (c1 << 12) | (c2 << 6) | (c3 & 0x3F));
                  }
                }
              }
            }
          }
#else
          soap_wchar c1, c2, c3, c4;
          c1 = (unsigned char)*s;
          if (c1)
          {
            s++;
            c1 &= 0x3F;
            if (c < 0xE0)
            {
              c = (wchar_t)(((soap_wchar)(c & 0x1F) << 6) | c1);
            }
            else
            {
              c2 = (unsigned char)*s;
              if (c2)
              {
                s++;
                c2 &= 0x3F;
                if (c < 0xF0)
                {
                  c = (wchar_t)(((soap_wchar)(c & 0x0F) << 12) | (c1 << 6) | c2);
                }
                else
                {
                  c3 = (unsigned char)*s;
                  if (c3)
                  {
                    s++;
                    c3 &= 0x3F;
                    if (c < 0xF8)
                    {
                      c = (wchar_t)(((soap_wchar)(c & 0x07) << 18) | (c1 << 12) | (c2 << 6) | c3);
                    }
                    else
                    {
                      c4 = (unsigned char)*s;
                      if (c4)
                      {
                        s++;
                        c4 &= 0x3F;
                        if (c < 0xFC)
                        {
                          c = (wchar_t)(((soap_wchar)(c & 0x03) << 24) | (c1 << 18) | (c2 << 12) | (c3 << 6) | c4);
                        }
                        else
                        {
                          c = (wchar_t)(((soap_wchar)(c & 0x01) << 30) | (c1 << 24) | (c2 << 18) | (c3 << 12) | (c4 << 6) | (unsigned char)(*s & 0x3F));
                          if (*s)
                            s++;
                        }
                      }
                    }
                  }
                }
              }
            }
          }
#endif
        }
        /* use UTF16 encoding when wchar_t is too small to hold UCS */
        if (sizeof(wchar_t) < 4 && c > 0xFFFF)
        {
          wchar_t c1, c2;
          c1 = 0xD800 - (0x10000 >> 10) + (c >> 10);
          c2 = 0xDC00 + (c & 0x3FF);
          if (soap_append_lab(soap, (const char*)&c1, sizeof(wchar_t)) || soap_append_lab(soap, (const char*)&c2, sizeof(wchar_t)))
            return NULL;
        }
        else if (soap_append_lab(soap, (const char*)&c, sizeof(wchar_t)))
        {
          return NULL;
        }
      }
    }
    l = soap->labidx / sizeof(wchar_t);
    c = L'\0';
    if (soap_append_lab(soap, (const char*)&c, sizeof(wchar_t)))
      return NULL;
    if ((maxlen >= 0 && l > (size_t)maxlen) || (minlen > 0 && l < (size_t)minlen))
    {
      soap->error = SOAP_LENGTH;
      return NULL;
    }
    t = (wchar_t*)(void*)soap->labbuf;
#ifndef WITH_LEAN
    if (flag >= 4 && t)
      t = soap_wcollapse(soap, t, flag, 1);
#endif
#ifndef WITH_LEANER
    if (pattern && soap->fwvalidate)
    {
      soap->error = soap->fwvalidate(soap, pattern, t);
      if (soap->error)
        return NULL;
    }
#endif
    return t;
  }
  return NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_LEAN
static wchar_t*
soap_wcollapse(struct soap *soap, wchar_t *s, int flag, int insitu)
{
  /* flag 4=normalizedString (replace), 5=token (collapse) */
  wchar_t *t;
  size_t n;
  if (!s)
    return NULL;
  if (flag == 4)
  {
    for (t = s; *t && (!soap_coblank((soap_wchar)*t) || *t == 32); t++)
      continue;
    if (*t)
    {
      /* replace blanks and control char by space */
      if (!insitu)
        s = soap_wstrdup(soap, s);
      if (s)
        for (t = s; *t; t++)
          if (soap_coblank((soap_wchar)*t))
            *t = L' ';
    }
    return s;
  }
  /* collapse white space */
  for (t = s; *t && soap_coblank((soap_wchar)*t); t++)
    continue;
  n = 0;
  while (t[n])
    n++;
  if (insitu && s < t)
    (void)soap_memmove(s, n + 1, t, n + 1);
  else
    s = t;
  if (n > 0)
  {
    if (!soap_coblank((soap_wchar)s[n-1]))
    {
      for (t = s; (*t && !soap_coblank((soap_wchar)*t)) || (*t == 32 && (!t[1] || !soap_coblank((soap_wchar)t[1]))); t++)
        continue;
      if (!*t)
        return s;
    }
    if (!insitu)
      s = soap_wstrdup(soap, s);
    if (s)
    {
      for (t = s; *t; t++)
      {
        if (soap_coblank((soap_wchar)*t))
        {
          wchar_t *r;
          *t = L' ';
          for (r = t + 1; *r && soap_coblank((soap_wchar)*r); r++)
            continue;
          if (r > t + 1)
            (void)soap_memmove(t + 1, sizeof(wchar_t) * (n - (t-s)), r, sizeof(wchar_t) * (n - (r-s) + 1));
        }
      }
      t--;
      if (t >= s && *t == 32)
        *t = L'\0';
    }
  }
  return s;
}
#endif

/******************************************************************************/

#ifndef WITH_LEAN
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_wchar2s(struct soap *soap, const wchar_t *s)
{
  soap_wchar c;
  char *r, *t;
  const wchar_t *q = s;
  size_t n = 0;
  if (!s)
    return NULL;
  while ((c = *q++))
  {
    if (c > 0 && c < 0x80)
      n++;
    else
#ifdef WITH_REPLACE_ILLEGAL_UTF8
      n += 4;
#else
      n += 6;
#endif
  }
  r = t = (char*)soap_malloc(soap, n + 1);
  if (r)
  {
    /* Convert wchar to UTF8 (chars above U+10FFFF are silently converted, but should not be used) */
    while ((c = *s++))
    {
      if (c > 0 && c < 0x80)
      {
        *t++ = (char)c;
      }
      else
      {
        /* check for UTF16 encoding when wchar_t is too small to hold UCS */
        if (sizeof(wchar_t) < 4 && (c & 0xFC00) == 0xD800)
        {
          soap_wchar d = *s;
          if ((d & 0xFC00) == 0xDC00)
          {
            c = ((c - 0xD800) << 10) + (d - 0xDC00) + 0x10000;
            s++;
          }
#ifdef WITH_REPLACE_ILLEGAL_UTF8
          else
          {
            c = SOAP_UNKNOWN_UNICODE_CHAR; /* Malformed UTF-16 */
          }
#endif
        }
        if (c < 0x0800)
        {
          *t++ = (char)(0xC0 | ((c >> 6) & 0x1F));
        }
        else
        {
#ifdef WITH_REPLACE_ILLEGAL_UTF8
          if (!((c >= 0x80 && c <= 0xD7FF) || (c >= 0xE000 && c <= 0xFFFD) || (c >= 0x10000 && c <= 0x10FFFF)))
            c = SOAP_UNKNOWN_UNICODE_CHAR;
#endif
          if (c < 0x010000)
          {
            *t++ = (char)(0xE0 | ((c >> 12) & 0x0F));
          }
          else
          {
            if (c < 0x200000)
            {
              *t++ = (char)(0xF0 | ((c >> 18) & 0x07));
            }
            else
            {
              if (c < 0x04000000)
              {
                *t++ = (char)(0xF8 | ((c >> 24) & 0x03));
              }
              else
              {
                *t++ = (char)(0xFC | ((c >> 30) & 0x01));
                *t++ = (char)(0x80 | ((c >> 24) & 0x3F));
              }
              *t++ = (char)(0x80 | ((c >> 18) & 0x3F));
            }
            *t++ = (char)(0x80 | ((c >> 12) & 0x3F));
          }
          *t++ = (char)(0x80 | ((c >> 6) & 0x3F));
        }
        *t++ = (char)(0x80 | (c & 0x3F));
      }
    }
    *t = '\0';
  }
  return r;
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outstring(struct soap *soap, const char *tag, int id, char *const*p, const char *type, int n)
{
  id = soap_element_id(soap, tag, id, *p, NULL, 0, type, n, NULL);
  if (id < 0)
    return soap->error;
  if (!**p)
  {
    if ((soap->mode & SOAP_C_NILSTRING))
      return soap_element_null(soap, tag, id, type);
    return soap_element_empty(soap, tag, id, type);
  }
  if (soap_element_begin_out(soap, tag, id, type)
   || soap_string_out(soap, *p, 0)
   || soap_element_end_out(soap, tag))
    return soap->error;
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
char **
SOAP_FMAC2
soap_instring(struct soap *soap, const char *tag, char **p, const char *type, int t, int flag, long minlen, long maxlen, const char *pattern)
{
  (void)type;
  if (soap_element_begin_in(soap, tag, 1, NULL))
  {
    if (!tag || *tag != '-' || soap->error != SOAP_NO_TAG)
      return NULL;
    soap->error = SOAP_OK;
  }
  if (!p)
  {
    p = (char**)soap_malloc(soap, sizeof(char*));
    if (!p)
      return NULL;
  }
  if (soap->null)
  {
    *p = NULL;
  }
  else if (soap->body)
  {
    *p = soap_string_in(soap, flag, minlen, maxlen, pattern);
    if (!*p || !(char*)soap_id_enter(soap, soap->id, *p, t, sizeof(char*), NULL, NULL, NULL, NULL))
      return NULL;
    if (!**p && tag && *tag == '-')
    {
      soap->error = SOAP_NO_TAG;
      return NULL;
    }
  }
  else if (tag && *tag == '-')
  {
    soap->error = SOAP_NO_TAG;
    return NULL;
  }
  else if (*soap->href != '#')
  {
    if (minlen > 0)
    {
      soap->error = SOAP_LENGTH;
      return NULL;
    }
    *p = soap_strdup(soap, SOAP_STR_EOS);
    if (!*p)
      return NULL;
  }
  if (*soap->href == '#')
    p = (char**)soap_id_lookup(soap, soap->href, (void**)p, t, sizeof(char**), 0, NULL);
  if (soap->body && soap_element_end_in(soap, tag))
    return NULL;
  return p;
}

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_outwstring(struct soap *soap, const char *tag, int id, wchar_t *const*p, const char *type, int n)
{
  id = soap_element_id(soap, tag, id, *p, NULL, 0, type, n, NULL);
  if (id < 0)
    return soap->error;
  if (!**p)
  {
    if ((soap->mode & SOAP_C_NILSTRING))
      return soap_element_null(soap, tag, id, type);
    return soap_element_empty(soap, tag, id, type);
  }
  if (soap_element_begin_out(soap, tag, id, type)
   || soap_wstring_out(soap, *p, 0)
   || soap_element_end_out(soap, tag))
    return soap->error;
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
wchar_t **
SOAP_FMAC2
soap_inwstring(struct soap *soap, const char *tag, wchar_t **p, const char *type, int t, int flag, long minlen, long maxlen, const char *pattern)
{
  (void)type;
  if (soap_element_begin_in(soap, tag, 1, NULL))
  {
    if (!tag || *tag != '-' || soap->error != SOAP_NO_TAG)
      return NULL;
    soap->error = SOAP_OK;
  }
  if (!p)
  {
    p = (wchar_t**)soap_malloc(soap, sizeof(wchar_t*));
    if (!p)
      return NULL;
  }
  if (soap->null)
  {
    *p = NULL;
  }
  else if (soap->body)
  {
    *p = soap_wstring_in(soap, flag, minlen, maxlen, pattern);
    if (!*p || !(wchar_t*)soap_id_enter(soap, soap->id, *p, t, sizeof(wchar_t*), NULL, NULL, NULL, NULL))
      return NULL;
    if (!**p && tag && *tag == '-')
    {
      soap->error = SOAP_NO_TAG;
      return NULL;
    }
  }
  else if (tag && *tag == '-')
  {
    soap->error = SOAP_NO_TAG;
    return NULL;
  }
  else if (*soap->href != '#')
  {
    if (minlen > 0)
    {
      soap->error = SOAP_LENGTH;
      return NULL;
    }
    *p = soap_wstrdup(soap, L"");
  }
  if (*soap->href == '#')
    p = (wchar_t**)soap_id_lookup(soap, soap->href, (void**)p, t, sizeof(wchar_t**), 0, NULL);
  if (soap->body && soap_element_end_in(soap, tag))
    return NULL;
  return p;
}
#endif

/******************************************************************************/

#ifndef WITH_LEAN
#ifdef UNDER_CE
/* WinCE mktime (based on the mingw-runtime, public domain) */
#define __FILETIME_to_ll(f) ((long long)(f).dwHighDateTime << 32 | (long long)(f).dwLowDateTime)
static time_t
mktime(struct tm *pt)
{
  SYSTEMTIME s, s1, s2;
  FILETIME f, f1, f2;
  long long diff;
  GetSystemTime(&s1);
  GetLocalTime(&s2);
  SystemTimeToFileTime(&s1, &f1);
  SystemTimeToFileTime(&s2, &f2);
  diff = (__FILETIME_to_ll(f2) - __FILETIME_to_ll(f1)) / 10000000LL;
  s.wYear = pt->tm_year + 1900;
  s.wMonth = pt->tm_mon  + 1;
  s.wDayOfWeek = pt->tm_wday;
  s.wDay = pt->tm_mday;
  s.wHour = pt->tm_hour;
  s.wMinute = pt->tm_min;
  s.wSecond = pt->tm_sec;
  s.wMilliseconds = 0;
  SystemTimeToFileTime(&s, &f);
  return (time_t)((__FILETIME_to_ll(f) - 116444736000000000LL) / 10000000LL) - (time_t)diff;
}
#endif
#endif

/******************************************************************************/

#ifndef WITH_LEAN
#ifdef UNDER_CE
/* WinCE gmtime_r (based on the mingw-runtime, public domain) */
#define HAVE_GMTIME_R
static struct tm*
gmtime_r(const time_t *t, struct tm *pt)
{
  FILETIME f, f1, f2;
  SYSTEMTIME s, s1 = {0};
  long long time = (long long)(*t) * 10000000LL + 116444736000000000LL;
  f.dwHighDateTime = (DWORD)((time >> 32) & 0x00000000FFFFFFFF);
  f.dwLowDateTime = (DWORD)(time & 0x00000000FFFFFFFF);
  FileTimeToSystemTime(&f, &s);
  pt->tm_year = s.wYear - 1900;
  pt->tm_mon = s.wMonth - 1;
  pt->tm_wday = s.wDayOfWeek;
  pt->tm_mday = s.wDay;
  s1.wYear = s.wYear;
  s1.wMonth = 1;
  s1.wDayOfWeek = 1;
  s1.wDay = 1;
  SystemTimeToFileTime(&s1, &f1);
  SystemTimeToFileTime(&s, &f2);
  pt->tm_yday = (((__FILETIME_to_ll(f2) - __FILETIME_to_ll(f1)) / 10000000LL) / (60 * 60 * 24));
  pt->tm_hour = s.wHour;
  pt->tm_min = s.wMinute;
  pt->tm_sec = s.wSecond;
  pt->tm_isdst = 0;
  return pt;
}
#endif
#endif

/******************************************************************************/

#ifndef WITH_LEAN
#ifdef UNDER_CE
/* WinCE very simple strftime for format "%Y-%m-%dT%H:%M:%SZ", note: %F and %T not supported by MS */
static size_t
strftime(char *buf, size_t len, const char *format, const struct tm *pT)
{
  (void)len; (void)format;
#ifndef WITH_NOZONE
  (SOAP_SNPRINTF(buf, len, 20), "%04d-%02d-%02dT%02d:%02d:%02dZ", pT->tm_year + 1900, pT->tm_mon + 1, pT->tm_mday, pT->tm_hour, pT->tm_min, pT->tm_sec);
#else
  (SOAP_SNPRINTF(buf, len, 20), "%04d-%02d-%02dT%02d:%02d:%02d", pT->tm_year + 1900, pT->tm_mon + 1, pT->tm_mday, pT->tm_hour, pT->tm_min, pT->tm_sec);
#endif
  return len;
}
#endif
#endif

/******************************************************************************/

#if !defined(WITH_LEAN) || defined(WITH_COOKIES)
SOAP_FMAC1
time_t
SOAP_FMAC2
soap_timegm(struct tm *T)
{
#if defined(HAVE_TIMEGM)
  return timegm(T);
#else
  time_t t, g, z;
  struct tm tm;
#ifndef HAVE_GMTIME_R
  struct tm *tp;
#endif
  t = mktime(T);
  if (t == (time_t)-1)
    return (time_t)-1;
#ifdef HAVE_GMTIME_R
  if (gmtime_r(&t, &tm) == SOAP_FUNC_R_ERR)
    return (time_t)-1;
#else
  tp = gmtime(&t);
  if (!tp)
    return (time_t)-1;
  tm = *tp;
#endif
  tm.tm_isdst = 0;
  g = mktime(&tm);
  if (g == (time_t)-1)
    return (time_t)-1;
  z = g - t;
  return t - z;
#endif
}
#endif

/******************************************************************************/

#ifndef WITH_LEAN
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_dateTime2s(struct soap *soap, time_t n)
{
  struct tm T, *pT = &T;
  size_t l = 0;
#if defined(HAVE_GMTIME_R) && !defined(WITH_NOZONE)
  if (gmtime_r(&n, pT) != SOAP_FUNC_R_ERR)
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%SZ", pT);
#elif defined(HAVE_GMTIME) && !defined(WITH_NOZONE)
  pT = gmtime(&n);
  if (pT)
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%SZ", pT);
#elif (defined(HAVE_TM_GMTOFF) || defined(HAVE_STRUCT_TM_TM_GMTOFF) || defined(HAVE_STRUCT_TM___TM_GMTOFF)) && !defined(WITH_NOZONE)
#if defined(HAVE_LOCALTIME_R)
  if (localtime_r(&n, pT) != SOAP_FUNC_R_ERR)
  {
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%S%z", pT);
    if (l)
    {
      (void)soap_memmove(soap->tmpbuf + 23, sizeof(soap->tmpbuf) - 23, soap->tmpbuf + 22, 3); /* 2000-03-01T02:00:00+0300 */
      soap->tmpbuf[22] = ':';                                                           /* 2000-03-01T02:00:00+03:00 */
    }
  }
#else
  pT = localtime(&n);
  if (pT)
  {
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%S%z", pT);
    if (l)
    {
      (void)soap_memmove(soap->tmpbuf + 23, sizeof(soap->tmpbuf) - 23, soap->tmpbuf + 22, 3); /* 2000-03-01T02:00:00+0300 */
      soap->tmpbuf[22] = ':';                                                           /* 2000-03-01T02:00:00+03:00 */
    }
  }
#endif
#elif defined(HAVE_GETTIMEOFDAY) && !defined(WITH_NOZONE)
#if defined(HAVE_LOCALTIME_R)
  if (localtime_r(&n, pT) != SOAP_FUNC_R_ERR)
  {
    struct timeval tv;
    struct timezone tz;
    memset((void*)&tz, 0, sizeof(tz));
    gettimeofday(&tv, &tz);
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%S", pT);
    if (l)
      (SOAP_SNPRINTF(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, 7), "%+03d:%02d", -tz.tz_minuteswest/60+(pT->tm_isdst!=0), abs(tz.tz_minuteswest)%60);
  }
#else
  pT = localtime(&n);
  if (pT)
  {
    struct timeval tv;
    struct timezone tz;
    memset((void*)&tz, 0, sizeof(tz));
    gettimeofday(&tv, &tz);
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%S", pT);
    if (l)
      (SOAP_SNPRINTF(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, 7), "%+03d:%02d", -tz.tz_minuteswest/60+(pT->tm_isdst!=0), abs(tz.tz_minuteswest)%60);
  }
#endif
#elif defined(HAVE_FTIME) && !defined(WITH_NOZONE)
#if defined(HAVE_LOCALTIME_R)
  if (localtime_r(&n, pT) != SOAP_FUNC_R_ERR)
  {
    struct timeb t;
    memset((void*)&t, 0, sizeof(t));
#ifdef __BORLANDC__
    ::ftime(&t);
#else
    ftime(&t);
#endif
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%S", pT);
    if (l)
      (SOAP_SNPRINTF(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, 7), "%+03d:%02d", -t.timezone/60+(pT->tm_isdst!=0), abs(t.timezone)%60);
  }
#else
  pT = localtime(&n);
  if (pT)
  {
    struct timeb t;
    memset((void*)&t, 0, sizeof(t));
#ifdef __BORLANDC__
    ::ftime(&t);
#else
    ftime(&t);
#endif
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%S", pT);
    if (l)
      (SOAP_SNPRINTF(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, 7), "%+03d:%02d", -t.timezone/60+(pT->tm_isdst!=0), abs(t.timezone)%60);
  }
#endif
#elif defined(HAVE_LOCALTIME_R)
  if (localtime_r(&n, pT) != SOAP_FUNC_R_ERR)
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%S", pT);
#else
  pT = localtime(&n);
  if (pT)
    l = strftime(soap->tmpbuf, sizeof(soap->tmpbuf), "%Y-%m-%dT%H:%M:%S", pT);
#endif
  if (!l)
    soap_strcpy(soap->tmpbuf, sizeof(soap->tmpbuf), "1969-12-31T23:59:59Z");
  return soap->tmpbuf;
}
#endif

/******************************************************************************/

#ifndef WITH_LEAN
SOAP_FMAC1
int
SOAP_FMAC2
soap_outdateTime(struct soap *soap, const char *tag, int id, const time_t *p, const char *type, int n)
{
  if (soap_element_begin_out(soap, tag, soap_embedded_id(soap, id, p, n), type)
   || soap_string_out(soap, soap_dateTime2s(soap, *p), 0))
    return soap->error;
  return soap_element_end_out(soap, tag);
}
#endif

/******************************************************************************/

#ifndef WITH_LEAN
SOAP_FMAC1
int
SOAP_FMAC2
soap_s2dateTime(struct soap *soap, const char *s, time_t *p)
{
  *p = 0;
  if (s)
  {
    unsigned long d;
    struct tm T;
    char *t;
    if (!*s)
      return soap->error = SOAP_EMPTY;
    memset((void*)&T, 0, sizeof(T));
    d = soap_strtoul(s, &t, 10);
    if (*t == '-')
    {
      /* YYYY-MM-DD */
      T.tm_year = (int)d;
      T.tm_mon = (int)soap_strtoul(t + 1, &t, 10);
      T.tm_mday = (int)soap_strtoul(t + 1, &t, 10);
    }
    else if (!(soap->mode & SOAP_XML_STRICT))
    {
      /* YYYYMMDD */
      T.tm_year = (int)(d / 10000);
      T.tm_mon = (int)(d / 100 % 100);
      T.tm_mday = (int)(d % 100);
    }
    else
    {
      return soap->error = SOAP_TYPE;
    }
    if (*t == 'T' || ((*t == 't' || *t == ' ') && !(soap->mode & SOAP_XML_STRICT)))
    {
      d = soap_strtoul(t + 1, &t, 10);
      if (*t == ':')
      {
        /* Thh:mm:ss */
        T.tm_hour = (int)d;
        T.tm_min = (int)soap_strtoul(t + 1, &t, 10);
        T.tm_sec = (int)soap_strtoul(t + 1, &t, 10);
      }
      else if (!(soap->mode & SOAP_XML_STRICT))
      {
        /* Thhmmss */
        T.tm_hour = (int)(d / 10000);
        T.tm_min = (int)(d / 100 % 100);
        T.tm_sec = (int)(d % 100);
      }
      else
      {
        return soap->error = SOAP_TYPE;
      }
    }
    if (T.tm_year == 1)
      T.tm_year = 70;
    else
      T.tm_year -= 1900;
    T.tm_mon--;
    if (*t == '.')
    {
      for (t++; *t; t++)
        if (*t < '0' || *t > '9')
          break;
    }
    if (*t == ' ' && !(soap->mode & SOAP_XML_STRICT))
      t++;
    if (*t)
    {
#ifndef WITH_NOZONE
      if (*t == '+' || *t == '-')
      {
        int h, m;
        m = (int)soap_strtol(t, &t, 10);
        if (*t == ':')
        {
          /* +hh:mm */
          h = m;
          m = (int)soap_strtol(t + 1, &t, 10);
          if (h < 0)
            m = -m;
        }
        else if (!(soap->mode & SOAP_XML_STRICT))
        {
          /* +hhmm */
          h = m / 100;
          m = m % 100;
        }
        else
        {
          /* +hh */
          h = m;
          m = 0;
        }
        if (*t)
          return soap->error = SOAP_TYPE;
        T.tm_min -= m;
        T.tm_hour -= h;
        /* put hour and min in range */
        T.tm_hour += T.tm_min / 60;
        T.tm_min %= 60;
        if (T.tm_min < 0)
        {
          T.tm_min += 60;
          T.tm_hour--;
        }
        T.tm_mday += T.tm_hour / 24;
        T.tm_hour %= 24;
        if (T.tm_hour < 0)
        {
          T.tm_hour += 24;
          T.tm_mday--;
        }
        /* note: day of the month may be out of range, timegm() handles it */
      }
      else if (*t != 'Z')
      {
        return soap->error = SOAP_TYPE;
      }
#endif
      *p = soap_timegm(&T);
    }
    else /* no UTC or timezone, so assume we got a localtime */
    {
      T.tm_isdst = -1;
      *p = mktime(&T);
    }
  }
  return soap->error;
}
#endif

/******************************************************************************/

#ifndef WITH_LEAN
SOAP_FMAC1
time_t *
SOAP_FMAC2
soap_indateTime(struct soap *soap, const char *tag, time_t *p, const char *type, int t)
{
  if (soap_element_begin_in(soap, tag, 0, NULL))
    return NULL;
  if (*soap->type
   && soap_match_tag(soap, soap->type, type)
   && soap_match_tag(soap, soap->type, ":dateTime"))
  {
    soap->error = SOAP_TYPE;
    soap_revert(soap);
    return NULL;
  }
  p = (time_t*)soap_id_enter(soap, soap->id, p, t, sizeof(time_t), NULL, NULL, NULL, NULL);
  if (!p)
    return NULL;
  if (*soap->href != '#')
  {
    int err = soap_s2dateTime(soap, soap_value(soap), p);
    if ((soap->body && soap_element_end_in(soap, tag)) || err)
      return NULL;
  }
  else
  {
    p = (time_t*)soap_id_forward(soap, soap->href, p, 0, t, t, sizeof(time_t), 0, NULL, NULL);
    if (soap->body && soap_element_end_in(soap, tag))
      return NULL;
  }
  return p;
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_outliteral(struct soap *soap, const char *tag, char *const*p, const char *type)
{
  if (tag && *tag != '-')
    if (soap_element_begin_out(soap, tag, 0, type))
      return soap->error;
  if (p && *p)
    if (soap_send(soap, *p)) /* send as-is */
      return soap->error;
  if (tag && *tag != '-')
    return soap_element_end_out(soap, tag);
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
char **
SOAP_FMAC2
soap_inliteral(struct soap *soap, const char *tag, char **p)
{
  if (soap_element_begin_in(soap, tag, 1, NULL))
  {
    if (soap->error != SOAP_NO_TAG || soap_peek(soap) == SOAP_TT)
      return NULL;
    soap->error = SOAP_OK;
  }
  if (!p)
  {
    p = (char**)soap_malloc(soap, sizeof(char*));
    if (!p)
      return NULL;
  }
  if (soap->body || (tag && *tag == '-'))
  {
    if (tag && *tag != '-')
      *p = soap_string_in(soap, -1, -1, -1, NULL);
    else
      *p = soap_string_in(soap, 0, -1, -1, NULL);
    if (!*p)
      return NULL;
    if (!**p && tag && *tag == '-')
    {
      soap->error = SOAP_NO_TAG;
      return NULL;
    }
  }
  else if (soap->null)
  {
    *p = NULL;
  }
  else
  {
    *p = soap_strdup(soap, SOAP_STR_EOS);
  }
  if (soap->body && soap_element_end_in(soap, tag))
    return NULL;
  return p;
}

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_outwliteral(struct soap *soap, const char *tag, wchar_t *const*p, const char *type)
{
  if (tag && *tag != '-')
    if (soap_element_begin_out(soap, tag, 0, type))
      return soap->error;
  if (p)
  {
    wchar_t c;
    const wchar_t *s = *p;
    while ((c = *s++))
    {
      if (soap_pututf8(soap, (unsigned long)c)) /* send as-is in UTF8 */
        return soap->error;
    }
  }
  if (tag && *tag != '-')
    return soap_element_end_out(soap, tag);
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
wchar_t **
SOAP_FMAC2
soap_inwliteral(struct soap *soap, const char *tag, wchar_t **p)
{
  if (soap_element_begin_in(soap, tag, 1, NULL))
  {
    if (soap->error != SOAP_NO_TAG || soap_peek(soap) == SOAP_TT)
      return NULL;
    soap->error = SOAP_OK;
  }
  if (!p)
  {
    p = (wchar_t**)soap_malloc(soap, sizeof(wchar_t*));
    if (!p)
      return NULL;
  }
  if (soap->body)
  {
    if (tag && *tag != '-')
      *p = soap_wstring_in(soap, -1, -1, -1, NULL);
    else
      *p = soap_wstring_in(soap, 0, -1, -1, NULL);
    if (!*p)
      return NULL;
    if (!**p && tag && *tag == '-')
    {
      soap->error = SOAP_NO_TAG;
      return NULL;
    }
  }
  else if (tag && *tag == '-')
  {
    soap->error = SOAP_NO_TAG;
    return NULL;
  }
  else if (soap->null)
  {
    *p = NULL;
  }
  else
  {
    *p = soap_wstrdup(soap, L"");
  }
  if (soap->body && soap_element_end_in(soap, tag))
    return NULL;
  return p;
}
#endif

/******************************************************************************/

SOAP_FMAC1
const char *
SOAP_FMAC2
soap_value(struct soap *soap)
{
  size_t i;
  soap_wchar c = 0;
  char *s = soap->tmpbuf;
  if (!soap->body)
    return SOAP_STR_EOS;
  do
  {
    c = soap_get(soap);
  } while (soap_coblank(c));
  for (i = 0; i < sizeof(soap->tmpbuf) - 1; i++)
  {
    if (c == SOAP_TT || c == SOAP_LT || (int)c == EOF)
      break;
    *s++ = (char)c;
    c = soap_get(soap);
  }
  for (s--; i > 0; i--, s--)
  {
    if (!soap_coblank((soap_wchar)*s))
      break;
  }
  s[1] = '\0';
  soap->tmpbuf[sizeof(soap->tmpbuf) - 1] = '\0'; /* appease */
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Element content value='%s'\n", soap->tmpbuf));
  if (c == SOAP_TT || c == SOAP_LT || (int)c == EOF)
  {
    soap_unget(soap, c);
  }
  else
  {
    soap->error = SOAP_LENGTH;
    return NULL;
  }
#ifdef WITH_DOM
  if ((soap->mode & SOAP_XML_DOM) && soap->dom)
  {
    soap->dom->text = soap_strdup(soap, soap->tmpbuf);
    if (!soap->dom->text)
      return NULL;
  }
#endif
  return soap->tmpbuf; /* return non-null pointer */
}

/******************************************************************************/

#if !defined(WITH_LEANER) || !defined(WITH_NOHTTP)
SOAP_FMAC1
int
SOAP_FMAC2
soap_getline(struct soap *soap, char *buf, int len)
{
  char *s = buf;
  int i = len;
  soap_wchar c = 0;
  for (;;)
  {
    while (i > 1)
    {
      c = soap_getchar(soap);
      if (c == '\r' || c == '\n')
        break;
      if ((int)c == EOF)
        return soap->error = SOAP_CHK_EOF;
      *s++ = (char)c;
      i--;
    }
    *s = '\0';
    if (c != '\n')
      c = soap_getchar(soap); /* got \r or something else, now get \n */
    if (c == '\n')
    {
      if (i == len) /* empty line: end of HTTP/MIME header */
        break;
      c = soap_get0(soap);
      if (c != ' ' && c != '\t') /* HTTP line continuation? */
        break;
    }
    else if ((int)c == EOF)
    {
      return soap->error = SOAP_CHK_EOF;
    }
    else if (i <= 1)
    {
      return soap->error = SOAP_HDR;
    }
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

static ULONG64
soap_count_attachments(struct soap *soap)
{
#ifndef WITH_LEANER
  struct soap_multipart *content;
  ULONG64 count = soap->count;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Calculating the message size with attachments, current count=" SOAP_ULONG_FORMAT "\n", count));
  if ((soap->mode & SOAP_ENC_DIME) && !(soap->mode & SOAP_ENC_MTOM))
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Calculating the size of DIME attachments\n"));
    for (content = soap->dime.first; content; content = content->next)
    {
      count += 12 + ((content->size+3)&(~3));
      if (content->id)
        count += ((strlen(content->id)+3)&(~3));
      if (content->type)
        count += ((strlen(content->type)+3)&(~3));
      if (content->options)
        count += ((((unsigned char)content->options[2] << 8) | ((unsigned char)content->options[3]))+7)&(~3);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Size of DIME attachment content is %lu bytes\n", (unsigned long)content->size));
    }
  }
  if ((soap->mode & SOAP_ENC_MIME) && soap->mime.boundary)
  {
    size_t n = strlen(soap->mime.boundary);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Calculating the size of MIME attachments\n"));
    for (content = soap->mime.first; content; content = content->next)
    {
      const char *s;
      /* count \r\n--boundary\r\n */
      count += 6 + n;
      /* count Content-Type: ...\r\n */
      if (content->type)
        count += 16 + strlen(content->type);
      /* count Content-Transfer-Encoding: ...\r\n */
      s = soap_code_str(mime_codes, content->encoding);
      if (s)
        count += 29 + strlen(s);
      /* count Content-ID: ...\r\n */
      if (content->id)
        count += 14 + strlen(content->id);
      /* count Content-Location: ...\r\n */
      if (content->location)
        count += 20 + strlen(content->location);
      /* count Content-Description: ...\r\n */
      if (content->description)
        count += 23 + strlen(content->description);
      /* count \r\n...content */
      count += 2 + content->size;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Size of MIME attachment content is %lu bytes\n", (unsigned long)content->size));
    }
    /* count \r\n--boundary-- */
    count += 6 + n;
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "New count=" SOAP_ULONG_FORMAT "\n", count));
  return count;
#else
  return soap->count;
#endif
}

/******************************************************************************/

#ifndef WITH_LEANER
static int
soap_putdimefield(struct soap *soap, const char *s, size_t n)
{
  if (soap_send_raw(soap, s, n))
    return soap->error;
  return soap_send_raw(soap, SOAP_STR_PADDING, -(long)n&3);
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
char *
SOAP_FMAC2
soap_dime_option(struct soap *soap, unsigned short optype, const char *option)
{
  size_t n;
  char *s = NULL;
  if (option)
  {
    n = strlen(option);
    s = (char*)soap_malloc(soap, n + 5);
    if (s)
    {
      s[0] = (char)(optype >> 8);
      s[1] = (char)(optype & 0xFF);
      s[2] = (char)(n >> 8);
      s[3] = (char)(n & 0xFF);
      soap_strcpy(s + 4, n + 1, option);
    }
  }
  return s;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_putdimehdr(struct soap *soap)
{
  unsigned char tmp[12];
  size_t optlen = 0, idlen = 0, typelen = 0;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Put DIME header id='%s'\n", soap->dime.id ? soap->dime.id : SOAP_STR_EOS));
  if (soap->dime.options)
    optlen = (((unsigned char)soap->dime.options[2] << 8) | ((unsigned char)soap->dime.options[3])) + 4;
  if (soap->dime.id)
  {
    idlen = strlen(soap->dime.id);
    if (idlen > 0x0000FFFF)
      idlen = 0x0000FFFF;
  }
  if (soap->dime.type)
  {
    typelen = strlen(soap->dime.type);
    if (typelen > 0x0000FFFF)
      typelen = 0x0000FFFF;
  }
  tmp[0] = SOAP_DIME_VERSION | (soap->dime.flags & 0x7);
  tmp[1] = soap->dime.flags & 0xF0;
  tmp[2] = (char)(optlen >> 8);
  tmp[3] = (char)(optlen & 0xFF);
  tmp[4] = (char)(idlen >> 8);
  tmp[5] = (char)(idlen & 0xFF);
  tmp[6] = (char)(typelen >> 8);
  tmp[7] = (char)(typelen & 0xFF);
  tmp[8] = (char)(soap->dime.size >> 24);
  tmp[9] = (char)((soap->dime.size >> 16) & 0xFF);
  tmp[10] = (char)((soap->dime.size >> 8) & 0xFF);
  tmp[11] = (char)(soap->dime.size & 0xFF);
  if (soap_send_raw(soap, (char*)tmp, 12)
   || soap_putdimefield(soap, soap->dime.options, optlen)
   || soap_putdimefield(soap, soap->dime.id, idlen)
   || soap_putdimefield(soap, soap->dime.type, typelen))
    return soap->error;
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_putdime(struct soap *soap)
{
  struct soap_multipart *content;
  if (!(soap->mode & SOAP_ENC_DIME))
    return SOAP_OK;
  for (content = soap->dime.first; content; content = content->next)
  {
    void *handle;
    soap->dime.size = content->size;
    soap->dime.id = content->id;
    soap->dime.type = content->type;
    soap->dime.options = content->options;
    soap->dime.flags = SOAP_DIME_VERSION | SOAP_DIME_MEDIA;
    if (soap->fdimereadopen && ((handle = soap->fdimereadopen(soap, (void*)content->ptr, content->id, content->type, content->options)) != NULL || soap->error))
    {
      size_t size = content->size;
      if (!handle)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "fdimereadopen failed\n"));
        return soap->error;
      }
      if (!size && ((soap->mode & SOAP_ENC_PLAIN) || (soap->mode & SOAP_IO) == SOAP_IO_CHUNK || (soap->mode & SOAP_IO) == SOAP_IO_STORE))
      {
        size_t chunksize = sizeof(soap->tmpbuf);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Chunked streaming DIME\n"));
        do
        {
          size = soap->fdimeread(soap, handle, soap->tmpbuf, chunksize);
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "fdimeread returned %lu bytes\n", (unsigned long)size));
          if (size < chunksize)
          {
            soap->dime.flags &= ~SOAP_DIME_CF;
            if (!content->next)
              soap->dime.flags |= SOAP_DIME_ME;
          }
          else
          {
            soap->dime.flags |= SOAP_DIME_CF;
          }
          soap->dime.size = size;
          if (soap_putdimehdr(soap)
           || soap_putdimefield(soap, soap->tmpbuf, size))
            break;
          if (soap->dime.id)
          {
            soap->dime.flags &= ~(SOAP_DIME_MB | SOAP_DIME_MEDIA);
            soap->dime.id = NULL;
            soap->dime.type = NULL;
            soap->dime.options = NULL;
          }
        } while (size >= chunksize);
      }
      else
      {
        if (!content->next)
          soap->dime.flags |= SOAP_DIME_ME;
        if (soap_putdimehdr(soap))
          return soap->error;
        do
        {
          size_t bufsize;
          if (size < sizeof(soap->tmpbuf))
            bufsize = size;
          else
            bufsize = sizeof(soap->tmpbuf);
          bufsize = soap->fdimeread(soap, handle, soap->tmpbuf, bufsize);
          if (!bufsize)
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "fdimeread failed: insufficient data (%lu bytes remaining from %lu bytes)\n", (unsigned long)size, (unsigned long)content->size));
            soap->error = SOAP_CHK_EOF;
            break;
          }
          if (soap_send_raw(soap, soap->tmpbuf, bufsize))
            break;
          size -= bufsize;
        } while (size);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "fdimereadclose\n"));
        if (soap_send_raw(soap, SOAP_STR_PADDING, -(long)soap->dime.size&3))
          return soap->error;
      }
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "fdimereadclose\n"));
      if (soap->fdimereadclose)
        soap->fdimereadclose(soap, handle);
    }
    else
    {
      if (!content->next)
        soap->dime.flags |= SOAP_DIME_ME;
      if (soap_putdimehdr(soap)
       || soap_putdimefield(soap, (char*)content->ptr, content->size))
        return soap->error;
    }
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
static char *
soap_getdimefield(struct soap *soap, size_t n)
{
  char *p = NULL;
  if (n > 0)
  {
    p = (char*)soap_malloc(soap, n + 1 > n ? n + 1 : n);
    if (p)
    {
      char *s = p;
      size_t i;
      for (i = n; i > 0; i--)
      {
        soap_wchar c = soap_get1(soap);
        if ((int)c == EOF)
        {
          soap->error = SOAP_CHK_EOF;
          return NULL;
        }
        *s++ = (char)c;
      }
      if (n + 1 > n)
        *s = '\0'; /* force NUL terminated */
      soap->error = soap_move(soap, (size_t)(-(long)n&3));
      if (soap->error)
        return NULL;
    }
    else
    {
      soap->error = SOAP_EOM;
    }
  }
  return p;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_getdimehdr(struct soap *soap)
{
  soap_wchar c;
  char *s;
  int i;
  unsigned char tmp[12];
  size_t optlen, idlen, typelen;
  if (!(soap->mode & SOAP_ENC_DIME))
    return soap->error = SOAP_DIME_END;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Get DIME header\n"));
  s = (char*)tmp;
  for (i = 12; i > 0; i--)
  {
    c = soap_getchar(soap);
    if ((int)c == EOF)
      return soap->error = SOAP_CHK_EOF;
    *s++ = (char)c;
  }
  if ((tmp[0] & 0xF8) != SOAP_DIME_VERSION)
    return soap->error = SOAP_DIME_MISMATCH;
  soap->dime.flags = (tmp[0] & 0x7) | (tmp[1] & 0xF0);
  optlen = (tmp[2] << 8) | tmp[3];
  idlen = (tmp[4] << 8) | tmp[5];
  typelen = (tmp[6] << 8) | tmp[7];
  soap->dime.size = ((size_t)tmp[8] << 24) | ((size_t)tmp[9] << 16) | ((size_t)tmp[10] << 8) | ((size_t)tmp[11]);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DIME size=%lu flags=0x%X\n", (unsigned long)soap->dime.size, soap->dime.flags));
  soap->dime.options = soap_getdimefield(soap, optlen);
  if (!soap->dime.options && soap->error)
    return soap->error;
  soap->dime.id = soap_getdimefield(soap, idlen);
  if (!soap->dime.id && soap->error)
    return soap->error;
  soap->dime.type = soap_getdimefield(soap, typelen);
  if (!soap->dime.type && soap->error)
    return soap->error;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DIME flags=%x id='%s', type='%s', options='%s'\n", soap->dime.flags, soap->dime.id ? soap->dime.id : SOAP_STR_EOS, soap->dime.type ? soap->dime.type : "", soap->dime.options ? soap->dime.options+4 : SOAP_STR_EOS));
  if ((soap->dime.flags & SOAP_DIME_ME))
    soap->mode &= ~SOAP_ENC_DIME;
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_getdime(struct soap *soap)
{
  if (soap->dime.buflen || soap->dime.chunksize)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Skip remainder of SOAP in DIME (%u bytes or %u bytes in chunk left)\n", (unsigned int)soap->dime.buflen, (unsigned int)soap->dime.chunksize));
    do
      if (soap_get1(soap) == (int)EOF)
        return soap->error = SOAP_CHK_EOF;
    while (soap->dime.buflen || soap->dime.chunksize);
    if (soap_move(soap, (size_t)(-(long)soap->dime.size&3)))
      return soap->error = SOAP_CHK_EOF;
    if (!(soap->mode & SOAP_ENC_DIME))
      return SOAP_OK;
  }
  else
  {
    if (soap_move(soap, (size_t)(((soap->dime.size+3)&(~3)) - soap_tell(soap))))
      return soap->error = SOAP_CHK_EOF;
  }
  for (;;)
  {
    struct soap_multipart *content;
    if (soap_getdimehdr(soap))
      break;
    if (soap->fdimewriteopen && ((soap->dime.ptr = (char*)soap->fdimewriteopen(soap, soap->dime.id, soap->dime.type, soap->dime.options)) != NULL || soap->error))
    {
      const char *id, *type, *options;
      size_t size, n;
      if (!soap->dime.ptr)
        return soap->error;
      id = soap->dime.id;
      type = soap->dime.type;
      options = soap->dime.options;
      for (;;)
      {
        size = soap->dime.size;
        for (;;)
        {
          n = soap->buflen - soap->bufidx;
          if (size < n)
            n = size;
          soap->error = soap->fdimewrite(soap, (void*)soap->dime.ptr, soap->buf + soap->bufidx, n);
          if (soap->error)
            break;
          size -= n;
          if (!size)
          {
            soap->bufidx += n;
            break;
          }
          if (soap_recv(soap))
          {
            soap->error = SOAP_EOF;
            goto end;
          }
        }
        if (soap_move(soap, (size_t)(-(long)soap->dime.size&3)))
        {
          soap->error = SOAP_EOF;
          break;
        }
        if (!(soap->dime.flags & SOAP_DIME_CF))
          break;
        if (soap_getdimehdr(soap))
          break;
      }
end:
      if (soap->fdimewriteclose)
        soap->fdimewriteclose(soap, (void*)soap->dime.ptr);
      soap->dime.size = 0;
      soap->dime.id = id;
      soap->dime.type = type;
      soap->dime.options = options;
    }
    else if ((soap->dime.flags & SOAP_DIME_CF))
    {
      const char *id, *type, *options;
      id = soap->dime.id;
      type = soap->dime.type;
      options = soap->dime.options;
      if (soap_alloc_block(soap) == NULL)
        return soap->error = SOAP_EOM;
      for (;;)
      {
        soap_wchar c;
        size_t i;
        char *s;
        if (soap->dime.size > SOAP_MAXDIMESIZE)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DIME size=%lu exceeds SOAP_MAXDIMESIZE=%lu\n", (unsigned long)soap->dime.size, (unsigned long)SOAP_MAXDIMESIZE));
          return soap->error = SOAP_DIME_ERROR;
        }
        s = (char*)soap_push_block(soap, NULL, soap->dime.size);
        if (!s)
          return soap->error = SOAP_EOM;
        for (i = soap->dime.size; i > 0; i--)
        {
          c = soap_get1(soap);
          if ((int)c == EOF)
            return soap->error = SOAP_EOF;
          *s++ = (char)c;
        }
        if (soap_move(soap, (size_t)(-(long)soap->dime.size&3)))
          return soap->error = SOAP_EOF;
        if (!(soap->dime.flags & SOAP_DIME_CF))
          break;
        if (soap_getdimehdr(soap))
          return soap->error;
      }
      soap->dime.size = soap->blist->size;
      if (soap->dime.size + 1 > soap->dime.size)
        soap->blist->size++; /* allocate one more byte in blist for the terminating '\0' */
      soap->dime.ptr = (char*)soap_save_block(soap, NULL, NULL, 0);
      if (!soap->dime.ptr)
        return soap->error;
      if (soap->dime.size + 1 > soap->dime.size)
        soap->dime.ptr[soap->dime.size] = '\0'; /* make 0-terminated, just in case even though this is binary data */
      soap->dime.id = id;
      soap->dime.type = type;
      soap->dime.options = options;
    }
    else
    {
      soap->dime.ptr = soap_getdimefield(soap, soap->dime.size);
    }
    content = soap_alloc_multipart(soap, &soap->dime.first, &soap->dime.last, soap->dime.ptr, soap->dime.size);
    if (!content)
      return soap->error = SOAP_EOM;
    content->id = soap->dime.id;
    content->type = soap->dime.type;
    content->options = soap->dime.options;
    if (soap->error)
      return soap->error;
    soap_resolve_attachment(soap, content);
  }
  if (soap->error != SOAP_DIME_END)
    return soap->error;
  return soap->error = SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_getmimehdr(struct soap *soap)
{
  struct soap_multipart *content;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Get MIME header\n"));
  do
  {
    if (soap_getline(soap, soap->msgbuf, sizeof(soap->msgbuf)))
      return soap->error;
  } while (!*soap->msgbuf);
  if (soap->msgbuf[0] == '-' && soap->msgbuf[1] == '-')
  {
    char *s = soap->msgbuf + strlen(soap->msgbuf) - 1;
    /* remove white space */
    while (soap_coblank((soap_wchar)*s))
      s--;
    s[1] = '\0';
    if (soap->mime.boundary)
    {
      if (strcmp(soap->msgbuf + 2, soap->mime.boundary))
        return soap->error = SOAP_MIME_ERROR;
    }
    else
    {
      soap->mime.boundary = soap_strdup(soap, soap->msgbuf + 2);
      if (!soap->mime.boundary)
        return soap->error = SOAP_EOM;
    }
    if (soap_getline(soap, soap->msgbuf, sizeof(soap->msgbuf)))
      return soap->error;
  }
  if (soap_set_mime_attachment(soap, NULL, 0, SOAP_MIME_NONE, NULL, NULL, NULL, NULL))
    return soap->error = SOAP_EOM;
  content = soap->mime.last;
  for (;;)
  {
    char *key = soap->msgbuf;
    char *val;
    if (!*key)
      break;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "MIME header: %s\n", key));
    val = strchr(soap->msgbuf, ':');
    if (val)
    {
      *val = '\0';
      do
      {
        val++;
      } while (*val && *val <= 32);
      if (!soap_tag_cmp(key, "Content-ID"))
        content->id = soap_strdup(soap, val);
      else if (!soap_tag_cmp(key, "Content-Location"))
        content->location = soap_strdup(soap, val);
      else if (!content->id && !soap_tag_cmp(key, "Content-Disposition"))
        content->id = soap_strdup(soap, soap_http_header_attribute(soap, val, "name"));
      else if (!soap_tag_cmp(key, "Content-Type"))
        content->type = soap_strdup(soap, val);
      else if (!soap_tag_cmp(key, "Content-Description"))
        content->description = soap_strdup(soap, val);
      else if (!soap_tag_cmp(key, "Content-Transfer-Encoding"))
        content->encoding = (enum soap_mime_encoding)soap_code_int(mime_codes, val, (LONG64)SOAP_MIME_NONE);
    }
    if (soap_getline(soap, key, sizeof(soap->msgbuf)))
      return soap->error;
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_getmime(struct soap *soap)
{
  while (soap_recv_mime_attachment(soap, NULL))
    continue;
  return soap->error;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
void
SOAP_FMAC2
soap_post_check_mime_attachments(struct soap *soap)
{
  soap->imode |= SOAP_MIME_POSTCHECK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_check_mime_attachments(struct soap *soap)
{
  if ((soap->mode & SOAP_MIME_POSTCHECK))
    return soap_recv_mime_attachment(soap, NULL) != NULL;
  return 0;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
struct soap_multipart *
SOAP_FMAC2
soap_recv_mime_attachment(struct soap *soap, void *handle)
{
  soap_wchar c = 0;
  size_t i, m = 0;
  char *s, *t = NULL;
  struct soap_multipart *content;
  short flag = 0;
  if (!(soap->mode & SOAP_ENC_MIME))
    goto post_check_exit;
  content = soap->mime.last;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Get MIME (%p)\n", (void*)content));
  if (!content)
  {
    if (soap_getmimehdr(soap))
      goto post_check_exit;
    content = soap->mime.last;
  }
  else if (content != soap->mime.first)
  {
    if (soap->fmimewriteopen && ((content->ptr = (char*)soap->fmimewriteopen(soap, (void*)handle, content->id, content->type, content->description, content->encoding)) != NULL || soap->error))
    {
      if (!content->ptr)
        goto post_check_exit;
    }
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Parsing MIME content id='%s' type='%s'\n", content->id ? content->id : SOAP_STR_EOS, content->type ? content->type : SOAP_STR_EOS));
  if (!content->ptr && soap_alloc_block(soap) == NULL)
  {
    soap->error = SOAP_EOM;
    goto post_check_exit;
  }
  for (;;)
  {
    if (content->ptr)
    {
      s = soap->tmpbuf;
    }
    else
    {
      s = (char*)soap_push_block(soap, NULL, sizeof(soap->tmpbuf));
      if (!s)
      {
        soap->error = SOAP_EOM;
        goto post_check_exit;
      }
    }
    for (i = 0; i < sizeof(soap->tmpbuf); i++)
    {
      if (m > 0)
      {
        *s++ = *t++;
        m--;
      }
      else
      {
        if (!flag)
        {
          c = soap_getchar(soap);
          if ((int)c == EOF)
          {
            if (content->ptr && soap->fmimewriteclose)
              soap->fmimewriteclose(soap, (void*)content->ptr);
            soap->error = SOAP_CHK_EOF;
            goto post_check_exit;
          }
        }
        if (flag || c == '\r')
        {
          memset((void*)soap->msgbuf, 0, sizeof(soap->msgbuf));
          soap_strcpy(soap->msgbuf, sizeof(soap->msgbuf), "\n--");
          if (soap->mime.boundary)
          {
            if (soap_strncat(soap->msgbuf, sizeof(soap->msgbuf), soap->mime.boundary, sizeof(soap->msgbuf) - 4))
            {
              soap->error = SOAP_MIME_ERROR;
              goto post_check_exit;
            }
          }
          t = soap->msgbuf;
          do
          {
            c = soap_getchar(soap);
          } while (c == *t++);
          if ((int)c == EOF)
          {
            if (content->ptr && soap->fmimewriteclose)
              soap->fmimewriteclose(soap, (void*)content->ptr);
            soap->error = SOAP_CHK_EOF;
            goto post_check_exit;
          }
          if (!*--t)
            goto end;
          *t = (char)c;
          flag = (c == '\r');
          m = t - soap->msgbuf + 1 - flag;
          t = soap->msgbuf;
          c = '\r';
        }
        *s++ = (char)c;
      }
    }
    if (content->ptr && soap->fmimewrite)
    {
      soap->error = soap->fmimewrite(soap, (void*)content->ptr, soap->tmpbuf, i);
      if (soap->error)
        break;
    }
  }
end:
  if (content->ptr)
  {
    if (!soap->error && soap->fmimewrite)
      soap->error = soap->fmimewrite(soap, (void*)content->ptr, soap->tmpbuf, i);
    if (soap->fmimewriteclose)
      soap->fmimewriteclose(soap, (void*)content->ptr);
    if (soap->error)
      goto post_check_exit;
  }
  else
  {
    *s = '\0'; /* make 0-terminated, just in case even though this is binary data */
    content->size = soap_size_block(soap, NULL, i + 1) - 1; /* last block with '\0' */
    content->ptr = (char*)soap_save_block(soap, NULL, NULL, 0);
  }
  soap_resolve_attachment(soap, content);
  if (c == '-' && soap_getchar(soap) == '-')
  {
    soap->mode &= ~SOAP_ENC_MIME;
    if ((soap->mode & SOAP_MIME_POSTCHECK))
    {
      if (soap_end_recv(soap))
        goto post_check_exit;
      if (soap->keep_alive == -2) /* special case to keep alive */
        soap->keep_alive = 0;
      (void)soap_closesock(soap);
    }
  }
  else
  {
    while (c != '\r' && (int)c != EOF && soap_coblank(c))
      c = soap_getchar(soap);
    if (c != '\r' || soap_getchar(soap) != '\n')
    {
      soap->error = SOAP_MIME_ERROR;
      goto post_check_exit;
    }
    if (soap_getmimehdr(soap))
      goto post_check_exit;
  }
  return content;
post_check_exit:
  if ((soap->mode & SOAP_MIME_POSTCHECK))
  {
    if (soap->keep_alive == -2) /* special case to keep alive */
      soap->keep_alive = 0;
    (void)soap_closesock(soap);
  }
  return NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_match_cid(struct soap *soap, const char *s, const char *t)
{
  size_t n;
  if (!s)
    return 1;
  if (!strcmp(s, t))
    return 0;
  if (!strncmp(s, "cid:", 4))
    s += 4;
  n = strlen(t);
  if (*t == '<')
  {
    t++;
    n -= 2;
  }
  if (!strncmp(s, t, n) && !s[n])
    return 0;
  (void)soap_decode(soap->tmpbuf, sizeof(soap->tmpbuf), s, SOAP_STR_EOS);
  if (!strncmp(soap->tmpbuf, t, n) && !soap->tmpbuf[n])
    return 0;
  return 1;
}
#endif

/******************************************************************************/

/* return UUID "<prefix>xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx" in a temporary buffer */
SOAP_FMAC1
const char*
SOAP_FMAC2
soap_rand_uuid(struct soap *soap, const char *prefix)
{
  int r1, r2, r3, r4;
#ifdef WITH_OPENSSL
  r1 = soap_random;
  r2 = soap_random;
#else
  size_t i;
  static int k = 0xFACEB00C;
  int lo = k % 127773;
  int hi = k / 127773;
# if defined(HAVE_GETTIMEOFDAY)
  struct timeval tv;
  gettimeofday(&tv, NULL);
  r1 = 10000000 * tv.tv_sec + tv.tv_usec;
# elif defined(UNDER_CE)
  r1 = (int)Random();
# elif !defined(WITH_LEAN)
  r1 = (int)time(NULL);
# else
  r1 = k;
# endif
  k = 16807 * lo - 2836 * hi;
  if (k <= 0)
    k += 0x7FFFFFFF;
  r2 = k;
  /* k &= 0x8FFFFFFF; */
  for (i = 0; i < (sizeof(soap->buf) < 16UL ? sizeof(soap->buf) : 16UL); i++)
    r2 += soap->buf[i];
#endif
  r3 = soap_random;
  r4 = soap_random;
  (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), prefix ? strlen(prefix) + 37 : 37), "%s%8.8x-%4.4hx-4%3.3hx-%4.4hx-%4.4hx%8.8x", prefix ? prefix : SOAP_STR_EOS, r1, (short)(r2 >> 16), (short)(((short)r2 >> 4) & 0x0FFF), (short)(((short)(r3 >> 16) & 0x3FFF) | 0x8000), (short)r3, r4);
  return soap->tmpbuf;
}

/******************************************************************************/

#ifndef WITH_LEANER
static void
soap_resolve_attachment(struct soap *soap, struct soap_multipart *content)
{
  if (content->id)
  {
    struct soap_xlist **xp = &soap->xlist;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Resolving attachment data for id='%s'\n", content->id));
    while (*xp)
    {
      struct soap_xlist *xq = *xp;
      if (!soap_match_cid(soap, xq->id, content->id))
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Found matching attachment id='%s' for content id='%s'\n", xq->id, content->id));
        *xp = xq->next;
        *xq->ptr = (unsigned char*)content->ptr;
        *xq->size = (int)content->size;
        *xq->type = (char*)content->type;
        if (content->options)
          *xq->options = (char*)content->options;
        else
          *xq->options = (char*)content->description;
        SOAP_FREE(soap, xq);
      }
      else
      {
        xp = &(*xp)->next;
      }
    }
  }
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_putmimehdr(struct soap *soap, struct soap_multipart *content)
{
  const char *s;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "MIME attachment type='%s'\n", content->type ? content->type : SOAP_STR_EOS));
  if (soap_send3(soap, "\r\n--", soap->mime.boundary, "\r\n"))
    return soap->error;
  if (content->type && soap_send3(soap, "Content-Type: ", content->type, "\r\n"))
    return soap->error;
  s = soap_code_str(mime_codes, content->encoding);
  if (s && soap_send3(soap, "Content-Transfer-Encoding: ", s, "\r\n"))
    return soap->error;
  if (content->id && soap_send3(soap, "Content-ID: ", content->id, "\r\n"))
    return soap->error;
  if (content->location && soap_send3(soap, "Content-Location: ", content->location, "\r\n"))
    return soap->error;
  if (content->description && soap_send3(soap, "Content-Description: ", content->description, "\r\n"))
    return soap->error;
  return soap_send_raw(soap, "\r\n", 2);
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_putmime(struct soap *soap)
{
  struct soap_multipart *content;
  if (!(soap->mode & SOAP_ENC_MIME) || !soap->mime.boundary)
    return SOAP_OK;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Sending MIME attachments\n"));
  for (content = soap->mime.first; content; content = content->next)
  {
    int err = SOAP_OK;
    void *handle;
    if (soap->fmimereadopen && ((handle = soap->fmimereadopen(soap, (void*)content->ptr, content->id, content->type, content->description)) != NULL || soap->error))
    {
      size_t size = content->size;
      if (!handle)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "fmimereadopen failed\n"));
        return soap->error;
      }
      if (soap_putmimehdr(soap, content))
        return soap->error;
      if (!size)
      {
        if ((soap->mode & SOAP_ENC_PLAIN) || (soap->mode & SOAP_IO) == SOAP_IO_CHUNK || (soap->mode & SOAP_IO) == SOAP_IO_STORE)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Chunked streaming MIME\n"));
          do
          {
            size = soap->fmimeread(soap, handle, soap->tmpbuf, sizeof(soap->tmpbuf));
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "fmimeread returned %lu bytes\n", (unsigned long)size));
            err = soap_send_raw(soap, soap->tmpbuf, size);
          } while (!err && size);
        }
        else
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Error: cannot chunk streaming MIME (no HTTP chunking)\n"));
        }
      }
      else
      {
        do
        {
          size_t bufsize;
          if (size < sizeof(soap->tmpbuf))
            bufsize = size;
          else
            bufsize = sizeof(soap->tmpbuf);
          bufsize = soap->fmimeread(soap, handle, soap->tmpbuf, bufsize);
          if (!bufsize)
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "fmimeread failed: insufficient data (%lu bytes remaining from %lu bytes)\n", (unsigned long)size, (unsigned long)content->size));
            err = SOAP_EOF;
            break;
          }
          err = soap_send_raw(soap, soap->tmpbuf, bufsize);
          size -= bufsize;
        } while (!err && size);
      }
      if (soap->fmimereadclose)
        soap->fmimereadclose(soap, handle);
      if (err)
        return err;
    }
    else
    {
      if (soap_putmimehdr(soap, content)
       || soap_send_raw(soap, content->ptr, content->size))
        return soap->error;
    }
  }
  return soap_send3(soap, "\r\n--", soap->mime.boundary, "--");
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
void
SOAP_FMAC2
soap_set_dime(struct soap *soap)
{
  soap->omode |= SOAP_ENC_DIME;
  soap->dime.first = NULL;
  soap->dime.last = NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
void
SOAP_FMAC2
soap_set_mime(struct soap *soap, const char *boundary, const char *start)
{
  soap->omode |= SOAP_ENC_MIME;
  soap->mime.first = NULL;
  soap->mime.last = NULL;
  soap->mime.boundary = soap_strdup(soap, boundary);
  soap->mime.start = soap_strdup(soap, start);
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
void
SOAP_FMAC2
soap_clr_dime(struct soap *soap)
{
  soap->omode &= ~SOAP_ENC_DIME;
  soap->dime.first = NULL;
  soap->dime.last = NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
void
SOAP_FMAC2
soap_clr_mime(struct soap *soap)
{
  soap->omode &= ~SOAP_ENC_MIME;
  soap->mime.first = NULL;
  soap->mime.last = NULL;
  soap->mime.boundary = NULL;
  soap->mime.start = NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
static int
soap_begin_attachments(struct soap *soap)
{
  if ((soap->mode & SOAP_ENC_MIME) && soap->mime.boundary && soap->mime.start)
  {
    const char *s;
    if (strlen(soap->mime.boundary) + strlen(soap->mime.start) + 140 > sizeof(soap->tmpbuf))
      return soap->error = SOAP_EOM;
    if ((soap->mode & SOAP_ENC_DIME) && !(soap->mode & SOAP_ENC_MTOM))
    {
      s = "application/dime";
    }
    else if (soap->version == 2)
    {
      if ((soap->mode & SOAP_ENC_MTOM))
        s = "application/xop+xml; charset=utf-8; type=\"application/soap+xml\"";
      else
        s = "application/soap+xml; charset=utf-8";
    }
    else if ((soap->mode & SOAP_ENC_MTOM))
    {
      s = "application/xop+xml; charset=utf-8; type=\"text/xml\"";
    }
    else
    {
      s = "text/xml; charset=utf-8";
    }
    (SOAP_SNPRINTF_SAFE(soap->tmpbuf, sizeof(soap->tmpbuf)), "--%s\r\nContent-Type: %s\r\nContent-Transfer-Encoding: binary\r\nContent-ID: %s\r\n\r\n", soap->mime.boundary, s, soap->mime.start);
    if (soap_send(soap, soap->tmpbuf))
      return soap->error;
  }
  if ((soap->mode & SOAP_IO_LENGTH))
    soap->dime.size = (size_t)soap->count; /* DIME in MIME correction, soap->count is small */
  if (!(soap->mode & SOAP_IO_LENGTH) && (soap->mode & SOAP_ENC_DIME))
  {
    if (soap_putdimehdr(soap))
      return soap->error;
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
static int
soap_end_attachments(struct soap *soap)
{
  if ((soap->mode & SOAP_IO_LENGTH) && (soap->mode & SOAP_ENC_DIME) && !(soap->mode & SOAP_ENC_MTOM))
  {
    if (soap->count > 0xFFFFFFFF)
      return soap->error = SOAP_DIME_ERROR;
    soap->dime.size = (size_t)soap->count - soap->dime.size;    /* DIME in MIME correction */
    (SOAP_SNPRINTF(soap->id, sizeof(soap->id), strlen(soap->dime_id_format) + 20), soap->dime_id_format, 0);
    soap->dime.id = soap->id;
    if (soap->local_namespaces && soap->local_namespaces[0].id)
    {
      if (soap->local_namespaces[0].out)
        soap->dime.type = (char*)soap->local_namespaces[0].out;
      else
        soap->dime.type = (char*)soap->local_namespaces[0].ns;
    }
    soap->dime.options = NULL;
    soap->dime.flags = SOAP_DIME_MB | SOAP_DIME_ABSURI;
    if (!soap->dime.first)
      soap->dime.flags |= SOAP_DIME_ME;
    soap->count += 12 + ((strlen(soap->dime.id)+3)&(~3)) + (soap->dime.type ? ((strlen(soap->dime.type)+3)&(~3)) : 0);
  }
  if ((soap->mode & SOAP_ENC_DIME) && !(soap->mode & SOAP_ENC_MTOM))
    return soap_send_raw(soap, SOAP_STR_PADDING, -(long)soap->dime.size&3);
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
static struct soap_multipart*
soap_alloc_multipart(struct soap *soap, struct soap_multipart **first, struct soap_multipart **last, const char *ptr, size_t size)
{
  struct soap_multipart *content;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "New DIME/MIME attachment %p (%lu)\n", (void*)ptr, (unsigned long)size));
  content = (struct soap_multipart*)soap_malloc(soap, sizeof(struct soap_multipart));
  if (content)
  {
    content->next = NULL;
    content->ptr = ptr;
    content->size = size;
    content->id = NULL;
    content->type = NULL;
    content->options = NULL;
    content->encoding = SOAP_MIME_NONE;
    content->location = NULL;
    content->description = NULL;
    if (!*first)
      *first = content;
    if (*last)
      (*last)->next = content;
    *last = content;
  }
  return content;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_set_dime_attachment(struct soap *soap, const char *ptr, size_t size, const char *type, const char *id, unsigned short optype, const char *option)
{
  struct soap_multipart *content = soap_alloc_multipart(soap, &soap->dime.first, &soap->dime.last, ptr, size);
  if (!content)
    return SOAP_EOM;
  content->id = soap_strdup(soap, id);
  content->type = soap_strdup(soap, type);
  content->options = soap_dime_option(soap, optype, option);
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
int
SOAP_FMAC2
soap_set_mime_attachment(struct soap *soap, const char *ptr, size_t size, enum soap_mime_encoding encoding, const char *type, const char *id, const char *location, const char *description)
{
  struct soap_multipart *content = soap_alloc_multipart(soap, &soap->mime.first, &soap->mime.last, ptr, size);
  if (!content)
    return SOAP_EOM;
  content->id = soap_strdup(soap, id);
  content->type = soap_strdup(soap, type);
  content->encoding = encoding;
  content->location = soap_strdup(soap, location);
  content->description = soap_strdup(soap, description);
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
SOAP_FMAC1
struct soap_multipart*
SOAP_FMAC2
soap_next_multipart(struct soap_multipart *content)
{
  if (content)
    return content->next;
  return NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
static void
soap_select_mime_boundary(struct soap *soap)
{
  while (!soap->mime.boundary || soap_valid_mime_boundary(soap))
  {
    char *s = soap->mime.boundary;
    size_t n = 0;
    if (s)
      n = strlen(s);
    if (n < 16)
    {
      n = 64;
      s = soap->mime.boundary = (char*)soap_malloc(soap, n + 1);
      if (!s)
        return;
    }
    *s++ = '=';
    *s++ = '=';
    n -= 4;
    while (n)
    {
      *s++ = soap_base64o[soap_random & 0x3F];
      n--;
    }
    *s++ = '=';
    *s++ = '=';
    *s = '\0';
  }
  if (!soap->mime.start)
    soap->mime.start = "<SOAP-ENV:Envelope>";
}
#endif

/******************************************************************************/

#ifndef WITH_LEANER
static int
soap_valid_mime_boundary(struct soap *soap)
{
  struct soap_multipart *content;
  size_t k;
  if (soap->fmimeread)
    return SOAP_OK;
  k = strlen(soap->mime.boundary);
  for (content = soap->mime.first; content; content = content->next)
  {
    if (content->ptr && content->size >= k)
    {
      const char *p = (const char*)content->ptr;
      size_t i;
      for (i = 0; i < content->size - k; i++, p++)
      {
        if (!strncmp(p, soap->mime.boundary, k))
          return SOAP_ERR;
      }
    }
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

#ifdef WITH_GZIP
static int
soap_getgziphdr(struct soap *soap)
{
  int i;
  soap_wchar c = 0, f = 0;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Get gzip header\n"));
  for (i = 0; i < 9; i++)
  {
    c = soap_get1(soap);
    if (i == 1 && c == 8)
      soap->z_dict = 0;
    if (i == 2)
      f = c;
  }
  if (f & 0x04) /* FEXTRA */
  {
    i = soap_get1(soap);
    i |= soap_get1(soap) << 8;
    while (i-- > 0)
    {
      if ((int)soap_get1(soap) == EOF)
        return soap->error = SOAP_ZLIB_ERROR;
    }
  }
  if (f & 0x08) /* skip FNAME */
  {
    do
    {
      c = soap_get1(soap);
    } while (c && (int)c != EOF);
  }
  if ((int)c != EOF && (f & 0x10)) /* skip FCOMMENT */
  {
    do
    {
      c = soap_get1(soap);
    } while (c && (int)c != EOF);
  }
  if ((int)c != EOF && (f & 0x02)) /* skip FHCRC (CRC32 is used) */
  {
    c = soap_get1(soap);
    if ((int)c != EOF)
      c = soap_get1(soap);
  }
  if ((int)c == EOF)
    return soap->error = SOAP_ZLIB_ERROR;
  return SOAP_OK;
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_begin_serve(struct soap *soap)
{
#ifdef WITH_FASTCGI
  if (FCGI_Accept() < 0)
  {
    soap->error = SOAP_EOF;
    return soap_send_fault(soap);
  }
#endif
  soap_begin(soap);
  if (soap_begin_recv(soap)
   || soap_envelope_begin_in(soap)
   || soap_recv_header(soap)
   || soap_body_begin_in(soap))
  {
    if (soap->error < SOAP_STOP)
    {
#ifdef WITH_FASTCGI
      (void)soap_send_fault(soap);
#else
      return soap_send_fault(soap);
#endif
    }
    return soap_closesock(soap);
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_begin_recv(struct soap *soap)
{
  soap_wchar c;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Initializing for input from socket=%d/fd=%d\n", (int)soap->socket, soap->recvfd));
  soap->error = SOAP_OK;
#ifndef WITH_LEANER
  soap->recverror = SOAP_OK;
#endif
  soap_free_temp(soap);
  soap_set_local_namespaces(soap);
  soap->version = 0;    /* don't assume we're parsing SOAP content by default */
#ifndef WITH_NOIDREF
  soap_free_iht(soap);
#endif
  if ((soap->imode & SOAP_IO) == SOAP_IO_CHUNK)
  {
    soap->imode &= ~SOAP_IO;
    soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_CHUNK;
  }
  soap->imode &= ~(SOAP_ENC_DIME | SOAP_ENC_MIME | SOAP_ENC_ZLIB);
  soap->mode = soap->imode;
  if (!(soap->mode & SOAP_IO_KEEPALIVE))
    soap->keep_alive = 0;
  if (!soap->keep_alive)
    soap->buflen = soap->bufidx = 0;
  soap->null = 0;
  soap->position = 0;
  soap->mustUnderstand = 0;
  soap->shaky = 0;
  soap->ahead = 0;
  soap->peeked = 0;
  soap->level = 0;
  soap->part = SOAP_BEGIN_RECV;
  soap->count = 0;
  soap->length = 0;
  soap->cdata = 0;
  *soap->endpoint = '\0';
  soap->action = NULL;
  soap->header = NULL;
  soap->fault = NULL;
  soap->status = 0;
  soap->fform = NULL;
  soap->body = 1;
#ifndef WITH_LEANER
  soap->dom = NULL;
  soap->dime.count = 0;
  soap->dime.chunksize = 0;
  soap->dime.buflen = 0;
  soap->dime.list = NULL;
  soap->dime.first = NULL;
  soap->dime.last = NULL;
  soap->mime.list = NULL;
  soap->mime.first = NULL;
  soap->mime.last = NULL;
  soap->mime.boundary = NULL;
  soap->mime.start = NULL;
#endif
#ifdef WIN32
#ifndef UNDER_CE
#ifndef WITH_FASTCGI
  if (!soap_valid_socket(soap->socket) && !soap->is && soap->recvfd >= 0) /* Set win32 stdin or soap->recvfd to BINARY, e.g. to support DIME */
#ifdef __BORLANDC__
    setmode(soap->recvfd, _O_BINARY);
#else
    _setmode(soap->recvfd, _O_BINARY);
#endif
#endif
#endif
#endif
#ifdef WITH_ZLIB
  soap->zlib_in = SOAP_ZLIB_NONE;
  soap->zlib_out = SOAP_ZLIB_NONE;
  if (!soap->d_stream)
  {
    soap->d_stream = (z_stream*)SOAP_MALLOC(soap, sizeof(z_stream));
    if (!soap->d_stream)
      return soap->error = SOAP_EOM;
    soap->d_stream->zalloc = Z_NULL;
    soap->d_stream->zfree = Z_NULL;
    soap->d_stream->opaque = Z_NULL;
    soap->d_stream->next_in = Z_NULL;
    soap->d_stream->msg = Z_NULL;
  }
  soap->d_stream->avail_in = 0;
  soap->d_stream->next_out = (Byte*)soap->buf;
  soap->d_stream->avail_out = sizeof(soap->buf);
  soap->z_ratio_in = 1.0;
#endif
#ifdef WITH_OPENSSL
  if (soap->ssl)
    ERR_clear_error();
#endif
#ifndef WITH_LEAN
  soap->start = (ULONG64)time(NULL);
#endif
#ifndef WITH_LEANER
  if (soap->fprepareinitrecv && (soap->error = soap->fprepareinitrecv(soap)) != SOAP_OK)
    return soap->error;
#endif
  c = soap_getchar(soap);
#ifdef WITH_GZIP
  if (c == 0x1F)
  {
    if (soap_getgziphdr(soap))
      return soap->error;
    if (inflateInit2(soap->d_stream, -MAX_WBITS) != Z_OK)
      return soap->error = SOAP_ZLIB_ERROR;
    if (soap->z_dict)
    {
      if (inflateSetDictionary(soap->d_stream, (const Bytef*)soap->z_dict, soap->z_dict_len) != Z_OK)
        return soap->error = SOAP_ZLIB_ERROR;
    }
    soap->zlib_state = SOAP_ZLIB_INFLATE;
    soap->mode |= SOAP_ENC_ZLIB;
    soap->zlib_in = SOAP_ZLIB_GZIP;
    soap->z_crc = crc32(0L, NULL, 0);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "gzip initialized\n"));
    if (!soap->z_buf)
      soap->z_buf = (char*)SOAP_MALLOC(soap, sizeof(soap->buf));
    (void)soap_memcpy((void*)soap->z_buf, sizeof(soap->buf), (const void*)soap->buf, sizeof(soap->buf));
    /* should not chunk over plain transport, so why bother to check? */
    /* if ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK) */
    /*   soap->z_buflen = soap->bufidx; */
    /* else */
    soap->d_stream->next_in = (Byte*)(soap->z_buf + soap->bufidx);
    soap->d_stream->avail_in = (unsigned int)(soap->buflen - soap->bufidx);
    soap->z_buflen = soap->buflen;
    soap->buflen = soap->bufidx;
    c = ' ';
  }
#endif
  while (soap_coblank(c))
    c = soap_getchar(soap);
#ifndef WITH_LEANER
  if (c == '-' && soap_get0(soap) == '-')
  {
    soap->mode |= SOAP_ENC_MIME;
  }
  else if ((c & 0xFFFC) == (SOAP_DIME_VERSION | SOAP_DIME_MB) && (soap_get0(soap) & 0xFFF0) == 0x20)
  {
    soap->mode |= SOAP_ENC_DIME;
  }
  else
#endif
  {
    /* skip BOM */
    if (c == 0xEF && soap_get0(soap) == 0xBB)
    {
      soap_get1(soap);
      c = soap_get1(soap);
      if (c == 0xBF)
      {
        soap->mode &= ~SOAP_ENC_LATIN;
        c = soap_getchar(soap);
      }
      else
      {
        c = (0x0F << 12) | (0xBB << 6) | (c & 0x3F); /* UTF-8 */
      }
    }
    else if ((c == 0xFE && soap_get0(soap) == 0xFF)  /* UTF-16 BE */
          || (c == 0xFF && soap_get0(soap) == 0xFE)) /* UTF-16 LE */
    {
      return soap->error = SOAP_UTF_ERROR;
    }
    /* skip space */
    while (soap_coblank(c))
      c = soap_getchar(soap);
  }
  if ((int)c == EOF)
    return soap->error = SOAP_CHK_EOF;
  soap_unget(soap, c);
#ifndef WITH_NOHTTP
  /* if not XML/MIME/DIME/ZLIB, assume HTTP method or status line */
  if (((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) && !(soap->mode & (SOAP_ENC_MIME | SOAP_ENC_DIME | SOAP_ENC_ZLIB | SOAP_ENC_PLAIN)))
  {
    soap_mode m = soap->imode;
    soap->error = soap->fparse(soap);
    soap->mode = soap->imode; /* if imode is changed, effectuate */
    soap->imode = m; /* restore imode */
    if ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK)
    {
      soap->chunkbuflen = soap->buflen;
      soap->buflen = soap->bufidx;
      soap->chunksize = 0;
    }
#ifdef WITH_ZLIB
    soap->mode &= ~SOAP_ENC_ZLIB;
    if (soap->zlib_in != SOAP_ZLIB_NONE)
    {
#ifdef WITH_GZIP
      if (soap->zlib_in != SOAP_ZLIB_DEFLATE)
      {
        c = soap_get1(soap);
        if (c == (int)EOF)
          return soap->error = SOAP_EOF;
        if (c == 0x1F)
        {
          if (soap_getgziphdr(soap))
            return soap->error;
          if (inflateInit2(soap->d_stream, -MAX_WBITS) != Z_OK)
            return soap->error = SOAP_ZLIB_ERROR;
          soap->z_crc = crc32(0L, NULL, 0);
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "gzip initialized\n"));
        }
        else
        {
          soap_revget1(soap);
          if (inflateInit(soap->d_stream) != Z_OK)
            return soap->error = SOAP_ZLIB_ERROR;
          soap->zlib_in = SOAP_ZLIB_DEFLATE;
        }
      }
      else
#endif
      if (inflateInit(soap->d_stream) != Z_OK)
        return soap->error = SOAP_ZLIB_ERROR;
      if (soap->z_dict)
      {
        if (inflateSetDictionary(soap->d_stream, (const Bytef*)soap->z_dict, soap->z_dict_len) != Z_OK)
          return soap->error = SOAP_ZLIB_ERROR;
      }
      soap->zlib_state = SOAP_ZLIB_INFLATE;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Inflate initialized\n"));
      soap->mode |= SOAP_ENC_ZLIB;
      if (!soap->z_buf)
        soap->z_buf = (char*)SOAP_MALLOC(soap, sizeof(soap->buf));
      (void)soap_memcpy((void*)soap->z_buf, sizeof(soap->buf), (const void*)soap->buf, sizeof(soap->buf));
      soap->d_stream->next_in = (Byte*)(soap->z_buf + soap->bufidx);
      soap->d_stream->avail_in = (unsigned int)(soap->buflen - soap->bufidx);
      soap->z_buflen = soap->buflen;
      soap->buflen = soap->bufidx;
    }
#endif
#ifndef WITH_LEANER
    if (soap->fpreparerecv && (soap->mode & SOAP_IO) != SOAP_IO_CHUNK && soap->buflen > soap->bufidx)
    {
      int r;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Invoking fpreparerecv\n"));
      r = soap->fpreparerecv(soap, soap->buf + soap->bufidx, soap->buflen - soap->bufidx);
      if (r)
        return soap->error = r;
    }
#endif
    if (soap->error && soap->error < SOAP_STOP)
    {
      if (soap->status >= 200 && soap->status < 600)
      {
        const char *s = soap_http_get_body(soap, NULL);
        (void)soap_end_recv(soap);
        if (soap->status >= 300)
          soap->keep_alive = 0; /* to force close */
        return soap_set_receiver_error(soap, "HTTP Error", s, soap->status);
      }
      return soap->error;
    }
    if (!soap->body && soap->status >= 200 && soap->status < 600)
      return soap->error = soap->status; /* client side received HTTP status code */
    if (soap->status > SOAP_POST)
    {
      soap->fform = NULL;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Invoking http method handler\n"));
      switch (soap->status)
      {
        case SOAP_GET:
          if (soap_http_skip_body(soap) || soap_end_recv(soap))
            return soap->error;
          soap->error = soap->fget(soap);
          break;
        case SOAP_PUT:
          soap->error = soap->fput(soap);
          break;
        case SOAP_PATCH:
          soap->error = soap->fpatch(soap);
          break;
        case SOAP_DEL:
          if (soap_http_skip_body(soap) || soap_end_recv(soap))
            return soap->error;
          soap->error = soap->fdel(soap);
          break;
        case SOAP_HEAD:
          if (soap_http_skip_body(soap) || soap_end_recv(soap))
            return soap->error;
          soap->error = soap->fhead(soap);
          break;
        case SOAP_OPTIONS:
          if (soap_http_skip_body(soap) || soap_end_recv(soap))
            return soap->error;
          soap->error = soap->fopt(soap);
          break;
        default:
          if (soap_http_skip_body(soap) || soap_end_recv(soap))
            return soap->error;
          return 405;
      }
      if (soap->error == SOAP_OK)
        return soap->error = SOAP_STOP; /* prevents further processing */
      if (soap->error != SOAP_FORM || !soap->fform) /* continue if handler returned SOAP_FORM */
        return soap->error;
      soap->error = SOAP_OK;
    }
    if (soap->fform)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Invoking http form handler\n"));
      soap->error = soap->fform(soap);
      if (soap->error == SOAP_OK)
        return soap->error = SOAP_STOP; /* prevents further processing */
      if (soap->status != SOAP_POST || soap->error != 404) /* continue with POST if handler returned HTTP not found */
        return soap->error;
      soap->error = SOAP_OK;
    }
    if (!soap->body)
      return soap->error = SOAP_NO_DATA;
  }
#endif
#ifndef WITH_LEANER
  if ((soap->mode & SOAP_ENC_MIME))
  {
    do /* skip preamble */
    {
      c = soap_getchar(soap);
      if ((int)c == EOF)
        return soap->error = SOAP_CHK_EOF;
    } while (c != '-' || soap_get0(soap) != '-');
    soap_unget(soap, c);
    if (soap_getmimehdr(soap))
      return soap->error;
    if (soap->mime.start)
    {
      do
      {
        if (!soap->mime.last->id)
          break;
        if (!soap_match_cid(soap, soap->mime.start, soap->mime.last->id))
          break;
      } while (soap_recv_mime_attachment(soap, NULL));
    }
    if (soap_http_header_attribute(soap, soap->mime.first->type, "application/dime"))
      soap->mode |= SOAP_ENC_DIME;
  }
  if ((soap->mode & SOAP_ENC_DIME))
  {
    if (soap_getdimehdr(soap))
      return soap->error;
    if ((soap->dime.flags & SOAP_DIME_CF))
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Chunked SOAP in DIME message\n"));
      soap->dime.chunksize = soap->dime.size;
      if (soap->buflen - soap->bufidx >= soap->dime.chunksize)
      {
        soap->dime.buflen = soap->buflen;
        soap->buflen = soap->bufidx + soap->dime.chunksize;
      }
      else
      {
        soap->dime.chunksize -= soap->buflen - soap->bufidx;
      }
    }
    soap->count = soap->buflen - soap->bufidx;
    if (soap->recv_maxlength && soap->count > soap->recv_maxlength)
      return soap->error = SOAP_EOF;
  }
#endif
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_envelope_begin_out(struct soap *soap)
{
  if (soap->version == 0)
    return SOAP_OK;
  soap->part = SOAP_IN_ENVELOPE;
  return soap_element_begin_out(soap, "SOAP-ENV:Envelope", 0, NULL);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_envelope_end_out(struct soap *soap)
{
  if (soap->version == 0)
    return SOAP_OK;
  if (soap_element_end_out(soap, "SOAP-ENV:Envelope")
   || soap_send_raw(soap, "\r\n", 2))   /* 2.8: always emit \r\n */
    return soap->error;
  soap->part = SOAP_END_ENVELOPE;
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_http_has_body(struct soap *soap)
{
  return soap->length || (soap->mode & SOAP_ENC_ZLIB) || (soap->mode & SOAP_IO) == SOAP_IO_CHUNK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_http_skip_body(struct soap *soap)
{
  ULONG64 k = soap->length;
  /* check HTTP body, return "" if none */
  if (!k && !(soap->mode & SOAP_ENC_ZLIB) && (soap->mode & SOAP_IO) != SOAP_IO_CHUNK)
    return SOAP_OK;
  /* do not consume DIME or MIME attachments, leave this to soap_end_recv */
  if ((soap->mode & SOAP_ENC_DIME) || (soap->mode & SOAP_ENC_MIME))
    return SOAP_OK;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Skipping HTTP body (mode=0x%x)\n", soap->mode));
  if (k && !(soap->mode & SOAP_ENC_ZLIB))
  {
    ULONG64 i;
    soap->length = 0;
    for (i = 0; i < k; i++)
    {
      soap_wchar c = soap_get1(soap);
      if ((int)c == EOF)
        break;
    }
  }
  else
  {
    for (;;)
    {
      soap_wchar c = soap_get1(soap);
      if ((int)c == EOF)
        break;
    }
  }
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
char *
SOAP_FMAC2
soap_http_get_body(struct soap *soap, size_t *len)
{
  return soap_http_get_body_prefix(soap, len, NULL);
}

/******************************************************************************/

SOAP_FMAC1
char *
SOAP_FMAC2
soap_http_get_form(struct soap *soap)
{
  return soap_http_get_body_prefix(soap, NULL, "?");
}
  
/******************************************************************************/

SOAP_FMAC1
char *
SOAP_FMAC2
soap_http_get_body_prefix(struct soap *soap, size_t *len, const char *prefix)
{
  char *s;
  ULONG64 k = soap->length;
  size_t n = 0;
  if (!prefix)
    prefix = SOAP_STR_EOS;
  else
    n = strlen(prefix);
  if (len)
    *len = 0;
  /* check HTTP body, return "" if none */
  if (!k && !(soap->mode & SOAP_ENC_ZLIB) && (soap->mode & SOAP_IO) != SOAP_IO_CHUNK)
    return soap_strdup(soap, prefix);
  /* do not consume DIME or MIME attachments, leave this to soap_end_recv */
  if ((soap->mode & SOAP_ENC_DIME) || (soap->mode & SOAP_ENC_MIME))
    return soap_strdup(soap, prefix);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Parsing HTTP body, prefixed with '%s' (mode=0x%x)\n", prefix, soap->mode));
  if (k && !(soap->mode & SOAP_ENC_ZLIB))
  {
    char *t;
    size_t j;
    soap->length = 0;
    /* http content length != 0 and uncompressed body and body length does not exceed size_t max size */
    if ((SOAP_MAXALLOCSIZE != 0 && n + k > SOAP_MAXALLOCSIZE) || n + k > (ULONG64)((size_t)-2))
    {
      soap->error = SOAP_EOM;
      return NULL;
    }
    j = (size_t)k; /* safe cast: k is in size_t range as guarded above */
    s = t = (char*)soap_malloc(soap, j + n + 1);
    if (s)
    {
      size_t i = 0;
      (void)soap_memcpy(t, n, prefix, n);
      t += n;
      while (i < j)
      {
        size_t m;
        if (soap->bufidx >= soap->buflen)
          if (soap_recv(soap))
            break;
        if (soap->buflen - soap->bufidx > j - i)
          m = j - i;
        else
          m = soap->buflen - soap->bufidx;
        (void)soap_memcpy(t, j + n + 1 - i, soap->buf + soap->bufidx, m);
        soap->bufidx += m;
        t += m;
        i += m;
      }
      *t = '\0';
      if (len)
        *len = n + i;
    }
    else
    {
      soap->error = SOAP_EOM;
      return NULL;
    }
  }
  else
  {
    size_t i, l = 0;
    if (soap_alloc_block(soap) == NULL)
      return NULL;
    if (n)
    {
      s = (char*)soap_push_block(soap, NULL, n);
      if (!s)
        return NULL;
      (void)soap_memcpy(s, n, prefix, n);
      l += n;
    }
    for (;;)
    {
      size_t k = SOAP_BLKLEN;
      s = (char*)soap_push_block(soap, NULL, k);
      if (!s)
        return NULL;
      i = 0;
      while (i < k)
      {
        size_t m;
        if (soap->bufidx >= soap->buflen)
          if (soap_recv(soap))
            goto end;
        if (soap->buflen - soap->bufidx > k - i)
          m = k - i;
        else
          m = soap->buflen - soap->bufidx;
        (void)soap_memcpy(s, k - i, soap->buf + soap->bufidx, m);
        soap->bufidx += m;
        s += m;
        i += m;
        if (l + m < l)
        {
          soap->error = SOAP_EOM;
          return NULL;
        }
        l += m;
      }
    }
end:
    *s = '\0';
    if (len)
      *len = l;
    soap_size_block(soap, NULL, i + 1);
    s = (char*)soap_save_block(soap, NULL, NULL, 0);
  }
  return s;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_envelope_begin_in(struct soap *soap)
{
  soap->part = SOAP_IN_ENVELOPE;
  if (soap_element_begin_in(soap, "SOAP-ENV:Envelope", 0, NULL))
  {
    if (soap->error == SOAP_TAG_MISMATCH)
    {
      if (!soap_element_begin_in(soap, "Envelope", 0, NULL))
        soap->error = SOAP_VERSIONMISMATCH;
      else if (soap->status == 0
           || (soap->status >= 200 && soap->status <= 299)
           || soap->status == 400
           || soap->status == 500)
        return SOAP_OK; /* allow non-SOAP (REST) XML content to be captured */
      soap->error = soap->status;
    }
    else if (soap->status)
    {
      soap->error = soap->status;
    }
    return soap->error;
  }
  soap_version(soap);
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_envelope_end_in(struct soap *soap)
{
  if (soap->version == 0)
    return SOAP_OK;
  soap->part = SOAP_END_ENVELOPE;
  return soap_element_end_in(soap, "SOAP-ENV:Envelope");
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_body_begin_out(struct soap *soap)
{
  if (soap->version == 1)
    soap->encoding = 1;
#ifndef WITH_LEAN
  if ((soap->mode & SOAP_SEC_WSUID) && soap_set_attr(soap, "wsu:Id", "Body", 1))
    return soap->error;
#endif
  if (soap->version == 0)
    return SOAP_OK;
  soap->part = SOAP_IN_BODY;
  return soap_element_begin_out(soap, "SOAP-ENV:Body", 0, NULL);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_body_end_out(struct soap *soap)
{
  if (soap->version == 0)
    return SOAP_OK;
  if (soap_element_end_out(soap, "SOAP-ENV:Body"))
    return soap->error;
  soap->part = SOAP_END_BODY;
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_body_begin_in(struct soap *soap)
{
  if (soap->version == 0)
    return SOAP_OK;
  soap->part = SOAP_IN_BODY;
  if (soap_element_begin_in(soap, "SOAP-ENV:Body", 0, NULL))
    return soap->error;
  if (!soap->body)
    soap->part = SOAP_NO_BODY;
  return SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_body_end_in(struct soap *soap)
{
  if (soap->version == 0)
    return SOAP_OK;
  if (soap->part == SOAP_NO_BODY)
    return soap->error = SOAP_OK;
  soap->part = SOAP_END_BODY;
  return soap_element_end_in(soap, "SOAP-ENV:Body");
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_recv_header(struct soap *soap)
{
  if (soap_getheader(soap) && soap->error == SOAP_TAG_MISMATCH)
    soap->error = SOAP_OK;
  if (soap->error == SOAP_OK && soap->fheader)
    soap->error = soap->fheader(soap);
  return soap->error;
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_set_endpoint(struct soap *soap, const char *endpoint)
{
  const char *s, *t;
  size_t i, n;
  soap->endpoint[0] = '\0';
  soap->host[0] = '\0';
  soap->path[0] = '/';
  soap->path[1] = '\0';
  soap->port = 80;
  if (!endpoint || !*endpoint)
    return;
#ifdef WITH_OPENSSL
  if (!soap_tag_cmp(endpoint, "https:*"))
    soap->port = 443;
#endif
  soap_strcpy(soap->endpoint, sizeof(soap->endpoint), endpoint);
  s = strchr(endpoint, ':');
  if (s && s[1] == '/' && s[2] == '/')
    s += 3;
  else
    s = endpoint;
#if !defined(WITH_NOHTTP) || !defined(WITH_LEANER)
  t = strchr(s, '@');
  if (t && *s != ':' && *s != '@')
  {
    size_t l = t - s + 1;
    char *r = (char*)soap_malloc(soap, l);
    n = s - endpoint;
    if (r)
    {
      s = soap_decode(r, l, s, ":@");
      soap->userid = r;
      soap->passwd = SOAP_STR_EOS;
      if (*s == ':')
      {
        s++;
        if (*s != '@' && s < t)
        {
          l = t - s + 1;
          r = r + strlen(r) + 1;
          s = soap_decode(r, l, s, "@");
          soap->passwd = r;
        }
      }
    }
    s++;
    soap_strcpy(soap->endpoint + n, sizeof(soap->endpoint) - n, s);
  }
#endif
  n = strlen(s);
  if (n >= sizeof(soap->host))
    n = sizeof(soap->host) - 1;
#ifdef WITH_IPV6
  if (s[0] == '[')
  {
    s++;
    for (i = 0; i < n; i++)
    {
      if (s[i] == ']')
      {
        s++;
        --n;
        break;
      }
      soap->host[i] = s[i];
    }
  }
  else
  {
    for (i = 0; i < n; i++)
    {
      soap->host[i] = s[i];
      if (s[i] == '/' || s[i] == ':' || s[i] == '?')
        break;
    }
  }
#else
  for (i = 0; i < n; i++)
  {
    soap->host[i] = s[i];
    if (s[i] == '/' || s[i] == ':' || s[i] == '?')
      break;
  }
#endif
  soap->host[i] = '\0';
  if (s[i] == ':')
  {
    soap->port = (int)soap_strtol(s + i + 1, NULL, 10);
    for (i++; i < n; i++)
      if (s[i] == '/')
        break;
  }
  if (i < n && s[i])
    soap_strcpy(soap->path, sizeof(soap->path), s + i);
  if (soap->override_host && *soap->override_host)
  {
    soap_strcpy(soap->host, sizeof(soap->host), soap->override_host);
    if (soap->override_port)
      soap->port = soap->override_port;
  }
  if (soap->userid && !soap->authrealm)
    soap->authrealm = soap->host;
}

/******************************************************************************/

#ifndef WITH_NOHTTP
SOAP_FMAC1
int
SOAP_FMAC2
soap_GET(struct soap *soap, const char *endpoint, const char *action)
{
  return soap_connect_command(soap, SOAP_GET, endpoint, action);
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
SOAP_FMAC1
int
SOAP_FMAC2
soap_PUT(struct soap *soap, const char *endpoint, const char *action, const char *type)
{
  if (!soap->http_content)
    soap->http_content = type;
  if ((soap->omode & SOAP_IO) != SOAP_IO_CHUNK)
  {
    soap->omode &= ~SOAP_IO;
    soap->omode |= SOAP_IO_STORE;
  }
  return soap_connect_command(soap, SOAP_PUT, endpoint, action);
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
SOAP_FMAC1
int
SOAP_FMAC2
soap_POST(struct soap *soap, const char *endpoint, const char *action, const char *type)
{
  if (!soap->http_content)
    soap->http_content = type;
  if ((soap->omode & SOAP_IO) != SOAP_IO_CHUNK)
  {
    soap->omode &= ~SOAP_IO;
    soap->omode |= SOAP_IO_STORE;
  }
  return soap_connect_command(soap, SOAP_POST_FILE, endpoint, action);
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
SOAP_FMAC1
int
SOAP_FMAC2
soap_PATCH(struct soap *soap, const char *endpoint, const char *action, const char *type)
{
  if (!soap->http_content)
    soap->http_content = type;
  if ((soap->omode & SOAP_IO) != SOAP_IO_CHUNK)
  {
    soap->omode &= ~SOAP_IO;
    soap->omode |= SOAP_IO_STORE;
  }
  return soap_connect_command(soap, SOAP_PATCH, endpoint, action);
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
SOAP_FMAC1
int
SOAP_FMAC2
soap_DELETE(struct soap *soap, const char *endpoint)
{
  if (soap_connect_command(soap, SOAP_DEL, endpoint, NULL)
   || soap_recv_empty_response(soap))
    return soap_closesock(soap);
  return SOAP_OK;
}
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_connect(struct soap *soap, const char *endpoint, const char *action)
{
  return soap_connect_command(soap, SOAP_POST, endpoint, action);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_connect_command(struct soap *soap, int http_command, const char *endpoints, const char *action)
{
  if (endpoints)
  {
    int retry = soap->connect_retry;
    unsigned int backoff = 1;
    for (;;)
    {
      struct timeval tv;
      const char *s;
      s = strchr(endpoints, ' ');
      if (s)
      {
        size_t l = strlen(endpoints);
        char *endpoint = NULL;
        if (SOAP_MAXALLOCSIZE == 0 || l <= SOAP_MAXALLOCSIZE)
          endpoint = (char*)SOAP_MALLOC(soap, l + 1);
        if (!endpoint)
          return soap->error = SOAP_EOM;
        for (;;)
        {
          (void)soap_strncpy(endpoint, l + 1, endpoints, s - endpoints);
          endpoint[s - endpoints] = '\0';
          if (soap_try_connect_command(soap, http_command, endpoint, action) != SOAP_TCP_ERROR)
            break;
          while (*s == ' ')
            s++;
          if (!*s)
            break;
          soap->error = SOAP_OK;
          endpoints = s;
          s = strchr(endpoints, ' ');
          if (!s)
            s = endpoints + strlen(endpoints);
        }
        SOAP_FREE(soap, endpoint);
      }
      else
      {
        soap_try_connect_command(soap, http_command, endpoints, action);
      }
      if (soap->error != SOAP_TCP_ERROR || retry <= 0)
        break;
      soap->error = SOAP_OK;
      tv.tv_sec = backoff;
      tv.tv_usec = 0;
      select(0, NULL, NULL, NULL, &tv);
      if (backoff < 32)
        backoff *= 2;
      --retry;
    }
  }
  return soap->error;
}

/******************************************************************************/

static int
soap_try_connect_command(struct soap *soap, int http_command, const char *endpoint, const char *action)
{
  char host[sizeof(soap->host)];
  int port;
  ULONG64 count;
  soap->error = SOAP_OK;
  soap_reset_errno;
  soap->errnum = 0;
  (void)soap_memcpy(host, sizeof(host), soap->host, sizeof(soap->host)); /* save previous host name: if != then reconnect */
  port = soap->port; /* save previous port to compare */
  soap->status = http_command;
  soap_set_endpoint(soap, endpoint);
  soap->action = soap_strdup(soap, action);
#ifndef WITH_LEANER
  if (soap->fconnect)
  {
    soap->error = soap->fconnect(soap, endpoint, soap->host, soap->port);
    if (soap->error)
      return soap->error;
  }
  else
#endif
  if (soap->fopen && *soap->host)
  {
    if (!soap->keep_alive || !soap_valid_socket(soap->socket) || strcmp(soap->host, host) || soap->port != port || !soap->fpoll || soap->fpoll(soap))
    {
      soap->error = SOAP_OK;
#ifndef WITH_LEAN
      if (!strncmp(endpoint, "soap.udp:", 9) || !strncmp(endpoint, "udp:", 4))
      {
        soap->omode |= SOAP_IO_UDP;
      }
      else
#endif
      {
        soap->keep_alive = 0; /* to force close */
        soap->omode &= ~SOAP_IO_UDP; /* to force close */
      }
      (void)soap_closesock(soap);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Connect/reconnect to '%s' host='%s' path='%s' port=%d\n", endpoint?endpoint:"(null)", soap->host, soap->path, soap->port));
      if (!soap->keep_alive || !soap_valid_socket(soap->socket))
      {
        soap->socket = soap->fopen(soap, endpoint, soap->host, soap->port);
        if (!soap_valid_socket(soap->socket) || soap->error)
        {
          if (soap->error)
            return soap->error;
          return soap->error = SOAP_TCP_ERROR;
        }
        soap->keep_alive = -((soap->omode & SOAP_IO_KEEPALIVE) != 0);
      }
    }
  }
#ifdef WITH_NTLM
  if (soap_ntlm_handshake(soap, SOAP_GET, endpoint, soap->host, soap->port))
    return soap->error;
#endif
  count = soap_count_attachments(soap);
  if (soap_init_send(soap))
    return soap->error;
#ifndef WITH_NOHTTP
  if (http_command == SOAP_GET || http_command == SOAP_DEL || http_command == SOAP_HEAD || http_command == SOAP_OPTIONS)
  {
    soap->mode &= ~SOAP_IO;
    soap->mode |= SOAP_IO_BUFFER;
  }
  if ((soap->mode & SOAP_IO) != SOAP_IO_STORE && !(soap->mode & SOAP_ENC_PLAIN) && endpoint)
  {
    soap_mode k = soap->mode;
    soap->mode &= ~(SOAP_IO | SOAP_ENC_ZLIB);
    if ((k & SOAP_IO) != SOAP_IO_FLUSH)
      soap->mode |= SOAP_IO_BUFFER;
    soap->error = soap->fpost(soap, endpoint, soap->host, soap->port, soap->path, action, count);
    if (soap->error)
      return soap->error;
    if ((k & SOAP_IO) == SOAP_IO_CHUNK)
    {
      if (soap_flush(soap))
        return soap->error;
    }
    soap->mode = k;
  }
  if (http_command == SOAP_GET || http_command == SOAP_DEL || http_command == SOAP_HEAD || http_command == SOAP_OPTIONS)
    return soap_end_send_flush(soap);
#endif
#ifndef WITH_LEANER
  if (soap_begin_attachments(soap))
    return soap->error;
#endif
  return SOAP_OK;
}

/******************************************************************************/

#ifdef WITH_NTLM
static int
soap_ntlm_handshake(struct soap *soap, int command, const char *endpoint, const char *host, int port)
{
  /* requires libntlm from http://www.nongnu.org/libntlm/ */
  const char *userid = (soap->proxy_userid ? soap->proxy_userid : soap->userid);
  const char *passwd = (soap->proxy_passwd ? soap->proxy_passwd : soap->passwd);
  struct SOAP_ENV__Header *oldheader;
  if (soap->ntlm_challenge && userid && passwd && soap->authrealm)
  {
    tSmbNtlmAuthRequest req;  
    tSmbNtlmAuthResponse res;
    tSmbNtlmAuthChallenge ch;
    int keep_alive = soap->keep_alive;
    ULONG64 length = soap->length;
    ULONG64 count = soap->count;
    soap_mode m = soap->mode;
    soap_mode om = soap->omode;
    int status = soap->status;
    char *action = soap->action;
    short version = soap->version;
    const char *http_content = soap->http_content;
    const char *http_extra_header = soap->http_extra_header;
    const char *bearer = soap->bearer;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "NTLM '%s'\n", soap->ntlm_challenge));
    if (!*soap->ntlm_challenge)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "NTLM S->C Type 1: received NTLM authentication challenge from server\n"));
      /* S -> C   401 Unauthorized
                  WWW-Authenticate: NTLM
      */
      buildSmbNtlmAuthRequest(&req, userid, soap->authrealm);
      soap->ntlm_challenge = soap_s2base64(soap, (unsigned char*)(void*)&req, NULL, SmbLength(&req));
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "NTLM C->S Type 2: sending NTLM authorization to server\nAuthorization: NTLM %s\n", soap->ntlm_challenge));
      /* C -> S   GET ...
                  Authorization: NTLM TlRMTVNTUAABAAAAA7IAAAoACgApAAAACQAJACAAAABMSUdIVENJVFlVUlNBLU1JTk9S
      */
      soap->omode = SOAP_IO_BUFFER;
      if (soap_init_send(soap))
        return soap->error;
      if (!soap->keep_alive)
        soap->keep_alive = -1; /* client keep alive */
      soap->status = command; /* GET or CONNECT for proxy */
      if (soap->fpost(soap, endpoint, host, port, soap->path, soap->action, 0)
       || soap_end_send_flush(soap))
        return soap->error;
      soap->mode = m;
      soap->keep_alive = keep_alive;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "NTLM S->C Type 2: waiting on server NTLM response\n"));
      oldheader = soap->header;
      if (soap_begin_recv(soap))
        if (soap->error == SOAP_EOF)
          return soap->error;
      (void)soap_end_recv(soap);
      soap->header = oldheader;
      soap->length = length;
      if (soap->status != 401 && soap->status != 407)
        return soap->error = SOAP_NTLM_ERROR;
      soap->error = SOAP_OK;
    }
    /* S -> C   401 Unauthorized
                WWW-Authenticate: NTLM TlRMTVNTUAACAAAAAAAAACgAAAABggAAU3J2Tm9uY2UAAAAAAAAAAA==
    */
    soap_base642s(soap, soap->ntlm_challenge, (char*)&ch, sizeof(tSmbNtlmAuthChallenge), NULL);
    buildSmbNtlmAuthResponse(&ch, &res, userid, passwd);
    soap->ntlm_challenge = soap_s2base64(soap, (unsigned char*)(void*)&res, NULL, SmbLength(&res));
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "NTLM C->S Type 3: sending NTLM authorization to server\nAuthorization: NTLM %s\n", soap->ntlm_challenge));
    /* C -> S   GET ...
                Authorization: NTLM TlRMTVNTUAADAAAAGAAYAHIAAAAYABgAigAAABQAFABAAAAADAAMAFQAAAASABIAYAAAAAAAAACiAAAAAYIAAFUAUgBTAEEALQBNAEkATgBPAFIAWgBhAHAAaABvAGQATABJAEcASABUAEMASQBUAFkArYfKbe/jRoW5xDxHeoxC1gBmfWiS5+iX4OAN4xBKG/IFPwfH3agtPEia6YnhsADT
    */
    soap->userid = NULL;
    soap->passwd = NULL;
    soap->proxy_userid = NULL;
    soap->proxy_passwd = NULL;
    soap->keep_alive = keep_alive;
    soap->length = length;
    soap->count = count;
    soap->mode = m;
    soap->omode = om;
    soap->status = status;
    soap->action = action;
    soap->version = version;
    soap->http_content = http_content;
    soap->http_extra_header = http_extra_header;
    soap->bearer = bearer;
  }
  return SOAP_OK;
}
#endif

/******************************************************************************/

SOAP_FMAC1
char*
SOAP_FMAC2
soap_s2base64(struct soap *soap, const unsigned char *s, char *t, int n)
{
  int i;
  unsigned long m;
  char *p;
  if (!t)
    t = (char*)soap_malloc(soap, (n + 2) / 3 * 4 + 1);
  if (!t)
    return NULL;
  p = t;
  t[0] = '\0';
  if (!s)
    return p;
  for (; n > 2; n -= 3, s += 3)
  {
    m = s[0];
    m = (m << 8) | s[1];
    m = (m << 8) | s[2];
    for (i = 4; i > 0; m >>= 6)
      t[--i] = soap_base64o[m & 0x3F];
    t += 4;
  }
  t[0] = '\0';
  if (n > 0) /* 0 < n <= 2 implies that t[0..4] is allocated (base64 scaling formula) */
  {
    m = 0;
    for (i = 0; i < n; i++)
      m = (m << 8) | *s++;
    for (; i < 3; i++)
      m <<= 8;
    for (i = 4; i > 0; m >>= 6)
      t[--i] = soap_base64o[m & 0x3F];
    for (i = 3; i > n; i--)
      t[i] = '=';
    t[4] = '\0';
  }
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_base642s(struct soap *soap, const char *s, char *t, size_t l, int *n)
{
  size_t i, j;
  soap_wchar c;
  unsigned long m;
  const char *p;
  if (!s || !*s)
  {
    if (n)
      *n = 0;
    if (soap->error)
      return NULL;
    return SOAP_NON_NULL;
  }
  if (!t)
  {
    l = (strlen(s) + 3) / 4 * 3 + 1;    /* space for raw binary and \0 */
    t = (char*)soap_malloc(soap, l);
  }
  if (!t)
    return NULL;
  p = t;
  if (n)
    *n = 0;
  for (i = 0; ; i += 3, l -= 3)
  {
    m = 0;
    j = 0;
    while (j < 4)
    {
      c = *s++;
      if (c == '=' || !c)
      {
        if (l >= j - 1)
        {
          switch (j)
          {
            case 2:
              *t++ = (char)((m >> 4) & 0xFF);
              i++;
              l--;
              break;
            case 3:
              *t++ = (char)((m >> 10) & 0xFF);
              *t++ = (char)((m >> 2) & 0xFF);
              i += 2;
              l -= 2;
          }
        }
        if (n)
          *n = (int)i;
        if (l)
          *t = '\0';
        return p;
      }
      c -= '+';
      if (c >= 0 && c <= 79)
      {
        int b = soap_base64i[c];
        if (b >= 64)
        {
          soap->error = SOAP_TYPE;
          return NULL;
        }
        m = (m << 6) + b;
        j++;
      }
      else if (!soap_coblank(c + '+'))
      {
        soap->error = SOAP_TYPE;
        return NULL;
      }
    }
    if (l < 3)
    {
      if (n)
        *n = (int)i;
      if (l)
        *t = '\0';
      return p;
    }
    *t++ = (char)((m >> 16) & 0xFF);
    *t++ = (char)((m >> 8) & 0xFF);
    *t++ = (char)(m & 0xFF);
  }
}

/******************************************************************************/

SOAP_FMAC1
char*
SOAP_FMAC2
soap_s2hex(struct soap *soap, const unsigned char *s, char *t, int n)
{
  char *p;
  if (!t)
    t = (char*)soap_malloc(soap, 2 * n + 1);
  if (!t)
    return NULL;
  p = t;
  t[0] = '\0';
  if (s)
  {
    for (; n > 0; n--)
    {
      int m = *s++;
      *t++ = (char)((m >> 4) + (m > 159 ? 'a' - 10 : '0'));
      m &= 0x0F;
      *t++ = (char)(m + (m > 9 ? 'a' - 10 : '0'));
    }
  }
  *t++ = '\0';
  return p;
}

/******************************************************************************/

SOAP_FMAC1
const char*
SOAP_FMAC2
soap_hex2s(struct soap *soap, const char *s, char *t, size_t l, int *n)
{
  const char *p;
  if (!s || !*s)
  {
    if (n)
      *n = 0;
    if (soap->error)
      return NULL;
    return SOAP_NON_NULL;
  }
  if (!t)
  {
    l = strlen(s) / 2 + 1;      /* make sure enough space for \0 */
    t = (char*)soap_malloc(soap, l);
  }
  if (!t)
    return NULL;
  p = t;
  while (l)
  {
    int d1, d2;
    d1 = *s++;
    if (!d1)
      break;
    d2 = *s++;
    if (!d2)
      break;
    *t++ = (char)(((d1 >= 'A' ? (d1 & 0x7) + 9 : d1 - '0') << 4) + (d2 >= 'A' ? (d2 & 0x7) + 9 : d2 - '0'));
    l--;
  }
  if (n)
    *n = (int)(t - p);
  if (l)
    *t = '\0';
  return p;
}

/******************************************************************************/

#ifndef WITH_NOHTTP
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_http_content_type(struct soap *soap, int status)
{
  if (soap->status != SOAP_GET && soap->status != SOAP_DEL && soap->status != SOAP_CONNECT)
  {
    const char *s = "text/xml; charset=utf-8";
#ifndef WITH_LEANER
    const char *r = NULL;
    size_t n;
#endif
    if (((status >= SOAP_FILE && status < SOAP_FILE + 600) || soap->status == SOAP_PUT || soap->status == SOAP_POST_FILE || soap->status == SOAP_PATCH) && soap->http_content && *soap->http_content && !strchr(soap->http_content, 10) && !strchr(soap->http_content, 13))
      s = soap->http_content;
    else if (status == SOAP_HTML)
      s = "text/html; charset=utf-8";
    else if (soap->version == 2)
      s = "application/soap+xml; charset=utf-8";
    soap->http_content = NULL; /* use http_content once (assign new value before each call) */
#ifndef WITH_LEANER
    if (soap->mode & (SOAP_ENC_DIME | SOAP_ENC_MTOM))
    {
      if ((soap->mode & SOAP_ENC_MTOM))
      {
        if (soap->version == 2)
          r = "application/soap+xml";
        else
          r = "text/xml";
        s = "application/xop+xml";
      }
      else
      {
        s = "application/dime";
      }
    }
    if ((soap->mode & SOAP_ENC_MIME) && soap->mime.boundary)
    {
      const char *t;
      size_t l;
      (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(soap->mime.boundary) + 53), "multipart/related; charset=utf-8; boundary=\"%s\"; type=\"", soap->mime.boundary);
      t = strchr(s, ';');
      if (t)
        n = t - s;
      else
        n = strlen(s);
      l = strlen(soap->tmpbuf);
      if (sizeof(soap->tmpbuf) > l + n)
        (void)soap_strncpy(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, s, n);
      if (soap->mime.start)
      {
        l = strlen(soap->tmpbuf);
        (SOAP_SNPRINTF(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, strlen(soap->mime.start) + 10), "\"; start=\"%s", soap->mime.start);
      }
      if (r)
      {
        l = strlen(soap->tmpbuf);
        (SOAP_SNPRINTF(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, strlen(r) + 15), "\"; start-info=\"%s", r);
      }
      l = strlen(soap->tmpbuf);
      if (sizeof(soap->tmpbuf) > l)
        soap_strcpy(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, "\"");
    }
    else
    {
      soap_strcpy(soap->tmpbuf, sizeof(soap->tmpbuf), s);
    }
    if (status == SOAP_OK && soap->version == 2 && soap->action)
    {
      size_t l = strlen(soap->tmpbuf);
      n = strlen(soap->action);
      (SOAP_SNPRINTF(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, n + 11), "; action=\"%s\"", soap->action);
    }
#else
    soap_strcpy(soap->tmpbuf, sizeof(soap->tmpbuf), s);
#endif
    return soap->tmpbuf;
  }
  return NULL;
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
SOAP_FMAC1
int
SOAP_FMAC2
soap_puthttphdr(struct soap *soap, int status, ULONG64 count)
{
  int err = SOAP_OK;
  if (soap_http_content_type(soap, status))
  {
    err = soap->fposthdr(soap, "Content-Type", soap->tmpbuf);
    if (err)
      return err;
#ifdef WITH_ZLIB
    if ((soap->omode & SOAP_ENC_ZLIB))
    {
#ifdef WITH_GZIP
      err = soap->fposthdr(soap, "Content-Encoding", soap->zlib_out == SOAP_ZLIB_DEFLATE ? "deflate" : "gzip");
#else
      err = soap->fposthdr(soap, "Content-Encoding", "deflate");
#endif
      if (err)
        return err;
    }
#endif
#ifndef WITH_LEANER
    if ((soap->omode & SOAP_IO) == SOAP_IO_CHUNK)
    {
      err = soap->fposthdr(soap, "Transfer-Encoding", "chunked");
    }
    else
#endif
    {
      (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 20), SOAP_ULONG_FORMAT, count);
      err = soap->fposthdr(soap, "Content-Length", soap->tmpbuf);
    }
    if (err)
      return err;
  }
  if (soap->http_extra_header)
  {
    const char *header = soap->http_extra_header;
    soap->http_extra_header = NULL; /* use http_extra_header once (assign new value before each call) */
    if (*header)
    {
      err = soap_send(soap, header);
      if (err)
        return err;
      err = soap_send_raw(soap, "\r\n", 2);
      if (err)
        return err;
    }
  }
  if (soap->keep_alive)
  {
    if (soap->keep_alive > 0 && soap->recv_timeout)
    {
      int t = soap->recv_timeout;
      if (t < 0)
        t = 1;
      (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), 20), "timeout=%d, max=%d", soap->recv_timeout, soap->keep_alive);
      err = soap->fposthdr(soap, "Keep-Alive", soap->tmpbuf);
      if (err)
        return err;
    }
    return soap->fposthdr(soap, "Connection", "keep-alive");
  }
  return soap->fposthdr(soap, "Connection", "close");
}
#endif

/******************************************************************************/

#ifndef WITH_LEAN
static const char*
soap_set_validation_fault(struct soap *soap, const char *s, const char *t)
{
  if (!t)
    t = SOAP_STR_EOS;
  if (*soap->tag)
    (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), strlen(s) + strlen(t) + strlen(soap->tag) + 47), "Validation constraint violation: %s%s in element '%s'", s, t, soap->tag);
  else
    (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), strlen(s) + strlen(t) + 33), "Validation constraint violation: %s%s", s, t);
  return soap->msgbuf;
}
#endif

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_set_fault(struct soap *soap)
{
  const char **c;
  const char **s;
  if (soap->version == 0)
    soap_version(soap);
  c = soap_faultcode(soap);
  s = soap_faultstring(soap);
  if (soap->fseterror)
    soap->fseterror(soap, c, s);
  if (!*c)
  {
    if (soap->version == 2)
      *c = "SOAP-ENV:Sender";
    else if (soap->version == 1)
      *c = "SOAP-ENV:Client";
    else
      *c = "";
  }
  if (*s)
    return;
  if (soap->error >= SOAP_POST)
    soap->error = SOAP_HTTP_METHOD;
  switch (soap->error)
  {
#ifndef WITH_LEAN
    case SOAP_CLI_FAULT:
      *s = "Client fault";
      break;
    case SOAP_SVR_FAULT:
      *s = "Server fault";
      break;
    case SOAP_TAG_MISMATCH:
      *s = soap_set_validation_fault(soap, "tag name or namespace mismatch", NULL);
      break;
    case SOAP_TYPE:
      if (*soap->type)
        *s = soap_set_validation_fault(soap, "type mismatch ", soap->type);
      else if (*soap->arrayType)
        *s = soap_set_validation_fault(soap, "array type mismatch", NULL);
      else
        *s = soap_set_validation_fault(soap, "invalid value", NULL);
      break;
    case SOAP_SYNTAX_ERROR:
      *s = soap_set_validation_fault(soap, "syntax error", NULL);
      break;
    case SOAP_NO_TAG:
      if (soap->version == 0 && soap->level == 0)
        *s = soap_set_validation_fault(soap, "root element expected", NULL);
      else if (soap->level == 0)
        *s = soap_set_validation_fault(soap, "SOAP message expected", NULL);
      else
        *s = soap_set_validation_fault(soap, "element tag expected", NULL);
      break;
    case SOAP_END_TAG:
      *s = soap_set_validation_fault(soap, "closing tag expected", NULL);
      break;
    case SOAP_MUSTUNDERSTAND:
      *c = "SOAP-ENV:MustUnderstand";
      (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), strlen(soap->tag) + 65), "The data in element '%s' must be understood but cannot be processed", soap->tag);
      *s = soap->msgbuf;
      break;
    case SOAP_VERSIONMISMATCH:
      *c = "SOAP-ENV:VersionMismatch";
      *s = "Invalid SOAP message or SOAP version mismatch";
      break;
    case SOAP_DATAENCODINGUNKNOWN:
      *c = "SOAP-ENV:DataEncodingUnknown";
      *s = "Unsupported SOAP data encoding";
      break;
    case SOAP_NAMESPACE:
      *s = soap_set_validation_fault(soap, "namespace error", NULL);
      break;
    case SOAP_USER_ERROR:
      *s = "User data access error";
      break;
    case SOAP_NO_METHOD:
      (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), strlen(soap->tag) + 66), "Method '%s' not implemented: method name or namespace not recognized", soap->tag);
      *s = soap->msgbuf;
      break;
    case SOAP_NO_DATA:
      *s = "Data required for operation";
      break;
    case SOAP_GET_METHOD:
      *s = "HTTP GET method not implemented";
      break;
    case SOAP_PUT_METHOD:
      *s = "HTTP PUT method not implemented";
      break;
    case SOAP_PATCH_METHOD:
      *s = "HTTP PATCH method not implemented";
      break;
    case SOAP_DEL_METHOD:
      *s = "HTTP DELETE method not implemented";
      break;
    case SOAP_HTTP_METHOD:
      *s = "HTTP method error";
      break;
    case SOAP_EOM:
      *s = "Out of memory";
      break;
    case SOAP_MOE:
      *s = "Memory overflow or memory corruption error";
      break;
    case SOAP_HDR:
      *s = "Header line too long";
      break;
    case SOAP_IOB:
      *s = "Array index out of bounds";
      break;
    case SOAP_NULL:
      *s = soap_set_validation_fault(soap, "nil not allowed", NULL);
      break;
    case SOAP_DUPLICATE_ID:
      *s = soap_set_validation_fault(soap, "multiple elements (use the SOAP_XML_TREE flag) with duplicate id ", soap->id);
      if (soap->version == 2)
        *soap_faultsubcode(soap) = "SOAP-ENC:DuplicateID";
      break;
    case SOAP_MISSING_ID:
      *s = soap_set_validation_fault(soap, "missing id for ref ", soap->id);
      if (soap->version == 2)
        *soap_faultsubcode(soap) = "SOAP-ENC:MissingID";
      break;
    case SOAP_HREF:
      *s = soap_set_validation_fault(soap, "incompatible object type id-ref ", soap->id);
      break;
    case SOAP_FAULT:
      break;
#ifndef WITH_NOIO
    case SOAP_UDP_ERROR:
      *s = "Message too large for UDP packet";
      break;
    case SOAP_TCP_ERROR:
      *s = tcp_error(soap);
      break;
#endif
    case SOAP_HTTP_ERROR:
      *s = "An HTTP processing error occurred";
      break;
    case SOAP_NTLM_ERROR:
      *s = "An HTTP NTLM authentication error occurred";
      break;
    case SOAP_SSL_ERROR:
#ifdef WITH_OPENSSL
      *s = "SSL/TLS error";
#else
      *s = "OpenSSL not installed: recompile with -DWITH_OPENSSL";
#endif
      break;
    case SOAP_PLUGIN_ERROR:
      *s = "Plugin registry error";
      break;
    case SOAP_DIME_ERROR:
      *s = "DIME format error or max DIME size exceeds SOAP_MAXDIMESIZE currently set to " SOAP_XSTRINGIFY(SOAP_MAXDIMESIZE);
      break;
    case SOAP_DIME_HREF:
      *s = "DIME href to missing attachment";
      break;
    case SOAP_DIME_MISMATCH:
      *s = "DIME version/transmission error";
      break;
    case SOAP_DIME_END:
      *s = "End of DIME error";
      break;
    case SOAP_MIME_ERROR:
      *s = "MIME format error";
      break;
    case SOAP_MIME_HREF:
      *s = "MIME href to missing attachment";
      break;
    case SOAP_MIME_END:
      *s = "End of MIME error";
      break;
    case SOAP_ZLIB_ERROR:
#ifdef WITH_ZLIB
      (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), (soap->d_stream && soap->d_stream->msg ? strlen(soap->d_stream->msg) : 0) + 19), "Zlib/gzip error: '%s'", soap->d_stream && soap->d_stream->msg ? soap->d_stream->msg : SOAP_STR_EOS);
      *s = soap->msgbuf;
#else
      *s = "Zlib/gzip not installed for (de)compression: recompile with -DWITH_GZIP";
#endif
      break;
    case SOAP_REQUIRED:
      *s = soap_set_validation_fault(soap, "missing required attribute", NULL);
      break;
    case SOAP_PROHIBITED:
      *s = soap_set_validation_fault(soap, "prohibited attribute present", NULL);
      break;
    case SOAP_LEVEL:
      *s = "Maximum XML nesting depth level exceeded: increase maxlevel";
      break;
    case SOAP_LENGTH:
      *s = soap_set_validation_fault(soap, "value range or content length violation", NULL);
      break;
    case SOAP_OCCURS:
      *s = soap_set_validation_fault(soap, "occurrence constraint violation", NULL);
      break;
    case SOAP_FIXED:
      *s = soap_set_validation_fault(soap, "value does not match the fixed value required", NULL);
      break;
    case SOAP_EMPTY:
      *s = soap_set_validation_fault(soap, "empty value provided where a value is required", NULL);
      break;
    case SOAP_FD_EXCEEDED:
      *s = "Maximum number of open connections was reached: increase FD_SETSIZE or define HAVE_POLL";
      break;
    case SOAP_UTF_ERROR:
      *s = "UTF content encoding error";
      break;
    case SOAP_STOP:
      *s = "Stopped: service request already handled by plugin (informative)";
      break;
#endif
    case SOAP_EOF:
#ifndef WITH_NOIO
      *s = soap_strerror(soap); /* *s = soap->msgbuf */
#ifndef WITH_LEAN
      if (strlen(soap->msgbuf) + 25 < sizeof(soap->msgbuf))
      {
        (void)soap_memmove((void*)(soap->msgbuf + 25), sizeof(soap->tmpbuf) - 25, (const void*)soap->msgbuf, strlen(soap->msgbuf) + 1);
        if (soap->is)
#if defined(__cplusplus) && !defined(WITH_COMPAT)
          (void)soap_memcpy((void*)soap->msgbuf, sizeof(soap->msgbuf), (const void*)"End or bad std::istream: ", 25);
#else
          (void)soap_memcpy((void*)soap->msgbuf, sizeof(soap->msgbuf), (const void*)"End at NUL buffer input: ", 25);
#endif
        else
          (void)soap_memcpy((void*)soap->msgbuf, sizeof(soap->msgbuf), (const void*)"End of file or no input: ", 25);
      }
#endif
      break;
#else
      *s = "End of file or no input";
      break;
#endif
    case SOAP_ERR:
      *s = "An unspecified error occurred";
      break;
    default:
#ifndef WITH_NOHTTP
#ifndef WITH_LEAN
      if (soap->error >= 200 && soap->error < 600)
      {
        const char *t = http_error(soap, soap->error);
        (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), strlen(t) + 54), "Error %d: HTTP %d %s", soap->error, soap->error, t);
        *s = soap->msgbuf;
      }
      else
#endif
#endif
      {
        (SOAP_SNPRINTF(soap->msgbuf, sizeof(soap->msgbuf), 26), "Error %d", soap->error);
        *s = soap->msgbuf;
      }
    }
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_send_fault(struct soap *soap)
{
  int status = soap->error;
  if (status == SOAP_OK || status == SOAP_STOP)
    return soap_closesock(soap);
#ifndef WITH_NOHTTP
  if (status >= 200 && status <= 299)
    return soap_send_empty_response(soap, status);
#endif
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Sending back fault struct for error code %d\n", soap->error));
  soap->keep_alive = 0; /* error: close connection later by disabling keep-alive here */
  soap_set_fault(soap);
  if (soap->error < 200 && soap->error != SOAP_FAULT)
    soap->header = NULL;
  if (status != SOAP_EOF || (!soap->recv_timeout && !soap->send_timeout))
  {
    int r = 1;
#ifndef WITH_NOIO
    if (soap->fpoll && soap->fpoll(soap))
    {
      r = 0;
    }
#ifndef WITH_LEAN
    else if (soap_valid_socket(soap->socket))
    {
      r = tcp_select(soap, soap->socket, SOAP_TCP_SELECT_RCV | SOAP_TCP_SELECT_SND, 0);
      if (r > 0)
      {
        int t;
        if (!(r & SOAP_TCP_SELECT_SND)
         || ((r & SOAP_TCP_SELECT_RCV)
          && recv(soap->socket, (char*)&t, 1, MSG_PEEK) < 0))
          r = 0;
      }
    }
#endif
#endif
    if (r > 0)
    {
      soap->error = SOAP_OK;
      if (soap->version > 0)
      {
        soap->encodingStyle = NULL; /* no encodingStyle in Faults */
        soap_serializeheader(soap);
        soap_serializefault(soap);
        (void)soap_begin_count(soap);
        if ((soap->mode & SOAP_IO_LENGTH))
        {
          if (soap_envelope_begin_out(soap)
           || soap_putheader(soap)
           || soap_body_begin_out(soap)
           || soap_putfault(soap)
           || soap_body_end_out(soap)
           || soap_envelope_end_out(soap))
          return soap_closesock(soap);
        }
        (void)soap_end_count(soap);
        if (soap_response(soap, status)
         || soap_envelope_begin_out(soap)
         || soap_putheader(soap)
         || soap_body_begin_out(soap)
         || soap_putfault(soap)
         || soap_body_end_out(soap)
         || soap_envelope_end_out(soap)
         || soap_end_send(soap))
          return soap_closesock(soap);
      }
      else
      {
        const char *s = *soap_faultstring(soap);
        const char **d = soap_faultdetail(soap);
        (void)soap_begin_count(soap);
        if ((soap->mode & SOAP_IO_LENGTH))
          if (soap_element_begin_out(soap, "fault", 0, NULL)
           || soap_outstring(soap, "reason", 0, (char*const*)&s, NULL, 0)
           || (d && *d && soap_outliteral(soap, "detail", (char*const*)d, NULL))
           || soap_element_end_out(soap, "fault"))
            return soap_closesock(soap);
        (void)soap_end_count(soap);
        if (soap_response(soap, status)
         || soap_element_begin_out(soap, "fault", 0, NULL)
         || soap_outstring(soap, "reason", 0, (char*const*)&s, NULL, 0)
         || (d && *d && soap_outliteral(soap, "detail", (char*const*)d, NULL))
         || soap_element_end_out(soap, "fault")
         || soap_end_send(soap))
          return soap_closesock(soap);
      }
    }
  }
  soap->error = status;
  return soap_closesock(soap);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_recv_fault(struct soap *soap, int check)
{
  int status = soap->status;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Check (%d) if receiving SOAP Fault (status = %d)\n", check, status));
  if (!check)
  {
    /* try getfault when no tag or tag mismatched at level 2, otherwise close and return SOAP_TAG_MISMATCH or HTTP error code */
    if (soap->error != SOAP_NO_TAG && (soap->error != SOAP_TAG_MISMATCH || soap->level != 2))
    {
      if (soap->error == SOAP_TAG_MISMATCH && soap->level == 0)
      {
        soap->error = SOAP_OK;
        if (!soap_element_begin_in(soap, "fault", 0, NULL))
        {
          char *s = NULL, *d = NULL;
          (void)soap_instring(soap, "reason", &s, NULL, 0, 1, 0, -1, NULL);
          (void)soap_inliteral(soap, "detail", &d);
          if (!soap_element_end_in(soap, "fault") && !soap_end_recv(soap))
          {
            *soap_faultstring(soap) = s;
            if (d && *d)
              *soap_faultdetail(soap) = d;
            if (status)
              soap->error = status;
            else
              soap->error = SOAP_FAULT;
            soap_set_fault(soap);
            return soap_closesock(soap);
          }
        }
        soap->error = SOAP_TAG_MISMATCH;
      }
      if (status && (status < 200 || status > 299))
        soap->error = status;
      return soap_closesock(soap);
    }
  }
  soap->error = SOAP_OK;
  if (soap_getfault(soap))
  {
    /* if check>0 and no SOAP Fault is present and no HTTP error then just return to parse request */
    if (check
     && (status == 0 || (status >= 200 && status <= 299))
     && ((soap->error == SOAP_TAG_MISMATCH && soap->level == 2) || soap->error == SOAP_NO_TAG))
      return soap->error = SOAP_OK;
    /* if check=0 and empty SOAP Body and encodingStyle is NULL and no HTTP error then just return */
    if (!check
     && (status == 0 || (status >= 200 && status <= 299))
     && !soap->encodingStyle
     && (soap->error == SOAP_NO_TAG && soap->level <= 2))
      return soap->error = SOAP_OK;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Error: soap_get_soapfault() failed with error %d at level %u tag '%s'\n", soap->error, soap->level, soap->tag));
    *soap_faultcode(soap) = (soap->version == 2 ? "SOAP-ENV:Sender" : "SOAP-ENV:Client");
    if (status)
      soap->error = status;
    else
      soap->error = status = SOAP_NO_DATA;
    soap_set_fault(soap);
  }
  else
  {
    const char *s = *soap_faultcode(soap);
    if (!soap_match_tag(soap, s, "SOAP-ENV:Server")
     || !soap_match_tag(soap, s, "SOAP-ENV:Receiver"))
    {
      status = SOAP_SVR_FAULT;
    }
    else if (!soap_match_tag(soap, s, "SOAP-ENV:Client")
          || !soap_match_tag(soap, s, "SOAP-ENV:Sender"))
    {
      status = SOAP_CLI_FAULT;
    }
    else if (!soap_match_tag(soap, s, "SOAP-ENV:MustUnderstand"))
    {
      status = SOAP_MUSTUNDERSTAND;
    }
    else if (!soap_match_tag(soap, s, "SOAP-ENV:VersionMismatch"))
    {
      status = SOAP_VERSIONMISMATCH;
    }
    else
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Received SOAP Fault code %s\n", s));
      status = SOAP_FAULT;
    }
    if (!soap_body_end_in(soap))
      soap_envelope_end_in(soap);
  }
  (void)soap_end_recv(soap);
  soap->error = status;
  return soap_closesock(soap);
}

/******************************************************************************/

#ifndef WITH_NOHTTP
SOAP_FMAC1
int
SOAP_FMAC2
soap_send_empty_response(struct soap *soap, int httpstatuscode)
{
  soap_mode m = soap->omode;
  if (!(m & SOAP_IO_UDP))
  {
    soap->count = 0;
    if ((m & SOAP_IO) == SOAP_IO_CHUNK)
      soap->omode = (m & ~SOAP_IO) | SOAP_IO_BUFFER;
    (void)soap_response(soap, httpstatuscode);
    (void)soap_end_send(soap); /* force end of sends */
    soap->error = SOAP_STOP; /* stops the server from returning another response */
    soap->omode = m;
  }
  return soap_closesock(soap);
}
#endif

/******************************************************************************/

#ifndef WITH_NOHTTP
SOAP_FMAC1
int
SOAP_FMAC2
soap_recv_empty_response(struct soap *soap)
{
  soap->error = SOAP_OK;
  if (!(soap->omode & SOAP_IO_UDP) && !(soap->omode & SOAP_ENC_PLAIN))
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Receiving empty response\n"));
    if (soap_begin_recv(soap) == SOAP_OK)
    {
      if (soap_http_skip_body(soap) || soap_end_recv(soap))
        return soap_closesock(soap);
    }
    else if (soap->error == 200 || soap->error == 201 || soap->error == 202)
    {
      soap->error = SOAP_OK;
    }
  }
#ifndef WITH_LEANER
  else if ((soap->fprepareinitrecv && (soap->error = soap->fprepareinitrecv(soap)))
        || (soap->fpreparefinalrecv && (soap->error = soap->fpreparefinalrecv(soap))))
  {
    return soap->error;
  }
#endif
  return soap_closesock(soap);
}
#endif

/******************************************************************************/

#ifndef WITH_NOIO
static const char*
soap_strerror(struct soap *soap)
{
  int err = soap->errnum;
  *soap->msgbuf = '\0';
  if (err)
  {
#ifndef WIN32
# ifdef HAVE_STRERROR_R
#  if !defined(_GNU_SOURCE) || (!(~_GNU_SOURCE+1) && ((!defined(_POSIX_C_SOURCE) && !defined(_XOPEN_SOURCE)) || (_POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600))) /* || !defined(__GLIBC__)) */
    err = strerror_r(err, soap->msgbuf, sizeof(soap->msgbuf)); /* XSI-compliant */
    if (err != 0)
      soap_strcpy(soap->msgbuf, sizeof(soap->msgbuf), "unknown error");
#  else
    return strerror_r(err, soap->msgbuf, sizeof(soap->msgbuf)); /* GNU-specific */
#  endif
# else
    return strerror(err);
# endif
#else
#ifndef UNDER_CE
    DWORD len;
    *soap->msgbuf = '\0';
    len = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, err, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)soap->msgbuf, (DWORD)sizeof(soap->msgbuf), NULL);
#else
    DWORD i, len;
    *soap->msgbuf = '\0';
    len = FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, err, 0, (LPTSTR)soap->msgbuf, (DWORD)(sizeof(soap->msgbuf)/sizeof(TCHAR)), NULL);
    for (i = 0; i <= len; i++)
    {
      if (((TCHAR*)soap->msgbuf)[i] < 0x80)
        soap->msgbuf[i] = (char)((TCHAR*)soap->msgbuf)[i];
      else
        soap->msgbuf[i] = '?';
    }
#endif
#endif
  }
  else
  {
    if (soap->recv_maxlength && soap->count > soap->recv_maxlength)
    {
      soap_strcpy(soap->msgbuf, sizeof(soap->msgbuf), "max message length exceeded");
    }
    else
    {
      int tt = soap->transfer_timeout, rt = soap->recv_timeout, st = soap->send_timeout;
#ifndef WITH_LEAN
      int tu = ' ', ru = ' ', su = ' ';
#endif
      soap_strcpy(soap->msgbuf, sizeof(soap->msgbuf), "message transfer interrupted");
      if (tt | rt || st)
        soap_strcpy(soap->msgbuf + 28, sizeof(soap->msgbuf) - 28, " or timed out");
#ifndef WITH_LEAN
      if (tt < 0)
      {
        tt = -tt;
        tu = 'u';
      }
      if (rt < 0)
      {
        rt = -rt;
        ru = 'u';
      }
      if (st < 0)
      {
        st = -st;
        su = 'u';
      }
      if (tt)
      {
        size_t l = strlen(soap->msgbuf);
        (SOAP_SNPRINTF(soap->msgbuf + l, sizeof(soap->msgbuf) - l, 43), " (%d%csec max transfer time)", tt, tu);
      }
      if (rt)
      {
        size_t l = strlen(soap->msgbuf);
        (SOAP_SNPRINTF(soap->msgbuf + l, sizeof(soap->msgbuf) - l, 40), " (%d%csec max recv delay)", rt, ru);
      }
      if (st)
      {
        size_t l = strlen(soap->msgbuf);
        (SOAP_SNPRINTF(soap->msgbuf + l, sizeof(soap->msgbuf) - l, 40), " (%d%csec max send delay)", st, su);
      }
#endif
    }
  }
  return soap->msgbuf;
}
#endif

/******************************************************************************/

static int
soap_set_error(struct soap *soap, const char *faultcode, const char *faultsubcodeQName, const char *faultstring, const char *faultdetailXML, int soaperror)
{
  *soap_faultcode(soap) = faultcode;
  if (faultsubcodeQName)
    *soap_faultsubcode(soap) = faultsubcodeQName;
  *soap_faultstring(soap) = faultstring;
  if (faultdetailXML && *faultdetailXML)
  {
    const char **s = soap_faultdetail(soap);
    if (s)
      *s = faultdetailXML;
  }
  return soap->error = soaperror;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_set_sender_error(struct soap *soap, const char *faultstring, const char *faultdetailXML, int soaperror)
{
  return soap_set_error(soap, soap->version == 2 ? "SOAP-ENV:Sender" : soap->version == 1 ? "SOAP-ENV:Client" : "at sender", NULL, faultstring, faultdetailXML, soaperror);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_set_receiver_error(struct soap *soap, const char *faultstring, const char *faultdetailXML, int soaperror)
{
  return soap_set_error(soap, soap->version == 2 ? "SOAP-ENV:Receiver" : soap->version == 1 ? "SOAP-ENV:Server" : "detected", NULL, faultstring, faultdetailXML, soaperror);
}

/******************************************************************************/

static int
soap_copy_fault(struct soap *soap, const char *faultcode, const char *faultsubcodeQName, const char *faultstring, const char *faultdetailXML)
{
  char *r = NULL, *s = NULL, *t = NULL;
  DBGFUN2("soap_copy_fault", "code=%s", faultcode ? faultcode : "(null)", "string=%s", faultstring ? faultstring : "(null)")
  if (faultsubcodeQName)
    r = soap_strdup(soap, faultsubcodeQName);
  if (faultstring)
    s = soap_strdup(soap, faultstring);
  if (faultdetailXML)
    t = soap_strdup(soap, faultdetailXML);
  return soap_set_error(soap, faultcode, r, s, t, SOAP_FAULT);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_sender_fault(struct soap *soap, const char *faultstring, const char *faultdetailXML)
{
  return soap_sender_fault_subcode(soap, NULL, faultstring, faultdetailXML);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_sender_fault_subcode(struct soap *soap, const char *faultsubcodeQName, const char *faultstring, const char *faultdetailXML)
{
  return soap_copy_fault(soap, soap->version == 2 ? "SOAP-ENV:Sender" : soap->version == 1 ? "SOAP-ENV:Client" : "at source", faultsubcodeQName, faultstring, faultdetailXML);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_receiver_fault(struct soap *soap, const char *faultstring, const char *faultdetailXML)
{
  return soap_receiver_fault_subcode(soap, NULL, faultstring, faultdetailXML);
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_receiver_fault_subcode(struct soap *soap, const char *faultsubcodeQName, const char *faultstring, const char *faultdetailXML)
{
  return soap_copy_fault(soap, soap->version == 2 ? "SOAP-ENV:Receiver" : soap->version == 1 ? "SOAP-ENV:Server" : "is internal", faultsubcodeQName, faultstring, faultdetailXML);
}

/******************************************************************************/

#ifndef WITH_NOSTDLIB
SOAP_FMAC1
void
SOAP_FMAC2
soap_print_fault(struct soap *soap, FILE *fd)
{
  if (soap_check_state(soap))
  {
    fprintf(fd, "Error: soap struct state not initialized\n");
  }
  else if (soap->error)
  {
    const char **c, *v = NULL, *s, *d;
    c = soap_faultcode(soap);
    if (!*c)
    {
      soap_set_fault(soap);
      c = soap_faultcode(soap);
    }
    if (soap->version == 2)
      v = soap_fault_subcode(soap);
    s = soap_fault_string(soap);
    d = soap_fault_detail(soap);
    fprintf(fd, "%s%d fault %s [%s]\n\"%s\"\nDetail: %s\n", soap->version ? "SOAP 1." : "Error ", soap->version ? (int)soap->version : soap->error, *c, v ? v : "no subcode", s ? s : "[no reason]", d ? d : "[no detail]");
  }
}
#endif

/******************************************************************************/

#ifdef __cplusplus
#ifndef WITH_LEAN
#ifndef WITH_NOSTDLIB
#ifndef WITH_COMPAT
SOAP_FMAC1
void
SOAP_FMAC2
soap_stream_fault(struct soap *soap, std::ostream& os)
{
  if (soap_check_state(soap))
  {
    os << "Error: soap struct state not initialized\n";
  }
  else if (soap->error)
  {
    const char **c, *v = NULL, *s, *d;
    c = soap_faultcode(soap);
    if (!*c)
    {
      soap_set_fault(soap);
      c = soap_faultcode(soap);
    }
    if (soap->version == 2)
      v = soap_fault_subcode(soap);
    s = soap_fault_string(soap);
    d = soap_fault_detail(soap);
    os << (soap->version ? "SOAP 1." : "Error ")
       << (soap->version ? (int)soap->version : soap->error)
       << " fault " << *c
       << "[" << (v ? v : "no subcode") << "]"
       << std::endl
       << "\"" << (s ? s : "[no reason]") << "\""
       << std::endl
       << "Detail: " << (d ? d : "[no detail]")
       << std::endl;
  }
}
#endif
#endif
#endif
#endif

/******************************************************************************/

#ifndef WITH_LEAN
#ifndef WITH_NOSTDLIB
SOAP_FMAC1
char*
SOAP_FMAC2
soap_sprint_fault(struct soap *soap, char *buf, size_t len)
{
  if (soap_check_state(soap))
  {
    soap_strcpy(buf, len, "Error: soap struct not initialized");
  }
  else if (soap->error)
  {
    const char **c, *v = NULL, *s, *d;
    c = soap_faultcode(soap);
    if (!*c)
    {
      soap_set_fault(soap);
      c = soap_faultcode(soap);
    }
    if (soap->version == 2)
      v = soap_fault_subcode(soap);
    if (!v)
      v = "no subcode";
    s = soap_fault_string(soap);
    if (!s)
      s = "[no reason]";
    d = soap_fault_detail(soap);
    if (!d)
      d = "[no detail]";
    (SOAP_SNPRINTF(buf, len, strlen(*c) + strlen(v) + strlen(s) + strlen(d) + 72), "%s%d fault %s [%s]\n\"%s\"\nDetail: %s\n", soap->version ? "SOAP 1." : "Error ", soap->version ? (int)soap->version : soap->error, *c, v, s, d);
  }
  else if (len > 0)
  {
    *buf = '\0';
  }
  return buf;
}
#endif
#endif

/******************************************************************************/

#ifndef WITH_NOSTDLIB
SOAP_FMAC1
void
SOAP_FMAC2
soap_print_fault_location(struct soap *soap, FILE *fd)
{
#ifndef WITH_LEAN
  int i, j, c1, c2;
  if (soap_check_state(soap) == SOAP_OK && soap->error && soap->error != SOAP_STOP && soap->bufidx <= soap->buflen && soap->buflen > 0 && soap->buflen <= sizeof(soap->buf))
  {
    i = (int)soap->bufidx - 1;
    if (i <= 0)
      i = 0;
    c1 = soap->buf[i];
    soap->buf[i] = '\0';
    if ((int)soap->buflen >= i + 1024)
      j = i + 1023;
    else
      j = (int)soap->buflen - 1;
    c2 = soap->buf[j];
    soap->buf[j] = '\0';
    fprintf(fd, "%s%c\n<!-- ** HERE ** -->\n", soap->buf, c1);
    if (soap->bufidx < soap->buflen)
      fprintf(fd, "%s\n", soap->buf + soap->bufidx);
    soap->buf[i] = (char)c1;
    soap->buf[j] = (char)c2;
  }
#else
  (void)soap;
  (void)fd;
#endif
}
#endif

/******************************************************************************/

#ifdef __cplusplus
#ifndef WITH_LEAN
#ifndef WITH_NOSTDLIB
#ifndef WITH_COMPAT
SOAP_FMAC1
void
SOAP_FMAC2
soap_stream_fault_location(struct soap *soap, std::ostream& os)
{
  int i, j, c1, c2;
  if (soap_check_state(soap) == SOAP_OK && soap->error && soap->error != SOAP_STOP && soap->bufidx <= soap->buflen && soap->buflen > 0 && soap->buflen <= sizeof(soap->buf))
  {
    i = (int)soap->bufidx - 1;
    if (i <= 0)
      i = 0;
    c1 = soap->buf[i];
    soap->buf[i] = '\0';
    if ((int)soap->buflen >= i + 1024)
      j = i + 1023;
    else
      j = (int)soap->buflen - 1;
    c2 = soap->buf[j];
    soap->buf[j] = '\0';
    os << soap->buf << (char)c1 << std::endl << "<!-- ** HERE ** -->" << std::endl;
    if (soap->bufidx < soap->buflen)
      os << soap->buf + soap->bufidx << std::endl;
    soap->buf[i] = (char)c1;
    soap->buf[j] = (char)c2;
  }
}
#endif
#endif
#endif
#endif

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_register_plugin_arg(struct soap *soap, int (*fcreate)(struct soap*, struct soap_plugin*, void*), void *arg)
{
  struct soap_plugin *p;
  int err;
  p = (struct soap_plugin*)SOAP_MALLOC(soap, sizeof(struct soap_plugin));
  if (!p)
    return soap->error = SOAP_EOM;
  p->id = NULL;
  p->data = NULL;
  p->fcopy = NULL;
  p->fdelete = NULL;
  err = fcreate(soap, p, arg);
  if (!err && p->fdelete && p->id)
  {
    if (!soap_lookup_plugin(soap, p->id))
    {
      p->next = soap->plugins;
      soap->plugins = p;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Registered '%s' plugin\n", p->id));
      return SOAP_OK;
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not register plugin '%s': plugin with the same ID already registered\n", p->id));
    SOAP_FREE(soap, p);
    return SOAP_OK;
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not register plugin '%s': plugin returned error %d or plugin ID not set or fdelete callback not set\n", p->id ? p->id : "plugin ID not set", err));
  SOAP_FREE(soap, p);
  soap->error = err ? err : SOAP_PLUGIN_ERROR;
  return soap->error;
}

/******************************************************************************/

static void *
fplugin(struct soap *soap, const char *id)
{
  struct soap_plugin *p;
  for (p = soap->plugins; p; p = p->next)
    if (p->id == id || !strcmp(p->id, id))
      return p->data;
  return NULL;
}

/******************************************************************************/

SOAP_FMAC1
void *
SOAP_FMAC2
soap_lookup_plugin(struct soap *soap, const char *id)
{
  return soap->fplugin(soap, id);
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif

/******************************************************************************\
 *
 *      C++ soap struct methods
 *
\******************************************************************************/

#ifdef __cplusplus
soap::soap()
{
  soap_init(this);
  /* no logs to prevent DEBUG mode leaks when the user calls a soap_init() on this context */
  soap_set_test_logfile(this, NULL);
  soap_set_sent_logfile(this, NULL);
  soap_set_recv_logfile(this, NULL);
}
#endif

/******************************************************************************/

#ifdef __cplusplus
soap::soap(soap_mode m)
{
  soap_init1(this, m);
}
#endif

/******************************************************************************/

#ifdef __cplusplus
soap::soap(soap_mode im, soap_mode om)
{
  soap_init2(this, im, om);
}
#endif

/******************************************************************************/

#ifdef __cplusplus
soap::soap(const struct soap& soap)
{
  soap_copy_context(this, &soap);
}
#endif

/******************************************************************************/

#ifdef __cplusplus
struct soap& soap::operator=(const struct soap& soap)
{
  soap_done(this);
  soap_copy_context(this, &soap);
  return *this;
}
#endif

/******************************************************************************/

#ifdef __cplusplus
void soap::destroy()
{
  soap_destroy(this);
  soap_end(this);
}
#endif

/******************************************************************************/

#ifdef __cplusplus
soap::~soap()
{
  soap_done(this);
}
#endif

/******************************************************************************/
