/*
	tandem.c

	Tandem platform support for gSOAP.
	See README.txt and tandem.h for usage instructions.

gSOAP XML Web services tools
Copyright (C) 2000-2010, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under ONE of the following licenses:
GPL.
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

#include "stdsoap2.h"
#include "tandem.h"

/* process name on the Tandem machine, used to set up sockets */
static const char *procname = NULL;

static int fsend(struct soap*, const char*, size_t);
static size_t frecv(struct soap*, char*, size_t);
static int tcp_connect(struct soap*, const char *endpoint, const char *host, int port);
static int tcp_disconnect(struct soap*);
static int tcp_closesocket(struct soap*, SOAP_SOCKET);
static int tcp_shutdownsocket(struct soap*, SOAP_SOCKET, int);

/**
@fn void tandem_init(struct soap *soap, const char *name)
@brief Initialize soap context for use on the Tandem platform. Immediately use
after creating a new soap context.
@param soap context
@param[in] name of process or NULL
*/
void
tandem_init(struct soap *soap, const char *name)
{ 
  procname = name;

  /* set callbacks */
  /* soap->fresolve = tcp_gethost; not needed */
  soap->fopen = tcp_connect;
  soap->fclose = tcp_disconnect;
  soap->fclosesocket = tcp_closesocket;
  soap->fshutdownsocket = tcp_shutdownsocket;
  soap->fsend = fsend;
  soap->frecv = frecv;
  /* soap->fpoll = soap_poll; enable when needed */
#ifdef SOAP_DEBUG
  /* in the new gsoap 2.7.16 we override this here rather than in stdsoap2.c: */
  soap_set_test_logfile(soap, "TESTlog");
  soap_set_sent_logfile(soap, "SENTlog");
  soap_set_recv_logfile(soap, "RECVlog");
#endif
}

/**
@fn void tandem_done(struct soap *soap)
@brief Disengages soap context for use on the Tandem platform.
@param soap context
*/
void
tandem_done(struct soap *soap)
{
}

/** 
@fn SOAP_SOCKET soap_bind(struct soap *soap, const char *host, int port, int backlog)
@brief Binds to port.
@param soap context
@param[in] host name of machine (optional NULL)
@param[in] port to bind
@param[in] backlog queue size
@return socket fd or -1
*/
SOAP_SOCKET
soap_bind(struct soap *soap, const char *host, int port, int backlog)
{ struct sockaddr_in sin;
  int optVal = 1;
  if (soap->master != -1)
  { soap->fclosesocket(soap, soap->master);
    soap->master = -1;
  }
  soap->socket = -1;
  if ((soap->master = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  { soap->errnum = errno;
    soap_set_receiver_error(soap, "", "TCP socket failed in soap_bind()", SOAP_TCP_ERROR);
    return -1;
  }
  setsockopt( soap->master, SOL_SOCKET, SO_REUSEADDR, (char*)&optVal, sizeof(optVal));
  memset((void*)&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_addr.s_addr = INADDR_ANY;
  sin.sin_port = port;
  if (bind(soap->master, (struct sockaddr*)&sin, (int)sizeof(sin)) || listen(soap->master, backlog))
  { soap->errnum = errno;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Could not bind to host\n"));
    soap_closesock(soap);
    soap_set_receiver_error(soap, "", "TCP bind failed in soap_bind()", SOAP_TCP_ERROR);
    return -1;
  }  
  return soap->master;
}

/** 
@fn SOAP_SOCKET soap_accept(struct soap *soap)
@brief Accepts new socket when bound to port.
@param soap context
@return socket fd or -1
*/
SOAP_SOCKET
soap_accept(struct soap *soap)
{ struct sockaddr_in sin;
  int val = SOAP_BUFLEN;
  int flen = sizeof(sin);
  memset((void*)&sin, 0, sizeof(sin));
  soap->socket = accept(soap->master, (struct sockaddr*)&sin, &flen);
  setsockopt(soap->socket, SOL_SOCKET, SO_RCVBUF, (char*)&val, sizeof(val));
  setsockopt(soap->socket, SOL_SOCKET, SO_SNDBUF, (char*)&val, sizeof(val));
  return soap->socket;
}

/**
@fn static size_t fsend(struct soap *soap, char *s, size_t n)
@brief Callback override for soap->fsend to write data.
@param soap context
@param[in] s points to buffer to send
@param[in] n size of buffer content
@return SOAP_OK or error
*/
static int
fsend(struct soap *soap, const char *s, size_t n)
{ if (soap->socket == -1)
    return SOAP_EOF;
  while (n)
  { register int nwritten = send(soap->socket, (char*)s, n, 0);
    if (nwritten < 0)
    { DBGLOG(TEST, SOAP_MESSAGE(fdebug, "send() error %s\n", errno)); 
      soap->errnum = errno;
      return SOAP_EOF;
    }
    n -= nwritten;
    s += nwritten;
  }
  return SOAP_OK;
}

/**
@fn static size_t frecv(struct soap *soap, char *s, size_t n)
@brief Callback override for soap->frecv to read data.
@param soap context
@param s points to buffer to populate
@param[in] n size of buffer space
@return number of bytes (less or equal to n) written into s
*/
static size_t
frecv(struct soap *soap, char *s, size_t n)
{ if (soap->socket == -1)
    return 0;
  soap->errnum = 0;
  for (;;)
  { register int r = recv(soap->socket, s, n, 0);
    if (r >= 0)
       return (size_t)r;
    /* We can check if there is a condition that warrants us to retry in the loop, here we just stop */
    if (r < 0)
    { DBGLOG(TEST, SOAP_MESSAGE(fdebug, "recv() error %s\n", errno)); 
      soap->errnum = errno;
      break;
    }
  }
  return 0;
}

/** 
@fn static int tcp_connect(struct soap *soap, const char *endpoint, const char *host, int port)
@brief Callback override for soap->fopen to open a new connection. Also starts
other communication-related objects such as SSL (when used).
@param soap context
@param[in] endpoint string
@param[in] host name
@param[in] port number
@return fd of socket
*/
static int
tcp_connect(struct soap *soap, const char *endpoint, const char *host, int port)
{ struct sockaddr_in sin;
  struct hostent *hostent;
  int val = SOAP_BUFLEN;
  if (soap->socket != -1)
    soap->fclosesocket(soap, soap->socket);
  fprintf(stderr, "Host=%s, port=%d\n", host, port);
  soap->socket = socket(AF_INET, SOCK_STREAM, 0);
  if (soap->socket < 0)
  { soap->errnum = errno;
    soap_set_sender_error(soap, "", "TCP socket failed in tcp_connect()", SOAP_TCP_ERROR);
    return -1;
  }
  memset((void*)&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  if ((sin.sin_addr.s_addr = inet_addr(host)) == INET_ERROR)
  { if (!(hostent = gethostbyname((char*)host)))
    { soap_set_sender_error(soap, "", "TCP get host by name failed in tcp_connect()", SOAP_TCP_ERROR);
      soap->fclosesocket(soap, soap->socket);
      return -1;
    }
    sin.sin_addr.s_addr = *(unsigned long*) (*(hostent->h_addr_list));
  }
  else
    sin.sin_addr.s_addr = inet_addr(host);
  sin.sin_port = port;
  if (connect(soap->socket, (struct sockaddr*)&sin, (int)(sizeof(sin))) < 0)
  { soap_set_sender_error(soap, "", "TCP connect failed in tcp_connect()", SOAP_TCP_ERROR);
    soap->fclosesocket(soap, soap->socket);
    return -1;
  }
  setsockopt(soap->socket, SOL_SOCKET, SO_RCVBUF, (char*)&val, sizeof(val));
  setsockopt(soap->socket, SOL_SOCKET, SO_SNDBUF, (char*)&val, sizeof(val));
  return soap->socket;
}

/** 
@fn static int tcp_disconnect(struct soap *soap)
@brief Callback override for soap->fclose that closes the socket only when it
is currently open. Also closes other communication-related objects such as SSL
(when used).
@param soap context
@return SOAP_OK when successful
*/
static int
tcp_disconnect(struct soap *soap)
{ if (soap->socket != -1)
  { DBGLOG(TEST,SOAP_MESSAGE(fdebug, "Closing socket %d\n", soap->socket));
    soap->fclosesocket(soap, soap->socket);
    soap->socket = -1;
  }
  return SOAP_OK;
}

/** 
@fn static int tcp_closesocket(struct soap *soap, SOAP_SOCKET fd)
@brief Callback override for soap->fclosesocket that closes the socket.
@param soap context
@param[in] fd of the socket
@return value of close, 0 = successful
*/
static int
tcp_closesocket(struct soap *soap, SOAP_SOCKET fd)
{ return FILE_CLOSE_(fd);
}

/** 
@fn static int tcp_shutdownsocket(struct soap *soap, SOAP_SOCKET fd, int how)
@brief Callback override for soap->fshutdownsocket that shuts down socket read
or write. May return 0 when shutdown operation is not supported on this
platform.  @param soap context
@param[in] fd of the socket
@param[in] how to shutdown the socket
@return value of shutdown, 0 = successful
*/
static int
tcp_shutdownsocket(struct soap *soap, SOAP_SOCKET fd, int how)
{ return shutdown(fd, how);
}
