/*
	magicserver.cpp

	Example CGI or stand-alone multi-threaded magic squares Web service.

	Install as a CGI application.
	Alternatively, run from command line with arguments IP (which must be
	the IP of the current machine you are using) and PORT to run this as a
	stand-alone server on a port. For example:
	$ magicserver.cgi 18081 &
	To let the 'magic' client application talk to this service, change the
	URL in magic.cpp into "http://localhost:18081"
	To show wsdl as stand-alone, use: http://localhost:18081/?wsdl

	This example illustrates two alternative server implementations with
	threads.  The first implementation recycles gSOAP resources but is
	bounded to a maximum number of threads. Each thread needs to be joined,
	so runaway processes will halt the server at some point. The second
	implementation has no thread limitation. Runaway threads are not
	controlled.

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
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
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "soapH.h"
#include "magic.nsmap"

#include <unistd.h>
#ifdef _POSIX_THREADS
#include <pthread.h>    // use Pthreads, NOTE: perhaps use gsoap/plugin/threads.h
#endif

#define BACKLOG (100)	// Max. request backlog
#define MAX_THR (8)	// Max. threads to serve requests
#define SLEEP	(0)	// >0 will make each thread sleep to mimic work load

////////////////////////////////////////////////////////////////////////////////
//
//	Magic Squares Server
//
////////////////////////////////////////////////////////////////////////////////

int http_get(struct soap*);
void *process_request(void*);

int main(int argc, char **argv)
{ struct soap soap;
  soap_init(&soap);
  if (argc < 2)		// no args: assume this is a CGI application
  { soap_serve(&soap);	// serve request
    soap_destroy(&soap);// cleanup class instances
    soap_end(&soap);	// cleanup
  }
  else
  {
#ifdef _POSIX_THREADS
    pthread_t tid;
    void *arg;
#endif
    SOAP_SOCKET m, s;
    int port = atoi(argv[1]);
    soap.recv_timeout = soap.send_timeout = 5; // max socket idle time (sec)
    soap.transfer_timeout = 10; // max transfer time (sec)
    // soap.accept_timeout = 60; // die if no requests are made within 1 minute
    // register a HTTP GET handler
    soap.fget = http_get;
    m = soap_bind(&soap, NULL, port, 100);
    if (!soap_valid_socket(m))
    { soap_print_fault(&soap, stderr);
      exit(1);
    }
    fprintf(stderr, "Socket connection successful %d\n", m);
    for (int i = 1; ; i++)
    { s = soap_accept(&soap);
      if (!soap_valid_socket(s))
      { if (soap.errnum)
          soap_print_fault(&soap, stderr);
	else
	  fprintf(stderr, "Server timed out\n");
	break;
      }
      fprintf(stderr, "%d: accepted %d IP=%d.%d.%d.%d ... ", i, s, (int)(soap.ip>>24)&0xFF, (int)(soap.ip>>16)&0xFF, (int)(soap.ip>>8)&0xFF, (int)soap.ip&0xFF);
#ifdef _POSIX_THREADS
      arg = (void*)soap_copy(&soap);
      while (pthread_create(&tid, NULL, (void*(*)(void*))process_request, arg))
	sleep(1);
#else
      soap_serve(&soap);	// process request
      fprintf(stderr, "served\n");
      soap_destroy(&soap);
      soap_end(&soap);	// clean up 
#endif
    }
  }
  soap_done(&soap);
  return 0;
}

#ifdef _POSIX_THREADS
void *process_request(void *soap)
{
  pthread_detach(pthread_self());
  soap_serve((struct soap*)soap);
  soap_destroy((struct soap*)soap);
  soap_end((struct soap*)soap);
  soap_free((struct soap*)soap);
  return NULL;
}
#endif

////////////////////////////////////////////////////////////////////////////////
//
//	Magic Square Algorithm
//
////////////////////////////////////////////////////////////////////////////////

int ns1__magic(struct soap *soap, int n, matrix *square)
{ int i, j, k, l, key = 2;
  if (n < 1)
    return soap_sender_fault(soap, "Negative or zero size", "<error xmlns=\"http://tempuri.org/\">The input parameter must be positive</error>");
  if (n > 100)
    return soap_sender_fault(soap, "size > 100", "<error xmlns=\"http://tempuri.org/\">The input parameter must not be too large</error>");
  square->resize(n, n);
  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      (*square)[i][j] = 0;
  i = 0;
  j = (n-1)/2;
  (*square)[i][j] = 1;
  while (key <= n*n)
  { if (i-1 < 0)
      k = n-1;
    else
      k = i-1;
    if (j-1 < 0)
      l = n-1;
    else
      l = j-1;
    if ((*square)[k][l])
      i = (i+1) % n;
    else
    { i = k;
      j = l;
    }
    (*square)[i][j] = key;
    key++;
  }
  sleep(SLEEP);		// mimic work load latency
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//	Class vector Methods
//
////////////////////////////////////////////////////////////////////////////////

vector::vector()
{ __ptr = 0;
  __size = 0;
}

vector::vector(int n)
{ __ptr = (int*)soap_malloc(soap, n*sizeof(int));
  __size = n;
}

vector::~vector()
{ soap_unlink(soap, this); // not required, but just to make sure if someone calls delete on this
}

void vector::resize(int n)
{ int *p;
  if (__size == n)
    return;
  p = (int*)soap_malloc(soap, n*sizeof(int));
  if (__ptr)
  { for (int i = 0; i < (n <= __size ? n : __size); i++)
      p[i] = __ptr[i];
    soap_unlink(soap, __ptr);
    free(__ptr);
  }
  __size = n;
  __ptr = p;
}

int& vector::operator[](int i) const
{ if (!__ptr || i < 0 || i >= __size)
    fprintf(stderr, "Array index out of bounds\n");
  return (__ptr)[i];
}

////////////////////////////////////////////////////////////////////////////////
//
//	Class matrix Methods
//
////////////////////////////////////////////////////////////////////////////////

matrix::matrix()
{ __ptr = 0;
  __size = 0;
}

matrix::matrix(int rows, int cols)
{ __ptr = soap_new_vector(soap, rows);
  for (int i = 0; i < cols; i++)
    __ptr[i].resize(cols);
  __size = rows;
}

matrix::~matrix()
{ soap_unlink(soap, this); // not required, but just to make sure if someone calls delete on this
}

void matrix::resize(int rows, int cols)
{ int i;
  vector *p;
  if (__size != rows)
  { if (__ptr)
    { p = soap_new_vector(soap, rows);
      for (i = 0; i < (rows <= __size ? rows : __size); i++)
      { if (this[i].__size != cols)
          (*this)[i].resize(cols);
	(p+i)->__ptr = __ptr[i].__ptr;
	(p+i)->__size = cols;
      }
      for (; i < rows; i++)
        __ptr[i].resize(cols);
    }
    else
    { __ptr = soap_new_vector(soap, rows);
      for (i = 0; i < rows; i++)
        __ptr[i].resize(cols);
      __size = rows;
    }
  }
  else
    for (i = 0; i < __size; i++)
      __ptr[i].resize(cols);
}

vector& matrix::operator[](int i) const
{ if (!__ptr || i < 0 || i >= __size)
    fprintf(stderr, "Array index out of bounds\n");
  return __ptr[i];
}

////////////////////////////////////////////////////////////////////////////////
//
//	A HTTP GET Handler to return WSDL (magic.wsdl)
//
////////////////////////////////////////////////////////////////////////////////

int http_get(struct soap * soap)
{ FILE *fd = NULL;
  char *s = strchr(soap->path, '?');  // soap->path has the URL path of soap->endpoint
  if (!s || strcmp(s, "?wsdl")) 
    return SOAP_GET_METHOD; 
  fd = fopen("magic.wsdl", "rb"); // open WSDL file to copy 
  if (!fd) 
    return 404; // return HTTP not found error 
  soap->http_content = "text/xml"; // HTTP header with text/xml content 
  soap_response(soap, SOAP_FILE); 
  for (;;) 
  { size_t r = fread(soap->tmpbuf, 1, sizeof(soap->tmpbuf), fd); 
    if (!r) 
      break; 
    if (soap_send_raw(soap, soap->tmpbuf, r)) 
      break; // can't send, but little we can do about that 
  } 
  fclose(fd); 
  soap_end_send(soap); 
  return SOAP_OK; 
}
