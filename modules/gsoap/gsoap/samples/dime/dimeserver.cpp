/*
	dimeserver.cpp

	Example streaming DIME server. Supports three methods:

	putData stores multiple data sets on the server and returns named keys
		to each data set. The keys are unique and provide access to
		the data. Once data is stored, it cannot be removed.
	getData retrieves data sets given a set of named keys.
	getImage is an example file-based image retrieval method

	Data is stored in the current directory or the directory specified
	by the TMPDIR environment variable.

	Runs as CGI (not multi-threaded) or multi-threaded stand-alone
	Web service

	Copyright (C) 2000-2003 Robert A. van Engelen, Genivia, Inc.
	All Rights Reserved.

	NOTE: THE SERVER WILL ONLY SEND FILES THAT ARE IN THE CURRENT DIR FOR
	SECURITY REASONS. HOWEVER, THE AUTHOR IS NOT RESPONSIBLE FOR ANY DAMAGES
	THAT MAY RESULT FROM THE USE OF THIS PROGRAM.
	
	Usage:
	For stand-alone Web service functionality, run from the command line
	with port number as command line argument, e.g.
	> dimeserver 8085 &
	Use port 80 to run as HTTP Web server accessible over the Web
	Change the 'endpoint' in 'dimeclient.cpp' to
	endpoint="http://machine:8085"
	where 'machine' is the host name of the machine on which the service
	runs, or 'localhost' if the server runs on the same machine. Be careful
	to run the client in a separate directory from the server. Otherwise,
	the client and server may interfere in their file access.

	The service is multi-threaded. Multi-threading is not required, but can
	improve QoS. Remove the pthread code to obtain a non-multi-threaded
	service.

	Unix/Linux: add a sigpipe handler to avoid broken pipes with
	stand-alone server.

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
#include "dime.nsmap"

#include <sys/stat.h>	// use fstat() for streaming DIME
#include <assert.h>
#include <unistd.h>
#ifdef _POSIX_THREADS
#include <pthread.h>	// use Pthreads
#endif

#define BACKLOG		(100)	// max request backlog
#define MAX_FILE_SIZE	(10000)	// max file size when buffering file for HTTP 1.0 and file size cannot be determined

////////////////////////////////////////////////////////////////////////////////
//
//	Forward decls
//
////////////////////////////////////////////////////////////////////////////////

static void *process_request(void*);
static int getdata(struct soap*, const char*, ns__Data&);
static void saveData(ns__Data&, const char*);
static void sigpipe_handle(int);

////////////////////////////////////////////////////////////////////////////////
//
//	Streaming DIME attachment content handlers
//
////////////////////////////////////////////////////////////////////////////////

static void *dime_read_open(struct soap*, void*, const char*, const char*, const char*);
static void dime_read_close(struct soap*, void*);
static size_t dime_read(struct soap*, void*, char*, size_t);
static void *dime_write_open(struct soap*, const char*, const char*, const char*);
static void dime_write_close(struct soap*, void*);
static int dime_write(struct soap*, void*, const char*, size_t);

////////////////////////////////////////////////////////////////////////////////
//
//	Data for streaming DIME write handler
//
////////////////////////////////////////////////////////////////////////////////

struct dime_write_handle
{ char *name;	// file name
  FILE *fd;	// file fd
};

////////////////////////////////////////////////////////////////////////////////
//
//	Static data
//
////////////////////////////////////////////////////////////////////////////////

static const char *TMPDIR = ".";

////////////////////////////////////////////////////////////////////////////////
//
//	Main
//
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{ struct soap soap;
  char *s = getenv("TMPDIR");
  if (s)
    TMPDIR = s;
  // use HTTP chunking when possible
  // chunking allows streaming of DIME content without requiring DIME attachment size to be set
  // DIME attachments can be streamed without chunking only if the attachment size is set
  soap_init1(&soap, SOAP_IO_KEEPALIVE | SOAP_IO_CHUNK);
  // set DIME callbacks
  soap.fdimereadopen = dime_read_open;
  soap.fdimereadclose = dime_read_close;
  soap.fdimeread = dime_read;
  soap.fdimewriteopen = dime_write_open;
  soap.fdimewriteclose = dime_write_close;
  soap.fdimewrite = dime_write;
  if (argc < 2)
  { // no args: assume this is a CGI application
    // serve request
    soap_serve(&soap);
    // cleanup class instances
    soap_destroy(&soap);
    // cleanup
    soap_end(&soap);
  }
  else
  {
#ifdef _POSIX_THREADS
    pthread_t tid;
#endif
    struct soap *tsoap;
    int port;
    int m, s, i;
    // Unix SIGPIPE, this is OS dependent:
    // soap.accept_flags = SO_NOSIGPIPE;	// some systems like this
    // soap.socket_flags = MSG_NOSIGNAL;	// others need this
    // signal(SIGPIPE, sigpipe_handle);		// or a sigpipe handler (portable)
    // port is first command line argument
    port = atoi(argv[1]);
    // bind to current host and specified port
    m = soap_bind(&soap, NULL, port, BACKLOG);
    // if we could not bind, exit
    if (m < 0)
    { soap_print_fault(&soap, stderr);
      exit(1);
    }
    fprintf(stderr, "Socket connection successful %d\n", m);
    // die after 24 hrs waiting for activity on port
    soap.accept_timeout = 24*60*60;
    // IO timeouts
    soap.send_timeout = 30;
    soap.recv_timeout = 30;
    // loop through requests
    for (i = 1; ; i++)
    { // accept request
      s = soap_accept(&soap);
      // if timeout or error, report
      if (s < 0)
      { if (soap.errnum)
          soap_print_fault(&soap, stderr);
	else
          fprintf(stderr, "Server timed out\n");
	break;
      }
      fprintf(stderr, "Thread %d accepts socket %d connection from IP %d.%d.%d.%d\n", i, s, (int)(soap.ip>>24)&0xFF, (int)(soap.ip>>16)&0xFF, (int)(soap.ip>>8)&0xFF, (int)soap.ip&0xFF);
      // copy soap environment and spawn thread (if Pthreads is installed)
      tsoap = soap_copy(&soap);
#ifdef _POSIX_THREADS
      while (pthread_create(&tid, NULL, (void*(*)(void*))process_request, (void*)tsoap))
	sleep(1);
#else
      process_request((void*)tsoap);
#endif
    }
  }
  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap);
  return 0;
}

void *process_request(void *soap)
{
#ifdef _POSIX_THREADS
  pthread_detach(pthread_self());
#endif
  // serve request (or multiple requests with keep-alive enabled)
  soap_serve((struct soap*)soap);
  // cleanup class instances
  soap_destroy((struct soap*)soap);
  // cleanup
  soap_end((struct soap*)soap);
  // detach and free thread's copy of soap environment
  soap_free((struct soap*)soap);
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
//
//	Server methods
//
////////////////////////////////////////////////////////////////////////////////

int ns__putData(struct soap *soap, arrayOfData *data, arrayOfName *names)
{ // gSOAP switches to SOAP_IO_STORE when SOAP_IO_CHUNK (HTTP chunking) is not supported by the client
  // Since it is undesirable to use SOAP_IO_STORE with streaming, we reset it to SOAP_IO_BUFFER
  if ((soap->omode & SOAP_IO) == SOAP_IO_STORE)
    soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_BUFFER;
  // return name (key) for each data item
  names->resize(data->size());
  for (int i = 0; i < data->size(); i++)
  { char *s, *name;
    // the id field is set when DIME was used so the dime_write callbacks already saved the file
    if ((*data)[i].id)
    { assert((*data)[i].__ptr);
      name = ((struct dime_write_handle*)(*data)[i].__ptr)->name;
    }
    else
    { name = tempnam(TMPDIR, "data");
      fprintf(stderr, "Saving file %s\n", name);
      saveData((*data)[i], name);
    }
    s = strrchr(name, '/');
    if (!s)
      s = strrchr(name, '\\');
    if (!s)
      s = name;
    else
      s++;
    (*names)[i] = soap_strdup(soap, s);
    if (!(*data)[i].id)
    { // free data alloced with tempnam()
      free(name);
    }
  }
  return SOAP_OK;
}

int ns__getData(struct soap *soap, arrayOfName *names, arrayOfData *data)
{ // gSOAP switches to SOAP_IO_STORE when SOAP_IO_CHUNK (HTTP chunking) is not supported by the client.
  // Since it is undesirable to use SOAP_IO_STORE, we reset it to SOAP_IO_BUFFER
  if ((soap->omode & SOAP_IO) == SOAP_IO_STORE)
    soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_BUFFER;
  if (!names)
    return soap_sender_fault(soap, "Names required", NULL);
  data->resize(names->size());
  for (int i = 0; i < names->__size; i++)
  { if (!(*names)[i])
      return soap_sender_fault(soap, "Name required", NULL);
    if (getdata(soap, (*names)[i], (*data)[i]))
      return soap_sender_fault(soap, "Access denied", NULL);
  }
  return SOAP_OK;
}

int ns__getImage(struct soap *soap, char *name, ns__Data& image)
{ if (!name)
    return soap_sender_fault(soap, "Name required", NULL);
  if (getdata(soap, name, image))
    return soap_sender_fault(soap, "Access denied", NULL);
  image.type = (char*)"image/jpeg";
  image.options = soap_dime_option(soap, 0, name);
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//	Helper functions
//
////////////////////////////////////////////////////////////////////////////////

static int getdata(struct soap *soap, const char *name, ns__Data& data)
{ struct stat sb;
  FILE *fd = NULL;
  if (name && !strchr(name, '/') && !strchr(name, '\\') && !strchr(name, ':'))
  { char *s = (char*)soap_malloc(soap, strlen(TMPDIR) + strlen(name) + 2);
    strcpy(s, TMPDIR);
    strcat(s, "/");
    strcat(s, name);
    fd = fopen(s, "rb");
    if (!fd)
    { strcpy(s, name);
      fd = fopen(s, "rb");
    }
  }
  if (!fd)
    return SOAP_EOF;
  if ((soap->omode & SOAP_IO) == SOAP_IO_CHUNK) // chunked response is possible
  { data.__ptr = (unsigned char*)fd; // must set to non-NULL (this is our fd handle which we need in the callbacks)
    data.__size = 0; // zero size streams data with HTTP chunking
  }
  else if (!fstat(fileno(fd), &sb) && sb.st_size > 0)
  { // since we can get the length of the file, we can stream it
    data.__ptr = (unsigned char*)fd; // must set to non-NULL (this is our fd handle which we need in the callbacks)
    data.__size = sb.st_size;
  }
  else // we can't use HTTP chunking and we don't know the size, so buffer it
  { int i;
    data.__ptr = (unsigned char*)soap_malloc(soap, MAX_FILE_SIZE);
    for (i = 0; i < MAX_FILE_SIZE; i++)
    { int c;
      if ((c = fgetc(fd)) == EOF)
        break;
      data.__ptr[i] = c;
    }
    fclose(fd);
    data.__size = i;
  }
  data.type = (char*)""; // specify non-NULL id or type to enable DIME
  data.options = soap_dime_option(soap, 0, name);
  return SOAP_OK;
}

static void saveData(ns__Data& data, const char *name)
{ char *buf = (char*)data.__ptr;
  int len = data.__size;
  FILE *fd = fopen(name, "wb");
  if (!fd)
  { fprintf(stderr, "Cannot save file %s\n", name);
    return;
  }
  while (len)
  { size_t nwritten = fwrite(buf, 1, len, fd);
    if (!nwritten)
    { fprintf(stderr, "Cannot write to %s\n", name);
      return;
    }
    len -= nwritten;
    buf += nwritten;
  }
  fclose(fd);
}

static void sigpipe_handle(int x) { }

////////////////////////////////////////////////////////////////////////////////
//
//	Streaming DIME attachment content handlers
//
////////////////////////////////////////////////////////////////////////////////

static void *dime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *options)
{ // we should return NULL without setting soap->error if we don't want to use the streaming callback for this DIME attachment. The handle contains the non-NULL __ptr field value which should have been set in the application.
  // return value of this function will be passed on to the fdimeread and fdimereadclose callbacks. The return value will not affect the __ptr field.
  return handle;
}

static void dime_read_close(struct soap *soap, void *handle)
{ fclose((FILE*)handle);
}

static size_t dime_read(struct soap *soap, void *handle, char *buf, size_t len)
{ return fread(buf, 1, len, (FILE*)handle);
}

static void *dime_write_open(struct soap *soap, const char *id, const char *type, const char *options)
{ // we can return NULL without setting soap->error if we don't want to use the streaming callback for this DIME attachment
  struct dime_write_handle *handle = (struct dime_write_handle*)soap_malloc(soap, sizeof(struct dime_write_handle));
  if (!handle)
  { soap->error = SOAP_EOM;
    return NULL;
  }
  char *name = tempnam(TMPDIR, "data");
  fprintf(stderr, "Saving file %s\n", name);
  handle->name = soap_strdup(soap, name);
  free(name);
  handle->fd = fopen(handle->name, "wb");
  if (!handle->fd)
  { soap->error = SOAP_EOF; // could not open file for writing
    soap->errnum = errno; // get reason
    return NULL;
  }
  return (void*)handle;
}

static void dime_write_close(struct soap *soap, void *handle)
{ fclose(((struct dime_write_handle*)handle)->fd);
}

static int dime_write(struct soap *soap, void *handle, const char *buf, size_t len)
{ while (len)
  { size_t nwritten = fwrite(buf, 1, len, ((struct dime_write_handle*)handle)->fd);
    if (!nwritten)
    { soap->errnum = errno; // get reason
      return SOAP_EOF;
    }
    len -= nwritten;
    buf += nwritten;
  }
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//	ns__Data class
//
////////////////////////////////////////////////////////////////////////////////

ns__Data::ns__Data()
{ __ptr = NULL;
  __size = 0;
  id = NULL;
  type = NULL;
  options = NULL;
  soap = NULL;
}

////////////////////////////////////////////////////////////////////////////////
//
//	arrayOfData class
//
////////////////////////////////////////////////////////////////////////////////

arrayOfData::arrayOfData()
{ __ptr = NULL;
  __size = 0;
  soap = NULL;
}

arrayOfData::arrayOfData(struct soap *soap, int n)
{ __ptr = NULL;
  __size = 0;
  this->soap = soap;
  resize(n);
}

arrayOfData::~arrayOfData()
{ resize(0);
}

int arrayOfData::size()
{ return __size;
}

void arrayOfData::resize(int n)
{ if (__ptr)
  { if (soap) // if created by soap environment
      soap_delete(soap, __ptr); // then delete
    else
      delete[] __ptr;
  }
  __size = n;
  if (n <= 0)
    __ptr = NULL;
  else if (soap)
    __ptr = soap_new_ns__Data(soap, n);
  else
    __ptr = new ns__Data[n];
}

ns__Data& arrayOfData::operator[](int i) const
{ assert(__ptr && i >= 0 && i < __size);
  return __ptr[i];
}

////////////////////////////////////////////////////////////////////////////////
//
//	arrayOfName class
//
////////////////////////////////////////////////////////////////////////////////

arrayOfName::arrayOfName()
{ __ptr = NULL;
  __size = 0;
  soap = NULL;
}

arrayOfName::arrayOfName(struct soap *soap, int n)
{ __ptr = NULL;
  __size = 0;
  this->soap = soap;
  resize(n);
}

arrayOfName::~arrayOfName()
{ resize(0);
}

int arrayOfName::size()
{ return __size;
}

void arrayOfName::resize(int n)
{ if (__ptr)
  { if (soap) // if created by soap environment
      soap_delete(soap, __ptr); // then delete
    else
      free(__ptr);
  }
  __size = n;
  if (n <= 0)
    __ptr = NULL;
  else
  { if (soap)
      __ptr = (char**)soap_malloc(soap, sizeof(char*) * n);
    else
      __ptr = (char**)malloc(sizeof(char*) * n);
    memset(__ptr, 0, n);
  }
}

char*& arrayOfName::operator[](int i) const
{ assert(__ptr && i >= 0 && i < __size);
  return __ptr[i];
}

