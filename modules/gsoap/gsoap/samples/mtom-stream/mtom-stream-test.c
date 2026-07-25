/*
	mtom-stream-test.c

	Example streaming MTOM client and server.

	Copyright (C) 2000-2006 Robert A. van Engelen, Genivia, Inc.
	All Rights Reserved.

	Usage (server):

	mtom-stream-test 8085 &

		Starts a server on your local host at port 8085.

	Usage (client):

	mtom-stream-test -p file1 file2 file3 ...

		Stores files file1, file2, etc. at the server side. The server
		saves them locally under a key. The storage keys are printed at
		the client side. The keys provide access to the data using
		option -g (get). The server saves files in its current
		directory, or the directory specified by the TMPDIR environment
		variable.

	mtom-stream-test -g name1 name2 name3 ...

		Retrieves files stored under keys name1, name2, etc.
		The keys must correspond to the keys returned when storing
		files. Files are stored by the server locally under the key
		name.

	Unix/Linux: add a sigpipe handler to avoid broken pipes.

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
#include "mtom_stream_test.nsmap"
#if defined(_POSIX_THREADS) || defined(_SC_THREADS)
#include <pthread.h>
#endif
#include <sys/stat.h>

/******************************************************************************\
 *
 *	Default endpoint
 *
\******************************************************************************/

const char *endpoint = "http://localhost:8085";

/******************************************************************************\
 *
 *	Server-Side Location to Store Files (Changed by TMPDIR env var)
 *
\******************************************************************************/

const char *TMPDIR = ".";

/******************************************************************************\
 *
 *	Server-Side Streaming MIME Handler for Saving File Data Under Keys
 *
\******************************************************************************/

struct mime_server_handle
{ char *key;	/* file name */
  FILE *fd;	/* file fd */
};

/******************************************************************************\
 *
 *	Forward Declarations
 *
\******************************************************************************/

int cgi_server();
int run_server(int port);
int run_client(int, char**);

int open_data(struct soap *soap, const char *file, struct x__Data *data);

int client_putData(struct soap *soap, int argc, char **argv);
int client_getData(struct soap *soap, int argc, char **argv);

void *mime_read_open(struct soap*, void*, const char*, const char*, const char*);
void mime_read_close(struct soap*, void*);
size_t mime_read(struct soap*, void*, char*, size_t);

void *mime_server_write_open(struct soap *soap, void *handle, const char *id, const char *type, const char *description, enum soap_mime_encoding encoding);
void mime_server_write_close(struct soap *soap, void *handle);
int mime_server_write(struct soap *soap, void *handle, const char *buf, size_t len);

void *mime_client_write_open(struct soap *soap, void *handle, const char *id, const char *type, const char *description, enum soap_mime_encoding encoding);
void mime_client_write_close(struct soap *soap, void *handle);
int mime_client_write(struct soap *soap, void *handle, const char *buf, size_t len);

char *file_type(const char *file);

/******************************************************************************\
 *
 *	Main
 *
\******************************************************************************/

int main(int argc, char **argv)
{ const char *tmp = getenv("TMPDIR");
  if (tmp)
    TMPDIR = tmp;
  if (argc < 2)
    return cgi_server();
  if (argc < 3)
    return run_server(atoi(argv[1]));
  return run_client(argc, argv);
}

/******************************************************************************\
 *
 *	CGI Server
 *
\******************************************************************************/

int cgi_server()
{ /* CGI-style: serve request from stdin to stdout */
  return soap_serve(soap_new1(SOAP_ENC_MTOM)); /* Enable MTOM XOP attachments */
}

#if !defined(_POSIX_THREADS) && !defined(_SC_THREADS)

/******************************************************************************\
 *
 *	Stand-Alone Iterative Server
 *
\******************************************************************************/

int run_server(int port)
{ struct soap soap;
  int ret;
  /* Enable MTOM */
#ifdef WITH_GZIP
  soap_init1(&soap, SOAP_ENC_MTOM | SOAP_ENC_ZLIB); /* Enable MTOM */
#else
  soap_init1(&soap, SOAP_ENC_MTOM); 
#endif
  /* Set the MIME callbacks */
  soap.fmimereadopen = mime_read_open;
  soap.fmimereadclose = mime_read_close;
  soap.fmimeread = mime_read;
  soap.fmimewriteopen = mime_server_write_open;
  soap.fmimewriteclose = mime_server_write_close;
  soap.fmimewrite = mime_server_write;
  /* Bind socket */
  if (!soap_valid_socket(soap_bind(&soap, NULL, port, 100)))
    soap_print_fault(&soap, stderr);
  else
  { fprintf(stderr, "Bind to port %d successful\n", port);
    /* Optional: let server time out after one hour */
    soap.accept_timeout = 3600;
    /* IO timeouts (sec) */
    soap.send_timeout = 10;
    soap.recv_timeout = 10;
    /* Unix/Linux SIGPIPE, this is OS dependent:
    soap.accept_flags = SO_NOSIGPIPE;	// some systems like this
    soap.socket_flags = MSG_NOSIGNAL;	// others need this
    signal(SIGPIPE, sigpipe_handle);	// or a sigpipe handler (more portable)
    */
    /* Server loop */
    for (;;)
    { int sock = soap_accept(&soap);
      if (!soap_valid_socket(sock))
      { if (soap.errnum)
          soap_print_fault(&soap, stderr);
        else
        { fprintf(stderr, "Server timed out (see code how to change this)\n");
          break;
        }
      }
      fprintf(stderr, "Accepting socket %d connection from IP %d.%d.%d.%d... ", sock, (int)(soap.ip>>24)&0xFF, (int)(soap.ip>>16)&0xFF, (int)(soap.ip>>8)&0xFF, (int)soap.ip&0xFF);
      if (soap_serve(&soap))
        soap_print_fault(&soap, stderr);
      fprintf(stderr, "done\n");
      soap_destroy(&soap);
      soap_end(&soap);
    } 
  }
  ret = soap.error;
  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap);
  return ret;
}

#else

/******************************************************************************\
 *
 *	Multi-Threaded Stand-Alone Server
 *
\******************************************************************************/

void *process_request(void*);

int run_server(int port)
{ struct soap soap;
  int i, ret;
  /* Enable MTOM */
#ifdef WITH_GZIP
  soap_init1(&soap, SOAP_ENC_MTOM | SOAP_ENC_ZLIB); /* Enable MTOM */
#else
  soap_init1(&soap, SOAP_ENC_MTOM); 
#endif
  /* Set the MIME callbacks */
  soap.fmimereadopen = mime_read_open;
  soap.fmimereadclose = mime_read_close;
  soap.fmimeread = mime_read;
  soap.fmimewriteopen = mime_server_write_open;
  soap.fmimewriteclose = mime_server_write_close;
  soap.fmimewrite = mime_server_write;
  /* Bind socket */
  if (!soap_valid_socket(soap_bind(&soap, NULL, port, 100)))
    soap_print_fault(&soap, stderr);
  else
  { fprintf(stderr, "Bind to port %d successful\n", port);
    /* Optional: let server time out after one hour */
    soap.accept_timeout = 3600;
    /* IO timeouts (sec) */
    soap.send_timeout = 10;
    soap.recv_timeout = 10;
    /* Unix/Linux SIGPIPE, this is OS dependent:
    soap.accept_flags = SO_NOSIGPIPE;	// some systems like this
    soap.socket_flags = MSG_NOSIGNAL;	// others need this
    signal(SIGPIPE, sigpipe_handle);	// or a sigpipe handler (more portable)
    */
    /* Main thread spawns server threads */
    for (i = 1; ; i++)
    { struct soap *tsoap;
      pthread_t tid;
      int sock = soap_accept(&soap);
      if (!soap_valid_socket(sock))
      { if (soap.errnum)
          soap_print_fault(&soap, stderr);
        else
        { fprintf(stderr, "Server timed out (see code how to change this)\n");
          break;
        }
      }
      fprintf(stderr, "Thread %d accepts socket %d connection from IP %d.%d.%d.%d\n", i, sock, (int)(soap.ip>>24)&0xFF, (int)(soap.ip>>16)&0xFF, (int)(soap.ip>>8)&0xFF, (int)soap.ip&0xFF);
      /* Copy soap environment and spawn thread */
      tsoap = soap_copy(&soap);
      pthread_create(&tid, NULL, (void*(*)(void*))process_request, (void*)tsoap);
    } 
  }
  ret = soap.error;
  soap_done(&soap);
  return ret;
}

void *process_request(void *soap)
{ pthread_detach(pthread_self());
  /* Serve request (or multiple requests with keep-alive enabled) */
  soap_serve((struct soap*)soap);
  /* Cleanup and delete deserialized data */
  soap_destroy((struct soap*)soap);
  soap_end((struct soap*)soap);
  /* Detach thread's copy of soap environment */
  soap_done((struct soap*)soap);
  /* Free soap environment */
  free(soap);
  fprintf(stderr, "done\n");
  return NULL;
}

#endif

/******************************************************************************\
 *
 *	Client
 *
\******************************************************************************/

int run_client(int argc, char **argv)
{ struct soap soap;
  int ret = 0;
#ifdef WITH_GZIP
  soap_init1(&soap, SOAP_ENC_MTOM | SOAP_ENC_ZLIB); /* Enable MTOM */
#else
  soap_init1(&soap, SOAP_ENC_MTOM); /* Enable MTOM */
#endif
  /* Set the MIME callbacks */
  soap.fmimereadopen = mime_read_open;
  soap.fmimereadclose = mime_read_close;
  soap.fmimeread = mime_read;
  soap.fmimewriteopen = mime_client_write_open;
  soap.fmimewriteclose = mime_client_write_close;
  soap.fmimewrite = mime_client_write;
  /* Connect timeout value (sec) (not supported by Linux) */
  soap.connect_timeout = 10;
  /* IO timeouts (sec) */
  soap.send_timeout = 10;
  soap.recv_timeout = 10;
  /* Unix/Linux SIGPIPE, this is OS dependent:
  soap.accept_flags = SO_NOSIGPIPE;	// some systems like this
  soap.socket_flags = MSG_NOSIGNAL;	// others need this
  signal(SIGPIPE, sigpipe_handle);	// or a sigpipe handler (more portable)
  */
  switch (argv[1][1])
  { case 'p':
      ret = client_putData(&soap, argc, argv);
      break;
    case 'g':
      ret = client_getData(&soap, argc, argv);
      break;
    default:
      fprintf(stderr, "Usage: mtom-stream-test -p file1 file2 file3 ...\n");
      fprintf(stderr, "       mtom-stream-test -g key1 key2 key3 ...\n");
  }
  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap);
  return ret;
}

int open_data(struct soap *soap, const char *file, struct x__Data *data)
{ struct stat sb;
  FILE *fd = NULL;
  int size;
  soap_default_x__Data(soap, data);
  fd = fopen(file, "rb");
  if (!fd)
  { fprintf(stderr, "Cannot open file %s\n", file);
    return soap->error = SOAP_EOF;
  }
  /* The handle for the streaming MIME callback is the open file fd */      
  data->xop__Include.__ptr = (unsigned char*)fd;
  if (!fstat(fileno(fd), &sb) && sb.st_size > 0)
    size = sb.st_size;
  else
  { /* File size is unknown, so must use HTTP chunking and set size = 0 */
    soap_set_omode(soap, SOAP_IO_CHUNK);
    size = 0;
  }
  data->xop__Include.__size = size;
  data->xmime5__contentType = file_type(file);
  data->xop__Include.id = NULL;
  data->xop__Include.type = data->xmime5__contentType;
  data->xop__Include.options = NULL;
  return SOAP_OK;
}

int client_putData(struct soap *soap, int argc, char **argv)
{ int i;
  struct x__DataSet data;
  struct m__PutDataResponse response;
  data.__size = argc - 2;
  data.item = soap_malloc(soap, (argc - 2)*sizeof(struct x__Data));
  for (i = 2; i < argc; i++)
    open_data(soap, argv[i], &data.item[i - 2]);
  if (soap_call_m__PutData(soap, endpoint, NULL, &data, &response))
    soap_print_fault(soap, stderr);
  else
  { printf("Data stored with keys:\n");
    for (i = 0; i < response.x__keys.__size; i++)
      printf("\t%s\n", response.x__keys.key[i]);
    printf("Use these keys to retrieve the data with -g key1 key2 ...\n");
  }
  return soap->error;
}

int client_getData(struct soap *soap, int argc, char **argv)
{ int i;
  struct x__Keys keys;
  struct m__GetDataResponse response;
  keys.__size = argc - 2;
  keys.key = soap_malloc(soap, (argc - 2)*sizeof(char*));
  for (i = 2; i < argc; i++)
    keys.key[i - 2] = argv[i];
  /* Pass this information to the callbacks using the soap->user variable */
  soap->user = (void*)keys.key;
  if (soap_call_m__GetData(soap, endpoint, NULL, &keys, &response))
    soap_print_fault(soap, stderr);
  else
    printf("Data retrieved\n");
  return soap->error;
}

/******************************************************************************\
 *
 *	Server Operations
 *
\******************************************************************************/

int m__PutData(struct soap *soap, struct x__DataSet *data, struct m__PutDataResponse *response)
{ int i;
  /* This operation is called AFTER the streaming MIME attachment callbacks
     handled the data.  gSOAP switches to SOAP_IO_STORE when SOAP_IO_CHUNK
     (HTTP chunking) is not supported by the client. Since it is undesirable to
     use SOAP_IO_STORE to buffer the entire message, we reset it to the default
     SOAP_IO_BUFFER for socket connections.
  */
  if ((soap->omode & SOAP_IO) == SOAP_IO_STORE)
    soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_BUFFER;
  if (!data)
    return soap_sender_fault(soap, "No data", NULL);
  /* Set up array of keys to return a key for each data item saved */
  response->x__keys.__size = data->__size;
  response->x__keys.key = soap_malloc(soap, data->__size*sizeof(char**));
  /* Each key is stored in the mime_server_handle object, set by the callbacks and accessible through the data __ptr, where .type must be set as this indicates the presence of an attachment with a type */
  for (i = 0; i < data->__size; i++)
  { if (data->item[i].xop__Include.__ptr && data->item[i].xop__Include.type)
    { const char *key = ((struct mime_server_handle*)(data->item[i].xop__Include.__ptr))->key;
      const char *s;
      if (!key)
        return soap_sender_fault(soap, "Missing name", NULL);
      s = strrchr(key, '/');
      if (!s)
        s = strrchr(key, '\\');
      if (!s)
        s = key;
      else
        s++;
      response->x__keys.key[i] = soap_strdup(soap, s);
    }
    else
      response->x__keys.key[i] = "-";
  }
  return SOAP_OK;
}

int m__GetData(struct soap *soap, struct x__Keys *keys, struct m__GetDataResponse *response)
{ int i;
  char *file = soap_malloc(soap, strlen(TMPDIR) + 80);
  if ((soap->omode & SOAP_IO) == SOAP_IO_STORE)
    soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_BUFFER;
  if (!keys)
    return soap_sender_fault(soap, "No keys", NULL);
  /* Set up array of attachments to return */
  response->x__data.__size = keys->__size;
  response->x__data.item = soap_malloc(soap, keys->__size*sizeof(struct x__Data));
  for (i = 0; i < keys->__size; ++i)
  { strcpy(file, TMPDIR);
    strcat(file, "/");
    if (strncmp(keys->key[i], "data", 4))
      strcat(file, "data");
    strcat(file, keys->key[i]);
    open_data(soap, file, &response->x__data.item[i]);
  }
  return SOAP_OK;
}

/******************************************************************************\
 *
 *	Streaming MIME Callbacks
 *
\******************************************************************************/

void *mime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *description)
{ FILE *fd = (FILE*)handle;
  /* we should return NULL without setting soap->error if we don't want to use
     the streaming callback for this MIME attachment. The handle contains the
     non-NULL __ptr field value of xop__Include which should have been set in
     the application. The return value of this function will be passed on to
     the fmimeread and fmimereadclose callbacks. The return value will not
     affect the value of the __ptr field.
  */
  fprintf(stderr, "Opening streaming outbound MIME channel for id=%s type=%s\n", id, type);
  return (void*)fd;
}

size_t mime_read(struct soap *soap, void *handle, char *buf, size_t len)
{ return fread(buf, 1, len, (FILE*)handle);
}

void mime_read_close(struct soap *soap, void *handle)
{ fprintf(stderr, "Closing streaming outbound MIME channel\n");
  fclose((FILE*)handle);
}

/******************************************************************************\
 *
 *	Server-Side Streaming MIME Callbacks
 *
\******************************************************************************/

void *mime_server_write_open(struct soap *soap, void *unused_handle, const char *id, const char *type, const char *description, enum soap_mime_encoding encoding)
{ /* Note: the 'unused_handle' is always NULL */
  /* Return NULL without setting soap->error if we don't want to use the streaming callback for this DIME attachment */
  const char *file;
  struct mime_server_handle *handle = soap_malloc(soap, sizeof(struct mime_server_handle));
  if (!handle)
  { soap->error = SOAP_EOM;
    return NULL;
  }
  /* Create a new file */
  file = tempnam(TMPDIR, "data");
  /* The file name is also the key */
  handle->key = soap_strdup(soap, file);
  handle->fd = fopen(file, "wb");
  free((void*)file);
  if (!handle->fd)
  { soap->error = soap_sender_fault(soap, "Cannot save data to file", handle->key);
    soap->errnum = errno; /* get reason */
    return NULL;
  }
  fprintf(stderr, "Saving file %s type %s\n", handle->key, type?type:"");
  return (void*)handle;
}

void mime_server_write_close(struct soap *soap, void *handle)
{ fclose(((struct mime_server_handle*)handle)->fd);
}

int mime_server_write(struct soap *soap, void *handle, const char *buf, size_t len)
{ FILE *fd = ((struct mime_server_handle*)handle)->fd;
  while (len)
  { size_t nwritten = fwrite(buf, 1, len, fd);
    if (!nwritten)
    { soap->errnum = errno;
      return SOAP_EOF;
    }
    len -= nwritten;
    buf += nwritten;
  }
  return SOAP_OK;
}

/******************************************************************************\
 *
 *	Client-Side Streaming MIME Callbacks
 *
\******************************************************************************/

void *mime_client_write_open(struct soap *soap, void *unused_handle, const char *id, const char *type, const char *description, enum soap_mime_encoding encoding)
{ /* Note: the 'unused_handle' is always NULL */
  FILE *fd;
  const char *file;
  fprintf(stderr, "Opening streaming inbound MIME channel for id=%s type=%s\n", id, type);
  /* soap->user points to array of keys (strings) that are file names */
  file = *(char**)soap->user;
  soap->user = (void*)(((char**)soap->user)+1);
  fd = fopen(file, "wb");
  if (!fd)
    soap->error = soap_receiver_fault(soap, "Cannot save data to file", file);
  return (void*)fd;
}

void mime_client_write_close(struct soap *soap, void *handle)
{ fprintf(stderr, "Closing streaming inbound MIME channel\n");
  fclose((FILE*)handle);
}

int mime_client_write(struct soap *soap, void *handle, const char *buf, size_t len)
{ FILE *fd = (FILE*)handle;
  while (len)
  { size_t nwritten = fwrite(buf, 1, len, fd);
    if (!nwritten)
    { soap->errnum = errno;
      return SOAP_EOF;
    }
    len -= nwritten;
    buf += nwritten;
  }
  return SOAP_OK;
}

/******************************************************************************\
 *
 *	Misc
 *
\******************************************************************************/

char *file_type(const char *file)
{ int n = strlen(file);
  if (n > 4 && (!strcmp(file + n - 4, ".xml")
             || !strcmp(file + n - 4, ".xsd")
             || !strcmp(file + n - 5, ".wsdl")))
    return "text/xml";
  if (n > 4 && !strcmp(file + n - 4, ".jpg"))
    return "image/jpg";
  return "*/*";
}
