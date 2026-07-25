/*
	dimeclient.cpp

	Example streaming DIME client for DIME server (dimeserver.cpp).
	Supports three methods:

	putData stores multiple data sets on the server and returns
		named references to each data set
	getData retrieves data sets given named references.
	getImage is an example file-based image retrieval method

	Change the endpoint in dime.h to your needs.

	Copyright (C) 2000-2003 Robert A. van Engelen, Genivia, Inc.
	All Rights Reserved.

	Usage (server):

	Start dimeserver on your host at port 8085 (see dimeserver.cpp):

	dimeserver 8085 &

	Usage (client):

	dimeclient
	dimeclient name
	dimeclient [-p] [-g] name ...

	dimeclient
		Without args retrieves image.jpg
	dimeclient name
		Retrieves image stored under name
	dimeclient -p name1 name2 ...
		Stores files name1, name2, etc. The storage keys are printed.
		The keys provide access to the data on the server.
	dimeclient -g name1 name2 ...
		Retrieves files stored under keys name1, name2, etc.
		The keys must correspond to the keys returned when storing
		files. Files are stored locally under the key name.

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
#include "dime.nsmap"
#include <assert.h>

// use the default endpoint set in dime.h for demo:
const char *endpoint = NULL;
// use the localhost for -p and -g (put and get):
const char *localhost = "http://localhost:8085";

////////////////////////////////////////////////////////////////////////////////
//
//	Forward decls
//
////////////////////////////////////////////////////////////////////////////////

static void putData(struct soap*, int, char**);
static void getData(struct soap*, int, char**);
static void getImage(struct soap*, char*);
static void saveData(ns__Data&, const char*);

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
//	Main
//
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{ struct soap soap;
  // use HTTP 1.1 chunking
  // HTTP chunking allows streaming of DIME content without requiring DIME attachment size to be set
  // DIME attachments can be streamed without chunking ONLY if the attachment size is set
  soap_init1(&soap, SOAP_IO_CHUNK);
  // set DIME callbacks
  soap.fdimereadopen = dime_read_open;
  soap.fdimereadclose = dime_read_close;
  soap.fdimeread = dime_read;
  soap.fdimewriteopen = dime_write_open;
  soap.fdimewriteclose = dime_write_close;
  soap.fdimewrite = dime_write;
  // connect timeout value (not supported by Linux)
  soap.connect_timeout = 10;
  // IO timeouts
  soap.send_timeout = 5;
  soap.recv_timeout = 5;
  soap.transfer_timeout = 30;
  // Unix/Linux SIGPIPE, this is OS dependent:
  // soap.accept_flags = SO_NOSIGPIPE;	// some systems like this
  // soap.socket_flags = MSG_NOSIGNAL;	// others need this
  // signal(SIGPIPE, sigpipe_handle);	// or a sigpipe handler (portable)
  if (argc < 3)
  { char *name;
    if (argc < 2)
      name = (char*)"image.jpg";
    else
      name = argv[1];
    getImage(&soap, name);
  }
  else
  { switch (argv[1][1])
    { case 'p':
	endpoint = localhost;
        putData(&soap, argc, argv);
        break;
      case 'g':
	endpoint = localhost;
        getData(&soap, argc, argv);
        break;
      default:
        fprintf(stderr, "Usage: [-p] [-g] name ...\n");
	exit(0);
    }
  }
  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap);
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//	Helper functions
//
////////////////////////////////////////////////////////////////////////////////

static void putData(struct soap *soap, int argc, char **argv)
{ arrayOfData data;
  arrayOfName names;
  data.resize(argc - 2);
  for (int i = 2; i < argc; i++)
  { data[i - 2].__ptr = (unsigned char*)argv[i];
    // MUST set id or type to enable DIME
    // zero size indicates streaming DIME (this requires HTTP chunking)
    data[i - 2].type = (char*)"";
  }
  if (soap_call_ns__putData(soap, endpoint, NULL, &data, &names))
    soap_print_fault(soap, stderr);
  else
  { printf("Data stored with keys:\n");
    for (int j = 0; j < names.size(); j++)
      printf("\t%s\n", names[j]);
    printf("Use these keys to retrieve the data\n");
  }
}

static void getData(struct soap *soap, int argc, char **argv)
{ arrayOfData data;
  arrayOfName names;
  names.resize(argc - 2);
  for (int i = 2; i < argc; i++)
    names[i - 2] = argv[i];
  soap->user = (void*)names.__ptr;
  if (soap_call_ns__getData(soap, endpoint, NULL, &names, &data))
    soap_print_fault(soap, stderr);
  else
  { for (int j = 0; j < data.size(); j++)
      if (!data[j].id)
        saveData(data[j], argv[j + 2]);
    printf("Data retrieved\n");
  }
}

static void getImage(struct soap *soap, char *name)
{ ns__Data image;
  arrayOfName temp;
  temp.resize(1);
  temp[0] = name;
  soap->user = (void*)temp.__ptr;
  if (soap_call_ns__getImage(soap, endpoint, NULL, name, image))
    soap_print_fault(soap, stderr);
  else if (image.id)
  { if (image.__size)
      printf("Got image %s size=%d type=%s through streaming DIME\n", name, image.__size, image.type?image.type:"");
    else
      printf("Got image %s type=%s through chunked streaming DIME\n", name, image.type?image.type:"");
  }
  else
  { printf("Got image %s\n", name);
    saveData(image, name);
  }
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
  printf("Saved file %s\n", name);
}

////////////////////////////////////////////////////////////////////////////////
//
//	Streaming DIME attachment content handlers
//
////////////////////////////////////////////////////////////////////////////////

static void *dime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *options)
{ FILE *fd;
  (void)soap; (void)id; (void)type; (void)options;
  // we should return NULL without setting soap->error if we don't want to use the streaming callback for this DIME attachment. The handle contains the non-NULL __ptr field value which should have been set in the application.
  // return value of this function will be passed on to the fdimeread and fdimereadclose callbacks. The return value will not affect the __ptr field.
  fd = fopen((char*)handle, "rb");
  return (void*)fd;
}

static void dime_read_close(struct soap *soap, void *handle)
{ (void)soap;
  fclose((FILE*)handle);
}

static size_t dime_read(struct soap *soap, void *handle, char *buf, size_t len)
{ (void)soap;
  return fread(buf, 1, len, (FILE*)handle);
}

static void *dime_write_open(struct soap *soap, const char *id, const char *type, const char *options)
{ // we can return NULL without setting soap->error if we don't want to use the streaming callback for this DIME attachment
  FILE *handle = NULL;
  char *name;
  (void)id; (void)type;
  // get file name from options (here we assume it's not '\0' terminated), but the gsoap engine adds NULs to DIME options after reading content
  if (options)
  { size_t len = ((unsigned char)options[2] << 8) | ((unsigned char)options[3]); // option string length
    name = (char*)soap_malloc(soap, len + 1);
    strncpy(name, options + 4, len);
    name[len] = '\0';
    handle = fopen(name, "wb");
    if (!handle)
    { soap->error = SOAP_EOF; // could not open file for writing
      soap->errnum = errno; // get reason
      return NULL;
    }
  }
  else 
    soap->error = soap_receiver_fault(soap, "Cannot save to file, because no file name was present in attachment", NULL);
  return (void*)handle;
}

static void dime_write_close(struct soap *soap, void *handle)
{ (void)soap;
  fclose((FILE*)handle);
}

static int dime_write(struct soap *soap, void *handle, const char *buf, size_t len)
{ while (len)
  { size_t nwritten = fwrite(buf, 1, len, (FILE*)handle);
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

