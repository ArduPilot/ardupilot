/*	dimeclient.cpp

	Example DIME client for simple image server. This DIME client
	demonstrates the new gSOAP DIME streaming feature.

	Copyright (C) 2000-2002 Robert A. van Engelen. All Rights Reserved.

	Run from command line. The first optional argument is the image file
	name. The second optional argument is the service endpoint URL. The
	third optional argument is the file name to save the image file to.
*/

#include "soapH.h"
#include "dime.nsmap"
#include <sys/stat.h>	// use fstat() for streaming DIME

// streaming DIME callbacks
static void *dime_write_open(struct soap*, const char*, const char*, const char*);
static void dime_write_close(struct soap*, void*);
static int dime_write(struct soap*, void*, const char*, size_t);

static void *dime_read_open(struct soap*, void*, const char*, const char*, const char*);
static void dime_read_close(struct soap*, void*);
static size_t dime_read(struct soap*, void*, char*, size_t);


static int getImage(const char *name, const char *url, const char *outputfile) {
	struct soap *soap = soap_new();
	xsd__base64Binary image;
    soap->user = (void*)outputfile;
    soap->fdimewriteopen = dime_write_open;
    soap->fdimewriteclose = dime_write_close;
    soap->fdimewrite = dime_write;
	soap->connect_timeout = 10;
	int nRet = soap_call_ns__getImage(soap, url, "", (char *)name, image);
	if (nRet != SOAP_OK) {
		soap_print_fault(soap, stderr);
	} else {
		printf("got an image, I suppose\n");
	}
	soap_destroy(soap);
	soap_end(soap);
	soap_free(soap);
	return nRet;
}
static int putImage(const char *name, const char *url, const char *inputfile) {
	FILE *fd = fopen(inputfile, "rb");
	if (NULL == fd) {
		printf("failed to open %s\n", inputfile);
		return 3;
	}
	struct stat sb;
	if (0 != fstat(fileno(fd), &sb) || sb.st_size <= 0) {
		printf("cannot find the length of file %s\n", inputfile);
		return 4;

	}
	struct soap *soap = soap_new();
	xsd__base64Binary *pimage = soap_new_xsd__base64Binary(soap, -1);
    soap->user = (void *)inputfile;
	soap->fdimereadopen = dime_read_open;
	soap->fdimereadclose = dime_read_close;
	soap->fdimeread = dime_read;
	pimage->__ptr = (unsigned char*)fd; 
	pimage->__size = sb.st_size; // must set size
    pimage->type = "image/jpeg";
	pimage->options = soap_dime_option(soap, 0, "My sent picture");
	soap->connect_timeout = 10;

	int nStatus = 0;
	int nRet = soap_call_ns__putImage(soap, url, "", (char *)name, pimage, nStatus);
	if (nRet != SOAP_OK) {
		soap_print_fault(soap, stderr);
	} else {
		printf("sent an image, I suppose");
	}
	soap_destroy(soap);
	soap_end(soap);
	soap_free(soap);
	return nRet;
}

int main(const int argc, const char *const *const argv) { 
	if (4 != argc) {
		printf("usage: %s imagename outputfilename\n", argv[0]);
		return 1;
	}
	int nRet = 0;
	nRet = getImage(argv[1], argv[2], argv[3]);
	if (0 == nRet) {
		nRet = putImage(argv[1], argv[2], argv[3]);
	}
	return nRet;
}

static void *dime_write_open(struct soap *soap, const char *id, const char *type, const char *options)
{ FILE *handle = NULL;
  // we can return NULL without setting soap->error if we don't want to use the streaming callback for this DIME attachment
  handle = fopen((char*)soap->user, "wb");
  if (handle)
    printf("Streaming image id=%s type=%s into file %s\n", id, type, (char*)soap->user);
  else
  { soap->error = SOAP_EOF; // could not open file for writing
    soap->errnum = errno; // get reason
  }
  return (void*)handle;
}

static void dime_write_close(struct soap *soap, void *handle)
{ fclose((FILE*)handle);
}

static int dime_write(struct soap *soap, void *handle, const char *buf, size_t len)
{ size_t nwritten;
  while (len)
  { nwritten = fwrite(buf, 1, len, (FILE*)handle);
    if (!nwritten)
    { soap->errnum = errno; // get reason
      return SOAP_EOF;
    }
    len -= nwritten;
    buf += nwritten;
  }
  return SOAP_OK;
}

//// reading routines for putImage:
/** save a file sent by the client to our server */
static void *dime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *options) {
  // we should return NULL without setting soap->error if we don't want to use the streaming callback for this DIME attachment. The handle contains the non-NULL __ptr field value which should have been set in the application.
  // the value of the handle can be changed and will be passed on to the fdimeread and fdimereadclose callbacks. The value will not affect the __ptr field.
  return handle;
}

static void dime_read_close(struct soap *soap, void *handle) {
	fclose((FILE*)handle);
}
static size_t dime_read(struct soap *soap, void *handle, char *buf, size_t len) {
	return fread(buf, 1, len, (FILE*)handle);
}

