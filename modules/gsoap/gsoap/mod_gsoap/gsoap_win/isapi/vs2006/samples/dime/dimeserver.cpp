/*	dimesrv.cpp

	Example simple image server using DIME,
	Adapted by Ch. Aberger to demonstrate dime for Win32 IIS. 
	Copyright (C) 2000-2002 Robert A. van Engelen. All Rights Reserved.
	Copyright (C) 2003 Christian T. Aberger.

	Runs as mod_gsoap (http://www.aberger.at/SOAP) as IIS SOAP service
	Web service

	NOTE: THE SERVER WILL ONLY SEND FILES THAT ARE IN THE CURRENT DIR FOR
	SECURITY REASONS. HOWEVER, THE AUTHOR IS NOT RESPONSIBLE FOR ANY DAMAGES
	THAT MAY RESULT FROM THE USE OF THIS PROGRAM.
	AND ALSO ABERGER IS RESPONSIBLE FOR NOTHING.
*/

#include "soapH.h"
#include "dime.nsmap"
#include <sys/stat.h>	// use fstat() for streaming DIME
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <windows.h>

#define MAX_FILE_SIZE (10000)	// Max. file size

// streaming DIME callbacks for sending data to client.
static void *dime_read_open(struct soap*, void*, const char*, const char*, const char*);
static void dime_read_close(struct soap*, void*);
static size_t dime_read(struct soap*, void*, char*, size_t);

// streaming DIME callbacks for receiving data from client.
static void *dime_write_open(struct soap*, const char*, const char*, const char*);
static void dime_write_close(struct soap*, void*);
static int dime_write(struct soap*, void*, const char*, size_t);


extern "C" {

/** This function is called by mod_gsoap after the dll was successfully loaded and before processing begins.
	You can do any one-time initialization here.
*/
int mod_gsoap_init() {
	// todo: add your initialization code here 
	return SOAP_OK;
}
/** This function is called after all processing was done before dll is unloaded.
	You can do any cleanup here.
*/
int mod_gsoap_terminate() {
	// todo: add your termination code here 
	return SOAP_OK;
}

}


int ns__getImage(struct soap *soap, char *name, xsd__base64Binary &image) { 
	/* in this sample, due to security reasons the file must be in the system's %TEMP% folder */
	TCHAR szPath[_MAX_PATH];
	if (0 == ::GetTempPath(sizeof szPath, szPath)) {
		return soap_receiver_fault(soap, "cannot find the TEMP folder", NULL);
	}
	if (name) { 
		FILE *fd = NULL;
		// do some checks on the file name to verify it is local:
		if (!strchr(name, '/') && !strchr(name, '\\') && !strchr(name, ':')) {
			strcat(szPath, name);
			fd = fopen(szPath, "rb");
		}
		if (!fd) {
			char szMsg[MAX_PATH + 64];
			strcpy(szMsg, "Cannot open file: ");
			strcat(szMsg, szPath);
			return soap_receiver_fault(soap, szMsg, NULL);
		}
		struct stat sb;
		if (!fstat(fileno(fd), &sb) && sb.st_size > 0) { // since we can get the length of the file, we can stream it
		  soap->fdimereadopen = dime_read_open;
		  soap->fdimereadclose = dime_read_close;
		  soap->fdimeread = dime_read;
		  image.__ptr = (unsigned char*)fd; // must set to non-NULL (this is our fd handle which we need in the callbacks)
		  image.__size = sb.st_size; // must set size
		} else { // don't know the size, so buffer it
			image.__ptr = (unsigned char*)soap_malloc(soap, MAX_FILE_SIZE);
			for (int i = 0; i < MAX_FILE_SIZE; i++) {
				int c;
				if ((c = fgetc(fd)) == EOF) {
					break;
				}
				image.__ptr[i] = c;
			}
			fclose(fd);
			image.__size = i;
		}
	    image.type = "image/jpeg";
		image.options = soap_dime_option(soap, 0, "My picture");
	} else {
		return soap_receiver_fault(soap, "Name required", NULL);
	}
	return SOAP_OK;
}

/** save a file sent by the client to our server */
static void *dime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *options)
{ // we should return NULL without setting soap->error if we don't want to use the streaming callback for this DIME attachment. The handle contains the non-NULL __ptr field value which should have been set in the application.
  // the value of the handle can be changed and will be passed on to the fdimeread and fdimereadclose callbacks. The value will not affect the __ptr field.
  return handle;
}

static void dime_read_close(struct soap *soap, void *handle)
{ fclose((FILE*)handle);
}

static size_t dime_read(struct soap *soap, void *handle, char *buf, size_t len)
{ return fread(buf, 1, len, (FILE*)handle);
}

int ns__putImage(struct soap *soap, char *name, xsd__base64Binary *image, int &status) { 
	int nRet = SOAP_OK;
	TCHAR szPath[_MAX_PATH];
	if (0 == ::GetTempPath(sizeof szPath, szPath)) {
		return soap_receiver_fault(soap, "cannot find the TEMP folder", NULL);
	}
	strcat(szPath, "upload");
	if (INVALID_FILE_ATTRIBUTES == ::GetFileAttributes(szPath)) {
		::CreateDirectory(szPath, NULL); // create it if it ain't exist.
		if (INVALID_FILE_ATTRIBUTES == ::GetFileAttributes(szPath)) {
			return soap_receiver_fault(soap, "Failed to create output directory", szPath);
		}
	}
	strcat(szPath, "\\");
	strcat(szPath, name);
	FILE *fp = fopen(szPath, "wb");
	status = 0;
	if (NULL != fp) {
		status = fwrite(image->__ptr, sizeof(char), image->__size, fp);
		fclose(fp);
	} else {
		return soap_receiver_fault(soap, "Failed to open output file", szPath);
	}
	if (status != image->__size) {
		return soap_receiver_fault(soap, "Failed to write all bytes to file.", szPath);
	}
	return SOAP_OK;
}
