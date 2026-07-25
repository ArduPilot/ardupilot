####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = asyncsoap asyncrest
SOAPHEADER = async.h

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = async.nsmap

asyncsoap_SOURCES = asyncsoap.c $(SOAPHEADER) $(SOAP_C_CLIENT)
asyncsoap_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread

asyncrest_SOURCES = asyncrest.c $(SOAPHEADER) $(SOAP_C_CORE)
asyncrest_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread
