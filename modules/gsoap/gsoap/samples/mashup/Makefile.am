####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = mashupclient mashupserver
SOAPHEADER = mashup.h

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = mashup.nsmap

mashupclient_SOURCES = mashupclient.c $(SOAPHEADER) $(SOAP_C_CLIENT)
mashupclient_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS)

mashupserver_SOURCES = mashupserver.c $(SOAPHEADER) $(SOAP_C_CLIENT) soapServer.c
mashupserver_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS)
