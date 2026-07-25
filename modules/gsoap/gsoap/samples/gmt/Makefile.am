####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = gmtclient gmtserver
SOAPHEADER = gmt.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = gmt.nsmap gmt.wsdl

gmtclient_SOURCES = gmtclient.cpp $(SOAPHEADER) $(SOAP_CPP_CLIENT)
gmtclient_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

gmtserver_SOURCES = gmtserver.cpp $(SOAPHEADER) $(SOAP_CPP_SERVER)
gmtserver_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)
