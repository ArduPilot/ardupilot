####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = luclient luserver.cgi
SOAPHEADER = lu.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = lu.nsmap lu.wsdl

luclient_SOURCES = luclient.cpp lumat.cpp $(SOAPHEADER) $(SOAP_CPP_CLIENT)
luclient_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

luserver_cgi_SOURCES = luserver.cpp lumat.cpp $(SOAPHEADER) $(SOAP_CPP_SERVER)
luserver_cgi_LDADD = $(SOAP_CPP_LIB) $(SAMPLE_EXTRA_LIBS)

