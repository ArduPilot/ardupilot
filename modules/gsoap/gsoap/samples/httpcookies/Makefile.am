####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = ck.cgi ckclient
SOAPHEADER = ck.h

include ../Makefile.defines
include ../Makefile.cpp_rules
SOAP_NS = ck.nsmap ck.wsdl

ck_cgi_CXXFLAGS = -DWITH_COOKIES -D$(platform)
ck_cgi_SOURCES = ckserver.cpp $(SOAPHEADER) $(SOAP_CPP_SERVER)
ck_cgi_LDADD = $(SOAP_CPP_LIB_CK) $(SAMPLE_EXTRA_LIBS)

ckclient_CXXFLAGS = -DWITH_COOKIES -D$(platform)
ckclient_SOURCES = ckclient.cpp $(SOAPHEADER) $(SOAP_CPP_CLIENT)
ckclient_LDADD = $(SOAP_CPP_LIB_CK) $(SAMPLE_EXTRA_LIBS)


