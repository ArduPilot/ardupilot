####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = foreign 1.4

noinst_PROGRAMS = sslclient sslserver
SOAPHEADER = ssl.h

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = $(SOAP_NS_DEF)

sslclient_CFLAGS = $(WSDL2H_EXTRA_FLAGS)
sslclient_SOURCES = sslclient.c $(SOAPHEADER) $(SOAP_C_CLIENT) ssl_setup.c ../../plugin/threads.c
sslclient_LDADD = $(SOAP_C_LIB_SSL) $(SAMPLE_EXTRA_LIBS) $(SAMPLE_SSL_LIBS) -lpthread

sslserver_CFLAGS = $(WSDL2H_EXTRA_FLAGS)
sslserver_SOURCES = sslserver.c $(SOAPHEADER) $(SOAP_C_SERVER) ../../plugin/threads.c
sslserver_LDADD = $(SOAP_C_LIB_SSL) $(SAMPLE_EXTRA_LIBS) $(SAMPLE_SSL_LIBS) -lpthread
