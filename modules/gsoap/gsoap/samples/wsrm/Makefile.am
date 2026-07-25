####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = foreign 1.4

noinst_PROGRAMS = wsrmdemo wsrmdemo_udp wsrmdemo_ssl
SOAPHEADER = wsrmdemo.h
SOAP_FLAGS = -a

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = wsrmdemo.nsmap wsrmdemo.wsdl

wsrmdemo_CFLAGS =
wsrmdemo_SOURCES = wsrmdemo.c $(SOAPHEADER) $(SOAP_C_SRC) ../../plugin/wsaapi.c ../../plugin/wsrmapi.c ../../plugin/threads.c ../../custom/duration.c
wsrmdemo_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread

wsrmdemo_udp_CFLAGS = -DWITH_UDP
wsrmdemo_udp_SOURCES = wsrmdemo.c $(SOAPHEADER) $(SOAP_C_SRC) ../../plugin/wsaapi.c ../../plugin/wsrmapi.c ../../plugin/threads.c ../../custom/duration.c
wsrmdemo_udp_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread

wsrmdemo_ssl_CFLAGS = $(WSDL2H_EXTRA_FLAGS)
wsrmdemo_ssl_SOURCES = wsrmdemo.c $(SOAPHEADER) $(SOAP_C_SRC) ../../plugin/wsaapi.c ../../plugin/wsrmapi.c ../../plugin/threads.c ../../custom/duration.c
wsrmdemo_ssl_LDADD = $(SOAP_C_LIB_SSL) $(SAMPLE_SSL_LIBS) $(SAMPLE_EXTRA_LIBS) -lpthread
