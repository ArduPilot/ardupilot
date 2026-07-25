####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = foreign 1.4

noinst_PROGRAMS = wsademo
SOAPHEADER = wsademo.h
SOAP_FLAGS = -a

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = wsademo.nsmap wsademo.wsdl

wsademo_CFLAGS = $(WSDL2H_EXTRA_FLAGS)
wsademo_SOURCES = wsademo.c $(SOAPHEADER) $(SOAP_C_SRC) ../../plugin/wsaapi.c
wsademo_LDADD = $(SOAP_C_LIB_SSL) $(SAMPLE_SSL_LIBS) $(SAMPLE_EXTRA_LIBS)


