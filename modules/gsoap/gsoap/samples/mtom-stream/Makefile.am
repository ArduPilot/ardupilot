####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = mtom-stream
SOAPHEADER = mtom-stream-test.h

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = mtom_stream_test.nsmap mtom_stream_test.wsdl

mtom_stream_SOURCES = mtom-stream-test.c $(SOAPHEADER) $(SOAP_C_SRC)
mtom_stream_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread

