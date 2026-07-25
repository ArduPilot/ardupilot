####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = event handler
SOAPHEADER = event.h

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = Event.nsmap Event.wsdl

event_SOURCES = event.c $(SOAPHEADER) $(SOAP_C_CLIENT)
event_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS)

handler_SOURCES = handler.c $(SOAPHEADER) $(SOAP_C_SRC)
handler_LDADD = $(SOAP_C_LIB) $(SAMPLE_EXTRA_LIBS) -lpthread


