####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = udpclient udpserver
SOAPHEADER = udp.h

include ../Makefile.defines
include ../Makefile.c_rules
SOAP_NS = udp.nsmap

udpclient_SOURCES = udpclient.c $(SOAPHEADER) $(SOAP_C_CLIENT)

udpserver_SOURCES = udpserver.c $(SOAPHEADER) $(SOAP_C_SERVER)

