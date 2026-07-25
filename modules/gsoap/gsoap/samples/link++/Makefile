# Simple Makefile for C++ example with muliple client proxy linkage
WSDL2H=../../wsdl/wsdl2h
GSOAP=../../src/soapcpp2 -I../..
SOAPH=../../stdsoap2.h
SOAPC=../../stdsoap2.c
SOAPCPP=../../stdsoap2.cpp
SOAPDOMC=../../dom.c
SOAPDOMCPP=../../dom.cpp
CC=gcc
CPP=g++
LIBS=
COFLAGS=-O2
CWFLAGS=-Wall
CIFLAGS=-I. -I.. -I../..
CMFLAGS=
CFLAGS= $(CWFLAGS) $(COFLAGS) $(CIFLAGS) $(CMFLAGS)
all:		xmas
xmas:		xmas.cpp envC.o stdsoap2.o gmtProxy.o calccalcProxy.o xmasmashupService.o
		$(CPP) $(CFLAGS) -o xmas xmas.cpp envC.o stdsoap2.o gmtProxy.o calccalcProxy.o xmasmashupService.o gmtC.cpp calcC.cpp xmasC.cpp
stdsoap2.o:	$(SOAPH) $(SOAPC)
		$(CPP) $(CFLAGS) -DWITH_NONAMESPACES -c ../../stdsoap2.cpp
envC.o:		env.h header.h fault.h
		$(GSOAP) -CS -penv env.h
		$(CPP) $(CFLAGS) -c envC.cpp
gmtProxy.cpp:	gmt.h
		$(GSOAP) -i -w -C -qgmt gmt.h
gmtProxy.o:	gmtProxy.cpp
		$(CPP) $(CFLAGS) -c gmtProxy.cpp
calccalcProxy.cpp:	calc.h
		$(GSOAP) -i -w -C -qcalc calc.h
		cp calcH.h soapH.h
calccalcProxy.o:	calccalcProxy.cpp
		$(CPP) $(CFLAGS) -c calccalcProxy.cpp
xmasmashupService.cpp:	xmas.hpp
		$(GSOAP) -i -S -qxmas xmas.hpp
xmasmashupService.o:	xmasmashupService.cpp
		$(CPP) $(CFLAGS) -c xmasmashupService.cpp
xmas.hpp:	xmas.wsdl
		$(WSDL2H) -s -o xmas.hpp xmas.wsdl
.PHONY: clean distclean
clean:
		rm -f *.o *H.h *Stub.h *C.cpp *Client.cpp *Server.cpp *ClientLib.cpp *ServerLib.cpp *Proxy.h *Proxy.cpp *Service.h *Service.cpp
distclean:
		rm -f *.o *H.h *Stub.h *C.cpp *Client.cpp *Server.cpp *ClientLib.cpp *ServerLib.cpp *Proxy.h *Proxy.cpp *Service.h *Service.cpp
		rm -f *.xsd *.xml *.nsmap *.log
		rm -f xmas
