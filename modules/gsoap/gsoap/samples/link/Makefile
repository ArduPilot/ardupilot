# Simple Makefile for C example with muliple client linkage
WSDL2H=../../wsdl/wsdl2h
GSOAP=../../src/soapcpp2
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
CIFLAGS=-I.. -I../..
CMFLAGS=
CFLAGS= $(CWFLAGS) $(COFLAGS) $(CIFLAGS) $(CMFLAGS)
all:		xmas
xmas:		xmas.c envC.o stdsoap2.o gmtClientLib.o calcClientLib.o xmasServerLib.o
		$(CC) $(CFLAGS) -o xmas xmas.c envC.o stdsoap2.o gmtClientLib.o calcClientLib.o xmasServerLib.o
stdsoap2.o:	$(SOAPH) $(SOAPC)
		$(CC) $(CFLAGS) -DWITH_NONAMESPACES -c $(SOAPC)
envC.o:		env.h header.h fault.h
		$(GSOAP) -c -CS -penv env.h
		$(CC) $(CFLAGS) -c envC.c
gmtClientLib.c:	gmt.h
		$(GSOAP) -c -C -n -pgmt gmt.h
gmtClientLib.o:	gmtClientLib.c
		$(CC) $(CFLAGS) -c gmtClientLib.c
calcClientLib.c:calc.h
		$(GSOAP) -c -C -n -pcalc calc.h
calcClientLib.o:calcClientLib.c
		$(CC) $(CFLAGS) -c calcClientLib.c
xmasServerLib.c:xmas.h
		$(GSOAP) -c -S -n -pxmas xmas.h
xmasServerLib.o:xmasServerLib.c
		$(CC) $(CFLAGS) -c xmasServerLib.c
xmas.h:		xmas.wsdl
		$(WSDL2H) -c xmas.wsdl
.PHONY: clean distclean
clean:
		rm -f *.o soapH.h soapStub.h soapC.cpp soapClient.cpp soapServer.cpp soapClientLib.cpp soapServerLib.cpp soap*Proxy.h soap*Proxy.cpp soap*Service.h soap*Service.cpp
distclean:
		rm -f *.o soapH.h soapStub.h soapC.cpp soapClient.cpp soapServer.cpp soapClientLib.cpp soapServerLib.cpp soap*Proxy.h soap*Proxy.cpp soap*Service.h soap*Service.cpp
		rm -f *.xsd *.xml *.nsmap *.log
		rm -f xmas
