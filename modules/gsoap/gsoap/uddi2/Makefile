.SUFFIXES: .wsdl .xsd
GSOAP=../bin/soapcpp2
WSDL2H=../bin/wsdl2h
SOAPH=../stdsoap2.h
SOAPC=../stdsoap2.c
SOAPCPP=../stdsoap2.cpp
SOAPDOMC=../dom.c
SOAPDOMCPP=../dom.cpp
CC=gcc
CPP=g++
LIBS=
COFLAGS=-O2
CWFLAGS=-Wall
CIFLAGS=-I../
CMFLAGS=-DWITH_NONAMESPACES
CFLAGS= $(CWFLAGS) $(COFLAGS) $(CIFLAGS) $(CMFLAGS)
all:			example1 example2
example1:		example1.cpp inqClientLib.o inquire_v2.o $(SOAPCPP) envC.o
			$(CPP) $(CFLAGS) -o example1 example1.cpp inqClientLib.o inquire_v2.o $(SOAPCPP) envC.o
example2:		example2.cpp inqClientLib.o inquire_v2.o $(SOAPCPP) envC.o
			$(CPP) $(CFLAGS) -o example2 example2.cpp inqClientLib.o inquire_v2.o $(SOAPCPP) envC.o
example3:		example3.cpp pubClientLib.o publish_v2.o $(SOAPCPP) envC.o
			$(CPP) $(CFLAGS) -DWITH_OPENSSL -o example3 example3.cpp pubClientLib.o publish_v2.o $(SOAPCPP) envC.o -lssl -lcrypto
uddiClientLib.cpp:	uddi_v2.h inquire_v2.h publish_v2.h
			$(GSOAP) -I.. -C -p uddi uddi_v2.h
			$(GSOAP) -I.. -C -p inq inquire_v2.h
			$(GSOAP) -I.. -C -p pub publish_v2.h
inqClientLib.cpp:	inquire_v2.h
			$(GSOAP) -I.. -C -p inq inquire_v2.h
pubClientLib.cpp:	publish_v2.h
			$(GSOAP) -I.. -C -p pub publish_v2.h
envC.cpp:
			echo '' > env.h
			$(GSOAP) -I.. -CSL -p env env.h
.c.o:
			$(CPP) $(CFLAGS) -c $<
.cpp.o:
			$(CPP) $(CFLAGS) -c $<
uddi_v2.h:		inquire_v2.wsdl publish_v2.wsdl uddi_v2.xsd uddi2-typemap.dat
			$(WSDL2H) -tuddi2-typemap.dat -ouddi_v2.h inquire_v2.wsdl publish_v2.wsdl
inquire_v2.h:		inquire_v2.wsdl uddi_v2.xsd uddi2-typemap.dat
			$(WSDL2H) -tuddi2-typemap.dat inquire_v2.wsdl
publish_v2.h:		publish_v2.wsdl uddi_v2.xsd uddi2-typemap.dat
			$(WSDL2H) -tuddi2-typemap.dat publish_v2.wsdl
.PHONY:			clean distclean
clean:		
			rm -f *.o *.log *.xml *.nsmap *Stub.h inqH.h pubH.h uddiH.h *Proxy.h *Object.h inqC*.cpp inqS*.cpp pubC*.cpp pubS*.cpp uddiC*.cpp uddiS*.cpp env*.h env*.cpp
distclean:		
			rm -f *.o *.log *.xml *.nsmap *Stub.h inqH.h pubH.h uddiH.h *Proxy.h *Object.h inqC*.cpp inqS*.cpp pubC*.cpp pubS*.cpp uddiC*.cpp uddiS*.cpp env*.h env*.cpp
			rm -f example1 example2 example3
