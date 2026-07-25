URL=http://192.168.2.2:8000/ServiceModelSamples/service
WSDL=../../../../wsdl/wsdl2h
GSOAP=../../../../src/soapcpp2
SOAPH=../../../../stdsoap2.h
SOAPC=../../../../stdsoap2.c
SOAPCPP=../../../../stdsoap2.cpp
SOAPDOMC=../../../../dom.c
SOAPDOMCPP=../../../../dom.cpp
TYPEMAP=../../../../typemap.dat
CC=gcc
CPP=g++
LIBS=-lssl -lcrypto
GSFLAGS=-L -I../../../../import
COFLAGS=-O2
CWFLAGS=-Wall -Wno-deprecated-declarations
CIFLAGS=-I. -I../../../.. -I../../../../plugin
CMFLAGS=-DWITH_DOM -DWITH_OPENSSL
#CMFLAGS=-DDEBUG -DWITH_DOM -DWITH_OPENSSL
CFLAGS= $(CWFLAGS) $(COFLAGS) $(CIFLAGS) $(CMFLAGS)
all:		cacert.cer calculator
calculator:	calculator.h calculator.cpp $(SOAPH) $(SOAPC)
		$(GSOAP) $(GSFLAGS) -a -j calculator.h
		$(CPP) $(CFLAGS) -o calculator calculator.cpp soapC.cpp soapBasicHttpBinding_USCOREICalculatorService.cpp soapBasicHttpBinding_USCOREICalculatorProxy.cpp $(SOAPDOMCPP) $(SOAPCPP) $(LIBS)
%.cer: %.pem
		openssl x509 -in $< -outform der -out $@
calculator.h:
		$(WSDL) -t $(TYPEMAP) -o calculator.h $(URL)?wsdl
.PHONY: clean distclean
clean:
		rm -f *.o soapH.h soapStub.h soapC.* soapClient.* soapServer.* soap*Proxy.* soap*Service.* soap*Object.*
distclean:
		rm -f *.o soapH.h soapStub.h soapC.* soapClient.* soapServer.* soap*Proxy.* soap*Service.* soap*Object.*
		rm -f *.xml *.nsmap *.log
		rm -f calculator
