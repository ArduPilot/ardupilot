URL=http://192.168.2.2:8000/ServiceModelSamples/service
WSDL=../../../../wsdl/wsdl2h
GSOAP=../../../../src/soapcpp2
SOAPH=../../../../stdsoap2.h
SOAPC=../../../../stdsoap2.c
SOAPCPP=../../../../stdsoap2.cpp
SOAPDOMC=../../../../dom.c
SOAPDOMCPP=../../../../dom.cpp
TYPEMAP=../../../../typemap.dat
CC=gcc -x c
CPP=g++ -x c++
LIBS=-lssl -lcrypto
GSFLAGS=-L -I../../../../import -I../../../..
COFLAGS=-O2
CWFLAGS=-Wall -Wno-deprecated-declarations
CIFLAGS=-I. -I../../../.. -I../../../../plugin
CMFLAGS=-DCB_THREAD -DWITH_WCF -DWITH_DOM -DWITH_OPENSSL
CFLAGS= $(CWFLAGS) $(COFLAGS) $(CIFLAGS) $(CMFLAGS)
all:		calculator
calculator:	calculator.h calculator.cpp $(SOAPH) $(SOAPC) wsrxClient.cpp
		$(GSOAP) $(GSFLAGS) -a -j calculator.h
		$(CPP) $(CFLAGS) -o calculator calculator.cpp soapC.cpp soapWSDualHttpBinding_USCOREICalculatorDuplexService.cpp soapWSDualHttpBinding_USCOREICalculatorDuplexProxy.cpp wsrxClient.cpp wsrxServer.cpp ../../../../plugin/wsrmapi.c ../../../../plugin/wsaapi.c ../../../../custom/duration.c $(SOAPDOMCPP) $(SOAPCPP) $(LIBS)
wsrxClient.cpp wsrxServer.cpp:
		$(GSOAP) $(GSFLAGS) -A -pwsrx ../../../../import/wsrm5.h
%.cer: %.pem
		openssl x509 -in $< -outform der -out $@
calculator.h:
		$(WSDL) -t $(TYPEMAP) -o calculator.h $(URL)?wsdl
.PHONY: clean distclean
clean:
		rm -f *.o soapH.h soapStub.h soapC.* soapClient.* soapServer.* soap*Proxy.* soap*Service.* soap*Object.*
		rm -f *.o wsrxH.h wsrxStub.h wsrxC.* wsrxClient.* wsrxServer.* wsrxwsrmObject.h wsrxwsrmProxy.h
distclean:
		rm -f *.o soapH.h soapStub.h soapC.* soapClient.* soapServer.* soap*Proxy.* soap*Service.* soap*Object.*
		rm -f *.o wsrxH.h wsrxStub.h wsrxC.* wsrxClient.* wsrxServer.* wsrxwsrmObject.h wsrxwsrmProxy.h
		rm -f *.xml *.nsmap *.log
		rm -f calculator
