GSOAP=../../src/soapcpp2
SOAPH=../../stdsoap2.h
SOAPC=../../stdsoap2.c
SOAPCPP=../../stdsoap2.cpp
CURLAPI=../../plugin/curlapi.c
CUSTOMDATE=../../custom/struct_tm_date.c
XMLRPC=../xml-rpc-json/xml-rpc.c
JSONC=../xml-rpc-json/json.c
CC=gcc
CPP=g++
LIBS=-lcurl -lcrypto -lssl
COFLAGS=-O2
CWFLAGS=-Wall -Wno-deprecated-declarations
CIFLAGS=-I. -I../.. -I../../plugin -I../xml-rpc
CMFLAGS=-DWITH_OPENSSL 
CFLAGS= $(CWFLAGS) $(COFLAGS) $(CIFLAGS) $(CMFLAGS)
all:		curlclient curlclient++ curlrest curl-json-currentTime
curlclient:	calc.h curlclient.c
		$(GSOAP) -c -CL calc.h
		$(CC) $(CFLAGS) -o curlclient curlclient.c soapC.c soapClient.c $(CURLAPI) $(SOAPC) $(LIBS)
curlclient++:	calc.h curlclient.cpp
		$(GSOAP) -j -C calc.h
		$(CPP) $(CFLAGS) -o curlclient++ curlclient.cpp soapC.cpp soapcalcProxy.cpp $(CURLAPI) $(SOAPCPP) $(LIBS)
curlrest:	calc.h curlrest.c
		$(GSOAP) -I../.. -c -CSL -0 person.h
		$(CC) $(CFLAGS) -o curlrest curlrest.c soapC.c $(CUSTOMDATE) $(CURLAPI) $(SOAPC) $(LIBS)
curl-json-currentTime:	curl-json-currentTime.c
		$(GSOAP) -c -CSL ../xml-rpc-json/xml-rpc.h
		$(CC) $(CFLAGS) -o curl-json-currentTime curl-json-currentTime.c soapC.c $(XMLRPC) $(JSONC) $(CURLAPI) $(SOAPC) $(LIBS)
.PHONY: clean distclean
clean:
		rm -f *.o soapH.h soapStub.h soapC.c soapC.cpp soapClient.c soapClient.cpp soapcalcProxy.h soapcalcProxy.cpp
distclean:
		rm -f *.o soapH.h soapStub.h soapC.c soapC.cpp soapClient.c soapClient.cpp soapcalcProxy.h soapcalcProxy.cpp
		rm -f *.wsdl *.xsd *.xml *.nsmap *.log
		rm -f curlclient curlclient++ curlrest curl-json-currentTime
