CC=cl
CPP=cl
LIBS=wsock32.lib
SECURE_LIBS=$(LIBS) libeay32.lib ssleay32.lib

GSOAP=..\\soapcpp2.exe

SOAPH=stdsoap2.h
SOAPC=stdsoap2.c
SOAPCPP=stdsoap2.cpp

COFLAGS=-O2
# COFLAGS=-ZI -DDEBUG

CFLAGS= -EHsc -nologo $(COFLAGS)

wsdl2h.exe:	wsdlC.obj wsdl.obj schema.obj soap.obj mime.obj wsp.obj bpel.obj types.obj service.obj wsdl2h.obj stdsoap2.obj
		$(CPP) $(CFLAGS) -Fewsdl2h.exe wsdl2h.obj wsdlC.obj wsdl.obj schema.obj \
			soap.obj mime.obj wsp.obj bpel.obj types.obj service.obj stdsoap2.obj $(LIBS)
#		cp -f wsdl2h ..

secure:
		touch wsdl2h.cpp
		make wsdl2h_secure
wsdl2h_secure:	wsdlC.obj wsdl.obj schema.cpp soap.obj mime.obj wsp.obj bpel.obj types.obj service.obj wsdl2h.obj httpda.obj md5evp.obj stdsoap2.obj
		$(CPP) $(CFLAGS) -DWITH_OPENSSL -Fewsdl2h.exe wsdl2h.obj wsdlC.obj wsdl.obj \
			schema.cpp soap.obj mime.obj wsp.obj bpel.obj types.obj service.obj httpda.obj md5evp.obj stdsoap2.obj $(SECURE_LIBS)

wsdl2h.obj:	wsdl2h.cpp
		$(CPP) -c $(CFLAGS) wsdl2h.cpp
wsdlC.obj:	wsdlC.cpp
		$(CPP) -c $(CFLAGS) wsdlC.cpp
wsdlC.cpp:	schema.h soap.h mime.h http.h wsdl.h includes.h imports.h
		$(GSOAP) -CS -pwsdl wsdl.h
types.obj:	types.h types.cpp
		$(CPP) -c $(CFLAGS) types.cpp
service.obj:	types.h service.h service.cpp
		$(CPP) -c $(CFLAGS) service.cpp
wsdl.obj:	wsdl.h wsdl.cpp includes.h imports.h
		$(CPP) -c $(CFLAGS) wsdl.cpp
schema.obj:	schema.h schema.cpp includes.h imports.h
		$(CPP) -c $(CFLAGS) schema.cpp
soap.obj:	soap.h soap.cpp includes.h imports.h
		$(CPP) -c $(CFLAGS) soap.cpp
mime.obj:	mime.h mime.cpp includes.h imports.h
		$(CPP) -c $(CFLAGS) mime.cpp
wsp.obj:	wsp.h wsp.cpp includes.h imports.h
		$(CPP) -c $(CFLAGS) wsp.cpp
bpel.obj:	bpel.h bpel.cpp includes.h imports.h
		$(CPP) -c $(CFLAGS) bpel.cpp
httpda.obj:	../plugin/httpda.c
		$(CC) -c -I../plugin $(CFLAGS) ../plugin/httpda.c
md5evp.obj:	../plugin/md5evp.c
		$(CC) -c -I../plugin $(CFLAGS) ../plugin/md5evp.c
stdsoap2.obj:	$(SOAPCPP)
		$(CPP) -c $(CFLAGS) $(SOAPCPP)

clean:
		del /f *.obj wsdlH.h wsdlStub.h wsdlC.cpp wsdlClient.cpp wsdlServer.cpp \
			wsdlClientLib.cpp wsdlServerLib.cpp
distclean:
		del /f *.obj *.wsdl. *.xsd *.xml *.nsmap wsdl2h.exe wsdlH.h wsdlStub.h \
			wsdlC.cpp wsdlClient.cpp wsdlServer.cpp wsdlClientLib.cpp wsdlServerLib.cpp
