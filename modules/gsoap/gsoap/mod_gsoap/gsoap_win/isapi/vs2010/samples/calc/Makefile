DELFILES= *.ncb *.opt *.plg *.clw *.pch *.aps *.pdb *.dll *.exe \
	*.nsmap *.xml *.wsdl ns.xsd \
	RECV.log SENT.log TEST.log \
	soapC.c?? soapcalcProxy.h soapClient.c?? soapServer.c?? soapStub.h soapH.h \
	soapcalcObject.h soapClientLib.* soapServerLib.*
DELDIRS= .\Debug .\Release .\Win32

all:
	MsBuild calc.vcxproj
	MsBuild calcclnt.vcxproj

clean:
	for %v in ($(DELDIRS)) DO if exist %v\*.* rmdir /s/q %v
	for %v in ($(DELFILES)) DO if exist %v del %v

