DELFILES= *.ncb *.opt *.plg *.clw *.pch *.aps *.pdb *.dll *.exe \
	*.nsmap *.xml *.wsdl ns.xsd \
	RECV.log SENT.log TEST.log \
	soapC.c?? soapClient.c?? soapServer.c?? soapStub.h soapH.h \
	soapClientLib.* soapServerLib.*
DELDIRS= .\Debug .\Release .\Win32

all:
	MsBuild dime.vcxproj
	MsBuild dimeclnt.vcxproj

clean:
	for %v in ($(DELDIRS)) DO if exist %v\*.* rmdir /s/q %v
	for %v in ($(DELFILES)) DO if exist %v del %v

