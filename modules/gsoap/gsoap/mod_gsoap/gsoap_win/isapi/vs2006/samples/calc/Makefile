DELFILES= *.ncb *.opt *.plg *.clw *.pch *.aps \
	*.nsmap *.xml *.wsdl ns.xsd \
	RECV.log SENT.log TEST.log \
	soapC.c?? soapcalcProxy.h soapClient.c?? soapServer.c?? soapStub.h soapH.h \
	soapcalcObject.h soapClientLib.* soapServerLib.*


all:
	echo done

clean:
	-rmdir /s/q .\Debug
	-rmdir /s/q .\Release
	for %v in ($(DELFILES)) DO if exist %v del %v

