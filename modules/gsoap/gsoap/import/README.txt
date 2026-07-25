This directory contains common #import-ed items for the soapcpp2 compiler.

Use the #import directive in the gSOAP header file processed by soapcpp2 to
include the features listed below.

dom.h		DOM for xs:anyType, xs:any, xs:anyAttribute (link with dom.c/pp)
stdstring.h	module for std::string to enable std::string serializer sharing
stldeque.h	std::deque serializer
stllist.h	std::list serializer
stlset.h	std::set serializer
stlvector.h	std::vector serializer
stl.h		std::deque, std::list, std::set, and std::vector serializer
soap12.h	SOAP 1.2 namespaces
wsa.h		WS-Addressing 2004/08 (see plugin/wsaapi.c)
wsa3.h		WS-Addressing 2003/03 (see plugin/wsaapi.c)
wsa4.h		WS-Addressing 2004/03 (see plugin/wsaapi.c)
wsa5.h		WS-Addressing 2005/08 (see plugin/wsaapi.c)
wsrm.h		WS-ReliableMessaging 1.1 schema with WS-Addressing 2005/08
wsrm4.h		WS-ReliableMessaging 1.0 (2004, deprecated)
wsrm5.h		WS-ReliableMessaging 1.0 schema with WS-Addressing 2005/08
wsrx.h		WS-ReliableMessaging 1.0/1.1 operations (see plugin/wsrmapi.c)
wsse.h		WS-Security (see plugin/wsseapi.c)
wsse2.h		WS-Security (2002 version, requires wsc2.h)
wsdd.h		WS-Discovery 1.1 schema with WS-Addressing 2005/08
wsdd5.h		WS-Discovery 1.0 schema with WS-Addressing 2005/08
wsdd10.h	WS-Discovery 1.0 schema with WS-Addressing 2004/08
wsdx.h		WS-Discovery 1.0/1.1 operations (see plugin/wsddapi.c)
wsc.h		WS-SecureConversation
wst.h		WS-Trust 2005/12 schema definitions
wst2.h		WS-Trust 2005/02 schema definitions
wstx.h		WS-Trust 2005/12 operations (see plugin/wstapi.c)
wstx2.h		WS-Trust 2005/02 operations (see plugin/wstapi.c)
wsu.h		Utility
xlink.h		Xlink bindings
xmlmime.h	XML MIME bindings
xop.h		XOP MTOM attachments

This directory further contains files to support a growing number of WS-*
specifications.

For example, wsa.h is generated from WS/WS-Addressing.xsd with:

wsdl2h -cgy -o wsa.h -t WS/WS-typemap.dat WS/WS-Addressing.xsd

The typemap.dat file that is needed by wsdl2h to convert WSDL to a gSOAP header
file defines imported namespaces as follows:

wsa = <http://schemas.xmlsoap.org/ws/2004/08/addressing>

This ensures that the WS-Addressing support is #import-ed from "wsa.h" when
wsdl2h sees WS-Addressing elements. Thus, it does not attempt to extract the
WS-Addressing schema but instead uses the pre-build wsa.h definitions.

Note that the regular XML namespace bindings are defined in typemap.dat using
quotes, for example:

aws = "urn:PI/DevCentral/SoapService"
