This directory contains instructions and an example application for Symbian.

NOTICE:

THIS IS AN OBSOLETE PART OF THE GSOAP PACKAGE. NO WARRANTIES (IMPLIED OR
OTHERWISE) OR GUARANTEES ON THE FUNCTIONALITY OF THE SOFTWARE ARE GIVEN.

INSTRUCTIONS:

The interop_all.h gSOAP header file defines the SOAP 1.1 RPC interop round 2
A, B, and C test services.

The interop_all.h was compiled with the gSOAP soapcpp2 compiler to produce the
serializers and stubs to access the interop services:

$ soapcpp2 -CLwx interop_all.h

The soapcpp2 compiler is platform-independent, so you can run the command on
Windows, Linux, or Mac OS X for example.

The generated files are:
soapH.h		header file for serializers
soapC.cpp	serializers
soapStub.cpp	header file for stubs
soapClient.cpp	stubs

Because the generated code is platform-independent, it is strongly advised to
try to build a simple test application on a non-Symbian platform first to verify
interoperability and data exchange. Because logging is disabled on Symbian, it
will be hard to find the source of a interop problem. After testing the
application, you can use the same sources to build a Symbian application.  The
stdsoap2.h and stdsoap2.cpp contains platform-dependent code and is required
to complete the build of your application.

To develop an application from a WSDL, run wsdl2h.exe (or a wsdl2h executable
for any other platform) on the WSDL file. Mind the command line options. For
example, wsdl2h.exe -c generates C code and wsdl2h.exe -s generates C++ code
without requiring STL. This command generated a C or C++ header file. After
this, you need to run soapcpp2.exe on the generated header file to create the
serializers and stubs.

See gsoapOnSymbian.doc for Symbian-specific build issues.

