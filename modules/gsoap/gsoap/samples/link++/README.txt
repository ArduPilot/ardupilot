
How to combine multiple clients and servers into one executable?

1. Run wsdl2h once on all WSDLs together.
   The wsdl2h tool can import multiple WSDL files at once to combine multiple
   clients and service into one executable. See the samples/mashup and
   samples/mashup++ examples.

2. Run wsdl2h on each WSDL and use soapcpp2 compiler options.
   If there is a need to build clients and services from multiple gSOAP header
   files, then an alternative approach is required demonstrated by the material
   included in this part of the gSOAP package.

The advantages of #1 is that you end up with smaller code, because redundant
definitions are eliminated.

The advantage of #2 is a more modular setup, thus supporting dynamic linking of 
separately compiled service modules. However, the build process requires
additional steps depending on C or C++. The C++ project build process is
simpler, because C++ namespaces can be used to separate the definitions as is
shown in the samples/link++ example.

The C++ examples in this directory illustrate how multiple clients and services
can be compiled and linked into one executable from multiple gSOAP header
files. The C++ examples accomplish this by using C++ namespaces (soapcpp2
option -q) to separate the service definitions and type serializers.

When linking multiple soapcpp2-generated files, a single file with shared SOAP
Header definitions is required. This means that all SOAP Header members must be
collected into a single SOAP Header definition. Similar requirements are needed
for SOAP Faults (the SOAP_ENV__Detail struct with details to be more precise).

So the first step is to generate header and fault handlers:
$ soapcpp2 -CS -penv env.h
Multiple SOAP_ENV__Header structs can be defined in env.h moved there from the
client and service gSOAP header files. All SOAP_ENV__Header members will be
combined into a single struct by soapcpp2. Also the struct SOAP_ENV__Detail can
be defined this way. Its definition is shared among the clients and services.

Then each file.h is compiled with:
$ soapcp2 -i -q<name> <file>.h
where <name> is a name for a C++ namespace. The -i option generates proxies and
server objects, which are declared in this namespace. Compile the
<name>xyzProxy.cpp or <name>xyzServer.cpp code and the <name>C.cpp serializers,
and envC.cpp.

