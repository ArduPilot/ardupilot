gSOAP UDDI v2

See directory html/index.html for documentation and examples of the UDDI v2 API
generated with gSOAP and Doxygen. The API is written in C++. However, wsdl2h
option -c can be used to generate C code serializers for UDDI v2.

The API implementation supports UDDI v2 inquiry and publish operations.

A Makefile is included to build two example C++ clients, example1 and example2,
to search services and businesses, respectively.

Code can be generated for the inquiry API, publish API, and both combined. To
do so, run wsdl2h on inquire_v2.wsdl and/or publish_v2.wsdl

COMPILATION

The build steps are generally as follows:

# Run wsdl2h with mapping specified in uddi2-typemap.dat on the UDDI WSDL(s):
$ wsdl2h -tuddi2-typemap.dat -ouddi_v2.h inquire_v2.wsdl publish_v2.wsdl

# Run soapcpp2 on the generated .h file (-puddi saves sources as uddiXYZ):
$ soapcpp2 -I.. -puddi uddi_v2.h

# Complete the build:
$ g++ -DWITH_NONAMESPACES -I.. -o main main.cpp inquire_v2.cpp publish_v2.cpp uddiC.cpp uddiClient.cpp ../stdsoap2.cpp

# To avoid link errors combining multiple services, replace the last step with:
$ echo '' > env.h
$ soapcpp2 -I.. -penv env.h
$ g++ -DWITH_NONAMESPACES -I.. -o main main.cpp inquire_v2.cpp publish_v2.cpp uddiClientLib.cpp ../stdsoap2.cpp

USING DOXYGEN

To generate documentation, use Doxygen on the generated header file, e.g.
uddi_v2.h. Part of the documentation is included in the uddi2-typemap.dat
file, which was used to generate the header file(s) with wsdl2h.
