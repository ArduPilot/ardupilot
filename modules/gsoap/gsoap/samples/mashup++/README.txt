
A simple example "mashup" service. The service computes the number of days to
christmas using the GMT time service and calculator service. 

The wsdl2h tool accepts multiple WSDLs and generates one gSOAP definitions file
to implement clients and/or services.  If necessary, use wsdl2h -Nname to
generate bindings for the separate WSDL services.

In this mashup example we create a new service to compute the days to christmas
by invoking the GMT and calc services within a service operation. A client for
the mashup service is also included.

The mashup.h gSOAP definitions file is generated from the GMT and calculator
WSDLs using:

wsdl2h -s -o mashup.h gmt.wsdl calc.wsdl mashup.wsdl

where gmt.wsdl is generated in samples/gmt, calc.wsdl is generated in
samples/calc, and mashup.wsdl is a new WSDL to expose the new "daystoxmas"
service operation.

The mashupserver is a CGI application, which can be easily changed into a
(multi-threaded) stand-alone Web server as per online tutorials and shown in
other examples.

The mashupclient is a stand-alone client for the service. Invoking mashupclient
will connect to the service, which returns the number of days to christmas.

