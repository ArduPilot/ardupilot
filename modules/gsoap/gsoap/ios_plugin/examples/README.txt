
INSTRUCIONS
===========

To compile the examples, first copy the following files to the source code
files folders of each example (Air, app2, calc):

- gsoapios.h		(located under gsoap/ios_plugin/)
- gsoapios.mm		(located under gsoap/ios_plugin/)
- stdsoap2.h		(located under gsoap/)
- stdsoap2.cpp		(located under gsoap/)
- dom.cpp		(located under gsoap/)

Then generate updated files with soapcpp2 for each example:

    cd Air
    soapcpp2 -j -CL -I../../../import airport.h

    cd app2
    soapcpp2 -j -CL -I../../../import GeoIPService.h

    cd calc
    soapcpp2 -CL -I../../../import calc.h


DOCUMENTATION
=============

See http://www.genivia.com/doc/ios/index.html
