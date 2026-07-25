
How container templates are serialized
======================================

In the gSOAP header file (the file processed by soapcpp2) a container template
can be declared as follows:

    template <class T> class mycontainer;

and mycontainer can be used as needed in the same gSOAP header file to
serialize mycontainers of values in XML similar to std::vector:

    mycontainer<int> ...
    mycontainer<someclass> ... etc

Compare this to:

    std::vector<int>

Note that the template class code itself should NOT be defined anywhere in the
header file, but separately in a header file.  You can include this header file
with (not import):

    #include "mycontainer.h"   // do not use #import: need mycontainer.h later

That is, The "real" container template must be defined in a regular header file
and implemented. For XML serialization to work with the template, the
template<typename T> class C container must define at least the following
public members:

    void              C::clear()
    C::iterator       C::begin()
    C::const_iterator C::begin() const
    C::iterator       C::end()
    C::const_iterator C::end() const
    size_t            C::size() const
    C::iterator       C::insert(C::iterator pos, const T& val)

These members allow the gSOAP serializers to read and write the container
content from/to XML. Of course, how you define these is up to you. A container
can also store just one value, or you can use it to serialize trees and graphs,
or even produce and consume content dynamically. You could define the templates
as processors of (de)serialized XML content embedded in XML messages. Because
the serializers are invoked twice to send a message over HTTP (one pass to
compute the content length when chunking is off, then to send the HTTP body),
you MUST ensure that the template instance's iterator produces the same content
in the two-phase HTTP count/send (or use chunking).

Example
-------

The example code in this directory is kept small to highlight the basic
features. The possibilities are endless.

In order to use container templates, you need at least one file that defines
the container and its implementation. In the example code here, we defined the
container template in:

    simple_vector.h

The application is a prime sieve:

    primes.cpp

which computes primes and stores them in the container. The primes application
uses the 'primes' class defined in the gSOAP header file:

    primes.h

which is parsed by soapcpp2 to generate all the 'primes' class serialization
routines to dump the XML content of the container, i.e. the list of primes.

Note that the 'primes' class is derived from the soap struct, which means that
a soap struct is associated with the class to allow a serialization context to
be carried along with it. This also minimizes the need to write code to setup
the soap context for XML serialization etc.

