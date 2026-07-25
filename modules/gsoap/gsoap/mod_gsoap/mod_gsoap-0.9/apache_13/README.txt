Apache 1.3 modules for gSOAP

The Apache modules for gSOAP are released under the gSOAP open source public
license and GPLv2.  The open source licensing is replaced by Genivia's license
for commercial use when a commercial-use license is purchased by customer.

This directory contains the sources for building the mod_gsoap Apache module
for Apache 1.3.

Here's the quick start way to build your own DSO.

./configure [--with-gsoap=/alternate/path/to/soapcpp2]
make

For more details you can see the apache_index.html, but it is not wholly up to
date.

Requirements:

apache compiled with --enable-module=so and installed
apxs in path and working
libtool installed.
