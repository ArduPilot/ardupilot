####### This is the input file for automake, which will generate Makefile.in ##########
## not a GNU package. You can remove this line, if
## you have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = subdir-objects foreign 1.4

noinst_PROGRAMS = aws
SOAPHEADER = aws-s3.h

include ../Makefile.defines
include ../Makefile.cpp_proxy_rules_j
SOAP_NS = AmazonS3SoapBinding.nsmap

aws_CPPFLAGS = $(WSDL2H_EXTRA_FLAGS)
aws_SOURCES = aws-s3.cpp $(SOAPHEADER) $(SOAP_CPP_CORE) soapAmazonS3SoapBindingProxy.cpp
aws_LDADD = $(SOAP_CPP_LIB_SSL) $(SAMPLE_EXTRA_LIBS) $(SAMPLE_SSL_LIBS)
