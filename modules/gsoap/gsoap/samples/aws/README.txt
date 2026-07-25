Example AWS S3 service invocation

Lists all S3 buckets of a user

Other service invocations can be easily added.  See this article for help:

http://www.codeproject.com/Articles/1108296/How-to-Use-Amazon-Simple-Storage-Service-S-in-Cplu

Build steps:

1. Make sure that typemap.dat has the following lines:

aws     = "urn:PI/DevCentral/SoapService"
s3      = "http://s3.amazonaws.com/doc/2006-03-01/"
_s3__CreateBucketResponse = $ s3__CreateBucketResult* CreateBucketResponse;
_s3__CopyObjectResponse   = $ s3__CopyObjectResult* CopyObjectResponse;

2. then execute:

$ wsdl2h -t typemap.dat -o aws-s3.h http://doc.s3.amazonaws.com/2006-03-01/AmazonS3.wsdl
$ soapcpp2 -C -j -r aws-s3.h
$ c++ -DWITH_OPENSSL -o aws-s3 aws-s3.cpp soapAmazonS3SoapBindingProxy.cpp soapC.cpp stdsoap2.cpp -lssl -lcrypto

A report is auto-generated in soapReadme.md

