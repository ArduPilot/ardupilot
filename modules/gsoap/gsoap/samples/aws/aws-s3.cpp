/*
	aws-s3.cpp

	Example AWS S3 service invocation

	Expects access key and signature to be provided as arguments

	Only one service operation is shown to demonstrate the API. Add AWS S3
	methods as needed:

        http://docs.aws.amazon.com/AmazonS3/latest/API/APISoap.html

	http://www.genivia.com/examples/aws

	HTTPS is required.

	Build steps:
	$ wsdl2h -t typemap.dat -o aws-s3.h http://doc.s3.amazonaws.com/2006-03-01/AmazonS3.wsdl
	$ soapcpp2 -C -j -r aws-s3.h
	$ c++ -DWITH_OPENSSL -o aws-s3 aws-s3.cpp soapAmazonS3SoapBindingProxy.cpp soapC.cpp stdsoap2.cpp -lssl -lcrypto

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "soapAmazonS3SoapBindingProxy.h"
#include "AmazonS3SoapBinding.nsmap"

// Make allocation and assigment of primitive values quick and easy:
template<class T>
T * soap_make(struct soap *soap, T val)
{
  T *p = (T*)soap_malloc(soap, sizeof(T));
  *p = val;
  return p;
}

// Make allocation and assignment of std::string quick and easy:
std::string * soap_make_string(struct soap *soap, const char *s)
{
  std::string *p = soap_new_std__string(soap);
  *p = s;
  return p;
}

int main(int argc, char **argv)
{
  // Create a proxy to invoke AWS S3 services
  AmazonS3SoapBindingProxy aws(SOAP_XML_INDENT);

  // Set the argument of the ListAllMyBuckets service operation
  _s3__ListAllMyBuckets arg;
  arg.AWSAccessKeyId = soap_make_string(aws.soap, argc > 1 ? argv[1] : "..."); // use your access key
  arg.Timestamp      = soap_make(aws.soap, time(0));
  arg.Signature      = soap_make_string(aws.soap, argc > 2 ? argv[2] : "..."); // use your signature

  // Store the result of the service
  _s3__ListAllMyBucketsResponse response;

  // Get list of my buckets
  if (aws.ListAllMyBuckets(&arg, response))
    aws.soap_stream_fault(std::cerr);
  else if (response.ListAllMyBucketsResponse)
  {
    s3__ListAllMyBucketsResult &result = *response.ListAllMyBucketsResponse;
    s3__CanonicalUser *owner = result.Owner;
    if (owner)
      std::cout << "ID = " << owner->ID << std::endl;
    if (owner && owner->DisplayName)
      std::cout << "DisplayName = " << *owner->DisplayName << std::endl;
    s3__ListAllMyBucketsList *buckets = result.Buckets;
    if (buckets)
    {
      for (std::vector<s3__ListAllMyBucketsEntry*>::const_iterator it = buckets->Bucket.begin(); it != buckets->Bucket.end(); ++it)
      {
	s3__ListAllMyBucketsEntry *entry = *it;
	if (entry)
	  std::cout << "Name = " << entry->Name << " created " << soap_dateTime2s(aws.soap, entry->CreationDate) << std::endl;
      }
    }
  }

  // Delete all managed data
  aws.destroy();

  return 0;
}
