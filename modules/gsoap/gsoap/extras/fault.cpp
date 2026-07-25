/*
	Contributed by Allan Kelly, June 17, 2002
	Provides C++ alternatives for
	soap_print_fault and soap_print_fault_location functions
  	This contributed code is covered under the MPL 1.1 license

	Note: soap_stream_fault in stdsoap2.cpp provides similar functionality
*/

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_print_fault(struct soap *soap, std::ostream& msg)
{ if (soap->error)
  { if (!*soap_faultcode(soap))
      soap_set_fault(soap);
    if (!*soap_faultstring(soap))
      *soap_faultstring(soap) = "";
    msg << "SOAP FAULT: "
        << *soap_faultcode(soap) << std::endl
        << "\"" << *soap_faultstring(soap) << "\"" << std::endl;
    if (*soap_faultdetail(soap))
      msg << "Detail: " << *soap_faultdetail(soap) << std::endl;
  }
}

/******************************************************************************/

SOAP_FMAC1
void
SOAP_FMAC2
soap_print_fault_location(struct soap *soap, std::ostream& msg)
{ int c;
  if (soap->error && soap->buflen > 0)
  { if (soap->bufidx == 0)
      soap->bufidx = 1;
    c = soap->buf[soap->bufidx-1];
    soap->buf[soap->bufidx-1] = '\0';
    if (soap->bufidx < soap->buflen)
      msg << soap->buf << static_cast<char> (c) << std::endl
          << "** HERE **" << std::endl << soap->buf+soap->bufidx
          << std::endl;
    else
      msg << soap->buf << static_cast<char> (c) << std::endl
          << "** HERE **" << std::endl;
  }
}

/******************************************************************************/
