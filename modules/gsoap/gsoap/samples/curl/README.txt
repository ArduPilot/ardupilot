This directory contains example client applications with CURL.  The CURL plugin
for gSOAP provides the bridge to libcurl.  While gSOAP provides a full HTTP
stack, libcurl can be used to support additional protocols and features.

To use the CURL plugin with a C gSOAP client application generated with
soapcpp2 with option -c, do the following:

    #include "soapH.h"
    #include "ns.nsmap"
    #include "plugin/curlapi.h"

    const char server[] = "server endpoint URL";

    int main(int argc, char **argv)
    {
      struct soap *soap = soap_new1(SOAP_IO_CHUNK | SOAP_XML_INDENT);
      /* CURL global init */
      curl_global_init(CURL_GLOBAL_ALL);
      /* create CURL handle */
      curl = curl_easy_init();
      /* set options */
      curl_easy_setopt(data->curl, CURLOPT_CONNECTTIMEOUT, 60L);
      curl_easy_setopt(data->curl, CURLOPT_TIMEOUT, 10L);
      /* register plugin with CURL handle */
      soap_register_plugin_arg(soap, soap_curl, curl);
      /* make calls */
      if (soap_call_ns__some_method(soap, server, ...))
      {
        soap_print_fault(soap, stderr);
	/* reset after errors, to ensure gSOAP IO works */
        soap_curl_reset(soap);
      }
      else
      {
        printf("OK!\n");
      }
      /* clean up gSOAP */
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
      /* clean up CURL handle after deleting the soap context */
      curl_easy_cleanup(curl);
      /* CURL global cleanup */
      curl_global_cleanup();
      return 0;
    }

To use the CURL plugin with a C++ gSOAP client application that uses a proxy
class generated with soapcpp2 with option -j, do the following:

    #include "soapProxy.h"
    #include "ns.nsmap"
    #include "plugin/curlapi.h"

    const char server[] = "server endpoint URL";

    int main(int argc, char **argv)
    {
      Proxy proxy(server, SOAP_IO_CHUNK | SOAP_XML_INDENT);
      /* CURL global init */
      curl_global_init(CURL_GLOBAL_ALL);
      /* create CURL handle */
      curl = curl_easy_init();
      /* set options */
      curl_easy_setopt(data->curl, CURLOPT_CONNECTTIMEOUT, 60L);
      curl_easy_setopt(data->curl, CURLOPT_TIMEOUT, 10L);
      /* register plugin with CURL handle */
      soap_register_plugin_arg(proxy.soap, soap_curl, curl);
      /* make calls */
      if (proxy.call(...))
      {
        proxy.soap_stream_fault(std:err);
	/* reset after errors, to ensure gSOAP IO works */
        soap_curl_reset(proxy.soap);
      }
      else
      {
        printf("OK!\n");
      }
      /* clean up gSOAP */
      proxy.destroy();
      /* clean up CURL handle after deleting the soap context */
      curl_easy_cleanup(curl);
      /* CURL global cleanup */
      curl_global_cleanup();
      return 0;
    }

For more information about the CURL plugin, see gsoap/doc/curl/html/index.html
