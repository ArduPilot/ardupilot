INSTRUCTIONS

Install the WCF samples by downloading the "Windows Communication Foundation
(WCF) and Windows Workflow Foundation (WF) Samples for .NET Framework 4." from
the Microsoft download site.

After downloading you should have:

  C:\WF_WCF_Samples\WCF\Basic\Binding\Basic\Http\CS

Open the Http.sln solution in that folder in Visual Studio.  This shows a Basic
Http WCF C# example client and service project.  If the Solution Explorer shows
"Project unloaded" then right-click the client project name in the Solution
Explorer and select "Reload project".  Do the same for the service.

Modify the configuration and source code as described below.

Notes:

  C:> denotes the Windows command line

  $ denotes the Unix/Linux command line

How to connect a WCF client to a gSOAP service
----------------------------------------------

In App.config of the example WCF client set the endpoint to the gSOAP server
host name or IP and port, which is the machine on which the gSOAP server runs,
say the server endpoint is "10.0.1.5" on port 8000:

<client>
  <endpoint address="http://10.0.1.5:8000" ... />

If you run the server on the same machine as the client, then you can use:

<client>
  <endpoint address="http://localhost:8000" ... />

Compile the gSOAP server with 'make' and run the gSOAP server on port 8000:

  $ make
  $ ./calculator 8000

Under Project Properties of the example WCF client, change the Output type to
Console Application.  Then After compiling the WCF client in Visual Studio, run
the WCF client:

  C:> cd WF_WCF_Samples\WCF\Basic\Binding\Basic\Http\CS\client\bin
  C:> client.exe

If the gSOAP server runs remotely and the client cannot connect to the machine
on which the server runs then check your firewall settings on both machines to
permit TCP connections on the server port (8000 for example).

How to self-host a WCF service
------------------------------

First obtain the machine host name or IP with ipconfig:

  C:> ipconfig /all

Say the output of this command shows us that the machine IP is 10.0.1.5.

Add a Program class with Main() to the WCF service example service.cs file to
self host a service with the following Program class put in the namespace:

using System;
using System.ServiceModel;
using System.Configuration;
using System.ServiceModel.Description;

namespace ...
{
  ...
  public class Program
  {
    public static void Main()
    {
      Uri httpUrl = new Uri("http://10.0.1.5:8000/ServiceModelSamples/service");
      using (ServiceHost serviceHost = new ServiceHost(typeof(CalculatorService), httpUrl))
      {
        BasicHttpBinding bhb = new BasicHttpBinding();
        serviceHost.AddServiceEndpoint(typeof(ICalculator), bhb, "");
  
        ServiceMetadataBehavior smb = new ServiceMetadataBehavior();
        smb.HttpGetEnabled = true;
        serviceHost.Description.Behaviors.Add(smb);
  
        serviceHost.Open();

        Console.WriteLine("Press <ENTER> to terminate service.");
        Console.ReadLine();
      }
    }
  }

Under Project Properties of the example WCF service, change the Output type to
Console Application.  Then rebuild the project to generate service.exe.

After compiling, run service.exe from the command prompt:

  C:\WF_WCF_Samples\WCF\Basic\Binding\Basic\Http\CS\service\bin\service.exe

Running this command on the command prompt may require administrator
privileges.  To do so, right click the Command Prompt application icon on your
desktop to select Run as Administrator.

WCF self-hosting settings must be configured for HTTP, see the Microsoft
documentation on Configuring HTTP and HTTPS for WCF.  For example, on Windows
7, 8 and 10 you need to add a URL reservation for the specified URL namespace
for your account:

  C:> netsh http add urlacl url=http://+:8000/MyUri user=YourUserName

Use a web browser to access the service at the endpoint URL
10.0.1.5:8000/ServiceModelSamples/service (replace 10.0.1.5 with the IP address
returned by ipconfig):

  http://10.0.1.5:8000/ServiceModelSamples/service

Use a browser to view the WSDL at the endpoint URL

  http://10.0.1.5:8000/ServiceModelSamples/service?wsdl

Using a browser to view the service from a remote machine is a good way to
check if the remote connection works.  If it fails, then check your firewall
settings on the machine on which the self-hosted service runs to allow
connections on port 8000.

To allow connections on port 8000 add a firewall rule:

  C:> netsh advfirewall firewall add rule name="WCF service example" dir=in action=allow protocol=TCP localport=8000

To remove the firewall rule later:

  C:> netsh advfirewall firewall delete rule name="WCF service example" dir=in action=allow protocol=TCP localport=8000

Once the service runs, you can now use the gSOAP wsdl2h tool to generate C++
source code to develop a client application:

   $ wsdl2h -t ../../../../typemap.dat -o calculator.h 'http://10.0.1.5:8000/ServiceModelSamples/service?wsdl'

This may take some time, since the self-hosted service is only an iterative web
server that allows only one open connection at a time, while multiple
connections are opened to access all files (WSDL and XSD files).  To avoid
this hassle, use:

   $ wsdl2h -t ../../../../typemap.dat -o calculator.h 'http://10.0.1.5:8000/ServiceModelSamples/service?singleWsdl'

A pre-generated calculator.h file is included in the build directory.  The
calculator.h file contains the service definitions and XML data bindings for
C++ (or C with wsdl2h option -c).  You can find the typemap.dat file in the
gsoap root directory.  This file binds XML namespace prefixes such as mssamh to
XML namespace URIs.  We use this prefix in our source code.

The next step is to run soapcpp2 on the calculator.h file to generate the
binding implementation source code:

   $ soapcpp2 -j calculator.h

This generates a BasicHttpBinding_USCOREICalculatorProxy class to invoke the
service.

The gSOAP client application uses this class as follows:

#include "soapBasicHttpBinding_USCOREICalculatorProxy.h"
#include "BasicHttpBinding_USCOREICalculator.nsmap"

const char *URI = "http://10.0.1.5:8000"; // the service URI

BasicHttpBinding_USCOREICalculatorProxy proxy(URI, SOAP_XML_INDENT);

proxy.soap->send_timeout = proxy.soap->recv_timeout = 10; // 10 sec timeouts

double n1 = 3.14, n2 = 1.41;

_mssamh__Add areq;
_mssamh__AddResponse ares;
areq.n1 = &n1;
areq.n2 = &n2;
if (proxy.Add(&areq, &ares) == SOAP_OK && ares.AddResult)
  std::cout << "Add(" << n1 << ", " << n2 << ") = " << *ares.AddResult << std::endl;
else
  proxy.soap_stream_fault(std::cerr);
proxy.destroy();

