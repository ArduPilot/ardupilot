INSTRUCTIONS

Install the WCF samples by downloading the "Windows Communication Foundation
(WCF) and Windows Workflow Foundation (WF) Samples for .NET Framework 4." from
the Microsoft download site (you may have to search).

You should have:

  C:\WF_WCF_Samples\WCF\Basic\Binding\Basic\MessageSecurity\CS

Open the Basic MessageSecurity WCF C# example client and service project in
Visual Studio. Modify the configuration and source code as described below.

Notes:

  C:> denotes the Windows command line

  $ denotes the Unix/Linux command line

To enable gSOAP to use WS-Security for BasicMessageSecurity interop, add to the
.h file to be processed by soapcpp2:

  #import "wsse.h"

See doc/wsse for instructions.

Generating keys and certificates
--------------------------------

First create the client key and certificate:

  C:> makecert -ss My -n "CN=client.com" -sk myKey -pe

Open the MMC console

  C:> mmc

Add the certificates snap-in to MMC if not already added, by pressing CTRL-M.

In Console Root/Certificates: select under Personal/Certificates "client.com"

Export "client.com" by right-clicking, All Tasks/Export...

Export "client.com" as "client.pfx", make sure you export the private key.

Copy client.pfx to the other machine for conversion to PEM format.

To create a PEM-formatted private key:

  $ openssl pkcs12 -in client.pfx -nocerts -out clientWCF.pem
  $ openssl rsa -in clientWCF.pem -text -noout

To create a PEM-formatted certificate:

  $ openssl pkcs12 -in client.pfx -clcerts -nokeys -out clientcertWCF.pem
  $ openssl x509 -in clientcertWCF.pem -text -noout

Obtain the host name of the machine on which the service runs, say we use
domain "server.com".

Generate the server certificate for this domain:

  C:> makecert -ss TrustedPeople -n "CN=server.com" -sky exchange -pe

Export the certificate under MMC Trusted People/Certificates to pfx format as
described above, and convert the key and certificate to PEM format:

  $ openssl pkcs12 -in server.pfx -nocerts -out serverWCF.pem
  $ openssl rsa -in serverWCF.pem -text -noout

  $ openssl pkcs12 -in server.pfx -clcerts -nokeys -out servercertWCF.pem
  $ openssl x509 -in servercertWCF.pem -text -noout

To connect a WCF client to a gSOAP service
------------------------------------------

In App.config of the WCF client, change the <endpoint>, <message>, and
<defaultCertificate> elements as follows:

  <client>
    <endpoint
      address="http://your_server_host:8000/ServiceModelSamples/service"
      behaviorConfiguration="..."
      binding="..."
      bindingConfiguration="..."
      contract="..."
      name="">
       <identity>
         <dns value="subject_name_of_your_server_certificate" />
       </identity>
    </endpoint>
    ...

  <bindings>
    <basicHttpBinding>
      <binding name="...">
        <security mode="Message">
          <message clientCredentialType="Certificate" algorithmSuite="Basic256Rsa15"/>

  <behaviors>
    <endpointBehaviors>
      <behavior name="...">
        <clientCredentials>
	   ...
	   <serviceCertificate>
	     <authentication certificateValidationMode="PeerOrChainTrust"/>
             <defaultCertificate
	       findValue="subject_name_of_your_server_certificate"
	       storeLocation="CurrentUser"
	       storeName="My"
	       x509FindType="FindBySubjectName" />

To connect a gSOAP client to a WCF service
------------------------------------------

Follow instructions to set up a WCF service using the keys and certificates,
where the server certificate should use the machine's host name or IP.

See gsoap\samples\wcf\Basic\Http\README.txt for help.

In Web.config (or App.config) of the service, change the <message> element:

  <bindings>
    <basicHttpBinding>
      <binding name="...">
        <security mode="Message">
          <message
	    clientCredentialType="Certificate"
	    algorithmSuite="Basic256Rsa15" />

Supported algorthm suites
-------------------------

All Basic suites with Rsa15 for keywrap. The rsa-oaep-mgf1p keywrap algorithm
is currently not supported, but possibly will be supported in future releases.

WS-Security with gSOAP
----------------------

You can use the following flags with the gSOAP client/server initialization using soap_new1() and soap_init1():

- SOAP_XML_CANONICAL is recommended to enable C14N
- SOAP_IO_CHUNK enables HTTP chunking, which is faster
- SOAP_ENC_GZIP enables HTTP compression

Do NOT use SOAP_XML_INDENT because interoperability with WCF WS-Security is not
guaranteed when SOAP_XML_INDENT is enabled.  The implementation of C14N in WCF
with respect to the normalization of white space between XML tags differs from
the protocol standards.

Build steps for gSOAP client/server calculator example
------------------------------------------------------

The calculator.h file with the service definitions and data binding interface
as pre-generated from the WSDL published by the WCF service as follows:

  $ wsdl2h -t ../../../../typemap.dat -o calculator.h \
    'http://your_server_host:8000/ServiceModelSamples/service?wsdl'

The soapcpp2 tool generates the data binding code and client/server classes:

  $ soapcpp2 -a -j calculator.h

Make sure that calculator.cpp connects to the WCF service by setting:

  const char *endpoint = "http://your_server_host:8000/ServiceModelSamples/service";

Compile the project with:

  $ c++ -DWITH_DOM -DWITH_OPENSSL -I. -I../../../.. -I../../../../plugin \
    -o calculator \
    calculator.cpp soapC.cpp \
    soapBasicHttpBinding_USCOREICalculatorService.cpp \
    soapBasicHttpBinding_USCOREICalculatorProxy.cpp \
    ../../../../plugin/wsseapi.c \
    ../../../../plugin/smdevp.c \
    ../../../../plugin/mecevp.c

This compiles the client/server that uses WS-Security signature and encryption.

Important: the MSVC++ compilers require .cpp files instead of .c files.
Otherwise compilation and link errors are likely.  Please rename the .c files
to .cpp files listed above for your project.

Enable certificate verification in the gSOAP client/server example
------------------------------------------------------------------

To enable certificate verification, build with -DCERTVERIFY:

  $ c++ -DCERTVERIFY -DWITH_DOM -DWITH_OPENSSL ...

How to disable encryption of the signature
------------------------------------------

To disable encryption of the signature:

  $ c++ -DUNENCRYPTED_SIGNATURE -DWITH_DOM -DWITH_OPENSSL ...

WCF-defined default bindings such as BasicHttpBinding require the signature to
be signed when receiving messages. WCF will throw an error such as "The primary
signature must be encrypted" if the signature is not encrypted.

To let WCF accept a message in which the signature is not encypted, a custom
binding is required. To do so, in the .config file of the WCF server replace
the original <binding> tag with the configuration below:

  <bindings>
    <customBinding>
      <binding name="Binding1">
        <security
          defaultAlgorithmSuite="Basic256Rsa15"
          authenticationMode="MutualCertificate"
          messageProtectionOrder="SignBeforeEncrypt"
          messageSecurityVersion="WSSecurity10WSTrust13WSSecureConversation13WSSecurityPolicy12BasicSecurityProfile10"
          securityHeaderLayout="LaxTimestampFirst"
          allowSerializedSigningTokenOnReply="true"
          requireSecurityContextCancellation="true">
        </security>
        <textMessageEncoding messageVersion="Soap11"></textMessageEncoding>
        <httpTransport></httpTransport>
      </binding>
    </customBinding>
  </bindings>   

Modify the value of attribute 'binding' in <endpoint> of Binding1 from
BasicHttpBinding to customBinding. The rest is unchanged.

  <services>
    <service name="Microsoft.Samples.MessageSecurity.CalculatorService" behaviorConfiguration="CalculatorServiceBehavior">
      <host>
        <baseAddresses>
          ...
        </baseAddresses>
      </host>
      <endpoint address="" binding="customBinding" bindingConfiguration="Binding1" contract="Microsoft.Samples.MessageSecurity.ICalculator"/>
      <endpoint address="mex" binding="mexHttpBinding" contract="IMetadataExchange"/>
    </service>
  </services>

See also https://blogs.msdn.microsoft.com/distributedservices/2011/03/07/wcf-security-interop-scenarios/

How to disable encryption
-------------------------

To disable encryption from the calculator.cpp client/server, comment the line:

  // #define ENCRYPTION

To disable encryption in service.cs of the WCF service, change interface
ICalculator by adding protecting level attribute on service contract and
operation contract.  For example, the code bellow shows only enable signature
on Add operation, and the rest three operation enable both signature and
encryption:

  [ServiceContract(Namespace = "http://Microsoft.Samples.MessageSecurity", ProtectionLevel = ProtectionLevel.EncryptAndSign)]
  interface ICalculator
  {
    [OperationContract(ProtectionLevel = ProtectionLevel.Sign)]
      double Add(double n1, double n2);
    [OperationContract]
      double Subtract(double n1, double n2);
    [OperationContract]
      double Multiply(double n1, double n2);
    [OperationContract]
      double Divide(double n1, double n2);
  }
  
In generatedClient.cs of the WCF client, enable only signature on the Add
operation by making similar changes:

  [System.CodeDom.Compiler.GeneratedCodeAttribute("System.ServiceModel", "3.0.0.0")]
  [System.ServiceModel.ServiceContractAttribute(Namespace = "http://Microsoft.Samples.MessageSecurity", ConfigurationName = "Microsoft.Samples.MessageSecurity.ICalculator", ProtectionLevel = System.Net.Security.ProtectionLevel.EncryptAndSign)]
  public interface ICalculator
  {
    [System.ServiceModel.OperationContractAttribute(ProtectionLevel = System.Net.Security.ProtectionLevel.Sign, Action = "http://Microsoft.Samples.MessageSecurity/ICalculator/Add", ReplyAction = "http://Microsoft.Samples.MessageSecurity/ICalculator/AddResponse")]
      double Add(double n1, double n2);
    [System.ServiceModel.OperationContractAttribute(Action="http://Microsoft.Samples.MessageSecurity/ICalculator/Subtract", ReplyAction="http://Microsoft.Samples.MessageSecurity/ICalculator/SubtractResponse")]
      double Subtract(double n1, double n2);
    [System.ServiceModel.OperationContractAttribute(Action="http://Microsoft.Samples.MessageSecurity/ICalculator/Multiply", ReplyAction="http://Microsoft.Samples.MessageSecurity/ICalculator/MultiplyResponse")]
      double Multiply(double n1, double n2);
    [System.ServiceModel.OperationContractAttribute(Action="http://Microsoft.Samples.MessageSecurity/ICalculator/Divide", ReplyAction="http://Microsoft.Samples.MessageSecurity/ICalculator/DivideResponse")]
      double Divide(double n1, double n2);
  }

