@mainpage The iOS Plugin

@section title The iOS Plugin

[TOC]

By Bethany Sanders, Robert van Engelen, Ning Xie, and Wei Zhang

@section ios_overview Overview

Consuming Web services on iOS (iPhone and iPad) platforms is increasingly in
demand today.  Xcode does not offer built-in tools that make consuming XML
Web services easy.  This means that detailed knowledge of XML processing
is required to send SOAP/XML and XML REST requests to the Web services and
parse the XML response.  By contrast, the gSOAP toolkit offers an automated XML
data binding toolkit for C and C++ to develop SOAP/XML Web services and
clients.  You can also use the iOS plugin to consume XML-RPC and JSON Web
services with the
[XML-RPC & JSON/JSON-Path](https://www.genivia.com/doc/xml-rpc-json/html/index.html)
plugin for gSOAP.  The plugin makes it easy to consume Web services on iOS
platforms, such as iPhone and iPad.  Moreover, the plugin takes advantage of
iOS network connectivity by supporting 3G/4G/LTE and wifi.

To use the iOS plugin for development of client applications on iOS platforms,
register the plugin with the gSOAP engine context as follows:

@code
    #import "gsoapios.h"

    struct soap *soap = soap_new();       // new engine context
    soap_register_plugin(soap, soap_ios); // register the iOS plugin

    ...

    soap_destroy(soap); // clean up deserialized data
    soap_end(soap);     // clean up temporaries
    soap_free(soap);    // free the context
@endcode

When coding in C++, it is recommended to generate C++ proxy classes with soapcpp2 option -j which are then used to instantiate proxy objects that manage the gSOAP engine context to register with the plugin as follows:

@code
    #import "gsoapios.h"

    Proxy proxy;
    soap_register_plugin(proxy.soap, soap_ios);

    ...

    proxy.destroy();
@endcode

There are no other plugin API calls necessary to make a Web service client
application work with iOS.  This means that you can develop iOS apps that
consume complex SOAP/XML, XML REST, and JSON Web services.

@section ios_start Getting Started

To start building Web services client applications for iPhone and/or iPad with
gSOAP, you will need:

- The [gSOAP toolkit](http://www.genivia.com/Products/downloads.html) version
  2.8.50 or greater;

- Xcode with the iOS SDK installed.

Developing Web services client applications on iOS is no different than
developing these applications on another OS with gSOAP when you use this iOS
Plugin.

The steps to create a client application on iOS:

- Use wsdl2h and soapcpp2 to generate C++ code with client stubs or client
  proxy classes (soapcpp2 option -j) from WSDL and XSD files;

- Create an iOS application project in Xcode;

- Add stdsoap2.cpp and stdsoap2.h from the gSOAP package to your iOS project;

- Add gsoapios.h and gsoapios.mm that are part of the iOS plugin files to your
  iOS project;

- In your client code create an engine context and register the iOS plugin;

- In your client code add the Web service methods invocations that you want,
  see the wsdl2h-generated header file (first step above) with interface
  declarations for instructions on how to invoke services.

All of the source code files should be of type Objective C++ source.  Mixing
C files with C++ files will cause errors.  Rename .c files to .cpp files as
necessary.

@section ios_cache_policy Specifying the Cache Policy

The interaction between the client and the Web serviceserver can be controlled
by specifying the cache policy. To specify the cache policy, use the API
function `soap_ios_setcachepolicy(struct soap *soap, unsigned int policy)`.
This API function cannot be called before the plugin is registered.

@code
    #import "gsoapios.h"

    struct soap *soap = soap_new();
    soap_register_plugin(soap, soap_ios);

    // Specify the cache policy for the request
    soap_ios_setchacepolicy(soap, NSURLRequestReturnCacheDataElseLoad);

    ...

    soap_destroy(soap); // clean up deserialized data
    soap_end(soap);     // clean up temporaries
    soap_free(soap);    // free the context
@endcode

The available cache policies that can be specified are:

@code
    enum {
      NSURLRequestUseProtocolCachePolicy = 0,
      NSURLRequestReloadIgnoringLocalCacheData = 1,
      NSURLRequestReloadIgnoringCacheData = NSURLRequestReloadIgnoringLocalCacheData,
      NSURLRequestReturnCacheDataElseLoad = 2,
      NSURLRequestReturnCacheDataDontLoad = 3,
      NSURLRequestReloadIgnoringLocalAndRemoteCacheData =4,
      NSURLRequestReloadRevalidatingCacheData = 5
    }
@endcode

The default cache policy is `NSURLRequestUseProtocolCachePolicy`.

@section ios_timeout_interval Specifying a Timeout Interval

The timeout of a network connection can be specified using the API function
`soap_ios_settimeoutinterval(struct soap *soap, double seconds)`:

@code
    #import "gsoapios.h"

    struct soap *soap = soap_new();
    soap_register_plugin(soap, soap_ios);

    // Specify the timout interval as 30 seconds
    soap_ios_settimeoutinterval(soap, 30.0);

    ...

    soap_destroy(soap); // clean up deserialized data
    soap_end(soap);     // clean up temporaries
    soap_free(soap);    // free the context
@endcode

The default timeout is 60 seconds.

@section ios_http_auth HTTP Authentication

To support authentication when access is denied (HTTP 401 error) when the
client tries to connect, enable HTTP authentication as follows.

Basic authentication is simply enabled at the client-side by setting the
`soap.userid` and `soap.passwd` strings to a username and password,
respectively:

@code
    struct soap *soap = soap_new();
    soap->userid = "someone";
    soap->passwd = "somepass";
@endcode

When using a generated C++ proxy class:

@code
    Proxy proxy;
    porxy.soap->userid = "someone";
    porxy.soap->passwd = "somepass";
@endcode

Make sure to **never use Basic authentication without HTTPS**.  HTTPS must be
used to ensure that Basic authentication is secure.

@section ios_example Examples

This section introduces four examples to demonstrate the development of client
applications consuming Web services on iOS platforms, such as iPhone and iPad,
by using the gSOAP tools and the iOS plugin.

The first example @ref ios_example_calc is a basic calculator client app.  The
second example @ref ios_example_geoip is a web service that locates the country
of a certain IP Adress. The third example @ref ios_example_weather returns
weather results for well-known US cities, and the fourth example
@ref ios_example_air shows information on every airport within a given country.

We assume you already have had some experience developing applications for
iPhone and iPad using Xcode with iOS SDK installed.  Experience is not required
to read on, but helpful in case you get lost in the details of the Xcode iOS
SDK.

General recommendations:

- Rename .m files to .mm files;

- Generate a C++ proxy class using soapcpp2 -j, which produces the source code
  you will need to access the Web service;

- Add stdsoap2.cpp and stdsoap2.h from the gSOAP source code tree to your iOS
  project;

- Add gsoapios.h and gsoapios.mm that are part of the iOS plugin files to your
  iOS project;

- In your client code register the iOS plugin.

@subsection ios_example_calc Simple Calculator Example (C++)

This example shows you how to develop a client application in C++ using gSOAP
and the ios plugin, which consumes a simple calculator service on iOS using
gSOAP.  The simple calculator service was developed and deployed as a demo to
[get started with gSOAP](http://www.genivia.com/dev.html).  The gSOAP
Calculator Service provides several operations such as add, sub, mul, div etc.
In this example, we use the operation add as a demo. Other operations are
applied in a similar way. The wsdl file for this service can be obtained at the
following link:

    http://www.genivia.com/calc.wsdl

The Xcode project for this example can be found in gsoap/ios_plugin/examples/calc.

@subsubsection ios_calc_step_1 Step 1: Generating stubs for C++ API

The gsoap/ios_plugin/examples/calc directory already contains calc.h so you can skip
this step.

To generate codes for the calculator Web service, we first run the wsdl2h tool
from the command line on the URL of the WSDL and use option -o to specify the
output file (Alternatively, you can download the calc.wsdl and use the local
file instead of the URL):

    wsdl2h -o calc.h http://www.genivia.com/calc.wsdl

This generates the calc.h service definition header file with service operation
definitions and types for the operation's data.  By default, gSOAP assumes you
will use C++ with STL.

We have not yet generated the stubs for the API. To do so, run the soapcpp2
compiler:

    soapcpp2 -CL -I$GSOAP_HOME/import calc.h

Option -CL indicates client-side only files (soapcpp2 generates both client and
server stubs and skeletons by default). This generates a number of source files
for client application development. We do not have to generate a proxy class in
this example because it does not use a Proxy class.

@subsubsection ios_calc_step_2 Step 2: Creating Xcode project for iPhone

Launch Xcode, create a Single View-based Application project and name it Calc.
Open the ViewController.xib (main storyboard) file in the Interface Builder.
Double-click on the View item and populate it with the views listed below and
shown in Figure 1:

- Two Labels ("+" and "A Simple Calculator")
- Two Text Fields (operand1 and operand2)
- One Button ("Add" button)

@image html calc-view.png "Figure 1. View of the Calculator Web Service Client App"

In Xcode, edit the file ViewController.h to make it look like the
following:

@code
    // File: ViewController.h

    #import <UIKit/UIKit.h>

    @interface ViewController : UIViewController {
      UITextField *op1; // operand1
      UITextField *op2; // operand2
    }
    @property (nonatomic, retain) IBOutlet UITextField *op1; // this code is generated when you link the text fields
    @property (nonatomic, retain) IBOutlet UITextField *op2; // this code is generated when you link the text fields

    - (IBAction) buttonPressed;
    @end
@endcode

Link the op1 and op2 to the two Text Fields and delegate the button action to
method buttonPressed.

In Xcode, edit the file info.plist to make it look like the following:

    <?xml version="1.0" encoding="UTF-8"?>
    <!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
    <plist version="1.0">
      <dict>
        <key>NSAppTransportSecurity</key>
        <dict>
          <key>NSAllowsArbitraryLoads</key>
          <true/>
        </dict>
        <key>CFBundleDevelopmentRegion</key>
        <string>en</string>
        <key>CFBundleExecutable</key>
        <string>$(EXECUTABLE_NAME)</string>
        <key>CFBundleIdentifier</key>
        <string>$(PRODUCT_BUNDLE_IDENTIFIER)</string>
        <key>CFBundleInfoDictionaryVersion</key>
        <string>6.0</string>
        <key>CFBundleName</key>
        <string>$(PRODUCT_NAME)</string>
        <key>CFBundlePackageType</key>
        <string>APPL</string>
        <key>CFBundleShortVersionString</key>
        <string>1.0</string>
        <key>CFBundleSignature</key>
        <string>????</string>
        <key>CFBundleVersion</key>
        <string>1</string>
        <key>LSRequiresIPhoneOS</key>
        <true/>
        <key>UILaunchStoryboardName</key>
        <string>LaunchScreen</string>
        <key>UIMainStoryboardFile</key>
        <string>Main</string>
        <key>UIRequiredDeviceCapabilities</key>
        <array>
          <string>armv7</string>
        </array>
        <key>UISupportedInterfaceOrientations</key>
        <array>
          <string>UIInterfaceOrientationPortrait</string>
          <string>UIInterfaceOrientationLandscapeLeft</string>
          <string>UIInterfaceOrientationLandscapeRight</string>
        </array>
      </dict>
    </plist>

These changes ensure that you can access the web services despite the added
layer of protection (App Transport Security).

@subsubsection ios_calc_step_3 Step 3: Adding generated source stubs to the Xcode project

Add the source files soapC.cpp, soapClient.cpp, soapH.h, and soapStub.h
generated in Step 1 of this tutorial to the project. Also add files stdsoap2.h
and stdsoap2.cpp to the project from gSOAP package as well as the iOS plugin
files gsoapios.h and gsoapios.mm.

Once all of your files have been added to the project, ensure they are all of
type "Objective C++ Source".  This ensures that there will be no issues with
mixing Objective C and C++ code.

@subsubsection ios_calc_step_4 Step 4: Implementing the Logic by calling the soap service

Firstly, edit file main.mm to import the file calc.nsmap.  Link errors may
arise without importing this XML namespace mapping table.

@code
    // File: main.mm

    #import <UIKit/UIKit.h>
    #import "../../calc.nsmap"

    int main(int argc, char * argv[]) {
      @autoreleasepool {
        return UIApplicationMain(argc, argv, nil, NSStringFromClass([AppDelegate class]));
      }
    }
@endcode

Then, implement the source file ViewController.mm as the following:

@code
    #import "ViewController.h"
    #import "soapStub.h"
    #import "gsoapios.h"
    
    @implementation ViewController
    
    @synthesize op1; // Set or get operand1
    @synthesize op2; // Set or get operand2
    
    - (IBAction) buttonPressed {
      double x = [op1.text doubleValue];
      double y = [op2.text doubleValue];

      struct soap *soap = soap_new1(SOAP_IO_DEFAULT|SOAP_IO_KEEPALIVE|SOAP_XML_INDENT|SOAP_XML_STRICT);

      if (soap_register_plugin(soap, soap_ios) == SOAP_OK)
      {
        // Specify the timeout interval (optional) to 45 seconds instead of
        // the default 60 seconds
        soap_ios_settimeoutinterval(soap, 45.0);

        double result;

        // Call Web service operation add
        int status = soap_call_ns2__add(soap, NULL, NULL,x, y, result);

        soap_free_temp(soap); // Cleanup temporary resources

        // Check soap response status
        if (status == SOAP_OK)
        {
          NSString *resultString;
          NSString *titleString;

          resultString = [NSString stringWithFormat:@"%f",result];
          titleString = [NSString stringWithFormat:@"%f + %f =",x, y];

          // Show the result in an alert
          UIAlertController * alert = [UIAlertController
            alertControllerWithTitle:titleString
            message:resultString
            preferredStyle:UIAlertControllerStyleAlert];
          UIAlertAction* cancelButton = [UIAlertAction
            actionWithTitle:@"OK"
            style:UIAlertActionStyleDefault
            handler:^(UIAlertAction * action){}];

          [alert addAction: cancelButton];
          [self presentViewController:alert animated:YES completion:nil];
        }
        else
          soap_print_fault(soap,stderr); // Print soap error in console
      }
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
    }
    @end
@endcode

A screen snapshot of the client is shown in Figure 2.

@image html calc-result.png "Figure 2: Snapshot of the CalcViewService result"

@subsection ios_example_geoip GeoIPService Example (C++)

[GeoIPService](http://www.webservicex.net/geoipservice.asmx?op=GetGeoIP) is a
live SOAP Web service that enables you to look up countries by IP address or by
Context.

This example shows you how to develop a client application in C++ using gSOAP
and the ios plugin, which consumes the GeoIPService on iOS using gSOAP.  The
WSDL file for this service can be downloaded at the following link:

    http://www.webservicex.net/geoipservice.asmx?WSDL

It is crucial to follow these directions in order for your app to work:

- Rename the .m files to .mm files.  The iOS plugin implementation file
  gsoapios.m must be renamed to gsoapios.mm. The main.m must be renamed to
  main.mm.  Other files such as your ViewController must also be renamed with
  .mm.

- Generate C++ proxy using proper options with wsdl2h and soapcpp2.

- Use stdsoap2.cpp from the gSOAP package in your iOS project.

The Xcode project for this example can be found in gsoap/ios_plugin/examples/GeoIPService.

@subsubsection ios_geoip_step_1 Step 1: Generating stubs for C++ Proxy

To generate codes for the GeoIPService Web service, we first run the wsdl2h
tool from the command line on the URL of the WSDL and use option -o to specify
the output file (alternatively, you can download the GeoIPService.wsdl file and
use the local file instead of the URL):

    wsdl2h -o GeoIPService.h -Ngeoip 'http://www.webservicex.net/geoipservice.asmx?WSDL'

This generates the GeoIPService.h service interface header file with service
operation definitions and types for the operation's data.  By default, gSOAP
assumes you will use C++ with STL.

To generate the stubs for the C++ proxy classes, run the soapcpp2:

    soapcpp2 -j -CL -I$GSOAP_HOME/import GeoIPService.h

Option -j tells the compiler to generate the C++ proxy class and option -CL
indicates client-side only files (soapcpp2 generates both client and server
stubs and skeletons by default). Option -I is needed to import the stlvector.h
file from the import directory in the gSOAP package to support serialization of
STL vectors.

@subsubsection ios_geoip_step_2 Step 2: Creating an Xcode project for iPhone

Create a Single View-based Application project and name it GeoIPService.Open
the ViewController.xib (main storyboard) file in the Interface
Builder. Double-click on the View item and populate it with the views listed
below and shown in Figure 3:

- A Label ("Enter IP Address")
- A Text Field
- One Round Rect Button ("Find Country" button)

@image html geoip-view.png "Figure 3. View of the GeoIPService Client"

In Xcode, edit the file ViewController.h to make it look like the
following:

@code
    // File: ViewController.h

    #import <UIKit/UIKit.h>

    @interface ViewController : UIViewController {
      UITextField* IPAddress;
    }

    @property (strong, nonatomic) IBOutlet UITextField *IPAddress; // this code generated when you link the text
    //field to IPAddress

    - (IBAction) buttonPressed;
    @end
@endcode

Set the ipAddress outlet and the buttonFindCountry:(id)sender method to
delegate action of the button.

In Xcode, edit the file info.plist to make it look like the following:

    <?xml version="1.0" encoding="UTF-8"?>
    <!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
    <plist version="1.0">
      <dict>
        <key>NSAppTransportSecurity</key>
        <dict>
          <key>NSAllowsArbitraryLoads</key>
          <true/>
        </dict>
          <key>CFBundleDevelopmentRegion</key>
          <string>en</string>
          <key>CFBundleExecutable</key>
          <string>$(EXECUTABLE_NAME)</string>
          <key>CFBundleIdentifier</key>
          <string>$(PRODUCT_BUNDLE_IDENTIFIER)</string>
          <key>CFBundleInfoDictionaryVersion</key>
          <string>6.0</string>
          <key>CFBundleName</key>
          <string>$(PRODUCT_NAME)</string>
          <key>CFBundlePackageType</key>
          <string>APPL</string>
          <key>CFBundleShortVersionString</key>
          <string>1.0</string>
          <key>CFBundleSignature</key>
          <string>????</string>
          <key>CFBundleVersion</key>
          <string>1</string>
          <key>LSRequiresIPhoneOS</key>
          <true/>
          <key>UILaunchStoryboardName</key>
          <string>LaunchScreen</string>
          <key>UIMainStoryboardFile</key>
          <string>Main</string>
          <key>UIRequiredDeviceCapabilities</key>
          <array>
            <string>armv7</string>
          </array>
          <key>UISupportedInterfaceOrientations</key>
          <array>
            <string>UIInterfaceOrientationPortrait</string>
            <string>UIInterfaceOrientationLandscapeLeft</string>
            <string>UIInterfaceOrientationLandscapeRight</string>
          </array>
      </dict>
    </plist>

These changes ensure that you can access the web services despite the added
layer of protection of App Transport Security).

@subsubsection ios_geoip_step_3 Step 3: Adding generated source stubs to the Xcode project

Add the source files soapC.cpp, soapGeoIPServiceSoapProxy.cpp,
soapGeoIPServiceSoapProxy.h, soapH.h, and soapStub.h generated in Step 1 of this
tutorial to the project. Also add files stdsoap2.h and stdsoap2.cpp to the
project from the gSOAP package and the iOS plugin files gsoapios.h and
gsoapios.mm.

Once all of your files have been added to the project, ensure they are all of
type "Objective C++ Source".  This ensures that there will be no issues with
mixing Objective C and C++ code.

@subsubsection ios_geoip_step_4 Step 4: Implementing the Logic by calling the soap service

Firstly, edit file main.mm to import the file GeoIPService.nsmap. Linking
errors would arise without importing this XML namespace mapping table.

@code
    // File: main.mm

    #import <UIKit/UIKit.h>
    #import "AppDelegate.h"
    #include "../../GeoIPServiceSoap.nsmap"

    int main(int argc, char * argv[]) {
      @autoreleasepool {
        return UIApplicationMain(argc, argv, nil, NSStringFromClass([AppDelegate class]));
      }
    }
@endcode

Then, implement the source file ViewController.mm as the following:

@code
    #import "ViewController.h"
    #include "soapGeoIPServiceSoapProxy.h"
    #import "soapStub.h"
    #import "gsoapios.h"
    using namespace std;

    typedef struct _ns1__GetGeoIP RequestStruct;
    typedef struct _ns1__GetGeoIPResponse ResponseStruct;

    @implementation ViewController

    @synthesize IPAddress;

    - (IBAction)buttonPressed:(id)sender {
      RequestStruct ip;
      ResponseStruct response;

      //creates proxy
      GeoIPServiceSoapProxy service(SOAP_IO_DEFAULT|SOAP_IO_KEEPALIVE|SOAP_XML_INDENT|SOAP_XML_STRICT);
      soap_init(service.soap);

      //sets IPAddress from input
      std::string ipAdd = std::string((char *)[IPAddress.text UTF8String]);
      ip.IPAddress = &ipAdd;

      // ----- register plugin for callbacks ------
      soap_register_plugin(service.soap, soap_ios);

      // Optional: timeout internal, the default is 60.0 seconds
      soap_ios_settimeoutinterval(service.soap, 30.0);

      //call web service
      int status = service.GetGeoIP(&ip, response);

      string* result;
      string err_msg = "Invalid IP address";

      if (status == SOAP_OK)
      {
        NSString *soapResult;
        NSString *titleMsg;

        if (response.GetGeoIPResult && response.GetGeoIPResult->CountryName)
          result = response.GetGeoIPResult->CountryName;
        else
          result = &err_msg;

        soapResult = [NSString stringWithUTF8String:result->c_str()];
        titleMsg = [NSString stringWithFormat: @"%@", @"Country Found"];

        //show result as an alert
        UIAlertController * alert = [UIAlertController
          alertControllerWithTitle:titleMsg
          message:soapResult
          preferredStyle:UIAlertControllerStyleAlert];
        UIAlertAction* cancelButton = [UIAlertAction
          actionWithTitle:@"OK"
          style:UIAlertActionStyleDefault
          handler:^(UIAlertAction * action){}];

        [alert addAction: cancelButton];
        [self presentViewController:alert animated:YES completion:nil];

      }
      else
        service.soap_stream_fault(std::cerr);
      service.destroy();
    }

    // Override to allow orientations other than the default portrait orientation.
    - (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation {
      // Return YES for supported orientations
      return (interfaceOrientation == UIInterfaceOrientationPortrait ||
          interfaceOrientation == UIInterfaceOrientationPortraitUpsideDown ||
          interfaceOrientation == UIInterfaceOrientationLandscapeLeft ||
          interfaceOrientation == UIInterfaceOrientationLandscapeRight);
    }

    - (void)didReceiveMemoryWarning {
      // Releases the view if it doesn't have a superview.
      [super didReceiveMemoryWarning];

      // Release any cached data, images, etc that aren't in use.
    }

    - (void)viewDidUnload {
      // Release any retained subviews of the main view.
      // e.g. self.myOutlet = nil;
    }

    - (void)dealloc {
      //    [ipAddress release];
      //    [super dealloc];
    }
    @end
@endcode

An image of the app is shown in Figure 4.

@image html geoip-result.png "Figure 4: Snapshot of the GeoIPServiceViewService result"

@subsection ios_example_weather Weather Example (C++)

GlobalWeather is a live SOAP Web service that enables you to look up the
Weather in popular cities around the world. For simplicity, this example is
limited to cities in the United States.  For some reason, some very popular
cities are not supported by this web service. For example, no results for New
York City will be returned. There is an error message within the app that shows
whenever one of these cities is entered. Know that the error message is not an
issue with the app you just built, but something that the web service itself
does not provide.

This example shows how to develop a client application in C++ using gSOAP
and the ios plugin, which consumes the Weather service on iOS using gSOAP.
The WSDL file for this service can be downloaded at the following link:

    http://www.webservicex.net/globalweather.asmx?WSDL

It is crucial to follow these directions in order for your app to work:

- Rename the .m files to .mm files.  The iOS plugin implementation file
  gsoapios.m must be renamed to gsoapios.mm. The main.m must be renamed to
  main.mm.  Other files such as your ViewController must also be renamed with
  .mm.

- Generate C++ proxy using proper options with wsdl2h and soapcpp2.

- Use stdsoap2.cpp from the gSOAP package in your iOS project.

The Xcode project for this example can be found in gsoap/ios_plugin/examples/Weather.

@subsubsection ios_weather_step_1 Step 1: Generating stubs for C++ Proxy

To generate codes for the GeoIPService Web service, we first run the wsdl2h
tool from the command line on the URL of the WSDL and use option -o to specify
the output file (Alternatively, you can download the GlobalWeather.wsdl file
and use the local file instead of the URL):

    wsdl2h -o weather.h 'http://www.webservicex.net/globalweather.asmx?WSDL'

This generates the weather.h service definition header file with service
operation definitions and types for the operation's data.  By default, gSOAP
assumes you will use C++ with STL.

Because we will be using the gSOAP DOM parser in this example, which is created
in our program with `xsd__anyType dom(ctx)` to extract plain XML content
received in string `xmlmessage` with `xmlmessage >> dom`, an additional step is
necessary.  Add the following line to the wsdl2h-generated airport.h file:

    #import "dom.h"

To generate the stubs for the C++ proxy classes, run the soapcpp2 tool:

    soapcpp2 -j -CL -I$GSOAP_HOME/import weather.h

Option -j tells soapcpp2 to generate the C++ proxy class and option -CL
indicates client-side only files (soapcpp2 generates both client and server
stubs and skeletons by default). Option -I is needed to import the
stlvector.h file from the import directory in the gSOAP package to support
serialization of STL vectors.

@subsubsection ios_weather_step_2 Step 2: Creating an Xcode project for iPhone

Create a Single View-based Application project and name it Weather. Open the
ViewController.xib(main storyboard) file in the Interface Builder.Double-click
on the View item and populate it with the views listed below and shown in
Figure 3:

- A Label ("Enter US City")
- A Text Field
- One Round Rect Button ("Get Weather Results" button)

In Xcode, edit the file ViewController.h to make it look like the following:

@code
    #import <UIKit/UIKit.h>

    @interface ViewController : UIViewController {
        UITextField* inputcity;
    }
    @property (strong, nonatomic) IBOutlet UITextField *inputcity;

    - (IBAction) buttonPressed;
    @end
@endcode

Set the inputcity outlet and the buttonFindCountry:(id)sender method to
delegate action of the button.

In Xcode, edit the file info.plist to make it look like the following:

     <?xml version="1.0" encoding="UTF-8"?>
     <!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
     <plist version="1.0">
       <dict>
         <key>NSAppTransportSecurity</key>
         <dict>
           <key>NSAllowsArbitraryLoads</key>
           <true/>
         </dict>
           <key>CFBundleDevelopmentRegion</key>
           <string>en</string>
           <key>CFBundleExecutable</key>
           <string>$(EXECUTABLE_NAME)</string>
           <key>CFBundleIdentifier</key>
           <string>$(PRODUCT_BUNDLE_IDENTIFIER)</string>
           <key>CFBundleInfoDictionaryVersion</key>
           <string>6.0</string>
           <key>CFBundleName</key>
           <string>$(PRODUCT_NAME)</string>
           <key>CFBundlePackageType</key>
           <string>APPL</string>
           <key>CFBundleShortVersionString</key>
           <string>1.0</string>
           <key>CFBundleSignature</key>
           <string>????</string>
           <key>CFBundleVersion</key>
           <string>1</string>
           <key>LSRequiresIPhoneOS</key>
           <true/>
           <key>UILaunchStoryboardName</key>
           <string>LaunchScreen</string>
           <key>UIMainStoryboardFile</key>
           <string>Main</string>
           <key>UIRequiredDeviceCapabilities</key>
           <array>
             <string>armv7</string>
           </array>
           <key>UISupportedInterfaceOrientations</key>
           <array>
             <string>UIInterfaceOrientationPortrait</string>
             <string>UIInterfaceOrientationLandscapeLeft</string>
             <string>UIInterfaceOrientationLandscapeRight</string>
           </array>
      </dict>
    </plist>

These changes ensure that you can access the web services despite the added
layer of protection of App Transport Security).

@subsubsection ios_weather_step_3 Step 3: Adding generated source stubs to the Xcode project

Add the source files soapC.cpp, soapGloabalWeatherSoapProxy.cpp,
soapGloabalWeatherSoapProxy.h, soapH.h, and soapStub.h generated in Step 1 of
this tutorial to the project. Also add files stdsoap2.h and stdsoap2.cpp to the
project from gsoap package and the iOS plugin files gsoapios.h and gsoapios.mm.

Once all of your files have been added to the project, ensure they are all of
type "Objective C++ Source".  This ensures that there will be no issues with
mixing Objective C and C++ code.

@subsubsection ios_weather_step_4 Step 4: Implementing the Logic by calling the soap service

Firstly, edit file main.mm to import the file GlobalWeatherSoap.nsmap. Linking
errors would arise without importing this XML namespace mapping table.

@code
    // File: main.mm

    #import <UIKit/UIKit.h>
    #import "AppDelegate.h"
    #include "../../GlobalWeatherSoap.nsmap"

    int main(int argc, char * argv[]) {
       @autoreleasepool {
         return UIApplicationMain(argc, argv, nil, NSStringFromClass([AppDelegate class]));
       }
    }
@endcode

Then, implement the source file ViewController.mm as the following, where we
used the [gSOAP domcpp](http://www.genivia.com/doc/dom/html) tool with option
-i on an example weather XML data file to produce DOM code to extract the XML
data (see further below):

@code
    #import "ViewController.h"
    #include "soapGlobalWeatherSoapProxy.h"
    #import "soapStub.h"
    #import "gsoapios.h"
    #include <fstream>

    typedef struct _ns__GetWeather RequestStruct;
    typedef struct _ns__GetWeatherResponse ResponseStruct;

    @implementation ViewController
    @synthesize inputcity;

    - (IBAction)buttonPressed:(id)sender {
      RequestStruct sending;
      ResponseStruct receiving;
      std::string result = "";

      GlobalWeatherSoapProxy service;
      soap_init(service.soap);

      // ----- register plugin for callbacks ------
      soap_register_plugin(service.soap, soap_ios);

      std::string cname= std::string((char *)[inputcity.text UTF8String]);
      sending.CityName = &cname;
      std::string USA = "United States";
      sending.CountryName = &USA;

      if (service.GetWeather(&sending, receiving) == SOAP_OK)
      {
        std::ofstream xmlwrite("/Users/bethanysanders/Documents/work/Weather/Weather/message.txt");
        std::ifstream xmlread("/Users/bethanysanders/Documents/work/Weather/Weather/message.txt");

        if(xmlwrite.is_open())
        {
          xmlwrite << *(receiving.GetWeatherResult);
          xmlwrite.close();

          struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_DOM_TREE);
          ctx->double_format = "%lG";

          xsd__anyType dom(ctx);
          if (xmlread.is_open())
            xmlread >> dom;
          if (dom.soap->error)
            result = "Error: Try a different city";

          xsd__anyAttribute *att;
          xsd__anyType *elt;

          if ((elt = dom.elt_get("Wind")))
          {
            xsd__anyType& dom_Wind = *elt;
            if (dom_Wind.get_text())
            {
              result += "Wind = ";
              result += dom_Wind.get_text();
            }
          }
          if ((elt = dom.elt_get("Visibility")))
          {
            xsd__anyType& dom_Visibility = *elt;
            if(dom_Visibility.get_text())
            {
              result += "\nVisibility = ";
              result += dom_Visibility.get_text();
            }

          }
          if ((elt = dom.elt_get("SkyConditions")))
          {
            xsd__anyType& dom_SkyConditions = *elt;
            if (dom_SkyConditions.get_text())
            {
              result += "\nSky Conditions = ";
              result += dom_SkyConditions.get_text();
            }
          }
          if ((elt = dom.elt_get("Temperature")))
          {
            xsd__anyType& dom_Temperature = *elt;
            if (dom_Temperature.get_text())
            {
              result += "\nTemperature = ";
              result += dom_Temperature.get_text();
            }
          }
          if ((elt = dom.elt_get("DewPoint")))
          {
            xsd__anyType& dom_DewPoint = *elt;
            if (dom_DewPoint.get_text())
            {
              result += "\nDew Point = ";
              result += dom_DewPoint.get_text();
            }
          }
          if ((elt = dom.elt_get("RelativeHumidity")))
          {
            xsd__anyType& dom_RelativeHumidity = *elt;
            if (dom_RelativeHumidity.get_text())
            {
              result += "\nRelative Humidity = ";
              result += dom_RelativeHumidity.get_text();
            }
          }
          if ((elt = dom.elt_get("Pressure")))
          {
            xsd__anyType& dom_Pressure = *elt;
            if (dom_Pressure.get_text())
            {
              result += "\nPressure = ";
              result += dom_Pressure.get_text();
            }
          }

          xmlread.close();

          soap_destroy(ctx); // delete objects
          soap_end(ctx);     // delete DOM data
          soap_free(ctx);    // free context
        }
      }

      NSString *titleMsg;
      NSString *Weather;
      titleMsg = [NSString stringWithFormat: @"%@", @"Weather Results"];
      Weather = [NSString stringWithUTF8String:result.c_str()];

      //show result as an alert
      UIAlertController * alert = [UIAlertController
        alertControllerWithTitle:titleMsg
        message:Weather
        preferredStyle:UIAlertControllerStyleAlert];
      UIAlertAction* cancelButton = [UIAlertAction
        actionWithTitle:@"OK"
        style:UIAlertActionStyleDefault
        handler:^(UIAlertAction * action){}];

      [alert addAction: cancelButton];
      [self presentViewController:alert animated:YES completion:nil];

      service.destroy();
    }

    - (void)viewDidLoad {
        [super viewDidLoad];
        // Do any additional setup after loading the view, typically from a nib.
    }

    - (void)didReceiveMemoryWarning {
        [super didReceiveMemoryWarning];
        // Dispose of any resources that can be recreated.
    }

    @end
@endcode

You will notice there is much more code in this example's ViewController.mm.
This is because this web service stores the whole XML response within a string
instead of appropriate variables. The dom parser can fix this situation so that
you can still access your results without having to parse the XML yourself. The
dom code in this example was generated via command line in UNIX. To do so, once
you have dom executable in your working directory, just execute the command

    ./domcpp -i weather.xml

where weather.xml is a file that stores an example xml response. Option -i
tells the domcpp tool to generate the code you need to parse your result.  To
obtain an example XML response, test the web service on
http://www.webservicex.net/New/Home/ServiceDetail/56.

The domcpp tool is found in gsoap/samples/dom and should be built in that
directory with:

    make domcpp

Then move or copy the domcpp executable to use it for your projects.

For more information about domcpp, read
[XML DOM and XPath](https://www.genivia.com/doc/dom/html/index.html) of the
gSOAP documentation.

@subsection ios_example_air Air Example (C++)

Airport Information Web Service is a live SOAP Web service that enables you to
look up Airport Information of countries around the world. For some reason,
some very well known countries are not supported by this web service. For
example, no results for Russia will be returned. There is an error message
within the app that shows whenever one of these countries is entered. Know that
the error message is not an issue with the app you just built, but information
that the web service itself does not provide.

This example shows how to develop a client application in C++ using gSOAP and
the ios plugin, which consumes the Weather service on iOS using gSOAP.  The
WSDL file for this service can be downloaded at the following link:
http://www.webservicex.net/airport.asmx?WSDL

It is crucial to follow these directions in order for your app to work:

- Rename the .m files to .mm files.  The iOS plugin implementation file
  gsoapios.m must be renamed to gsoapios.mm. The main.m must be renamed to
  main.mm.  Other files such as your ViewController must also be renamed with
  .mm.

- Generate C++ proxy using proper options with wsdl2h and soapcpp2.

- Use the stdsoap2.cpp instead of stdsoap2.c from the gSOAP package in your iOS
  project.

The Xcode project for this example can be found in gsoap/ios_plugin/examples/Air.

@subsubsection ios_air_step_1 Step 1: Generating stubs for C++ Proxy

To generate codes for the GeoIPService Web service, we first run the wsdl2h
tool from the command line on the URL of the WSDL and use option -o to specify
the output file (Alternatively, you can download the airport.wsdl file and use
the local file instead of the URL):

    wsdl2h -o airport.h 'http://www.webservicex.net/airport.asmx?WSDL'

This generates the airport.h service definition header file with service
operation definitions and types for the operation's data.  By default, gSOAP
assumes you will use C++ with STL.

Because we will be using the gSOAP DOM parser in this example, which is created
in our program with `xsd__anyType dom(ctx)` to extract plain XML content
received in string `xmlmessage` with `xmlmessage >> dom`, an additional step is
necessary.  Add the following line to the wsdl2h-generated airport.h file:

    #import "dom.h"

To generate the stubs for the C++ proxy classes, run the soapcpp2:

    soapcpp2 -j -CL -I$GSOAP_HOME/import airport.h

Option -j tells the compiler to generate the C++ proxy class and option -CL
indicates client-side only files (soapcpp2 generates both client and server
stubs and skeletons by default). Option -I is needed to import the stlvector.h
file from the import directory in the gSOAP package to support serialization of
STL vectors.

@subsubsection ios_air_step_2 Step 2: Creating an Xcode project for iPhone

Create a Single View-based Application project and name it Air. Open the
ViewController.xib(main storyboard) file in the Interface Builder.Double-click
on the View item and populate it with the views listed below and shown in
Figure 3:

- A Label ("Enter Country")
- A Text Field
- One Round Rect Button ("Get Airport Information" button)
- One Text View (to show the results)

In Xcode, edit the file ViewController.h to make it look like the following:

@code
    #import <UIKit/UIKit.h>

    @interface ViewController : UIViewController {
        UITextField* country_name;
        UITextView* showResults;
    }
    @property (strong, nonatomic) IBOutlet UITextField *country_name;
    @property (strong, nonatomic) IBOutlet UITextView *showResults;


    -(IBAction) buttonPressed;

    @end
@endcode

Set the `country_name` and `showResults` outlets and the
`buttonFindCountry:(id)sender` method to delegate action of the button.

In Xcode, edit the file info.plist to make it look like the following:

    <?xml version="1.0" encoding="UTF-8"?>
    <!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
    <plist version="1.0">
      <dict>
        <key>NSAppTransportSecurity</key>
        <dict>
          <key>NSAllowsArbitraryLoads</key>
          <true/>
        </dict>
          <key>CFBundleDevelopmentRegion</key>
          <string>en</string>
          <key>CFBundleExecutable</key>
          <string>$(EXECUTABLE_NAME)</string>
          <key>CFBundleIdentifier</key>
          <string>$(PRODUCT_BUNDLE_IDENTIFIER)</string>
          <key>CFBundleInfoDictionaryVersion</key>
          <string>6.0</string>
          <key>CFBundleName</key>
          <string>$(PRODUCT_NAME)</string>
          <key>CFBundlePackageType</key>
          <string>APPL</string>
          <key>CFBundleShortVersionString</key>
          <string>1.0</string>
          <key>CFBundleSignature</key>
          <string>????</string>
          <key>CFBundleVersion</key>
          <string>1</string>
          <key>LSRequiresIPhoneOS</key>
          <true/>
          <key>UILaunchStoryboardName</key>
          <string>LaunchScreen</string>
          <key>UIMainStoryboardFile</key>
          <string>Main</string>
          <key>UIRequiredDeviceCapabilities</key>
          <array>
            <string>armv7</string>
          </array>
          <key>UISupportedInterfaceOrientations</key>
          <array>
            <string>UIInterfaceOrientationPortrait</string>
            <string>UIInterfaceOrientationLandscapeLeft</string>
            <string>UIInterfaceOrientationLandscapeRight</string>
          </array>
      </dict>
    </plist>

These changes ensure that you can access the web services despite the added
layer of protection of App Transport Security).

@subsubsection ios_air_step_3 Step 3: Adding generated source stubs to the Xcode project

Add the source files soapC.cpp, soapairportSoapProxy.cpp,
soapairportSoapProxy.h, soapH.h, and soapStub.h generated in Step 1 of this
tutorial to the project. Also add files stdsoap2.h and stdsoap2.cpp to the
project from the gSOAP package and the iOS plugin files gsoapios.h and
gsoapios.mm.

Once all of your files have been added to the project, ensure they are all of
type "Objective C++ Source".  This ensures that there will be no issues with
mixing Objective C and C++ code.

@subsubsection ios_air_step_4 Step 4: Implementing the Logic by calling the soap service 

Firstly, edit file main.mm to import the file airportSoap.nsmap. Linking errors
would arise without importing this XML namespace mapping table.

@code
    // File: main.mm

    #import <UIKit/UIKit.h>
    #import "AppDelegate.h"
    #include "../../airportSoap.nsmap"

    int main(int argc, char * argv[]) {
       @autoreleasepool {
         return UIApplicationMain(argc, argv, nil, NSStringFromClass([AppDelegate class]));
       }
    }
@endcode

Then, we implement the source file ViewController.mm as the following, where we
used the [gSOAP domcpp](http://www.genivia.com/doc/dom/html) tool with option
-i on an example weather XML data file to produce DOM code to extract the XML
data (see further below)

@code
    #import "ViewController.h"
    #include "soapairportSoapProxy.h"
    #import "soapStub.h"
    #import "gsoapios.h"
    #include <fstream>

    @interface ViewController ()

    @end

    @implementation ViewController
    @synthesize country_name;
    @synthesize showResults;

    - (IBAction)buttonPressed:(id)sender {
      _ns1__GetAirportInformationByCountry sending;
      _ns1__GetAirportInformationByCountryResponse receiving;
      std::string result = "";      //stores resulting data
      bool webserviceresult = true; //keeps track of if the web service returns a readable result

      airportSoapProxy service;
      soap_init(service.soap);

      // ----- register plugin for callbacks ------
      soap_register_plugin(service.soap, soap_ios);

      std::string countryname = std::string((char*)[country_name.text UTF8String]);
      sending.country = &countryname;

      std::ofstream out("/Users/bethanysanders/Documents/work/Air/Air/xmlmessage.txt");
      std::ifstream in("/Users/bethanysanders/Documents/work/Air/Air/xmlmessage.txt");

      if (service.GetAirportInformationByCountry(&sending,receiving) == SOAP_OK)
        if (out.is_open())
          out << *(receiving.GetAirportInformationByCountryResult);

      out.close();

      struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_DOM_TREE);
      ctx->double_format = "%lG";

      xsd__anyType dom(ctx);
      if (in.is_open())
        in >> dom;
      if (dom.soap->error)
        webserviceresult = false;
      xsd__anyType *elt;

      for (xsd__anyType *it = dom.elt_get("Table"); it; it = it->get_next())
      {
        xsd__anyType& dom_Table = *it;
        if ((elt = dom_Table.elt_get("AirportCode")))
        {
          xsd__anyType& dom_Table_AirportCode = *elt;
          if (dom_Table_AirportCode.get_text())
          {
            result += "\n\nAirport Code = ";
            result += dom_Table_AirportCode.get_text();
          }

        }
        if ((elt = dom_Table.elt_get("CityOrAirportName")))
        {
          xsd__anyType& dom_Table_CityOrAirportName = *elt;
          if (dom_Table_CityOrAirportName.get_text())
          {
            result += "\nCity or Airport Name = ";
            result += dom_Table_CityOrAirportName.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("Country")))
        {
          xsd__anyType& dom_Table_Country = *elt;
          if (dom_Table_Country.get_text())
          {
            result += "\nCountry = ";
            result += dom_Table_Country.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("CountryAbbrviation")))
        {
          xsd__anyType& dom_Table_CountryAbbrviation = *elt;
          if (dom_Table_CountryAbbrviation.get_text())
          {
            result += "\nCountry Abbreviation = ";
            result += dom_Table_CountryAbbrviation.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("CountryCode")))
        {
          xsd__anyType& dom_Table_CountryCode = *elt;
          if (dom_Table_CountryCode.get_text())
          {
            result += "\nCountry Code = ";
            result += dom_Table_CountryCode.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("GMTOffset")))
        {
          xsd__anyType& dom_Table_GMTOffset = *elt;
          if (dom_Table_GMTOffset.get_text())
          {
            result += "\nGMT Offset = ";
            result += dom_Table_GMTOffset.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("RunwayLengthFeet")))
        {
          xsd__anyType& dom_Table_RunwayLengthFeet = *elt;
          if (dom_Table_RunwayLengthFeet.get_text())
          {
            result += "\nRunway Length (feet) = ";
            result += dom_Table_RunwayLengthFeet.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("RunwayElevationFeet")))
        {
          xsd__anyType& dom_Table_RunwayElevationFeet = *elt;
          if (dom_Table_RunwayElevationFeet.get_text())
          {
            result += "\nRunway Elevation (feet) = ";
            result += dom_Table_RunwayElevationFeet.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("LatitudeDegree")))
        {
          xsd__anyType& dom_Table_LatitudeDegree = *elt;
          if (dom_Table_LatitudeDegree.get_text())
          {
            result += "\nLatitude Degree = ";
            result += dom_Table_LatitudeDegree.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("LatitudeMinute")))
        {
          xsd__anyType& dom_Table_LatitudeMinute = *elt;
          if (dom_Table_LatitudeMinute.get_text())
          {
            result += "\nLatitude Minute = ";
            result += dom_Table_LatitudeMinute.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("LatitudeSecond")))
        {
          xsd__anyType& dom_Table_LatitudeSecond = *elt;
          if (dom_Table_LatitudeSecond.get_text())
          {
            result += "\nLatitude Second = ";
            result += dom_Table_LatitudeSecond.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("LatitudeNpeerS")))
        {
          xsd__anyType& dom_Table_LatitudeNpeerS = *elt;
          if (dom_Table_LatitudeNpeerS.get_text())
          {
            result += "\nLatitude N or S = ";
            result += dom_Table_LatitudeNpeerS.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("LongitudeDegree")))
        {
          xsd__anyType& dom_Table_LongitudeDegree = *elt;
          if (dom_Table_LongitudeDegree.get_text())
          {
            result += "\nLongitude Degree = ";
            result += dom_Table_LongitudeDegree.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("LongitudeMinute")))
        {
          xsd__anyType& dom_Table_LongitudeMinute = *elt;
          if (dom_Table_LongitudeMinute.get_text())
          {
            result += "\nLongitude Minute = ";
            result += dom_Table_LongitudeMinute.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("LongitudeSeconds")))
        {
          xsd__anyType& dom_Table_LongitudeSeconds = *elt;
          if (dom_Table_LongitudeSeconds.get_text())
          {
            result += "\nLongitude Second = ";
            result += dom_Table_LongitudeSeconds.get_text();
          }
        }
        if ((elt = dom_Table.elt_get("LongitudeEperW")))
        {
          xsd__anyType& dom_Table_LongitudeEperW = *elt;
          if (dom_Table_LongitudeEperW.get_text())
          {
            result += "\nLongitude E or W = ";
            result += dom_Table_LongitudeEperW.get_text();
          }
        }
      }

      NSString *airPorts = [NSString stringWithUTF8String:result.c_str()];
      if (result == "" || !webserviceresult)
        airPorts = [NSString stringWithFormat: @"%@", @"Error: no know information. Try a different country."];

      showResults.editable = NO;
      showResults.showsVerticalScrollIndicator = YES;
      showResults.text = airPorts;

      soap_destroy(ctx); // delete objects
      soap_end(ctx);     // delete DOM data
      soap_free(ctx);    // free context

      service.destroy();
    }

    - (void)viewDidLoad {
        [super viewDidLoad];
        // Do any additional setup after loading the view, typically from a nib.
    }

    - (void)didReceiveMemoryWarning {
        [super didReceiveMemoryWarning];
        // Dispose of any resources that can be recreated.
    }
    @end
@endcode

You will notice there is much more code in this example's ViewController.mm.
This is because this web service stores the whole XML response within a string
instead of appropriate variables. The dom parser can fix this situation so that
you can still access your results without having to parse the XML yourself. The
dom code in this example was generated via command line in UNIX. To do so, once
you have dom executable in your working directory, just execute the command

    ./domcpp -i airPorts.xml

where airPorts.xml is a file that stores an example xml response. The option -i
is what tells the dom tool to generate the code you need to parse your result.
To obtain an example XML response, test the web service on
http://www.webservicex.net/New/Home/ServiceDetail/20.

The domcpp tool is found in gsoap/samples/dom and should be built in that
directory with:

    make domcpp

Then move or copy the domcpp executable to use it for your projects.

For more information about domcpp, read
[XML DOM and XPath](https://www.genivia.com/doc/dom/html/index.html) of the
gSOAP documentation.

