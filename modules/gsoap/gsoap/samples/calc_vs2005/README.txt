Microsoft Visual Studio 2005 project example for the simple calculator service.

To create a new gSOAP-based VS2005 project, follow these additional steps:

Add a custom-build step to invoke soapcpp2 on the gSOAP header file (xyz.h):
Right-button select xyz.h and select Properties
In Configuration Propertes select Custom Build Step
  Command Line: soapcpp2.exe -i -wx -C $(InputFileName)
  Outputs: soapH.h soapStub.h soapC.cpp

IMPORTANT: make sure to compile all sources in C++ compilation mode. If you
migrate to a project file .vcproj, please set CompileAs="2" in your .vcproj
file.

Notes:

1. soapcpp2.exe must be installed as an executable, or you need to copy it to
   your project directory to enable custom-build step execution.
2. do not #include "xyz.h" in our source code directly, but #include the
   soapxyzProxy.h (for proxies) or soapH.h file and the .nsmap file

The calc_vs2005 executable is a simple console application to invoke the
calculator service from the DOS command line:

$ calc_vs2005 add 3 4
result = 7

