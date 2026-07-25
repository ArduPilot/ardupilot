A MAC OS X Xcode example client for the simple calculator service.

To create a new gSOAP-based Xcode project, follow these additional steps:

Add a new run script to invoke soapcpp2:
Project -> New Build Phase -> New Run Script Build Phase
In the New script edit box:
Script content:
./../src/soapcpp2 -i -wx -C calc.h
Input files:
$(SRCROOT)/calc.h
Output files:
$(DERIVED_FILE_DIR)/soapcalcProxy.h
$(DERIVED_FILE_DIR)/soapcalcProxy.cpp
$(DERIVED_FILE_DIR)/soapStub.h
$(DERIVED_FILE_DIR)/soapH.h
$(DERIVED_FILE_DIR)/soapC.cpp

In the project overview move the new Run script to the start of the build:
Groups & Files window pane:
Targets
-> calc_xcode
   -> Run script
   -> Compile Sources
   ...

The calcclient application invokes the service and is used from the command
line as follows:

$ calcclient add 3 4
result = 7

The calcserver application is a CGI application that can be installed under the
cgibin of your Web server.

