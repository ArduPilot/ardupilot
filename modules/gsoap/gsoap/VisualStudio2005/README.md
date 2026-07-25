
Flex and Bison for Windows
==========================

Visual Studio 2005 Solution files for soapcpp2 and wsdl2h are included.

If not already installed, install the Platform SDK (R2) for `winsock2.h`.

To build `soapcpp2.exe`, first install Bison and Flex (in the default dirs):

<http://gnuwin32.sourceforge.net/packages/bison.htm>
<http://gnuwin32.sourceforge.net/packages/flex.htm>

then add the FlexBison.rules as explained here:

<http://msdn2.microsoft.com/en-us/library/aa730877(VS.80).aspx>

These custom-build rules are used to build the scanner and parser for
`soapcpp2.exe`.

To build `wsdl2h.exe`, you first need to build `soapcpp2.exe` and install it in
Program Files or copy it to the `wsdl` directory. This is needed to execute the
custom-build step on `wsdl.h` to generate `wsdlStub.h`, `wsdlH.h`, and
`wsdlC.cpp`.

Build Rules for `soapcpp2.exe` VS 2008 and 2010
===============================================

To build `soapcpp2.exe` you need to install Flex and Bison. To do so, you need
to create custom build rules to compile `.l` and `.y` files with Flex and
Bison.

Please see:

<http://msdn.microsoft.com/en-us/library/aa730877(VS.80).aspx#vccustombr_topic3>

In VS2008, there is UI available to help you create the custom build rule.

Make sure you have `flex.exe`, `bison.exe`, and `m4.exe` on the system search
path.

- Right click on the `soapcpp2_lex.l` file and select properties
- Configuration -> All Configurations
- General -> Item Type -> Custom Build Tool
- Apply
- Custom Build Tool -> General -> Command Line -> `flex soapcpp2_lex.l`
- Custom Build Tool -> General -> Outputs -> `lex.yy.c`
- Custom Build Tool -> General -> Additional Dependencies -> `soapcpp2.h soapcpp2_yacc.tab.h`
- Apply
- Select the `soapcpp2_yacc.y` file in the solution explorer
- Configuration -> All Configurations
- General -> Item Type -> Custom Build Tool
- Apply
- Custom Build Tool -> General -> Command Line -> `bison -d -v soapcpp2_yacc.y`
- Custom Build Tool -> General -> Outputs -> `soapcpp2_yacc.tab.c soapcpp2_yacc.tab.h`

In VS2010, there may not be a UI available to create the custom build rules.
To add or modify build rules in VS2010 you need to edit:

`%ProgramFiles%\MSBuild\Microsoft.Cpp\v4.0\BuildCustomizations`

and/or

`%ProgramFiles(x86)%\MSBuild\Microsoft.Cpp\v4.0\BuildCustomizations`


