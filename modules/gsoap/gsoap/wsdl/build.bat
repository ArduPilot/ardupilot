REM Copying stdsoap2.h and stdsoap2.cpp to source dir
COPY /B ..\stdsoap2.h .
COPY /B ..\stdsoap2.cpp .
..\..\soapcpp2\Release\soapcpp2.exe -I../import -SC -pwsdl wsdl.h
REM You need these header files to compile:
REM stdsoap2.h
REM include.h
REM service.h
REM types.h
REM wsdlH.h
REM wsdlStub.h
REM You need these source code files to compile:
REM stdsoap2.cpp
REM bpel.cpp
REM mime.cpp
REM schema.cpp
REM service.cpp
REM soap.cpp
REM types.cpp
REM wadl.cpp
REM wsdl.cpp
REM wsdl2h.cpp
REM wsp.cpp
REM wsdlC.cpp
