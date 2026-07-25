REM Copying stdsoap2.h and stdsoap2.cpp to source dir
COPY /B ..\..\stdsoap2.h .
COPY /B ..\..\stdsoap2.cpp .
COPY /B ..\..\import\dom.h .
COPY /B ..\..\dom.cpp .
..\..\soapcpp2\Release\soapcpp2.exe -CSL dom.h
REM You need these header files to compile:
REM stdsoap2.h
REM soapH.h
REM soapStub.h
REM You need these source code files to compile:
REM stdsoap2.cpp
REM dom.cpp
REM domcpp.cpp
REM soapC.cpp
