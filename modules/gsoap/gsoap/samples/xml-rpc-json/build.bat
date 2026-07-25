REM Copying stdsoap2.h and stdsoap2.cpp to source dir
COPY /B ..\..\stdsoap2.h .
COPY /B ..\..\stdsoap2.cpp .
..\..\soapcpp2\Release\soapcpp2.exe -CSL xml-rpc.h
REM You need these header files to compile:
REM stdsoap2.h
REM json.h
REM xml-rpc-iters.h
REM soapH.h
REM soapStub.h
REM You need these source code files to compile:
REM stdsoap2.cpp
REM xml-rpc.cpp
REM json.cpp
REM jsoncpp.cpp
REM soapC.cpp
