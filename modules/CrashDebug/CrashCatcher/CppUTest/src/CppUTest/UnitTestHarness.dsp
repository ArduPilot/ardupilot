# Microsoft Developer Studio Project File - Name="UnitTestHarness" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=UnitTestHarness - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "UnitTestHarness.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "UnitTestHarness.mak" CFG="UnitTestHarness - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "UnitTestHarness - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "UnitTestHarness - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "UnitTestHarness - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ELSEIF  "$(CFG)" == "UnitTestHarness - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "..\Platforms\VisualCpp" /I ".." /I "$(CPP_TEST_TOOLS)" /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /FR /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Cmds=copy Debug\*.lib ..\lib
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "UnitTestHarness - Win32 Release"
# Name "UnitTestHarness - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\CommandLineTestRunner.cpp
# End Source File
# Begin Source File

SOURCE=.\EqualsFailure.cpp
# End Source File
# Begin Source File

SOURCE=.\Failure.cpp
# End Source File
# Begin Source File

SOURCE=.\FailureTest.cpp
# End Source File
# Begin Source File

SOURCE=.\MemoryLeakWarningTest.cpp
# End Source File
# Begin Source File

SOURCE=.\NullTest.cpp
# End Source File
# Begin Source File

SOURCE=.\NullTestTest.cpp
# End Source File
# Begin Source File

SOURCE=.\SimpleString.cpp
# End Source File
# Begin Source File

SOURCE=.\SimpleStringTest.cpp
# End Source File
# Begin Source File

SOURCE=.\Test.cpp
# End Source File
# Begin Source File

SOURCE=.\TestInstaller.cpp
# End Source File
# Begin Source File

SOURCE=.\TestInstallerTest.cpp
# End Source File
# Begin Source File

SOURCE=.\TestOutput.cpp
# End Source File
# Begin Source File

SOURCE=.\TestOutputTest.cpp
# End Source File
# Begin Source File

SOURCE=.\TestRegistry.cpp
# End Source File
# Begin Source File

SOURCE=.\TestResult.cpp
# End Source File
# Begin Source File

SOURCE=.\TestTest.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\CommandLineTestRunner.h
# End Source File
# Begin Source File

SOURCE=.\EqualsFailure.h
# End Source File
# Begin Source File

SOURCE=.\Failure.h
# End Source File
# Begin Source File

SOURCE=.\MemoryLeakWarning.h
# End Source File
# Begin Source File

SOURCE=.\MockPrinter.h
# End Source File
# Begin Source File

SOURCE=.\NullTest.h
# End Source File
# Begin Source File

SOURCE=.\RealPrinter.h
# End Source File
# Begin Source File

SOURCE=.\SimpleString.h
# End Source File
# Begin Source File

SOURCE=.\Test.h
# End Source File
# Begin Source File

SOURCE=.\TestHarness.h
# End Source File
# Begin Source File

SOURCE=.\TestInstaller.h
# End Source File
# Begin Source File

SOURCE=.\TestOutput.h
# End Source File
# Begin Source File

SOURCE=.\TestRegistry.h
# End Source File
# Begin Source File

SOURCE=.\TestResult.h
# End Source File
# Begin Source File

SOURCE=.\UnitTestHarnessTests.h
# End Source File
# End Group
# End Target
# End Project
