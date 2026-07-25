# Microsoft Developer Studio Generated NMAKE File, Based on CppUTest.dsp
!IF "$(CFG)" == ""
CFG=CppUTest - Win32 Debug
!MESSAGE No configuration specified. Defaulting to CppUTest - Win32 Debug.
!ENDIF 

!IF "$(CFG)" != "CppUTest - Win32 Release" && "$(CFG)" != "CppUTest - Win32 Debug"
!MESSAGE Invalid configuration "$(CFG)" specified.
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "CppUTest.mak" CFG="CppUTest - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "CppUTest - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "CppUTest - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 
!ERROR An invalid configuration is specified.
!ENDIF 

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE 
NULL=nul
!ENDIF 

CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "CppUTest - Win32 Release"

OUTDIR=.\Release
INTDIR=.\Release
# Begin Custom Macros
OutDir=.\Release
# End Custom Macros

ALL : "$(OUTDIR)\CppUTest.lib"


CLEAN :
	-@erase "$(INTDIR)\CommandLineArguments.obj"
	-@erase "$(INTDIR)\CommandLineTestRunner.obj"
	-@erase "$(INTDIR)\Failure.obj"
	-@erase "$(INTDIR)\JUnitTestOutput.obj"
	-@erase "$(INTDIR)\MemoryLeakAllocator.obj"
	-@erase "$(INTDIR)\MemoryLeakDetector.obj"
	-@erase "$(INTDIR)\MemoryLeakWarningPlugin.obj"
	-@erase "$(INTDIR)\SimpleString.obj"
	-@erase "$(INTDIR)\TestHarness_c.obj"
	-@erase "$(INTDIR)\TestOutput.obj"
	-@erase "$(INTDIR)\TestPlugin.obj"
	-@erase "$(INTDIR)\TestRegistry.obj"
	-@erase "$(INTDIR)\TestResult.obj"
	-@erase "$(INTDIR)\Utest.obj"
	-@erase "$(INTDIR)\UtestPlatform.obj"
	-@erase "$(INTDIR)\vc60.idb"
	-@erase "$(OUTDIR)\CppUTest.lib"

"$(OUTDIR)" :
    if not exist "$(OUTDIR)/$(NULL)" mkdir "$(OUTDIR)"

CPP_PROJ=/nologo /ML /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /Fp"$(INTDIR)\CppUTest.pch" /YX /Fo"$(INTDIR)\\" /Fd"$(INTDIR)\\" /FD /c 
BSC32=bscmake.exe
BSC32_FLAGS=/nologo /o"$(OUTDIR)\CppUTest.bsc" 
BSC32_SBRS= \
	
LIB32=link.exe -lib
LIB32_FLAGS=/nologo /out:"$(OUTDIR)\CppUTest.lib" 
LIB32_OBJS= \
	"$(INTDIR)\CommandLineArguments.obj" \
	"$(INTDIR)\CommandLineTestRunner.obj" \
	"$(INTDIR)\Failure.obj" \
	"$(INTDIR)\JUnitTestOutput.obj" \
	"$(INTDIR)\MemoryLeakAllocator.obj" \
	"$(INTDIR)\MemoryLeakDetector.obj" \
	"$(INTDIR)\MemoryLeakWarningPlugin.obj" \
	"$(INTDIR)\SimpleString.obj" \
	"$(INTDIR)\TestHarness_c.obj" \
	"$(INTDIR)\TestOutput.obj" \
	"$(INTDIR)\TestPlugin.obj" \
	"$(INTDIR)\TestRegistry.obj" \
	"$(INTDIR)\TestResult.obj" \
	"$(INTDIR)\Utest.obj" \
	"$(INTDIR)\UtestPlatform.obj"

"$(OUTDIR)\CppUTest.lib" : "$(OUTDIR)" $(DEF_FILE) $(LIB32_OBJS)
    $(LIB32) @<<
  $(LIB32_FLAGS) $(DEF_FLAGS) $(LIB32_OBJS)
<<

!ELSEIF  "$(CFG)" == "CppUTest - Win32 Debug"

OUTDIR=.\Debug
INTDIR=.\Debug

ALL : ".\lib\CppUTest.lib"


CLEAN :
	-@erase "$(INTDIR)\CommandLineArguments.obj"
	-@erase "$(INTDIR)\CommandLineTestRunner.obj"
	-@erase "$(INTDIR)\Failure.obj"
	-@erase "$(INTDIR)\JUnitTestOutput.obj"
	-@erase "$(INTDIR)\MemoryLeakAllocator.obj"
	-@erase "$(INTDIR)\MemoryLeakDetector.obj"
	-@erase "$(INTDIR)\MemoryLeakWarningPlugin.obj"
	-@erase "$(INTDIR)\SimpleString.obj"
	-@erase "$(INTDIR)\TestHarness_c.obj"
	-@erase "$(INTDIR)\TestOutput.obj"
	-@erase "$(INTDIR)\TestPlugin.obj"
	-@erase "$(INTDIR)\TestRegistry.obj"
	-@erase "$(INTDIR)\TestResult.obj"
	-@erase "$(INTDIR)\Utest.obj"
	-@erase "$(INTDIR)\UtestPlatform.obj"
	-@erase "$(INTDIR)\vc60.idb"
	-@erase "$(INTDIR)\vc60.pdb"
	-@erase ".\lib\CppUTest.lib"

"$(OUTDIR)" :
    if not exist "$(OUTDIR)/$(NULL)" mkdir "$(OUTDIR)"

CPP_PROJ=/nologo /MDd /W3 /GX /ZI /Od /I ".\include" /I ".\include\Platforms\VisualCpp" /D "_LIB" /D "WIN32" /D "_DEBUG" /D "_MBCS" /Fo"$(INTDIR)\\" /Fd"$(INTDIR)\\" /FD /GZ /c 
BSC32=bscmake.exe
BSC32_FLAGS=/nologo /o"$(OUTDIR)\CppUTest.bsc" 
BSC32_SBRS= \
	
LIB32=link.exe -lib
LIB32_FLAGS=/nologo /out:"lib\CppUTest.lib" 
LIB32_OBJS= \
	"$(INTDIR)\CommandLineArguments.obj" \
	"$(INTDIR)\CommandLineTestRunner.obj" \
	"$(INTDIR)\Failure.obj" \
	"$(INTDIR)\JUnitTestOutput.obj" \
	"$(INTDIR)\MemoryLeakAllocator.obj" \
	"$(INTDIR)\MemoryLeakDetector.obj" \
	"$(INTDIR)\MemoryLeakWarningPlugin.obj" \
	"$(INTDIR)\SimpleString.obj" \
	"$(INTDIR)\TestHarness_c.obj" \
	"$(INTDIR)\TestOutput.obj" \
	"$(INTDIR)\TestPlugin.obj" \
	"$(INTDIR)\TestRegistry.obj" \
	"$(INTDIR)\TestResult.obj" \
	"$(INTDIR)\Utest.obj" \
	"$(INTDIR)\UtestPlatform.obj"

".\lib\CppUTest.lib" : "$(OUTDIR)" $(DEF_FILE) $(LIB32_OBJS)
    $(LIB32) @<<
  $(LIB32_FLAGS) $(DEF_FLAGS) $(LIB32_OBJS)
<<

!ENDIF 

.c{$(INTDIR)}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cpp{$(INTDIR)}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cxx{$(INTDIR)}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.c{$(INTDIR)}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cpp{$(INTDIR)}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cxx{$(INTDIR)}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<


!IF "$(NO_EXTERNAL_DEPS)" != "1"
!IF EXISTS("CppUTest.dep")
!INCLUDE "CppUTest.dep"
!ELSE 
!MESSAGE Warning: cannot find "CppUTest.dep"
!ENDIF 
!ENDIF 


!IF "$(CFG)" == "CppUTest - Win32 Release" || "$(CFG)" == "CppUTest - Win32 Debug"
SOURCE=.\src\CppUTest\CommandLineArguments.cpp

"$(INTDIR)\CommandLineArguments.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\CommandLineTestRunner.cpp

"$(INTDIR)\CommandLineTestRunner.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\Failure.cpp

"$(INTDIR)\Failure.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\JUnitTestOutput.cpp

"$(INTDIR)\JUnitTestOutput.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\MemoryLeakAllocator.cpp

"$(INTDIR)\MemoryLeakAllocator.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\MemoryLeakDetector.cpp

"$(INTDIR)\MemoryLeakDetector.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\MemoryLeakWarningPlugin.cpp

"$(INTDIR)\MemoryLeakWarningPlugin.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\SimpleString.cpp

"$(INTDIR)\SimpleString.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\TestHarness_c.cpp

"$(INTDIR)\TestHarness_c.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\TestOutput.cpp

"$(INTDIR)\TestOutput.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\TestPlugin.cpp

"$(INTDIR)\TestPlugin.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\TestRegistry.cpp

"$(INTDIR)\TestRegistry.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\TestResult.cpp

"$(INTDIR)\TestResult.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\CppUTest\Utest.cpp

"$(INTDIR)\Utest.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


SOURCE=.\src\Platforms\VisualCpp\UtestPlatform.cpp

"$(INTDIR)\UtestPlatform.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)



!ENDIF 

