# Microsoft Developer Studio Generated NMAKE File, Based on AllTests.dsp
!IF "$(CFG)" == ""
CFG=AllTests - Win32 Debug
!MESSAGE No configuration specified. Defaulting to AllTests - Win32 Debug.
!ENDIF 

!IF "$(CFG)" != "AllTests - Win32 Release" && "$(CFG)" != "AllTests - Win32 Debug"
!MESSAGE Invalid configuration "$(CFG)" specified.
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "AllTests.mak" CFG="AllTests - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "AllTests - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "AllTests - Win32 Debug" (based on "Win32 (x86) Console Application")
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

!IF  "$(CFG)" == "AllTests - Win32 Release"

OUTDIR=.\Release
INTDIR=.\Release
# Begin Custom Macros
OutDir=.\Release
# End Custom Macros

!IF "$(RECURSE)" == "0" 

ALL : "$(OUTDIR)\AllTests.exe"

!ELSE 

ALL : "CppUTest - Win32 Release" "$(OUTDIR)\AllTests.exe"

!ENDIF 

!IF "$(RECURSE)" == "1" 
CLEAN :"CppUTest - Win32 ReleaseCLEAN" 
!ELSE 
CLEAN :
!ENDIF 
	-@erase "$(INTDIR)\AllocationInCFile.obj"
	-@erase "$(INTDIR)\AllocationInCppFile.obj"
	-@erase "$(INTDIR)\AllTests.obj"
	-@erase "$(INTDIR)\CommandLineArgumentsTest.obj"
	-@erase "$(INTDIR)\CommandLineTestRunnerTest.obj"
	-@erase "$(INTDIR)\FailureTest.obj"
	-@erase "$(INTDIR)\JUnitOutputTest.obj"
	-@erase "$(INTDIR)\MemoryLeakAllocator.obj"
	-@erase "$(INTDIR)\MemoryLeakAllocatorTest.obj"
	-@erase "$(INTDIR)\MemoryLeakDetectorTest.obj"
	-@erase "$(INTDIR)\MemoryLeakOperatorOverloadsTest.obj"
	-@erase "$(INTDIR)\MemoryLeakWarningTest.obj"
	-@erase "$(INTDIR)\NullTestTest.obj"
	-@erase "$(INTDIR)\PluginTest.obj"
	-@erase "$(INTDIR)\SetPluginTest.obj"
	-@erase "$(INTDIR)\SimpleStringTest.obj"
	-@erase "$(INTDIR)\TestHarness_cTest.obj"
	-@erase "$(INTDIR)\TestInstallerTest.obj"
	-@erase "$(INTDIR)\TestOutputTest.obj"
	-@erase "$(INTDIR)\TestRegistryTest.obj"
	-@erase "$(INTDIR)\TestResultTest.obj"
	-@erase "$(INTDIR)\UtestTest.obj"
	-@erase "$(INTDIR)\vc60.idb"
	-@erase "$(OUTDIR)\AllTests.exe"

"$(OUTDIR)" :
    if not exist "$(OUTDIR)/$(NULL)" mkdir "$(OUTDIR)"

CPP_PROJ=/nologo /ML /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /Fp"$(INTDIR)\AllTests.pch" /YX /Fo"$(INTDIR)\\" /Fd"$(INTDIR)\\" /FD /c 
BSC32=bscmake.exe
BSC32_FLAGS=/nologo /o"$(OUTDIR)\AllTests.bsc" 
BSC32_SBRS= \
	
LINK32=link.exe
LINK32_FLAGS=kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /incremental:no /pdb:"$(OUTDIR)\AllTests.pdb" /machine:I386 /out:"$(OUTDIR)\AllTests.exe" 
LINK32_OBJS= \
	"$(INTDIR)\AllTests.obj" \
	"$(INTDIR)\CommandLineArgumentsTest.obj" \
	"$(INTDIR)\CommandLineTestRunnerTest.obj" \
	"$(INTDIR)\FailureTest.obj" \
	"$(INTDIR)\JUnitOutputTest.obj" \
	"$(INTDIR)\MemoryLeakAllocator.obj" \
	"$(INTDIR)\MemoryLeakAllocatorTest.obj" \
	"$(INTDIR)\MemoryLeakDetectorTest.obj" \
	"$(INTDIR)\MemoryLeakWarningTest.obj" \
	"$(INTDIR)\NullTestTest.obj" \
	"$(INTDIR)\PluginTest.obj" \
	"$(INTDIR)\SetPluginTest.obj" \
	"$(INTDIR)\SimpleStringTest.obj" \
	"$(INTDIR)\TestHarness_cTest.obj" \
	"$(INTDIR)\TestInstallerTest.obj" \
	"$(INTDIR)\TestOutputTest.obj" \
	"$(INTDIR)\TestRegistryTest.obj" \
	"$(INTDIR)\TestResultTest.obj" \
	"$(INTDIR)\UtestTest.obj" \
	"$(INTDIR)\AllocationInCppFile.obj" \
	"$(INTDIR)\MemoryLeakOperatorOverloadsTest.obj" \
	"$(INTDIR)\AllocationInCFile.obj" \
	"..\Release\CppUTest.lib"

"$(OUTDIR)\AllTests.exe" : "$(OUTDIR)" $(DEF_FILE) $(LINK32_OBJS)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(LINK32_OBJS)
<<

!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"

OUTDIR=.\Debug
INTDIR=.\Debug
# Begin Custom Macros
OutDir=.\Debug
# End Custom Macros

!IF "$(RECURSE)" == "0" 

ALL : "$(OUTDIR)\AllTests.exe" "$(OUTDIR)\AllTests.bsc"

!ELSE 

ALL : "CppUTest - Win32 Debug" "$(OUTDIR)\AllTests.exe" "$(OUTDIR)\AllTests.bsc"

!ENDIF 

!IF "$(RECURSE)" == "1" 
CLEAN :"CppUTest - Win32 DebugCLEAN" 
!ELSE 
CLEAN :
!ENDIF 
	-@erase "$(INTDIR)\AllocationInCFile.obj"
	-@erase "$(INTDIR)\AllocationInCFile.sbr"
	-@erase "$(INTDIR)\AllocationInCppFile.obj"
	-@erase "$(INTDIR)\AllocationInCppFile.sbr"
	-@erase "$(INTDIR)\AllTests.obj"
	-@erase "$(INTDIR)\AllTests.sbr"
	-@erase "$(INTDIR)\CommandLineArgumentsTest.obj"
	-@erase "$(INTDIR)\CommandLineArgumentsTest.sbr"
	-@erase "$(INTDIR)\CommandLineTestRunnerTest.obj"
	-@erase "$(INTDIR)\CommandLineTestRunnerTest.sbr"
	-@erase "$(INTDIR)\FailureTest.obj"
	-@erase "$(INTDIR)\FailureTest.sbr"
	-@erase "$(INTDIR)\JUnitOutputTest.obj"
	-@erase "$(INTDIR)\JUnitOutputTest.sbr"
	-@erase "$(INTDIR)\MemoryLeakAllocator.obj"
	-@erase "$(INTDIR)\MemoryLeakAllocator.sbr"
	-@erase "$(INTDIR)\MemoryLeakAllocatorTest.obj"
	-@erase "$(INTDIR)\MemoryLeakAllocatorTest.sbr"
	-@erase "$(INTDIR)\MemoryLeakDetectorTest.obj"
	-@erase "$(INTDIR)\MemoryLeakDetectorTest.sbr"
	-@erase "$(INTDIR)\MemoryLeakOperatorOverloadsTest.obj"
	-@erase "$(INTDIR)\MemoryLeakOperatorOverloadsTest.sbr"
	-@erase "$(INTDIR)\MemoryLeakWarningTest.obj"
	-@erase "$(INTDIR)\MemoryLeakWarningTest.sbr"
	-@erase "$(INTDIR)\NullTestTest.obj"
	-@erase "$(INTDIR)\NullTestTest.sbr"
	-@erase "$(INTDIR)\PluginTest.obj"
	-@erase "$(INTDIR)\PluginTest.sbr"
	-@erase "$(INTDIR)\SetPluginTest.obj"
	-@erase "$(INTDIR)\SetPluginTest.sbr"
	-@erase "$(INTDIR)\SimpleStringTest.obj"
	-@erase "$(INTDIR)\SimpleStringTest.sbr"
	-@erase "$(INTDIR)\TestHarness_cTest.obj"
	-@erase "$(INTDIR)\TestHarness_cTest.sbr"
	-@erase "$(INTDIR)\TestInstallerTest.obj"
	-@erase "$(INTDIR)\TestInstallerTest.sbr"
	-@erase "$(INTDIR)\TestOutputTest.obj"
	-@erase "$(INTDIR)\TestOutputTest.sbr"
	-@erase "$(INTDIR)\TestRegistryTest.obj"
	-@erase "$(INTDIR)\TestRegistryTest.sbr"
	-@erase "$(INTDIR)\TestResultTest.obj"
	-@erase "$(INTDIR)\TestResultTest.sbr"
	-@erase "$(INTDIR)\UtestTest.obj"
	-@erase "$(INTDIR)\UtestTest.sbr"
	-@erase "$(INTDIR)\vc60.idb"
	-@erase "$(INTDIR)\vc60.pdb"
	-@erase "$(OUTDIR)\AllTests.bsc"
	-@erase "$(OUTDIR)\AllTests.exe"
	-@erase "$(OUTDIR)\AllTests.ilk"
	-@erase "$(OUTDIR)\AllTests.pdb"

"$(OUTDIR)" :
    if not exist "$(OUTDIR)/$(NULL)" mkdir "$(OUTDIR)"

CPP_PROJ=/nologo /MDd /W3 /GX /ZI /Od /I "..\include" /I "..\include\Platforms\VisualCpp" /D "_CONSOLE" /D "WIN32" /D "_DEBUG" /D "_MBCS" /FR"$(INTDIR)\\" /Fo"$(INTDIR)\\" /Fd"$(INTDIR)\\" /FD /GZ /c 
BSC32=bscmake.exe
BSC32_FLAGS=/nologo /o"$(OUTDIR)\AllTests.bsc" 
BSC32_SBRS= \
	"$(INTDIR)\AllTests.sbr" \
	"$(INTDIR)\CommandLineArgumentsTest.sbr" \
	"$(INTDIR)\CommandLineTestRunnerTest.sbr" \
	"$(INTDIR)\FailureTest.sbr" \
	"$(INTDIR)\JUnitOutputTest.sbr" \
	"$(INTDIR)\MemoryLeakAllocator.sbr" \
	"$(INTDIR)\MemoryLeakAllocatorTest.sbr" \
	"$(INTDIR)\MemoryLeakDetectorTest.sbr" \
	"$(INTDIR)\MemoryLeakWarningTest.sbr" \
	"$(INTDIR)\NullTestTest.sbr" \
	"$(INTDIR)\PluginTest.sbr" \
	"$(INTDIR)\SetPluginTest.sbr" \
	"$(INTDIR)\SimpleStringTest.sbr" \
	"$(INTDIR)\TestHarness_cTest.sbr" \
	"$(INTDIR)\TestInstallerTest.sbr" \
	"$(INTDIR)\TestOutputTest.sbr" \
	"$(INTDIR)\TestRegistryTest.sbr" \
	"$(INTDIR)\TestResultTest.sbr" \
	"$(INTDIR)\UtestTest.sbr" \
	"$(INTDIR)\AllocationInCppFile.sbr" \
	"$(INTDIR)\MemoryLeakOperatorOverloadsTest.sbr" \
	"$(INTDIR)\AllocationInCFile.sbr"

"$(OUTDIR)\AllTests.bsc" : "$(OUTDIR)" $(BSC32_SBRS)
    $(BSC32) @<<
  $(BSC32_FLAGS) $(BSC32_SBRS)
<<

LINK32=link.exe
LINK32_FLAGS=..\lib\CppUTest.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib winmm.lib /nologo /subsystem:console /incremental:yes /pdb:"$(OUTDIR)\AllTests.pdb" /debug /machine:I386 /out:"$(OUTDIR)\AllTests.exe" /pdbtype:sept 
LINK32_OBJS= \
	"$(INTDIR)\AllTests.obj" \
	"$(INTDIR)\CommandLineArgumentsTest.obj" \
	"$(INTDIR)\CommandLineTestRunnerTest.obj" \
	"$(INTDIR)\FailureTest.obj" \
	"$(INTDIR)\JUnitOutputTest.obj" \
	"$(INTDIR)\MemoryLeakAllocator.obj" \
	"$(INTDIR)\MemoryLeakAllocatorTest.obj" \
	"$(INTDIR)\MemoryLeakDetectorTest.obj" \
	"$(INTDIR)\MemoryLeakWarningTest.obj" \
	"$(INTDIR)\NullTestTest.obj" \
	"$(INTDIR)\PluginTest.obj" \
	"$(INTDIR)\SetPluginTest.obj" \
	"$(INTDIR)\SimpleStringTest.obj" \
	"$(INTDIR)\TestHarness_cTest.obj" \
	"$(INTDIR)\TestInstallerTest.obj" \
	"$(INTDIR)\TestOutputTest.obj" \
	"$(INTDIR)\TestRegistryTest.obj" \
	"$(INTDIR)\TestResultTest.obj" \
	"$(INTDIR)\UtestTest.obj" \
	"$(INTDIR)\AllocationInCppFile.obj" \
	"$(INTDIR)\MemoryLeakOperatorOverloadsTest.obj" \
	"$(INTDIR)\AllocationInCFile.obj" \
	"..\lib\CppUTest.lib"

"$(OUTDIR)\AllTests.exe" : "$(OUTDIR)" $(DEF_FILE) $(LINK32_OBJS)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(LINK32_OBJS)
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
!IF EXISTS("AllTests.dep")
!INCLUDE "AllTests.dep"
!ELSE 
!MESSAGE Warning: cannot find "AllTests.dep"
!ENDIF 
!ENDIF 


!IF "$(CFG)" == "AllTests - Win32 Release" || "$(CFG)" == "AllTests - Win32 Debug"
SOURCE=.\AllocationInCFile.c

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\AllocationInCFile.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\AllocationInCFile.obj"	"$(INTDIR)\AllocationInCFile.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\AllocationInCppFile.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\AllocationInCppFile.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\AllocationInCppFile.obj"	"$(INTDIR)\AllocationInCppFile.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\AllTests.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\AllTests.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\AllTests.obj"	"$(INTDIR)\AllTests.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\CommandLineArgumentsTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\CommandLineArgumentsTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\CommandLineArgumentsTest.obj"	"$(INTDIR)\CommandLineArgumentsTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\CommandLineTestRunnerTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\CommandLineTestRunnerTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\CommandLineTestRunnerTest.obj"	"$(INTDIR)\CommandLineTestRunnerTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\FailureTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\FailureTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\FailureTest.obj"	"$(INTDIR)\FailureTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\JUnitOutputTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\JUnitOutputTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\JUnitOutputTest.obj"	"$(INTDIR)\JUnitOutputTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=..\src\CppUTest\MemoryLeakAllocator.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\MemoryLeakAllocator.obj" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\MemoryLeakAllocator.obj"	"$(INTDIR)\MemoryLeakAllocator.sbr" : $(SOURCE) "$(INTDIR)"
	$(CPP) $(CPP_PROJ) $(SOURCE)


!ENDIF 

SOURCE=.\MemoryLeakAllocatorTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\MemoryLeakAllocatorTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\MemoryLeakAllocatorTest.obj"	"$(INTDIR)\MemoryLeakAllocatorTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\MemoryLeakDetectorTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\MemoryLeakDetectorTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\MemoryLeakDetectorTest.obj"	"$(INTDIR)\MemoryLeakDetectorTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\MemoryLeakOperatorOverloadsTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\MemoryLeakOperatorOverloadsTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\MemoryLeakOperatorOverloadsTest.obj"	"$(INTDIR)\MemoryLeakOperatorOverloadsTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\MemoryLeakWarningTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\MemoryLeakWarningTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\MemoryLeakWarningTest.obj"	"$(INTDIR)\MemoryLeakWarningTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\NullTestTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\NullTestTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\NullTestTest.obj"	"$(INTDIR)\NullTestTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\PluginTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\PluginTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\PluginTest.obj"	"$(INTDIR)\PluginTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\SetPluginTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\SetPluginTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\SetPluginTest.obj"	"$(INTDIR)\SetPluginTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\SimpleStringTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\SimpleStringTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\SimpleStringTest.obj"	"$(INTDIR)\SimpleStringTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\TestHarness_cTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\TestHarness_cTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\TestHarness_cTest.obj"	"$(INTDIR)\TestHarness_cTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\TestInstallerTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\TestInstallerTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\TestInstallerTest.obj"	"$(INTDIR)\TestInstallerTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\TestOutputTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\TestOutputTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\TestOutputTest.obj"	"$(INTDIR)\TestOutputTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\TestRegistryTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\TestRegistryTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\TestRegistryTest.obj"	"$(INTDIR)\TestRegistryTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\TestResultTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\TestResultTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\TestResultTest.obj"	"$(INTDIR)\TestResultTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

SOURCE=.\UtestTest.cpp

!IF  "$(CFG)" == "AllTests - Win32 Release"


"$(INTDIR)\UtestTest.obj" : $(SOURCE) "$(INTDIR)"


!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"


"$(INTDIR)\UtestTest.obj"	"$(INTDIR)\UtestTest.sbr" : $(SOURCE) "$(INTDIR)"


!ENDIF 

!IF  "$(CFG)" == "AllTests - Win32 Release"

"CppUTest - Win32 Release" : 
   cd "\workspace\CppUTest"
   $(MAKE) /$(MAKEFLAGS) /F .\CppUTest.mak CFG="CppUTest - Win32 Release" 
   cd ".\tests"

"CppUTest - Win32 ReleaseCLEAN" : 
   cd "\workspace\CppUTest"
   $(MAKE) /$(MAKEFLAGS) /F .\CppUTest.mak CFG="CppUTest - Win32 Release" RECURSE=1 CLEAN 
   cd ".\tests"

!ELSEIF  "$(CFG)" == "AllTests - Win32 Debug"

"CppUTest - Win32 Debug" : 
   cd "\workspace\CppUTest"
   $(MAKE) /$(MAKEFLAGS) /F .\CppUTest.mak CFG="CppUTest - Win32 Debug" 
   cd ".\tests"

"CppUTest - Win32 DebugCLEAN" : 
   cd "\workspace\CppUTest"
   $(MAKE) /$(MAKEFLAGS) /F .\CppUTest.mak CFG="CppUTest - Win32 Debug" RECURSE=1 CLEAN 
   cd ".\tests"

!ENDIF 


!ENDIF 

