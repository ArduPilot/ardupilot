#---------
#
# MakefileWorker.mk
#
# Include this helper file in your makefile
# It makes
#    A static library
#    A test executable
#
# See this example for parameter settings 
#    examples/Makefile
#
#----------
# Inputs - these variables describe what to build
#
#   INCLUDE_DIRS - Directories used to search for include files.
#                   This generates a -I for each directory
#	SRC_DIRS - Directories containing source file to built into the library
#   SRC_FILES - Specific source files to build into library. Helpful when not all code 
#				in a directory can be built for test (hopefully a temporary situation)
#	TEST_SRC_DIRS - Directories containing unit test code build into the unit test runner
#				These do not go in a library. They are explicitly included in the test runner
#	MOCKS_SRC_DIRS - Directories containing mock source files to build into the test runner
#				These do not go in a library. They are explicitly included in the test runner
#----------
# You can adjust these variables to influence how to build the test target
# and where to put and name outputs
# See below to determine defaults
#   COMPONENT_NAME - the name of the thing being built
#   TEST_TARGET - name the test executable. By default it is 
#			$(COMPONENT_NAME)_tests
#		Helpful if you want 1 > make files in the same directory with different
#		executables as output.
#   CPPUTEST_HOME - where CppUTest home dir found
#   CPPUTEST_OBJS_DIR - a directory where o and d files go
#   CPPUTEST_LIB_DIR - a directory where libs go
#   CPPUTEST_ENABLE_DEBUG - build for debug
#   CPPUTEST_USE_MEM_LEAK_DETECTION - Links with overridden new and delete
#   CPPUTEST_USE_STD_CPP_LIB - Set to N to keep the standard C++ library out
#		of the test harness
#   CPPUTEST_USE_GCOV - Turn on coverage analysis
#		Clean then build with this flag set to Y, then 'make gcov'
#   CPPUTEST_USE_REAL_GTEST - Expect to link to gtest too. This enables the ability to
#       run Google Test tests as CppUTest tests using the GTestConvertor.
#   CPPUTEST_MAPFILE - generate a map file
#   CPPUTEST_WARNINGFLAGS - overly picky by default
#	OTHER_MAKEFILE_TO_INCLUDE - a hook to use this makefile to make 
#		other targets. Like CSlim, which is part of fitnesse
#	CPPUTEST_USE_VPATH - Use Make's VPATH functionality to support user 
#		specification of source files and directories that aren't below 
#		the user's Makefile in the directory tree, like:
#			SRC_DIRS += ../../lib/foo
#		It defaults to N, and shouldn't be necessary except in the above case.
#----------
#
#  Other flags users can initialize to sneak in their settings
#	CPPUTEST_CXXFLAGS - flags for the C++ compiler
#	CPPUTEST_CPPFLAGS - flags for the C++ AND C preprocessor
#	CPPUTEST_CFLAGS - C complier
#	CPPUTEST_LDFLAGS - Linker flags
#----------

# Some behavior is weird on some platforms. Need to discover the platform.
UNAME_OUTPUT = "$(shell uname -a)"
MACOSX_STR = Darwin
MINGW_STR = MINGW
UNKNWOWN_OS_STR = Unknown
UNAME_OS = $(UNKNWOWN_OS_STR)

ifeq ($(findstring $(MINGW_STR),$(UNAME_OUTPUT)),$(MINGW_STR))
	UNAME_OS = $(MINGW_STR)
endif

ifeq ($(findstring $(MACOSX_STR),$(UNAME_OUTPUT)),$(MACOSX_STR))
	UNAME_OS = $(MACOSX_STR)
	#lion has a problem with the 'v' part of -a
	UNAME_OUTPUT = "$(shell uname -pmnrs)"
endif

#Kludge for mingw, it does not have cc.exe, but gcc.exe will do
ifeq ($(UNAME_OS),$(MINGW_STR))
	CC := gcc
endif

#Kludge for MacOsX gcc compiler on Darwin9 who can't handle pendantic

ifeq ($(UNAME_OS),$(MACOSX_STR))
ifeq ($(findstring Version 9,$(UNAME_OUTPUT)),Version 9)
	CPPUTEST_PEDANTIC_ERRORS = N
endif
endif

ifndef COMPONENT_NAME
    COMPONENT_NAME = name_this_in_the_makefile
endif

# Debug on by default
ifndef CPPUTEST_ENABLE_DEBUG
	CPPUTEST_ENABLE_DEBUG = Y
endif

# new and delete for memory leak detection on by default
ifndef CPPUTEST_USE_MEM_LEAK_DETECTION
	CPPUTEST_USE_MEM_LEAK_DETECTION = Y
endif

# Use the standard C library
ifndef CPPUTEST_USE_STD_C_LIB
	CPPUTEST_USE_STD_C_LIB = Y
endif

# Use the standard C++ library
ifndef CPPUTEST_USE_STD_CPP_LIB
	CPPUTEST_USE_STD_CPP_LIB = Y
endif

# Use the real gtest or use the fake simulation
ifdef CPPUTEST_USE_REAL_GTEST
	CPPUTEST_USE_REAL_GTEST = Y
else
	CPPUTEST_USE_REAL_GTEST = N
endif

# Use gmock
ifdef CPPUTEST_USE_REAL_GMOCK
	CPPUTEST_USE_REAL_GMOCK = Y
else
	CPPUTEST_USE_REAL_GMOCK = N
endif


# Use gcov, off by default
ifndef CPPUTEST_USE_GCOV
	CPPUTEST_USE_GCOV = N
endif

ifndef CPPUTEST_PEDANTIC_ERRORS
	CPPUTEST_PEDANTIC_ERRORS = Y
endif

# Default warnings
ifndef CPPUTEST_WARNINGFLAGS
ifeq ($(CPPUTEST_USE_REALGTEST), N)
	CPPUTEST_WARNINGFLAGS =  -Wall -Wextra -Werror -Wshadow -Wswitch-default -Wswitch-enum -Wconversion
ifeq ($(CPPUTEST_PEDANTIC_ERRORS), Y)
	CPPUTEST_WARNINGFLAGS += -pedantic-errors
endif 
	CPPUTEST_CXX_WARNINGFLAGS = -Woverloaded-virtual
endif
endif

# Default dir for temporary files (d, o)
ifndef CPPUTEST_OBJS_DIR
    CPPUTEST_OBJS_DIR = objs
endif

# Default dir for the outout library
ifndef CPPUTEST_LIB_DIR
    CPPUTEST_LIB_DIR = lib
endif

# No map by default
ifndef CPPUTEST_MAP_FILE
	CPPUTEST_MAP_FILE = N
endif

# No extentions is default
ifndef CPPUTEST_USE_EXTENSIONS
	CPPUTEST_USE_EXTENSIONS = N
endif

# No VPATH is default
ifndef CPPUTEST_USE_VPATH
	CPPUTEST_USE_VPATH := N
endif
# Make empty, instead of 'N', for usage in $(if ) conditionals
ifneq ($(CPPUTEST_USE_VPATH), Y)
	CPPUTEST_USE_VPATH := 
endif

# --------------------------------------
# derived flags in the following area
# --------------------------------------

# Without the C library, we'll need to disable the C++ library and ... 
ifeq ($(CPPUTEST_USE_STD_C_LIB), N)
	CPPUTEST_USE_STD_CPP_LIB = N
	CPPUTEST_USE_MEM_LEAK_DETECTION = N
	CPPUTEST_CPPFLAGS += -DCPPUTEST_STD_C_LIB_DISABLED
	CPPUTEST_CPPFLAGS += -nostdinc
endif

CPPUTEST_CPPFLAGS += -DCPPUTEST_COMPILATION

ifeq ($(CPPUTEST_USE_MEM_LEAK_DETECTION), N)
	CPPUTEST_CPPFLAGS += -DCPPUTEST_MEM_LEAK_DETECTION_DISABLED
else
    ifndef CPPUTEST_MEMLEAK_DETECTOR_NEW_MACRO_FILE
	    	CPPUTEST_MEMLEAK_DETECTOR_NEW_MACRO_FILE = -include $(CPPUTEST_HOME)/include/CppUTest/MemoryLeakDetectorNewMacros.h
    endif
    ifndef CPPUTEST_MEMLEAK_DETECTOR_MALLOC_MACRO_FILE
	    CPPUTEST_MEMLEAK_DETECTOR_MALLOC_MACRO_FILE = -include $(CPPUTEST_HOME)/include/CppUTest/MemoryLeakDetectorMallocMacros.h
	endif	
endif

ifeq ($(CPPUTEST_ENABLE_DEBUG), Y)
	CPPUTEST_CXXFLAGS += -g
	CPPUTEST_CFLAGS += -g
endif

ifeq ($(CPPUTEST_USE_STD_CPP_LIB), N)
	CPPUTEST_CPPFLAGS += -DCPPUTEST_STD_CPP_LIB_DISABLED
	CPPUTEST_CXXFLAGS += -nostdinc++
endif

ifeq ($(CPPUTEST_USE_REAL_GMOCK), Y)
	ifndef GMOCK_HOME
$(error CPPUTEST_USE_REAL_GMOCK defined, but GMOCK_HOME not, so can't use real gmock! Please define GMOCK_HOME to the gmock location)
	endif
	GTEST_HOME = $(GMOCK_HOME)/gtest
	CPPUTEST_USE_REAL_GTEST = Y
	CPPUTEST_CPPFLAGS += -I$(GMOCK_HOME)/include
	GMOCK_LIBRARY = $(GMOCK_HOME)/lib/.libs/libgmock.a
	LD_LIBRARIES += $(GMOCK_LIBRARY)
	CPPUTEST_CPPFLAGS += -DCPPUTEST_USE_REAL_GMOCK
else
	CPPUTEST_CPPFLAGS += -Iinclude/CppUTestExt/CppUTestGMock
endif

ifeq ($(CPPUTEST_USE_REAL_GTEST), Y)
	ifndef GTEST_HOME
$(error CPPUTEST_USE_REAL_GTEST defined, but GTEST_HOME not, so can't use real gtest! Please define GTEST_HOME to the gtest location)
	endif
	CPPUTEST_CPPFLAGS += -I$(GTEST_HOME)/include -I$(GTEST_HOME)
	GTEST_LIBRARY = $(GTEST_HOME)/lib/.libs/libgtest.a
	LD_LIBRARIES += $(GTEST_LIBRARY)
	CPPUTEST_CPPFLAGS += -DCPPUTEST_USE_REAL_GTEST
else
	CPPUTEST_CPPFLAGS += -Iinclude/CppUTestExt/CppUTestGTest
endif

ifeq ($(CPPUTEST_USE_GCOV), Y)
    CPPUTEST_GCOV_FLAGS = -fprofile-arcs -ftest-coverage
	CPPUTEST_CXXFLAGS += $(CPPUTEST_GCOV_FLAGS)
	CPPUTEST_CFLAGS += $(CPPUTEST_GCOV_FLAGS)
else
    CPPUTEST_GCOV_FLAGS =
endif

CPPUTEST_CXXFLAGS += $(CPPUTEST_WARNINGFLAGS) $(CPPUTEST_CXX_WARNINGFLAGS)
CPPUTEST_CPPFLAGS += $(CPPUTEST_WARNINGFLAGS)
CPPUTEST_CXXFLAGS += $(CPPUTEST_MEMLEAK_DETECTOR_NEW_MACRO_FILE)
CPPUTEST_CPPFLAGS += $(CPPUTEST_MEMLEAK_DETECTOR_MALLOC_MACRO_FILE) 

TARGET_MAP = $(COMPONENT_NAME).map.txt
ifeq ($(CPPUTEST_MAP_FILE), Y)
	CPPUTEST_LDFLAGS += -Wl,-map,$(TARGET_MAP)
endif

# Link with CppUTest lib
CPPUTEST_LIB = $(CPPUTEST_HOME)/lib/libCppUTest.a 

ifeq ($(CPPUTEST_USE_EXTENSIONS), Y)
CPPUTEST_LIB += $(CPPUTEST_HOME)/lib/libCppUTestExt.a
endif

LD_LIBRARIES += -lstdc++

# Xcode uses a different library for code coverage.
ifeq ($(UNAME_OS),$(MACOSX_STR))
    LD_LIBRARIES += $(CPPUTEST_GCOV_FLAGS)
else
    LD_LIBRARIES += -lgcov
endif

TARGET_LIB = \
    $(CPPUTEST_LIB_DIR)/lib$(COMPONENT_NAME).a
    
ifndef TEST_TARGET
TEST_TARGET = $(COMPONENT_NAME)_tests
endif

#Helper Functions
get_src_from_dir  = $(wildcard $1/*.cpp) $(wildcard $1/*.c)
get_dirs_from_dirspec  = $(wildcard $1)
get_src_from_dir_list = $(foreach dir, $1, $(call get_src_from_dir,$(dir)))
__src_to = $(subst .c,$1, $(subst .cpp,$1,$(if $(CPPUTEST_USE_VPATH),$(notdir $2),$2)))
src_to = $(addprefix $(CPPUTEST_OBJS_DIR)/,$(call __src_to,$1,$2))
src_to_o = $(call src_to,.o,$1)
src_to_d = $(call src_to,.d,$1)
src_to_gcda = $(call src_to,.gcda,$1)
src_to_gcno = $(call src_to,.gcno,$1)
time = $(shell date +%s)
delta_t = $(eval minus, $1, $2)
debug_print_list = $(foreach word,$1,echo "  $(word)";) echo;

#Derived
STUFF_TO_CLEAN += $(TEST_TARGET) $(TEST_TARGET).exe $(TARGET_LIB) $(TARGET_MAP)

SRC += $(call get_src_from_dir_list, $(SRC_DIRS)) $(SRC_FILES)			        
OBJ = $(call src_to_o,$(SRC))

STUFF_TO_CLEAN += $(OBJ)

TEST_SRC = $(call get_src_from_dir_list, $(TEST_SRC_DIRS))
TEST_OBJS = $(call src_to_o,$(TEST_SRC))
STUFF_TO_CLEAN += $(TEST_OBJS)


MOCKS_SRC = $(call get_src_from_dir_list, $(MOCKS_SRC_DIRS))
MOCKS_OBJS = $(call src_to_o,$(MOCKS_SRC))
STUFF_TO_CLEAN += $(MOCKS_OBJS)

ALL_SRC = $(SRC) $(TEST_SRC) $(MOCKS_SRC)

# If we're using VPATH
ifeq ($(CPPUTEST_USE_VPATH), Y)
	# gather all the source directories and add them
	VPATH += $(sort $(dir $(ALL_SRC)))
	# Add the component name to the objs dir path, to differentiate between same-name objects
	CPPUTEST_OBJS_DIR := $(addsuffix /$(COMPONENT_NAME),$(CPPUTEST_OBJS_DIR))
endif

#Test coverage with gcov
GCOV_OUTPUT = gcov_output.txt
GCOV_REPORT = gcov_report.txt
GCOV_ERROR = gcov_error.txt
GCOV_GCDA_FILES = $(call src_to_gcda, $(ALL_SRC))
GCOV_GCNO_FILES = $(call src_to_gcno, $(ALL_SRC))
TEST_OUTPUT = $(TEST_TARGET).txt
STUFF_TO_CLEAN += \
	$(GCOV_OUTPUT)\
	$(GCOV_REPORT)\
	$(GCOV_REPORT).html\
	$(GCOV_ERROR)\
	$(GCOV_GCDA_FILES)\
	$(GCOV_GCNO_FILES)\
	$(TEST_OUTPUT)

#The gcda files for gcov need to be deleted before each run
#To avoid annoying messages.
GCOV_CLEAN = $(SILENCE)rm -f $(GCOV_GCDA_FILES) $(GCOV_OUTPUT) $(GCOV_REPORT) $(GCOV_ERROR)
RUN_TEST_TARGET = $(SILENCE)  $(GCOV_CLEAN) ; echo "Running $(TEST_TARGET)"; ./$(TEST_TARGET) $(CPPUTEST_EXE_FLAGS)

INCLUDES_DIRS_EXPANDED = $(call get_dirs_from_dirspec, $(INCLUDE_DIRS))
INCLUDES += $(foreach dir, $(INCLUDES_DIRS_EXPANDED), -I$(dir))
MOCK_DIRS_EXPANDED = $(call get_dirs_from_dirspec, $(MOCKS_SRC_DIRS))
INCLUDES += $(foreach dir, $(MOCK_DIRS_EXPANDED), -I$(dir))

CPPUTEST_CPPFLAGS +=  $(INCLUDES) 

DEP_FILES = $(call src_to_d, $(ALL_SRC))
STUFF_TO_CLEAN += $(DEP_FILES) $(PRODUCTION_CODE_START) $(PRODUCTION_CODE_END)
STUFF_TO_CLEAN += $(STDLIB_CODE_START) $(MAP_FILE) cpputest_*.xml junit_run_output

# We'll use the CPPUTEST_CFLAGS etc so that you can override AND add to the CppUTest flags
CFLAGS = $(CPPUTEST_CFLAGS) $(CPPUTEST_ADDITIONAL_CFLAGS)
CPPFLAGS = $(CPPUTEST_CPPFLAGS) $(CPPUTEST_ADDITIONAL_CPPFLAGS)
CXXFLAGS = $(CPPUTEST_CXXFLAGS) $(CPPUTEST_ADDITIONAL_CXXFLAGS)
LDFLAGS = $(CPPUTEST_LDFLAGS) $(CPPUTEST_ADDITIONAL_LDFLAGS)

DEP_FLAGS=-MMD -MP

# Some macros for programs to be overridden. For some reason, these are not in Make defaults
RANLIB = ranlib

# Targets

.PHONY: all
all: start $(TEST_TARGET)  
	$(RUN_TEST_TARGET)	

.PHONY: start
start: $(TEST_TARGET) 
	$(SILENCE)START_TIME=$(call time)

.PHONY: all_no_tests
all_no_tests: $(TEST_TARGET)

.PHONY: flags
flags: 
	@echo
	@echo "OS ${UNAME_OS}"
	@echo "Compile C and C++ source with CPPFLAGS:"
	@$(call debug_print_list,$(CPPFLAGS))
	@echo "Compile C++ source with CXXFLAGS:"
	@$(call debug_print_list,$(CXXFLAGS))
	@echo "Compile C source with CFLAGS:"
	@$(call debug_print_list,$(CFLAGS))
	@echo "Link with LDFLAGS:"
	@$(call debug_print_list,$(LDFLAGS))
	@echo "Link with LD_LIBRARIES:"
	@$(call debug_print_list,$(LD_LIBRARIES))
	@echo "Create libraries with ARFLAGS:"
	@$(call debug_print_list,$(ARFLAGS))

$(TEST_TARGET): $(TEST_OBJS) $(MOCKS_OBJS)  $(PRODUCTION_CODE_START) $(TARGET_LIB) $(USER_LIBS) $(PRODUCTION_CODE_END) $(CPPUTEST_LIB) $(STDLIB_CODE_START) 
	@echo Linking $@
	$(SILENCE)$(LINK.o) -o $@ $^ $(LD_LIBRARIES)

$(TARGET_LIB): $(OBJ)
	@echo Building archive $@
	$(SILENCE)mkdir -p $(CPPUTEST_LIB_DIR)
	$(SILENCE)$(AR) $(ARFLAGS) $@ $^
	$(SILENCE)$(RANLIB) $@

test: $(TEST_TARGET)
	$(RUN_TEST_TARGET) | tee $(TEST_OUTPUT)
	
vtest: $(TEST_TARGET)
	$(RUN_TEST_TARGET) -v  | tee $(TEST_OUTPUT)

$(CPPUTEST_OBJS_DIR)/%.o: %.cpp
	@echo compiling $(notdir $<)
	$(SILENCE)mkdir -p $(dir $@)
	$(SILENCE)$(COMPILE.cpp) $(DEP_FLAGS) $(OUTPUT_OPTION) $<

$(CPPUTEST_OBJS_DIR)/%.o: %.c
	@echo compiling $(notdir $<)
	$(SILENCE)mkdir -p $(dir $@)
	$(SILENCE)$(COMPILE.c) $(DEP_FLAGS)  $(OUTPUT_OPTION) $<

ifneq "$(MAKECMDGOALS)" "clean"
-include $(DEP_FILES)
endif

.PHONY: clean
clean:
	@echo Making clean
	$(SILENCE)$(RM) $(STUFF_TO_CLEAN)
	$(SILENCE)rm -rf gcov $(CPPUTEST_OBJS_DIR)
	$(SILENCE)find . -name "*.gcno" | xargs rm -f
	$(SILENCE)find . -name "*.gcda" | xargs rm -f
	
#realclean gets rid of all gcov, o and d files in the directory tree
#not just the ones made by this makefile
.PHONY: realclean
realclean: clean
	$(SILENCE)rm -rf gcov
	$(SILENCE)find . -name "*.gdcno" | xargs rm -f
	$(SILENCE)find . -name "*.[do]" | xargs rm -f	

gcov: test
ifeq ($(CPPUTEST_USE_VPATH), Y)
	$(SILENCE)for d in $(OBJ) ; do \
		gcov -object-directory=$(CPPUTEST_OBJS_DIR) $(call __src_to,.o,$$d) >> $(GCOV_OUTPUT) 2>>$(GCOV_ERROR) ; \
	done
else
	$(SILENCE)for d in $(SRC_DIRS) ; do \
		for f in $(CPPUTEST_OBJS_DIR)/$$d/*.o ; do \
			gcov -object-directory=$(CPPUTEST_OBJS_DIR)/$$d $$f >> $(GCOV_OUTPUT) 2>>$(GCOV_ERROR) ; \
		done \
	done
endif
	$(CPPUTEST_HOME)/scripts/filterGcov.sh $(GCOV_OUTPUT) $(GCOV_ERROR) $(GCOV_REPORT) $(TEST_OUTPUT)
	$(SILENCE)cat $(GCOV_REPORT)
	$(SILENCE)mkdir -p gcov
	$(SILENCE)mv *.gcov gcov
	$(SILENCE)mv gcov_* gcov
	@echo "See gcov directory for details"
 
.PHONEY: format
format: 
	$(CPPUTEST_HOME)/scripts/reformat.sh $(PROJECT_HOME_DIR)
	
.PHONEY: debug
debug:
	@echo
	@echo "Target Source files:"
	@$(call debug_print_list,$(SRC))
	@echo "Target Object files:"
	@$(call debug_print_list,$(OBJ))
	@echo "Test Source files:"
	@$(call debug_print_list,$(TEST_SRC))
	@echo "Test Object files:"
	@$(call debug_print_list,$(TEST_OBJS))
	@echo "Mock Source files:"
	@$(call debug_print_list,$(MOCKS_SRC))
	@echo "Mock Object files:"
	@$(call debug_print_list,$(MOCKS_OBJS))
	@echo "All Input Dependency files:"
	@$(call debug_print_list,$(DEP_FILES))
	@echo Stuff to clean:
	@$(call debug_print_list,$(STUFF_TO_CLEAN))
	@echo Includes:
	@$(call debug_print_list,$(INCLUDES))

ifneq "$(OTHER_MAKEFILE_TO_INCLUDE)" ""
-include $(OTHER_MAKEFILE_TO_INCLUDE)
endif
