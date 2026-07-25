# Copyright 2017 Adam Green (https://github.com/adamgreen)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# User can set VERBOSE variable to have all commands echoed to console for debugging purposes.
ifdef VERBOSE
    Q :=
else
    Q := @
endif


#######################################
#  Forwards Declaration of Main Rules
#######################################
.PHONY : all test clean gcov

all:
gcov:
clean:


#  Names of tools for compiling binaries to run on this host system.
HOST_GCC := gcc
HOST_GPP := g++
HOST_AS  := gcc
HOST_LD  := g++
HOST_AR  := ar

# Handle Windows and *nix differences.
ifeq "$(OS)" "Windows_NT"
    MAKEDIR = mkdir $(subst /,\,$(dir $@))
    REMOVE := del /q
    REMOVE_DIR := rd /s /q
    QUIET := >nul 2>nul & exit 0
    EXE := .exe
else
ifeq "$(shell uname)" "Darwin"
    GCOV_OBJDIR_FLAG := -object-directory
else
    GCOV_OBJDIR_FLAG := --object-directory
endif
    MAKEDIR = mkdir -p $(dir $@)
    REMOVE := rm
    REMOVE_DIR := rm -r -f
    QUIET := > /dev/null 2>&1 ; exit 0
    EXE :=
endif

# Flags to use when compiling binaries to run on this host system.
HOST_GCCFLAGS := -O2 -g3 -ffunction-sections -fdata-sections -fno-common -MMD -MP
HOST_GCCFLAGS += -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unknown-warning-option -Wno-format-truncation
HOST_GCCFLAGS += -include mri/CppUTest/include/CppUTest/MemoryLeakDetectorMallocMacros.h
HOST_GPPFLAGS := $(HOST_GCCFLAGS) -std=gnu++98 -include mri/CppUTest/include/CppUTest/MemoryLeakDetectorNewMacros.h
HOST_GCCFLAGS += -std=gnu90
HOST_ASFLAGS  := -g -x assembler-with-cpp -MMD -MP

# Output directories for intermediate object files.
OBJDIR        := obj
HOST_OBJDIR   := $(OBJDIR)

# Output directory for gcov files.
GCOVDIR := gcov

# Output directories for final libraries.
LIBDIR        := lib
HOST_LIBDIR   := $(LIBDIR)

# Customize some variables for code coverage builds.
GCOV_HOST_OBJDIR        := $(GCOVDIR)/$(HOST_OBJDIR)
GCOV_HOST_LIBDIR        := $(GCOVDIR)/$(HOST_LIBDIR)
GCOV_HOST_GCCFLAGS      := $(HOST_GCCFLAGS) -fprofile-arcs -ftest-coverage
GCOV_HOST_GPPFLAGS      := $(HOST_GPPFLAGS) -fprofile-arcs -ftest-coverage
GCOV_HOST_LDFLAGS       := $(HOST_LDFLAGS) -fprofile-arcs -ftest-coverage

# Start out with empty pre-req lists.  Add modules as we go.
ALL_TARGETS  :=
GCOV_TARGETS :=

# Start out with an empty header file dependency list.  Add module files as we go.
DEPS :=

# Useful macros.
objs = $(addprefix $2/,$(addsuffix .o,$(basename $(wildcard $1/*.c $1/*.cpp $1/*.S))))
host_objs = $(call objs,$1,$(HOST_OBJDIR))
gcov_host_objs = $(call objs,$1,$(GCOV_HOST_OBJDIR))
add_deps = $(patsubst %.o,%.d,$(HOST_$1_OBJ) $(GCOV_HOST_$1_OBJ))
obj_to_gcda = $(patsubst %.o,%.gcda,$1)
includes = $(patsubst %,-I%,$1)
define build_lib
	@echo Building $@
	$Q $(MAKEDIR) $(QUIET)
	$Q $($1_AR) -rc $@ $?
endef
define link_exe
	@echo Building $@
	$Q $(MAKEDIR) $(QUIET)
	$Q $($1_LD) $($1_LDFLAGS) $^ -o $@
endef
define gcov_link_exe
	@echo Building $@
	$Q $(MAKEDIR) $(QUIET)
	$Q $($1_LD) $(GCOV_$1_LDFLAGS) $^ -o $@
endef
ifeq "$(OS)" "Windows_NT"
define run_gcov
    GCOV_TARGETS += GCOV_$1
    .PHONY : GCOV_$1
    GCOV_$1 : GCOV_RUN_$1_TESTS
		$Q $(REMOVE) $1_output.txt $(QUIET)
		$Q mkdir $(subst /,\,gcov/$1_tests) $(QUIET)
		$Q $(foreach i,$(GCOV_HOST_$1_OBJ),gcov $(dir $i)$(notdir $i)  >> $1_output.txt 2>nul &&) REM
		$Q move $1_output.txt gcov/$1_tests/ $(QUIET)
		$Q move *.gcov gcov/$1_tests/ $(QUIET)
		$Q mri\CppUTest\scripts\filterGcov.cmd gcov\$1_tests\$1_output.txt gcov\$1_tests\$1.txt
		$Q type gcov\$1_tests\$1.txt
endef
else
define run_gcov
    GCOV_TARGETS += GCOV_$1
    .PHONY : GCOV_$1
    GCOV_$1 : GCOV_RUN_$1_TESTS
		$Q $(REMOVE) $1_output.txt $(QUIET)
		$Q mkdir -p gcov/$1_tests $(QUIET)
		$Q $(foreach i,$(GCOV_HOST_$1_OBJ),gcov $(GCOV_OBJDIR_FLAG)=$(dir $i) $(notdir $i) >> $1_output.txt ;)
		$Q mv $1_output.txt gcov/$1_tests/ $(QUIET)
		$Q mv *.gcov gcov/$1_tests/ $(QUIET)
		$Q mri/CppUTest/scripts/filterGcov.sh gcov/$1_tests/$1_output.txt /dev/null gcov/$1_tests/$1.txt
		$Q cat gcov/$1_tests/$1.txt
endef
endif
define make_library # ,LIBRARY,src_dirs,libname.a,includes
    HOST_$1_OBJ      := $(foreach i,$2,$(call host_objs,$i))
    GCOV_HOST_$1_OBJ := $(foreach i,$2,$(call gcov_host_objs,$i))
    HOST_$1_LIB      := $(HOST_LIBDIR)/$3
    GCOV_HOST_$1_LIB := $(GCOV_HOST_LIBDIR)/$3
    DEPS             += $$(call add_deps,$1)
    ALL_TARGETS      += $$(HOST_$1_LIB)
    $$(HOST_$1_LIB)      : INCLUDES := $4
    $$(GCOV_HOST_$1_LIB) : INCLUDES := $4
    $$(HOST_$1_LIB) : $$(HOST_$1_OBJ)
		$$(call build_lib,HOST)
    $$(GCOV_HOST_$1_LIB) : $$(GCOV_HOST_$1_OBJ)
		$$(call build_lib,HOST)
endef
define make_tests # ,LIB2TEST,test_src_dirs,includes,other_libs
    HOST_$1_TESTS_OBJ      := $(foreach i,$2,$(call host_objs,$i))
    GCOV_HOST_$1_TESTS_OBJ := $(foreach i,$2,$(call gcov_host_objs,$i))
    HOST_$1_TESTS_EXE      := $1_tests
    GCOV_HOST_$1_TESTS_EXE := $1_tests_gcov
    DEPS                   += $$(call add_deps,$1_TESTS)
    ALL_TARGETS += RUN_$1_TESTS
    $$(HOST_$1_TESTS_EXE)      : INCLUDES := mri/CppUTest/include $3
    $$(GCOV_HOST_$1_TESTS_EXE) : INCLUDES := mri/CppUTest/include $3
    $$(HOST_$1_TESTS_EXE) : $$(HOST_$1_TESTS_OBJ) $(HOST_$1_LIB) $(HOST_CPPUTEST_LIB) $4
		$$(call link_exe,HOST)
    .PHONY : RUN_$1_TESTS GCOV_RUN_$1_TESTS
    RUN_$1_TESTS : $$(HOST_$1_TESTS_EXE)
		@echo Runnning $$^
		$Q ./$$^
    $$(GCOV_HOST_$1_TESTS_EXE) : $$(GCOV_HOST_$1_TESTS_OBJ) $(GCOV_HOST_$1_LIB) $(GCOV_HOST_CPPUTEST_LIB) $4
		$$(call gcov_link_exe,HOST)
    GCOV_RUN_$1_TESTS : $$(GCOV_HOST_$1_TESTS_EXE)
		@echo Runnning $$^
		$Q $(REMOVE) $(call obj_to_gcda,$(GCOV_HOST_$1_OBJ)) $(QUIET)
		$Q ./$$^
endef
define make_app # ,APP2BUILD,app_src_dirs,includes,other_libs
    HOST_$1_APP_OBJ        := $(foreach i,$2,$(call host_objs,$i))
    HOST_$1_APP_EXE        := $1
    DEPS                   += $$(call add_deps,$1_APP)
    ALL_TARGETS += $$(HOST_$1_APP_EXE)
    $$(HOST_$1_APP_EXE) : INCLUDES := $3
    $$(HOST_$1_APP_EXE) : $$(HOST_$1_APP_OBJ) $4
		$$(call link_exe,HOST)
endef

#######################################
# libCppUtest.a
$(eval $(call make_library,CPPUTEST,mri/CppUTest/src/CppUTest mri/CppUTest/src/Platforms/Gcc,libCppUTest.a,mri/CppUTest/include))
$(eval $(call make_tests,CPPUTEST,mri/CppUTest/tests,,))

#######################################
# Native memory access objects
HOST_NATIVE_MEM_OBJ   := $(call host_objs,mri/memory/native)
DEPS += $(call add_deps,NATIVE_MEM)

#######################################
# libmricore.a
$(eval $(call make_library,LIBMRICORE,mri/core,libmricore.a,mri/include))
$(eval $(call make_tests,LIBMRICORE,\
                         mri/tests/tests mri/tests/mocks,\
                         mri/include mri/tests/mocks,\
                         $(HOST_NATIVE_MEM_OBJ)))
$(eval $(call run_gcov,LIBMRICORE))

#######################################
# libmocks.a
$(eval $(call make_library,LIBMOCKS,libmocks/src,libmocks.a,include))
$(eval $(call make_tests,LIBMOCKS,libmocks/tests,include,))
$(eval $(call run_gcov,LIBMOCKS))

#######################################
# libcommon.a
$(eval $(call make_library,LIBCOMMON,libcommon/src,libcommon.a,include))
$(eval $(call make_tests,LIBCOMMON,libcommon/tests,include,$(HOST_LIBMOCKS_LIB)))
$(eval $(call run_gcov,LIBCOMMON))

#######################################
# libCrashDebug.a
$(eval $(call make_library,LIBCRASHDEBUG,libCrashDebug/src,libCrashDebug.a,\
                         include mri/include libCrashDebug/mocks CrashCatcher/include CrashCatcher/Core/src))
$(eval $(call make_tests,LIBCRASHDEBUG, \
                         libCrashDebug/tests libCrashDebug/mocks, \
                         include mri/include libCrashDebug/mocks libCrashDebug/src CrashCatcher/include CrashCatcher/Core/src, \
                         $(HOST_LIBCOMMON_LIB) \
                         $(HOST_LIBMOCKS_LIB) \
                         $(HOST_LIBMRICORE_LIB)))
$(eval $(call run_gcov,LIBCRASHDEBUG))

#######################################
# CrashDebug Executable
$(eval $(call make_app,CrashDebug,main,include,$(HOST_LIBCRASHDEBUG_LIB) \
                                               $(HOST_LIBMRICORE_LIB) \
                                               $(HOST_LIBMEMSIM_LIB) \
                                               $(HOST_LIBCOMMON_LIB)))



#######################################
#  Actual Definition of Main Rules
#######################################
all : $(ALL_TARGETS)

gcov : RUN_CPPUTEST_TESTS $(GCOV_TARGETS)

clean :
	@echo Cleaning CrashDebug
	$Q $(REMOVE_DIR) $(OBJDIR) $(QUIET)
	$Q $(REMOVE_DIR) $(LIBDIR) $(QUIET)
	$Q $(REMOVE_DIR) $(GCOVDIR) $(QUIET)
	$Q $(REMOVE) *_tests$(EXE) $(QUIET)
	$Q $(REMOVE) *_tests_gcov$(EXE) $(QUIET)
	$Q $(REMOVE) CrashDebug$(EXE) $(QUIET)


# *** Pattern Rules ***
$(HOST_OBJDIR)/%.o : %.c
	@echo Compiling $<
	$Q $(MAKEDIR) $(QUIET)
	$Q $(EXTRA_COMPILE_STEP)
	$Q $(HOST_GCC) $(HOST_GCCFLAGS) $(call includes,$(INCLUDES)) -c $< -o $@

$(HOST_OBJDIR)/%.o : %.cpp
	@echo Compiling $<
	$Q $(MAKEDIR) $(QUIET)
	$Q $(EXTRA_COMPILE_STEP)
	$Q $(HOST_GPP) $(HOST_GPPFLAGS) $(call includes,$(INCLUDES)) -c $< -o $@

$(GCOV_HOST_OBJDIR)/%.o : %.c
	@echo Compiling $<
	$Q $(MAKEDIR) $(QUIET)
	$Q $(REMOVE) $(call obj_to_gcda,$@) $(QUIET)
	$Q $(HOST_GCC) $(GCOV_HOST_GCCFLAGS) $(call includes,$(INCLUDES)) -c $< -o $@

$(GCOV_HOST_OBJDIR)/%.o : %.cpp
	@echo Compiling $<
	$Q $(MAKEDIR) $(QUIET)
	$Q $(REMOVE) $(call obj_to_gcda,$@) $(QUIET)
	$Q $(HOST_GPP) $(GCOV_HOST_GPPFLAGS) $(call includes,$(INCLUDES)) -c $< -o $@


# *** Pull in header dependencies if not performing a clean build. ***
ifneq "$(findstring clean,$(MAKECMDGOALS))" "clean"
    -include $(DEPS)
endif
