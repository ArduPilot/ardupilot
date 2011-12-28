#
# Copyright (c) 2010 Andrew Tridgell. All rights reserved.
# based on Arduino.mk, Copyright (c) 2010 Michael Smith
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.	IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#

#
# Save the system type for later use.
#
SYSTYPE			:=	$(shell uname)

# force LANG to C so awk works sanely on MacOS
export LANG=C

#
# Locate the sketch sources based on the initial Makefile's path
#
SRCROOT			:=	$(PWD)

#
# We need to know the location of the sketchbook.  If it hasn't been overridden,
# try the parent of the current directory.  If there is no libraries directory
# there, assume that we are in a library's examples directory and try backing up
# further.
#
ifeq ($(SKETCHBOOK),)
  SKETCHBOOK		:=	$(shell cd $(SRCROOT)/.. && pwd)
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    SKETCHBOOK		:=	$(shell cd $(SRCROOT)/../.. && pwd)
  endif
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    SKETCHBOOK		:=	$(shell cd $(SRCROOT)/../../.. && pwd)
  endif
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    SKETCHBOOK		:=	$(shell cd $(SRCROOT)/../../../.. && pwd)
  endif
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    $(error ERROR: cannot determine sketchbook location - please specify on the commandline with SKETCHBOOK=<path>)
  endif
else
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    $(warning WARNING: sketchbook directory $(SKETCHBOOK) contains no libraries)
  endif
endif

#
# Work out the sketch name from the name of the source directory.
#
SKETCH			:=	$(lastword $(subst /, ,$(SRCROOT)))
# Workaround a $(lastword ) bug on cygwin
ifeq ($(SKETCH),)
  WORDLIST		:=	$(subst /, ,$(SRCROOT))
  SKETCH		:=	$(word $(words $(WORDLIST)),$(WORDLIST))
endif

#
# Work out where we are going to be building things
#
TMPDIR			?=	/tmp
BUILDROOT		:=	$(abspath $(TMPDIR)/$(SKETCH).build)
ifneq ($(findstring CYGWIN, $(SYSTYPE)),)
  # Workaround a $(abspath ) bug on cygwin
  ifeq ($(BUILDROOT),)
    BUILDROOT	:=	C:$(TMPDIR)/$(SKETCH).build
    $(warning your abspath function is not working)
    $(warning > setting BUILDROOT to $(BUILDROOT))
  endif
endif

# Jump over the next makefile sections when runing a "make configure"
ifneq ($(MAKECMDGOALS),configure)

HARDWARE=desktop
BOARD=desktop

ifeq ($(SYSTYPE),Darwin)
  AWK			:=	awk
  CXX			:=	c++
  CC			:=	cc
  AS			:=	cc
  AR			:=	ar
endif

CXX			?=	g++
CC			?=	gcc
AS			?=	gcc
AR			?=	ar
LD			:=	$(CXX)

# Find awk
AWK			?=	gawk
ifeq ($(shell which $(AWK)),)
$(error ERROR: cannot find $(AWK) - you may need to install GNU awk)
endif

#
# Tool options
#
DEFINES			=	$(EXTRAFLAGS) -DSKETCH=\"$(SKETCH)\"
OPTFLAGS		=	-g -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2 -Wno-reorder
DEPFLAGS		=	-MD -MT $@

# XXX warning options TBD
CXXOPTS			= 	-fno-exceptions -D__AVR_ATmega2560__ -I$(SKETCHBOOK)/libraries/Desktop/include -DDESKTOP_BUILD=1
COPTS			=	-I$(SKETCHBOOK)/libraries/Desktop/include -DDESKTOP_BUILD=1
ASOPTS			=	-assembler-with-cpp

CXXFLAGS		=	-g $(DEFINES) $(OPTFLAGS) $(DEPFLAGS) $(CXXOPTS)
CFLAGS			=	-g $(DEFINES) $(OPTFLAGS) $(DEPFLAGS) $(COPTS)
ASFLAGS			=	-g $(DEFINES) $(DEPFLAGS) $(ASOPTS)
LDFLAGS			=	-g $(OPTFLAGS)

LIBS			=	-lm

SRCSUFFIXES		=	*.cpp *.c

ifeq ($(VERBOSE),)
v			=	@
else
v			=
endif


################################################################################
# Sketch
#

# Sketch source files
SKETCHPDESRCS	:=	$(wildcard $(SRCROOT)/*.pde)
SKETCHSRCS		:=	$(wildcard $(addprefix $(SRCROOT)/,$(SRCSUFFIXES)))
SKETCHPDE		:=	$(wildcard $(SRCROOT)/$(SKETCH).pde)
SKETCHCPP		:=	$(BUILDROOT)/$(SKETCH).cpp
ifeq ($(SKETCHPDE),)
$(error ERROR: sketch $(SKETCH) is missing $(SKETCH).pde)
endif

# Sketch object files
SKETCHOBJS		:=	$(subst $(SRCROOT),$(BUILDROOT),$(SKETCHSRCS)) $(SKETCHCPP)
SKETCHOBJS		:=	$(addsuffix .o,$(basename $(SKETCHOBJS)))

# List of input files to the sketch.cpp file in the order they should
# be appended to create it
SKETCHCPP_SRC		:=	$(SKETCHPDE) $(sort $(filter-out $(SKETCHPDE),$(SKETCHPDESRCS)))

################################################################################
# Libraries
#
# Pick libraries to add to the include path and to link with based on
# #include directives in the sketchfiles.
#
# For example:
#
#   #include <Foo.h>
#
# implies that there might be a Foo library.
#
# Note that the # and $ require special treatment to avoid upsetting
# make.
#
SEXPR			=	's/^[[:space:]]*\#include[[:space:]][<\"]([^>\"./]+).*$$/\1/p'
ifeq ($(SYSTYPE),Darwin)
  LIBTOKENS        :=    $(sort $(shell cat $(SKETCHPDESRCS) $(SKETCHSRCS) | sed -nEe $(SEXPR)))
else
  LIBTOKENS        :=    $(sort $(shell cat $(SKETCHPDESRCS) $(SKETCHSRCS) | sed -nre $(SEXPR)))
endif

# these are library objects we don't want in the desktop build (maybe we'll add them later)
NODESKTOP		:= I2C/I2C.cpp DataFlash/DataFlash_APM1.cpp FastSerial/FastSerial.cpp AP_Compass/AP_Compass_HMC5843.cpp AP_Baro/AP_Baro_BMP085.cpp

#
# Find sketchbook libraries referenced by the sketch.
#
# Include paths for sketch libraries
#
SKETCHLIBS		:=	$(wildcard $(addprefix $(SKETCHBOOK)/libraries/,$(LIBTOKENS)))
SKETCHLIBNAMES		:=	$(notdir $(SKETCHLIBS))
SKETCHLIBSRCDIRS	:=	$(SKETCHLIBS) $(SKETCHBOOK)/libraries/Desktop/support $(addsuffix /utility,$(SKETCHLIBS))
SKETCHLIBSRCS		:=	$(wildcard $(foreach suffix,$(SRCSUFFIXES),$(addsuffix /$(suffix),$(SKETCHLIBSRCDIRS))))
FILTEREDSRCS		:=	$(filter-out $(addprefix $(SKETCHBOOK)/libraries/,$(NODESKTOP)),$(SKETCHLIBSRCS))
SKETCHLIBOBJS		:=	$(addsuffix .o,$(basename $(subst $(SKETCHBOOK),$(BUILDROOT),$(FILTEREDSRCS))))
SKETCHLIBINCLUDES	:=	$(addprefix -I,$(SKETCHLIBS))

#
# Find Arduino libraries referenced by the sketch.  Exclude any that
# are overloaded by versions in the sketchbook.
#
ARDUINOLIBS		:=	$(wildcard $(addprefix $(ARDUINO)/libraries/,$(filter-out $(SKETCHLIBNAMES),$(LIBTOKENS))))
ARDUINOLIBNAMES		:=	$(notdir $(ARDUINOLIBS))
ARDUINOLIBSRCDIRS	:=	$(ARDUINOLIBS) $(addsuffix /utility,$(ARDUINOLIBS))
ARDUINOLIBSRCS		:=	$(wildcard $(foreach suffix,$(SRCSUFFIXES),$(addsuffix /$(suffix),$(ARDUINOLIBSRCDIRS))))
ARDUINOLIBOBJS		:=	$(addsuffix .o,$(basename $(subst $(ARDUINO),$(BUILDROOT),$(ARDUINOLIBSRCS))))
ARDUINOLIBINCLUDES	:=	$(addprefix -I,$(ARDUINOLIBS))
ARDUINOLIBINCLUDES	:=	$(ARDUINOLIBINCLUDES)

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)

################################################################################
# Built products
#

# The ELF file
SKETCHELF		=	$(BUILDROOT)/$(SKETCH).elf

# Map file
SKETCHMAP		=	$(BUILDROOT)/$(SKETCH).map

# The core library
CORELIB			=

# All of the objects that may be built
ALLOBJS			=	$(SKETCHOBJS) $(LIBOBJS) $(CORELIBOBJS)

# All of the dependency files that may be generated
ALLDEPS			=	$(ALLOBJS:%.o=%.d)
endif

################################################################################
# Targets
#

all:	$(SKETCHELF)

clean:
	@rm -fr $(BUILDROOT)

################################################################################
# Rules
#

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

# common header for rules, prints what is being built
define RULEHDR
	@echo %% $(subst $(BUILDROOT)/,,$@)
	@mkdir -p $(dir $@)
endef

showsources:
	@echo "SKETCHLIBSRCDIRS=$(SKETCHLIBSRCDIRS)"
	@echo "SKETCHLIBSRCS=$(SKETCHLIBSRCS)"
	@echo "SKETCHOBJS=$(SKETCHOBJS)"
	@echo "LIBOBJS=$(LIBOBJS)"
	@echo "CORELIB=$(CORELIB)"

# Link the final object
$(SKETCHELF):	$(SKETCHOBJS) $(LIBOBJS) $(CORELIB)
	$(RULEHDR)
	$(v)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)
	@echo Built $@

#
# Build sketch objects
#
SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) $(COREINCLUDES)

$(BUILDROOT)/%.o: $(BUILDROOT)/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< -I$(SRCROOT) $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

#
# Build library objects from sources in the sketchbook
#
SLIB_INCLUDES	=	-I$(dir $<)/utility $(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) $(COREINCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

#
# Build library objects from Ardiuno library sources
#
ALIB_INCLUDES	=	-I$(dir $<)/utility $(ARDUINOLIBINCLUDES) $(COREINCLUDES)

$(BUILDROOT)/libraries/%.o: $(ARDUINO)/libraries/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(ALIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(ARDUINO)/libraries/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(ALIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(ARDUINO)/libraries/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(ALIB_INCLUDES)

#
# Build objects from the hardware core
#
$(BUILDROOT)/$(HARDWARE)/%.o: $(CORESRC_DIR)/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(filter-out -W%,$(CXXFLAGS)) -c -o $@ $< -I$(CORESRC_DIR)

$(BUILDROOT)/$(HARDWARE)/%.o: $(CORESRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(v)$(CC) $(filter-out -W%,$(CFLAGS)) -c -o $@ $< -I$(CORESRC_DIR)

$(BUILDROOT)/$(HARDWARE)/%.o: $(CORESRC_DIR)/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< -I$(CORESRC_DIR)

#
# Build the core library
#
$(CORELIB): $(CORELIBOBJS)
	$(RULEHDR)
	$(v)$(AR) -rcs $@ $^

#
# Build the sketch.cpp file
#
# This process strives to be as faithful to the Arduino implementation as
# possible.  Conceptually, the process is as follows:
#
# * All of the .pde files are concatenated, starting with the file named
#   for the sketch and followed by the others in alphabetical order.
# * An insertion point is created in the concatenated file at
#   the first statement that isn't a preprocessor directive or comment.
# * An include of "WProgram.h" is inserted at the insertion point.
# * The file following the insertion point is scanned for function definitions
#   and prototypes for these functions are inserted at the insertion point.
#
# In addition, we add #line directives wherever the originating file changes
# to help backtrack from compiler messages and in the debugger.
#
$(SKETCHCPP):	$(SKETCHCPP_SRC)
	$(RULEHDR)
	$(v)$(AWK) -v mode=header '$(SKETCH_SPLITTER)'   $(SKETCHCPP_SRC) >  $@
	$(v)echo "#line 1 \"autogenerated\""                              >> $@
	$(v)echo "#include \"WProgram.h\""                                >> $@
	$(v)$(AWK)                '$(SKETCH_PROTOTYPER)' $(SKETCHCPP_SRC) >> $@
	$(v)$(AWK) -v mode=body   '$(SKETCH_SPLITTER)'   $(SKETCHCPP_SRC) >> $@

# delete the sketch.cpp file if a processing error occurs
.DELETE_ON_ERROR: $(SKETCHCPP)

#
# The sketch splitter is an awk script used to split off the
# header and body of the concatenated .pde files.  It also
# inserts #line directives to help in backtracking from compiler
# and debugger messages to the original source file.
#
# Note that # and $ require special treatment here to avoid upsetting
# make.
#
# This script requires BWK or GNU awk.
#
define SKETCH_SPLITTER
  BEGIN { 							\
    scanning = 1; 						\
    printing = (mode ~ "header") ? 1 : 0;			\
  }								\
  { toggles = 1 }						\
  (FNR == 1) && printing { 					\
    printf "#line %d \"%s\"\n", FNR, FILENAME;			\
  }								\
  /^[[:space:]]*\/\*/,/\*\// { 					\
    toggles = 0;						\
  }								\
  /^[[:space:]]*$$/ || /^[[:space:]]*\/\/.*/ || /^\#.*$$/ { 	\
    toggles = 0;						\
  }								\
  scanning && toggles { 					\
    scanning = 0; 						\
    printing = !printing;					\
    if (printing) { 						\
      printf "#line %d \"%s\"\n", FNR, FILENAME;		\
    }								\
  }								\
  printing
endef

#
# The prototype scanner is an awk script used to generate function
# prototypes from the concantenated .pde files.
#
# Function definitions are expected to follow the form
#
#   <newline><type>[<qualifier>...]<name>([<arguments>]){
#
# with whitespace permitted between the various elements.  The pattern
# is assembled from separate subpatterns to make adjustments easier.
#
# Note that $ requires special treatment here to avoid upsetting make,
# and backslashes are doubled in the partial patterns to satisfy
# escaping rules.
#
# This script requires BWK or GNU awk.
#
define SKETCH_PROTOTYPER
  BEGIN {								\
    RS="{";								\
    type       = "((\\n)|(^))[[:space:]]*[[:alnum:]_]+[[:space:]]+";	\
    qualifiers = "([[:alnum:]_\\*&]+[[:space:]]*)*";			\
    name       = "[[:alnum:]_]+[[:space:]]*";				\
    args       = "\\([[:space:][:alnum:]_,&\\*\\[\\]]*\\)";		\
    bodycuddle = "[[:space:]]*$$";					\
    pattern    = type qualifiers name args bodycuddle;			\
  }									\
  match($$0, pattern) {							\
    proto = substr($$0, RSTART, RLENGTH);				\
    gsub("\n", " ", proto);						\
    printf "%s;\n", proto;						\
  }
endef
