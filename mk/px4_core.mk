############################################################################
#
#   Copyright (C) 2012 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# Common Makefile for nsh command modules and utility libraries; should be 
# included by the module-specific Makefile.
#
# To build an app that appears as an nsh external command, the caller 
# must define:
#
# LIBNAME	- the name of the library, defaults to the name of the
#		  directory
#
# The calling makefile may also set:
#
# ASRCS		- list of assembly source files, defaults to all .S 
#		  files in the directory
#
# CSRCS		- list of C source files, defaults to all .c files
#		  in the directory
#
# CXXSRCS	- list of C++ source files, defaults to all .cpp
#		  files in the directory
#
# INCLUDES	- list of directories to be added to the include
#		  search path
#
# PRIORITY	- thread priority for the command (defaults to 
#		  SCHED_PRIORITY_DEFAULT)
#
# STACKSIZE	- stack size for the command (defaults to 4096)
#
# Symbols in the module are private to the module unless deliberately exported
# using the __EXPORT tag.
#

############################################################################
# No user-serviceable parts below
############################################################################



# build the variables to emulate Arduino.mk build system
SRCROOT	:= $(realpath $(dir $(firstword $(MAKEFILE_LIST))))
SKETCH  := $(lastword $(subst /, ,$(SRCROOT)))

# this is where we put the generated cpp file, created from the *.pde/*.ino
TMPDIR			?=	/tmp
BUILDROOT		:=	$(abspath $(TMPDIR)/$(SKETCH).build)

HARDWARE		?=	px4
AWK			?=	gawk

SRCSUFFIXES		=	*.cpp *.c *.S

SKETCHPDESRCS		:=	$(wildcard $(SRCROOT)/*.pde $(SRCROOT)/*.ino)
SKETCHSRCS		:=	$(wildcard $(addprefix $(SRCROOT)/,$(SRCSUFFIXES)))
SKETCHPDE		:=	$(wildcard $(SRCROOT)/$(SKETCH).pde $(SRCROOT)/$(SKETCH).ino)
SKETCHCPP		:=	$(BUILDROOT)/$(SKETCH).cpp
ifneq ($(words $(SKETCHPDE)),1)
$(error ERROR: sketch $(SKETCH) must contain exactly one of $(SKETCH).pde or $(SKETCH).ino)
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
SYSTYPE			:=	$(shell uname)
SEXPR			=	's/^[[:space:]]*\#include[[:space:]][<\"]([^>\"./]+).*$$/\1/p'
ifeq ($(SYSTYPE),Darwin)
  LIBTOKENS        :=    $(sort $(shell cat $(SKETCHPDESRCS) $(SKETCHSRCS) | sed -nEe $(SEXPR)))
else
  LIBTOKENS        :=    $(sort $(shell cat $(SKETCHPDESRCS) $(SKETCHSRCS) | sed -nre $(SEXPR)))
endif

#
# Find sketchbook libraries referenced by the sketch.
#
# Include paths for sketch libraries 
#
SKETCHBOOK=$(MK_DIR)/..
SKETCHLIBS		:=	$(wildcard $(addprefix $(SKETCHBOOK)/libraries/,$(LIBTOKENS)))
SKETCHLIBNAMES		:=	$(notdir $(SKETCHLIBS))
SKETCHLIBSRCDIRS	:=	$(SKETCHLIBS) $(addsuffix /utility,$(SKETCHLIBS))
SKETCHLIBSRCS		:=	$(wildcard $(foreach suffix,$(SRCSUFFIXES),$(addsuffix /$(suffix),$(SKETCHLIBSRCDIRS))))
SKETCHLIBOBJS		:=	$(addsuffix .o,$(basename $(subst $(SKETCHBOOK),$(BUILDROOT),$(SKETCHLIBSRCS))))
SKETCHLIBINCLUDES	:=	$(addprefix -I,$(SKETCHLIBS))

# add sketch libs to includes
INCLUDES	+= $(SKETCHLIBS)

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)

# common header for rules, prints what is being built
define RULEHDR
	@echo %% $(subst $(BUILDROOT)/,,$@)
	@mkdir -p $(dir $@)
endef

############################################################################
# Work out who included us so we can report decent errors
#
THIS_MAKEFILE	:= $(lastword $(MAKEFILE_LIST))
PARENT_MAKEFILE	:= $(lastword $(filter-out $(THIS_MAKEFILE),$(MAKEFILE_LIST)))

############################################################################
# Get configuration
#
-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# this allows us to generate the main function
SKETCHFLAGS     = -DCONFIG_HAL_BOARD=HAL_BOARD_PX4 -DSKETCHNAME=\"$(SKETCH)\" -DSKETCH_MAIN=$(SKETCH)_main
CFLAGS		+= $(SKETCHFLAGS)
CXXFLAGS	+= $(SKETCHFLAGS)

############################################################################
# Sanity-check the information we've been given and set any defaults
#
SRCDIR		?= $(dir $(PARENT_MAKEFILE))
PRIORITY	?= SCHED_PRIORITY_DEFAULT
STACKSIZE	?= 4096

INCLUDES	+= $(APPDIR)

ASRCS		?= $(wildcard $(SRCDIR)/*.S)
CSRCS		?= $(wildcard $(SRCDIR)/*.c)
CXXSRCS		?= $(wildcard $(SRCDIR)/*.cpp) $(SKETCHCPP)
PDESRCS		?= $(wildcard $(SRCDIR)/*.pde)

APPNAME          = $(SKETCH)

# there has to be a source file
ifeq ($(ASRCS)$(CSRCS)$(CXXSRCS)$(PDESRCS),)
$(error $(realpath $(PARENT_MAKEFILE)): at least one of ASRCS, CSRCS, CXXSRCS or PDESRCS must be set)
endif

# check that C++ is configured if we have C++ source files and we are building
ifneq ($(CXXSRCS),)
ifneq ($(CONFIG_HAVE_CXX),y)
ifeq ($(MAKECMDGOALS),build)
$(error $(realpath $(PARENT_MAKEFILE)): cannot set CXXSRCS if CONFIG_HAVE_CXX not set in configuration)
endif
endif
endif

############################################################################
# Adjust compilation flags to implement EXPORT
#
CFLAGS		+= -fsigned-char -fvisibility=hidden -include $(APPDIR)/systemlib/visibility.h
CXXFLAGS	+= -fsigned-char -fvisibility=hidden -include $(APPDIR)/systemlib/visibility.h

############################################################################
# Add extra include directories
#
CFLAGS		+= $(addprefix -I,$(INCLUDES))
CXXFLAGS	+= $(addprefix -I,$(INCLUDES))

############################################################################
# Things we are going to build
#

SRCS		 = $(ASRCS) $(CSRCS) $(CXXSRCS)
AOBJS		 = $(patsubst %.S,%.o,$(ASRCS))
COBJS		 = $(patsubst %.c,%.o,$(CSRCS))
CXXOBJS		 = $(patsubst %.cpp,%.o,$(CXXSRCS))
OBJS		 = $(AOBJS) $(COBJS) $(CXXOBJS) $(SKETCHLIBOBJS)

# The prelinked object that we are ultimately going to build
ifneq ($(APPNAME),)
PRELINKOBJ	 = $(APPNAME).pre.o
else
PRELINKOBJ	 = $(LIBNAME).pre.o
endif

# The archive that the object file will be placed in
# XXX does WINTOOL ever get set?
ifeq ($(WINTOOL),y)
  INCDIROPT	= -w
  BIN		 = "$(shell cygpath -w  $(APPDIR)/libapps$(LIBEXT))"
else
  BIN		 = "$(APPDIR)/libapps$(LIBEXT)"
endif

############################################################################
# Rules for building things
#

all:		.built
.PHONY:		clean depend distclean

#
# Top-level build; add prelinked object to the apps archive
#
.built:		$(PRELINKOBJ)
	$(RULEHDR)
	@$(call ARCHIVE, $(BIN), $(PRELINKOBJ))
	@touch $@

#
# Source dependencies
#
depend:		.depend
.depend:	$(MAKEFILE_LIST) $(SRCS)
	@$(MKDEP) --dep-path . $(CC) -- $(CFLAGS) -- $(CSRCS) >Make.dep
	@$(MKDEP) --dep-path . $(CXX) -- $(CXXFLAGS) -- $(CXXSRCS) >>Make.dep
	@touch $@

ifneq ($(APPNAME),)
#
# App registration
#
context:	.context
.context:	$(MAKEFILE_LIST)
	$(call REGISTER,$(APPNAME),$(PRIORITY),$(STACKSIZE),$(APPNAME)_main)
	@touch $@
else
context:
endif

#
# Object files
#
$(PRELINKOBJ):	$(OBJS)
	$(call PRELINK, $@, $(OBJS))

$(AOBJS): %.o : %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %.o : %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS): %.o : %.cpp
	$(call COMPILEXX, $<, $@)

#
# Tidying up
#
clean:
	@rm -f $(OBJS) $(PRELINKOBJ) Make.dep .built
	$(call CLEAN)

distclean:	clean
	@rm -f Make.dep .depend


#
# Build the sketch.cpp file
#
# This process strives to be as faithful to the Arduino implementation as
# possible.  Conceptually, the process is as follows:
#
# * All of the .pde/.ino files are concatenated, starting with the file named 
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
	$(v)$(AWK)                '$(SKETCH_PROTOTYPER)' $(SKETCHCPP_SRC) >> $@
	$(v)$(AWK) -v mode=body   '$(SKETCH_SPLITTER)'   $(SKETCHCPP_SRC) >> $@


# delete the sketch.cpp file if a processing error occurs
.DELETE_ON_ERROR: $(SKETCHCPP)

#
# The sketch splitter is an awk script used to split off the
# header and body of the concatenated .pde/.ino files.  It also
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
# prototypes from the concantenated .pde/.ino files.
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

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.cpp
	$(RULEHDR)
	$(call COMPILEXX, $<, $@)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.c
	$(RULEHDR)
	$(call COMPILE, $<, $@)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.S
	$(RULEHDR)
	$(call ASSEMBLE, $<, $@)

-include Make.dep
