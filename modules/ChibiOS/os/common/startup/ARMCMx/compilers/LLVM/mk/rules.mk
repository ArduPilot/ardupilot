# ARM Cortex-Mx common makefile scripts and rules.

##############################################################################
# Processing options coming from the upper Makefile.
#

# Compiler options
OPT    := $(USE_OPT)
COPT   := $(USE_COPT)
CPPOPT := $(USE_CPPOPT)

# Garbage collection
ifeq ($(USE_LINK_GC),yes)
  OPT   += -ffunction-sections -fdata-sections -fno-common
  LDOPT := ,--gc-sections
else
  LDOPT :=
endif

# Linker extra options
ifneq ($(USE_LDOPT),)
  LDOPT := $(LDOPT),$(USE_LDOPT)
endif

# Link time optimizations
ifeq ($(USE_LTO),yes)
  OPT += -flto
endif

# FPU options default (Cortex-M4 and Cortex-M7 single precision).
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16
endif

# FPU-related options
ifeq ($(USE_FPU),)
  USE_FPU = no
endif
ifneq ($(USE_FPU),no)
  OPT    += $(USE_FPU_OPT)
  DDEFS  += -DCORTEX_USE_FPU=TRUE
  DADEFS += -DCORTEX_USE_FPU=TRUE
else
  OPT    += -mfloat-abi=soft
  DDEFS  += -DCORTEX_USE_FPU=FALSE
  DADEFS += -DCORTEX_USE_FPU=FALSE
endif

# Process stack size
ifeq ($(USE_PROCESS_STACKSIZE),)
  LDOPT := $(LDOPT),--defsym=__process_stack_size__=0x400
else
  LDOPT := $(LDOPT),--defsym=__process_stack_size__=$(USE_PROCESS_STACKSIZE)
endif

# Exceptions stack size
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  LDOPT := $(LDOPT),--defsym=__main_stack_size__=0x400
else
  LDOPT := $(LDOPT),--defsym=__main_stack_size__=$(USE_EXCEPTIONS_STACKSIZE)
endif

# Output directory and files
ifeq ($(BUILDDIR),)
  BUILDDIR = build
endif
ifeq ($(BUILDDIR),.)
  BUILDDIR = build
endif

# Dependencies directory
ifeq ($(DEPDIR),)
  DEPDIR = .dep
endif
ifeq ($(DEPDIR),.)
  DEPDIR = .dep
endif

OUTFILES := $(BUILDDIR)/$(PROJECT).elf \
            $(BUILDDIR)/$(PROJECT).hex \
            $(BUILDDIR)/$(PROJECT).bin \
            $(BUILDDIR)/$(PROJECT).dmp \
            $(BUILDDIR)/$(PROJECT).list

ifdef SREC
  OUTFILES += $(BUILDDIR)/$(PROJECT).srec
endif

# Source files groups and paths
TCSRC     += $(CSRC)
TCPPSRC   += $(CPPSRC)
TSRC      := $(TCSRC) $(TCPPSRC)
SRCPATHS  := $(sort $(dir $(ASMXSRC)) $(dir $(ASMSRC)) $(dir $(TSRC)))

# Various directories
OBJDIR    := $(BUILDDIR)/obj
LSTDIR    := $(BUILDDIR)/lst

# Object files groups
TCOBJS    := $(addprefix $(OBJDIR)/, $(notdir $(TCSRC:.c=.o)))
#TCPPOBJS  := $(addprefix $(OBJDIR)/, $(notdir $(TCPPSRC:.cpp=.o)))
TCPPOBJS  := $(addprefix $(OBJDIR)/, $(notdir $(patsubst %.cpp, %.o, $(filter %.cpp, $(TCPPSRC)))))
TCCOBJS   := $(addprefix $(OBJDIR)/, $(notdir $(patsubst %.cc, %.o, $(filter %.cc, $(TCPPSRC)))))
ASMOBJS   := $(addprefix $(OBJDIR)/, $(notdir $(ASMSRC:.s=.o)))
ASMXOBJS  := $(addprefix $(OBJDIR)/, $(notdir $(ASMXSRC:.S=.o)))
#OBJS      := $(ASMXOBJS) $(ASMOBJS) $(ACOBJS) $(TCOBJS) $(ACPPOBJS) $(TCPPOBJS)
OBJS      := $(ASMXOBJS) $(ASMOBJS) $(ACOBJS) $(TCOBJS) $(ACPPOBJS) $(TCPPOBJS) $(TCCOBJS)

# Macros
DEFS      := $(DDEFS) $(UDEFS)
ADEFS     := $(DADEFS) $(UADEFS)

# Libs
LIBS      := $(DLIBS) $(ULIBS)

# Various settings
MCFLAGS   := -mcpu=$(MCU) -mthumb
ODFLAGS   = -x --syms
ASFLAGS   = $(MCFLAGS) $(OPT) $(ADEFS)
ASXFLAGS  = $(MCFLAGS) $(OPT) $(ADEFS)
CFLAGS    = $(MCFLAGS) $(OPT) $(COPT) $(CWARN) $(DEFS)
CPPFLAGS  = $(MCFLAGS) $(OPT) $(CPPOPT) $(CPPWARN) $(DEFS)

ARM_CORTEXM_SYSROOT := \
	$(shell $(GCC_CC) $(CFLAGS) -print-sysroot 2>&1)
ARM_CORTEXM_MULTI_DIR := \
	$(shell $(GCC_CC) $(CFLAGS) -print-multi-directory 2>&1)
ARM_CORTEXM_BUILTINS := \
	$(shell $(GCC_CC) $(CFLAGS) -print-libgcc-file-name 2>&1)

ASFLAGS   += --target=arm-none-eabi --sysroot=$(ARM_CORTEXM_SYSROOT)
ASXFLAGS  += --target=arm-none-eabi --sysroot=$(ARM_CORTEXM_SYSROOT)
CFLAGS    += --target=arm-none-eabi --sysroot=$(ARM_CORTEXM_SYSROOT)
CPPFLAGS  += --target=arm-none-eabi --sysroot=$(ARM_CORTEXM_SYSROOT)

# Generate dependency information
ASFLAGS  += -MD -MP -MF $(DEPDIR)/$(@F).d
ASXFLAGS += -MD -MP -MF $(DEPDIR)/$(@F).d
CFLAGS   += -MD -MP -MF $(DEPDIR)/$(@F).d
CPPFLAGS += -MD -MP -MF $(DEPDIR)/$(@F).d

# Paths
IINCDIR   := $(patsubst %,-I%,$(INCDIR) $(DINCDIR) $(UINCDIR))
LLIBDIR   := $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR) $(ARM_CORTEXM_SYSROOT)/lib/$(ARM_CORTEXM_MULTI_DIR))
LDFLAGS   = $(MCFLAGS) $(OPT) -nostartfiles $(LLIBDIR) $(ARM_CORTEXM_BUILTINS) -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch,--library-path=$(STARTUPLD),--script=$(LDSCRIPT)$(LDOPT)

# Paths where to search for sources
VPATH     = $(SRCPATHS)

#
# Makefile rules
#

all: PRE_MAKE_ALL_RULE_HOOK $(OBJS) $(OUTFILES) POST_MAKE_ALL_RULE_HOOK

PRE_MAKE_ALL_RULE_HOOK:

POST_MAKE_ALL_RULE_HOOK:

$(OBJS): | PRE_MAKE_ALL_RULE_HOOK $(BUILDDIR) $(OBJDIR) $(LSTDIR) $(DEPDIR)

$(BUILDDIR):
ifneq ($(USE_VERBOSE_COMPILE),yes)
	@echo Compiler Options
	@echo $(CC) -c $(CFLAGS) -I. $(IINCDIR) main.c -o main.o
	@echo
endif
	@mkdir -p $(BUILDDIR)

$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(LSTDIR):
	@mkdir -p $(LSTDIR)

$(DEPDIR):
	@mkdir -p $(DEPDIR)

$(TCPPOBJS) : $(OBJDIR)/%.o : %.cpp $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CPPC) -c $(CPPFLAGS) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CPPC) -c $(CPPFLAGS) -I. $(IINCDIR) $< -o $@
endif

$(TCCOBJS) : $(OBJDIR)/%.o : %.cc $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CPPC) -c $(CPPFLAGS) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CPPC) -c $(CPPFLAGS) -I. $(IINCDIR) $< -o $@
endif

$(TCOBJS) : $(OBJDIR)/%.o : %.c $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@
endif

$(ASMOBJS) : $(OBJDIR)/%.o : %.s $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(AS) -c $(ASFLAGS) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(AS) -c $(ASFLAGS) -I. $(IINCDIR) $< -o $@
endif

$(ASMXOBJS) : $(OBJDIR)/%.o : %.S $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) -c $(ASXFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CC) -c $(ASXFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
endif

$(BUILDDIR)/$(PROJECT).elf: $(OBJS) $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(LD) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
else
	@echo Linking $@
	@$(LD) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
endif

%.hex: %.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(HEX) $< $@
else
	@echo Creating $@
	@$(HEX) $< $@
endif

%.bin: %.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(BIN) $< $@
else
	@echo Creating $@
	@$(BIN) $< $@
endif

%.srec: %.elf
ifdef SREC
  ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(SREC) $< $@
  else
	@echo Creating $@
	@$(SREC) $< $@
  endif
endif

%.dmp: %.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(OD) $(ODFLAGS) $< > $@
	$(SZ) $<
else
	@echo Creating $@
	@$(OD) $(ODFLAGS) $< > $@
	@echo
	@$(SZ) $<
endif

%.list: %.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(OD) -S $< > $@
else
	@echo Creating $@
	@$(OD) -S $< > $@
	@echo
	@echo Done
endif

lib: $(OBJS) $(BUILDDIR)/lib$(PROJECT).a

$(BUILDDIR)/lib$(PROJECT).a: $(OBJS)
	@$(AR) -r $@ $^
	@echo
	@echo Done

clean: CLEAN_RULE_HOOK
	@echo Cleaning
	@echo - $(DEPDIR)
	@-rm -fR $(DEPDIR)/* $(BUILDDIR)/* 2>/dev/null
	@-if [ -d "$(DEPDIR)" ]; then rmdir -p --ignore-fail-on-non-empty $(subst ./,,$(DEPDIR)) 2>/dev/null; fi
	@echo - $(BUILDDIR)
	@-if [ -d "$(BUILDDIR)" ]; then rmdir -p --ignore-fail-on-non-empty $(subst ./,,$(BUILDDIR)) 2>/dev/null; fi
	@echo
	@echo Done

CLEAN_RULE_HOOK:

#
# Include the dependency files, should be the last of the makefile
#
-include $(wildcard $(DEPDIR)/*)

# *** EOF ***
