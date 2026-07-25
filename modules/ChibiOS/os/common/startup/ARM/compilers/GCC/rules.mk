# ARM Cortex-Mx common makefile scripts and rules.

##############################################################################
# Processing options coming from the upper Makefile.
#

# Compiler options
OPT = $(USE_OPT)
COPT = $(USE_COPT)
CPPOPT = $(USE_CPPOPT)

# Garbage collection
ifeq ($(USE_LINK_GC),yes)
  OPT += -ffunction-sections -fdata-sections -fno-common
  LDOPT := ,--gc-sections
else
  LDOPT :=
endif

# Linker extra options
ifneq ($(USE_LDOPT),)
  LDOPT := $(LDOPT),$(USE_LDOPT)
endif

# Link time optimizations
ifneq ($(USE_LTO),no)
  ifeq ($(USE_LTO),yes)
    OPT += -flto=auto
  else
    OPT += -flto=$(USE_LTO)
  endif
endif

# Undefined state stack size
ifeq ($(USE_UND_STACKSIZE),)
  LDOPT := $(LDOPT),--defsym=__und_stack_size__=8
else
  LDOPT := $(LDOPT),--defsym=__und_stack_size__=$(USE_UND_STACKSIZE)
endif

# Abort stack size
ifeq ($(USE_ABT_STACKSIZE),)
  LDOPT := $(LDOPT),--defsym=__abt_stack_size__=8
else
  LDOPT := $(LDOPT),--defsym=__abt_stack_size__=$(USE_ABT_STACKSIZE)
endif

# FIQ stack size
ifeq ($(USE_FIQ_STACKSIZE),)
  LDOPT := $(LDOPT),--defsym=__fiq_stack_size__=64
else
  LDOPT := $(LDOPT),--defsym=__fiq_stack_size__=$(USE_FIQ_STACKSIZE)
endif

# IRQ stack size
ifeq ($(USE_IRQ_STACKSIZE),)
  LDOPT := $(LDOPT),--defsym=__irq_stack_size__=0x400
else
  LDOPT := $(LDOPT),--defsym=__irq_stack_size__=$(USE_IRQ_STACKSIZE)
endif

# Supervisor stack size
ifeq ($(USE_SUPERVISOR_STACKSIZE),)
  LDOPT := $(LDOPT),--defsym=__svc_stack_size__=8
else
  LDOPT := $(LDOPT),--defsym=__svc_stack_size__=$(USE_SUPERVISOR_STACKSIZE)
endif

# System stack size
ifeq ($(USE_SYSTEM_STACKSIZE),)
  LDOPT := $(LDOPT),--defsym=__sys_stack_size__=0x400
else
  LDOPT := $(LDOPT),--defsym=__sys_stack_size__=$(USE_SYSTEM_STACKSIZE)
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

OUTFILES = $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).hex \
           $(BUILDDIR)/$(PROJECT).bin $(BUILDDIR)/$(PROJECT).dmp \
           $(BUILDDIR)/$(PROJECT).list

# Source files groups and paths
ifeq ($(USE_THUMB),yes)
  TCSRC += $(CSRC)
  TCPPSRC += $(CPPSRC)
else
  ACSRC += $(CSRC)
  ACPPSRC += $(CPPSRC)
endif
ASRC      = $(ACSRC) $(ACPPSRC)
TSRC      = $(TCSRC) $(TCPPSRC)
SRCPATHS  = $(sort $(dir $(ASMXSRC)) $(dir $(ASMSRC)) $(dir $(ASRC)) $(dir $(TSRC)))

# Various directories
OBJDIR    = $(BUILDDIR)/obj
LSTDIR    = $(BUILDDIR)/lst

# Object files groups
ACOBJS    = $(addprefix $(OBJDIR)/, $(notdir $(ACSRC:.c=.o)))
#ACPPOBJS  = $(addprefix $(OBJDIR)/, $(notdir $(ACPPSRC:.cpp=.o)))
ACPPOBJS  := $(addprefix $(OBJDIR)/, $(notdir $(patsubst %.cpp, %.o, $(filter %.cpp, $(TCPPSRC)))))
ACCOBJS   := $(addprefix $(OBJDIR)/, $(notdir $(patsubst %.cc, %.o, $(filter %.cc, $(TCPPSRC)))))
TCOBJS    = $(addprefix $(OBJDIR)/, $(notdir $(TCSRC:.c=.o)))
#TCPPOBJS  = $(addprefix $(OBJDIR)/, $(notdir $(TCPPSRC:.cpp=.o)))
TCPPOBJS  := $(addprefix $(OBJDIR)/, $(notdir $(patsubst %.cpp, %.o, $(filter %.cpp, $(TCPPSRC)))))
TCCOBJS   := $(addprefix $(OBJDIR)/, $(notdir $(patsubst %.cc, %.o, $(filter %.cc, $(TCPPSRC)))))
ASMOBJS   = $(addprefix $(OBJDIR)/, $(notdir $(ASMSRC:.s=.o)))
ASMXOBJS  = $(addprefix $(OBJDIR)/, $(notdir $(ASMXSRC:.S=.o)))
#OBJS      = $(ASMXOBJS) $(ASMOBJS) $(ACOBJS) $(TCOBJS) $(ACPPOBJS) $(TCPPOBJS)
OBJS      = $(ASMXOBJS) $(ASMOBJS) $(ACOBJS) $(TCOBJS) $(ACPPOBJS) $(ACCOBJS) $(TCPPOBJS) $(TCOBJS)

# Paths
IINCDIR   = $(sort $(patsubst %,-I%,$(INCDIR) $(DINCDIR) $(UINCDIR)))
LLIBDIR   = $(sort $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR)))

# Macros
DEFS      = $(DDEFS) $(UDEFS)
ADEFS     = $(DADEFS) $(UADEFS)

# Libs
LIBS      = $(DLIBS) $(ULIBS)

# Various settings
MCFLAGS   = -mcpu=$(MCU)
ODFLAGS   = -x --syms
ASFLAGS   = $(MCFLAGS) $(OPT) -Wa,-amhls=$(LSTDIR)/$(notdir $(<:.s=.lst)) $(ADEFS)
ASXFLAGS  = $(MCFLAGS) $(OPT) -Wa,-amhls=$(LSTDIR)/$(notdir $(<:.S=.lst)) $(ADEFS)
CFLAGS    = $(MCFLAGS) $(OPT) $(COPT) $(CWARN) -Wa,-alms=$(LSTDIR)/$(notdir $(<:.c=.lst)) $(DEFS)
CPPFLAGS  = $(MCFLAGS) $(OPT) $(CPPOPT) $(CPPWARN) -Wa,-alms=$(LSTDIR)/$(notdir $(<:.cpp=.lst)) $(DEFS)
LDFLAGS   = $(MCFLAGS) $(OPT) -nostartfiles $(LLIBDIR) -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch,--library-path=$(RULESPATH)/ld,--script=$(LDSCRIPT)$(LDOPT)

# Thumb interwork enabled only if needed because it kills performance.
ifneq ($(strip $(TSRC)),)
  CFLAGS   += -DTHUMB_PRESENT
  CPPFLAGS += -DTHUMB_PRESENT
  ASFLAGS  += -DTHUMB_PRESENT
  ASXFLAGS += -DTHUMB_PRESENT
  ifneq ($(strip $(ASRC)),)
    # Mixed ARM and THUMB mode.
    CFLAGS   += -mthumb-interwork
    CPPFLAGS += -mthumb-interwork
    ASFLAGS  += -mthumb-interwork
    ASXFLAGS += -mthumb-interwork
    LDFLAGS  += -mthumb-interwork
  else
    # Pure THUMB mode, THUMB C code cannot be called by ARM asm code directly.
    CFLAGS   += -mno-thumb-interwork -DTHUMB_NO_INTERWORKING
    CPPFLAGS += -mno-thumb-interwork -DTHUMB_NO_INTERWORKING
    ASFLAGS  += -mno-thumb-interwork -DTHUMB_NO_INTERWORKING -mthumb
    ASXFLAGS += -mno-thumb-interwork -DTHUMB_NO_INTERWORKING -mthumb
    LDFLAGS  += -mno-thumb-interwork -mthumb
  endif
else
  # Pure ARM mode
  CFLAGS   += -mno-thumb-interwork
  CPPFLAGS += -mno-thumb-interwork
  ASFLAGS  += -mno-thumb-interwork
  ASXFLAGS += -mno-thumb-interwork
  LDFLAGS  += -mno-thumb-interwork
endif

# Generate dependency information
ASFLAGS  += -MD -MP -MF $(DEPDIR)/$(@F).d
ASXFLAGS += -MD -MP -MF $(DEPDIR)/$(@F).d
CFLAGS   += -MD -MP -MF $(DEPDIR)/$(@F).d
CPPFLAGS += -MD -MP -MF $(DEPDIR)/$(@F).d

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

$(ACPPOBJS) : $(OBJDIR)/%.o : %.cpp $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CPPC) -c $(CPPFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CPPC) -c $(CPPFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
endif

$(TCPPOBJS) : $(OBJDIR)/%.o : %.cpp $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CPPC) -c $(CPPFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CPPC) -c $(CPPFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
endif

$(ACCOBJS) : $(OBJDIR)/%.o : %.cc $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CPPC) -c $(CPPFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CPPC) -c $(CPPFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
endif

$(TCCOBJS) : $(OBJDIR)/%.o : %.cc $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CPPC) -c $(CPPFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CPPC) -c $(CPPFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
endif

$(ACOBJS) : $(OBJDIR)/%.o : %.c $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) -c $(CFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CC) -c $(CFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
endif

$(TCOBJS) : $(OBJDIR)/%.o : %.c $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) -c $(CFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CC) -c $(CFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
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
