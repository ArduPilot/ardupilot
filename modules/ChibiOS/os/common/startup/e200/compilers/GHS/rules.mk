# e200z common makefile scripts and rules.

##############################################################################
# Processing options coming from the upper Makefile.
#

# Compiler options
OPT = $(USE_OPT)
COPT = $(USE_COPT)
CPPOPT = $(USE_CPPOPT)

# Garbage collection
ifeq ($(USE_LINK_GC),yes)
  OPT += --no_commons
  LDOPT := -delete
endif

# Linker extra options
ifneq ($(USE_LDOPT),)
  LDOPT := $(LDOPT) $(USE_LDOPT)
endif

# Link time optimizations
ifeq ($(USE_LTO),yes)
  OPT += -Owholeprogram
endif

# VLE option handling.
ifeq ($(USE_VLE),yes)
  DDEFS += -DPPC_USE_VLE=1
  DADEFS += -DPPC_USE_VLE=1
  OPT += -vle
  COPT += -vle
else
  DDEFS += -DPPC_USE_VLE=0
  DADEFS += -DPPC_USE_VLE=0
endif

# Process stack size
ifeq ($(USE_PROCESS_STACKSIZE),)
  LDOPT := $(LDOPT) -C__process_stack_size__=0x400 #,--defsym=__process_stack_size__=0x400
else
  LDOPT := $(LDOPT) -C__process_stack_size__=$(USE_PROCESS_STACKSIZE) #,--defsym=__process_stack_size__=$(USE_PROCESS_STACKSIZE)
endif

# Exceptions stack size
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  LDOPT := $(LDOPT) -C__irq_stack_size__=0x400 #,--defsym=__irq_stack_size__=0x400
else
  LDOPT := $(LDOPT) -C__irq_stack_size__=$(USE_EXCEPTIONS_STACKSIZE) #,--defsym=__irq_stack_size__=$(USE_EXCEPTIONS_STACKSIZE)
endif

# Output directory and files
ifeq ($(BUILDDIR),)
  BUILDDIR = build
endif
ifeq ($(BUILDDIR),.)
  BUILDDIR = build
endif

OUTFILES = $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).hex \
           $(BUILDDIR)/$(PROJECT).mot $(BUILDDIR)/$(PROJECT).bin \
           $(BUILDDIR)/$(PROJECT).dmp

# Source files groups and paths
SRC       = $(CSRC)$(CPPSRC)
SRCPATHS  = $(sort $(dir $(ASMXSRC)) $(dir $(ASMSRC)) $(dir $(SRC)))

# Various directories
OBJDIR    = $(BUILDDIR)/obj
LSTDIR    = $(BUILDDIR)/lst

# Object files groups
COBJS     = $(addprefix $(OBJDIR)/, $(notdir $(CSRC:.c=.o)))
CPPOBJS   = $(addprefix $(OBJDIR)/, $(notdir $(CPPSRC:.cpp=.o)))
ASMOBJS   = $(addprefix $(OBJDIR)/, $(notdir $(ASMSRC:.s=.o)))
ASMXOBJS  = $(addprefix $(OBJDIR)/, $(notdir $(ASMXSRC:.S=.o)))
OBJS      = $(ASMXOBJS) $(ASMOBJS) $(COBJS) $(CPPOBJS)

# Paths
IINCDIR   = $(patsubst %,-I%,$(INCDIR) $(DINCDIR) $(UINCDIR))
LLIBDIR   = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

# Macros
DEFS      = $(DDEFS) $(UDEFS)
ADEFS     = $(DADEFS) $(UADEFS)

# Libs
LIBS      = $(DLIBS) $(ULIBS)

# Various settings
WARN      = --incorrect_pragma_warnings --unknown_pragma_warnings --prototype_warnings --diag_error 236
LIST      = -list -tmp=$(OBJDIR)  

MCFLAGS   = -cpu=$(MCU)

ODFLAGS   = -ysec -full

ASFLAGS   = $(MCFLAGS) $(OPT) $(LIST) -list=$(LSTDIR)/$(notdir $(<:.s=.lst))  $(ADEFS) $(WARN)  -preprocess_assembly_files
ASXFLAGS  = $(MCFLAGS) $(OPT) $(LIST) -list=$(LSTDIR)/$(notdir $(<:.S=.lst))  $(ADEFS) $(WARN)  -preprocess_assembly_files    
CFLAGS    = $(MCFLAGS) $(OPT) $(COPT)   $(CWARN)   $(LIST) -list=$(LSTDIR)/$(notdir $(<:.c=.lst))   $(DEFS)               
CPPFLAGS  = $(MCFLAGS) $(OPT) $(CPPOPT) $(CPPWARN) $(LIST) -list=$(LSTDIR)/$(notdir $(<:.cpp=.lst)) $(DEFS)                                

LDFLAGS  := $(MCFLAGS) $(OPT) -nostartfiles $(LLIBDIR) 
LDFLAGS  += -map=$(BUILDDIR)/$(PROJECT).map
LDFLAGS  += -Mx
LDFLAGS  += $(LDOPT)
LDFLAGS  += $(LDSCRIPT)

# Generate dependency information
ASFLAGS  += -MD
ASXFLAGS += -MD 
CFLAGS   += -MD
CPPFLAGS += -MD

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

$(CPPOBJS) : $(OBJDIR)/%.o : %.cpp $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CPPC) -c $(CPPFLAGS) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CPPC) -c $(CPPFLAGS) -I. $(IINCDIR) $< -o $@
endif

$(COBJS) : $(OBJDIR)/%.o : %.c $(MAKEFILE_LIST)
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
	$(CC) -c $(ASXFLAGS) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CC) -c $(ASXFLAGS) -I. $(IINCDIR) $< -o $@
endif

%.elf: $(OBJS) $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(LD) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
else
	@echo Linking $@
	@$(LD) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
endif

%.hex: %.elf $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(HEX) $< -o $@
else
	@echo Creating $@
	@$(HEX) $< -o $@
endif

%.mot: %.elf $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(MOT) $< -o $@
else
	@echo Creating $@
	@$(MOT) $< -o $@
endif

%.bin: %.elf $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(BIN) $< -o $@
else
	@echo Creating $@
	@$(BIN) $< -o $@
endif

%.dmp: %.elf $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(OD) $(ODFLAGS) $< > $@
else
	@echo Creating $@
	@$(OD) $(ODFLAGS) $< > $@
	@echo
#	@$(SZ) -addr -file -hex $<
#	@echo
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
