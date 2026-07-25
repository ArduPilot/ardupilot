# AVR common makefile scripts and rules.

##############################################################################
# Processing options coming from the upper Makefile.
#

# Compiler options
OPT    := $(USE_OPT)
COPT   := $(USE_COPT)
CPPOPT := $(USE_CPPOPT)

# Output directory and files
ifeq ($(BUILDDIR),)
  BUILDDIR = build
endif
ifeq ($(BUILDDIR),.)
  BUILDDIR = build
endif
OUTFILES := $(BUILDDIR)/$(PROJECT).elf \
            $(BUILDDIR)/$(PROJECT).hex \
            $(BUILDDIR)/$(PROJECT).bin \
            $(BUILDDIR)/$(PROJECT).eep \
            $(BUILDDIR)/$(PROJECT).lss \
            $(BUILDDIR)/$(PROJECT).sym

ifdef SREC
  OUTFILES += $(BUILDDIR)/$(PROJECT).srec
endif

# Source files groups and paths
ASRC      := $(CSRC) $(CPPSRC)
SRCPATHS  := $(sort $(dir $(ASMXSRC)) $(dir $(ASMSRC)) $(dir $(ASRC)))

# Various directories
OBJDIR    := $(BUILDDIR)/obj
LSTDIR    := $(BUILDDIR)/lst

# Object files groups
ACOBJS    := $(addprefix $(OBJDIR)/, $(notdir $(CSRC:.c=.o)))
ACPPOBJS  := $(addprefix $(OBJDIR)/, $(notdir $(CPPSRC:.cpp=.o)))
ASMOBJS   := $(addprefix $(OBJDIR)/, $(notdir $(ASMSRC:.s=.o)))
ASMXOBJS  := $(addprefix $(OBJDIR)/, $(notdir $(ASMXSRC:.S=.o)))
OBJS	  := $(ASMXOBJS) $(ASMOBJS) $(ACOBJS) $(ACPPOBJS)

# Paths
IINCDIR   := $(patsubst %, -I%,$(INCDIR) $(DINCDIR) $(UINCDIR))
LLIBDIR   := $(patsubst %, -L%,$(DLIBDIR) $(ULIBDIR))
LLIBDIR   += -L$(dir $(LDSCRIPT))

# Macros
DEFS      := $(DDEFS) $(UDEFS)
ADEFS 	  := $(DADEFS) $(UADEFS)

# Libs
LIBS      := $(DLIBS) $(ULIBS)

# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu11

# Place -D or -U options here for C sources
CDEFS = -DF_CPU=$(F_CPU)UL

# Place -D or -U options here for ASM sources
ADEFS = -DF_CPU=$(F_CPU)

# Place -D or -U options here for C++ sources
CPPDEFS = -DF_CPU=$(F_CPU)UL

# Paths where to search for sources
VPATH     = $(SRCPATHS)

# Various settings for C flags.
MCFLAGS := -mmcu=$(MCU)
CFLAGS  = $(MCFLAGS) -I. -gdwarf-2 $(CDEFS) $(OPT) -funsigned-char
CFLAGS  += -funsigned-bitfields -fpack-struct -fshort-enums $(CWARN)
CFLAGS  += -Wa,-adhlns=$(LSTDIR)/$(notdir $(<:.c=.lst)) $(DEFS)
CFLAGS  += -std=gnu11 -mrelax -fdata-sections -ffunction-sections
CFLAGS  += -Wundef -MMD -MP #-MF

# Various settings for CPP flags.
CPPFLAGS = $(MCFLAGS) -I. -gdwarf-2 $(CDEFS) $(OPT) -funsigned-char
CPFLAGS  += -funsigned-bitfields -fpack-struct -fshort-enums $(CWARN)
CPFLAGS  += -Wa,-adhlns=$(LSTDIR)/$(notdir $(<:.c=.lst)) $(DEFS)
CPFLAGS  += -std=gnu++11 -mrelax -fdata-sections -ffunction-sections
CPFLAGS  += -Wundef -MMD -MP #-MF

#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -adhlns:   create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
#  -listing-cont-lines: Sets the maximum number of continuation lines of hex
#       dump that will be displayed for a given single line of source input.
ASFLAGS = $(ADEFS) -Wa,-adhlns=$(<:%.S=$(OBJDIR)/%.lst),-gstabs, \
          --listing-cont-lines=100

#---------------- Library Options ----------------
# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

# If this is left blank, then it will use the Standard printf version.
PRINTF_LIB = $(PRINTF_LIB_MIN)
#PRINTF_LIB = $(PRINTF_LIB_MIN)
#PRINTF_LIB = $(PRINTF_LIB_FLOAT)

# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

# If this is left blank, then it will use the Standard scanf version.
SCANF_LIB = $(SCANF_LIB_MIN)
#SCANF_LIB = $(SCANF_LIB_MIN)
#SCANF_LIB = $(SCANF_LIB_FLOAT)

MATH_LIB = -lm

#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file

#LDFLAGS = -Wl,-Map=$(TARGET).map,--cref,--gc-sections
#LDFLAGS += $(EXTMEMOPTS)
#LDFLAGS += $(patsubst %,-L%,$(EXTRALIBDIRS))
#LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)

LDFLAGS = -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--gc-sections
LDFLAGS += -Wl,-u,vfprintf -lprintf_min -Wl,-u,vfscanf -lscanf_min -lm

# Linker extra options
ifneq ($(USE_LDOPT),)
  LDFLAGS += -Wl,$(USE_LDOPT)
endif

#
# Makefile rules
#

all: PRE_MAKE_ALL_RULE_HOOK $(OBJS) $(OUTFILES) POST_MAKE_ALL_RULE_HOOK

PRE_MAKE_ALL_RULE_HOOK:

POST_MAKE_ALL_RULE_HOOK:

$(OBJS): | $(BUILDDIR) $(OBJDIR) $(LSTDIR)

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

$(ACPPOBJS) : $(OBJDIR)/%.o : %.cpp Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CPPC) -c $(CPPFLAGS) $(MOPT) $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CPPC) -c $(CPPFLAGS) $(MOPT) $(IINCDIR) $< -o $@
endif

$(ACOBJS) : $(OBJDIR)/%.o : %.c  Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) -c $(CFLAGS) $(MOPT) $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CC) -c $(CFLAGS) $(MOPT) $(IINCDIR) $< -o $@
endif

$(ASMOBJS) : $(OBJDIR)/%.o : %.s Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(AS) -c $(ASFLAGS) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(AS) -c $(ASFLAGS) -I. $(IINCDIR) $< -o $@
endif

$(ASMXOBJS) : $(OBJDIR)/%.o : %.S Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) -c $(ASXFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@$(CC) -c $(ASXFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
endif

$(BUILDDIR)/$(PROJECT).elf: $(OBJS)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) $(CFLAGS) $^ --output $@ $(LDFLAGS)
else
	@echo Linking $@
	@$(CC) $(CFLAGS) $^ --output $@ $(LDFLAGS)
endif

%.hex: %.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(HEX) $< $@
else
	@echo Creating $@
	@$(HEX) $< $@
endif

%.bin: %.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(BIN) $< $@
else
	@echo Creating $@
	@$(BIN) $< $@
endif

%.eep: %.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	-$(CP) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 --no-change-warnings -O $(FORMAT) \
  $< $@ || exit 0
else
	@echo Creating $@
	@-$(CP) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 --no-change-warnings -O $(FORMAT) \
  $< $@ || exit 0
endif

%.lss: %.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(OD) -h -S $< > $@
else
	@echo Creating $@
	@$(OD) -h -S $< > $@
endif

%.sym: %.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(NM) -n $< > $@
	$(SZ) $<
	@echo
	@echo Done
else
	@echo Creating $@
	@$(NM) -n $< > $@
	@echo
	@$(SZ) $<
	@echo
	@echo Done
endif

lib: $(OBJS) $(BUILDDIR)/lib$(PROJECT).a

$(BUILDDIR)/lib$(PROJECT).a: $(OBJS)
	@$(AR) $@ $^
	@echo
	@echo Done

clean: CLEAN_RULE_HOOK
	@echo Cleaning
	-rm -fR .dep $(BUILDDIR)
	@echo
	@echo Done

CLEAN_RULE_HOOK:

#
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
