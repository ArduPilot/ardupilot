##############################################################################
# Rust compiler rules
#

# Compiler executable
RUSTC := rustc

# Compiler options, from makefile
RUSTOPT := $(USE_RUSTOPT)

# Rust static flags
RUSTFLAGS = --emit=obj --target thumbv7em-none-eabihf $(RUSTOPT)

# Rust objects
RUSTOBJS := $(addprefix $(BUILDDIR)/obj/, $(notdir $(RUSTSRC:.rs=.o)))

# Adding to the set of objects to be linked
OBJS     += $(RUSTOBJS)

# Adding to the set of source paths
SRCPATHS += $(sort $(dir $(RUSTSRC)))

$(RUSTOBJS) :$(BUILDDIR)/obj/%.o : %.rs $(MAKEFILE_LIST)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	unset MAKEFLAGS MFLAGS MAKELEVEL ; $(RUSTC) $(RUSTFLAGS) $< -o $@
else
	@echo Compiling $(<F)
	@unset MAKEFLAGS MFLAGS MAKELEVEL ; $(RUSTC) $(RUSTFLAGS) $< -o $@
endif

#
# Rust compiler rules
##############################################################################
