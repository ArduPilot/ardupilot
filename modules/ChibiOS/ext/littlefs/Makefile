ifdef BUILDDIR
# make sure BUILDDIR ends with a slash
override BUILDDIR := $(BUILDDIR)/
# bit of a hack, but we want to make sure BUILDDIR directory structure
# is correct before any commands
$(if $(findstring n,$(MAKEFLAGS)),, $(shell mkdir -p \
	$(BUILDDIR) \
	$(BUILDDIR)bd \
	$(BUILDDIR)tests))
endif

# overridable target/src/tools/flags/etc
ifneq ($(wildcard test.c main.c),)
TARGET ?= $(BUILDDIR)lfs
else
TARGET ?= $(BUILDDIR)lfs.a
endif


CC ?= gcc
AR ?= ar
SIZE ?= size
CTAGS ?= ctags
NM ?= nm
LCOV ?= lcov

SRC ?= $(wildcard *.c)
OBJ := $(SRC:%.c=$(BUILDDIR)%.o)
DEP := $(SRC:%.c=$(BUILDDIR)%.d)
ASM := $(SRC:%.c=$(BUILDDIR)%.s)

ifdef DEBUG
override CFLAGS += -O0 -g3
else
override CFLAGS += -Os
endif
ifdef TRACE
override CFLAGS += -DLFS_YES_TRACE
endif
override CFLAGS += -I.
override CFLAGS += -std=c99 -Wall -pedantic
override CFLAGS += -Wextra -Wshadow -Wjump-misses-init -Wundef

ifdef VERBOSE
override TESTFLAGS += -v
override CODEFLAGS += -v
override COVERAGEFLAGS += -v
endif
ifdef EXEC
override TESTFLAGS += --exec="$(EXEC)"
endif
ifdef BUILDDIR
override TESTFLAGS += --build-dir="$(BUILDDIR:/=)"
override CODEFLAGS += --build-dir="$(BUILDDIR:/=)"
endif
ifneq ($(NM),nm)
override CODEFLAGS += --nm-tool="$(NM)"
endif


# commands
.PHONY: all build
all build: $(TARGET)

.PHONY: asm
asm: $(ASM)

.PHONY: size
size: $(OBJ)
	$(SIZE) -t $^

.PHONY: tags
tags:
	$(CTAGS) --totals --c-types=+p $(shell find -H -name '*.h') $(SRC)

.PHONY: code
code: $(OBJ)
	./scripts/code.py $^ $(CODEFLAGS)

.PHONY: test
test:
	./scripts/test.py $(TESTFLAGS)
.SECONDEXPANSION:
test%: tests/test$$(firstword $$(subst \#, ,%)).toml
	./scripts/test.py $@ $(TESTFLAGS)

.PHONY: coverage
coverage:
	./scripts/coverage.py $(BUILDDIR)tests/*.toml.info $(COVERAGEFLAGS)

# rules
-include $(DEP)
.SUFFIXES:

$(BUILDDIR)lfs: $(OBJ)
	$(CC) $(CFLAGS) $^ $(LFLAGS) -o $@

$(BUILDDIR)%.a: $(OBJ)
	$(AR) rcs $@ $^

$(BUILDDIR)%.o: %.c
	$(CC) -c -MMD $(CFLAGS) $< -o $@

$(BUILDDIR)%.s: %.c
	$(CC) -S $(CFLAGS) $< -o $@

# clean everything
.PHONY: clean
clean:
	rm -f $(TARGET)
	rm -f $(OBJ)
	rm -f $(DEP)
	rm -f $(ASM)
	rm -f $(BUILDDIR)tests/*.toml.*
