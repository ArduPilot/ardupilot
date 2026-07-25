# Versions to build
DIRS := mbed1768-StdIO.mk \
        mbed1768-LocalFileSystem.mk \
        mbed11u24-StdIO.mk \
        mbed11u24-LocalFileSystem.mk \
        k64F-StdIO.mk

DIRSCLEAN := $(addsuffix .clean,$(DIRS))


# Set VERBOSE make variable to 1 to output all tool commands.
VERBOSE?=0
ifeq "$(VERBOSE)" "0"
Q=@
else
Q=
endif


# Rules
all: $(DIRS)

clean: $(DIRSCLEAN)

$(DIRS):
	@echo Building $@
	$(Q) $(MAKE) --no-print-directory -f $@ all

$(DIRSCLEAN): %.clean:
	$(Q) $(MAKE) --no-print-directory -f $* clean

.PHONY: all clean $(DIRS) $(DIRSCLEAN)
