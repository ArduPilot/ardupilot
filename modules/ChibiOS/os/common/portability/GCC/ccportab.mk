# Compiler portability subsystem build.

# Required files.
ifndef CCPORTABSRC
  CCPORTABSRC =
  ALLCSRC += $(CCPORTABSRC)
endif

# Required include directories
ifndef CCPORTABINC
  CCPORTABINC = ${CHIBIOS}/os/common/portability/GCC
  ALLINC += $(CCPORTABINC)
endif
