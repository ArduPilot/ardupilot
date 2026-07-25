##############################################################################
# Compiler settings
#

TRGT   = llvm-
CC     = clang
GCC_CC = arm-none-eabi-gcc
CPPC   = clang++
LD     = $(GCC_CC)
CP     = $(TRGT)objcopy
AS     = $(TRGT)as -x assembler-with-cpp
AR     = $(TRGT)ar
OD     = $(TRGT)objdump
SZ     = $(TRGT)size
HEX    = $(CP) -O ihex
BIN    = $(CP) -O binary

#
# Compiler settings
##############################################################################
