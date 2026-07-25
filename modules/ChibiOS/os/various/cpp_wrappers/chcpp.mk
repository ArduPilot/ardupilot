# C++ wrapper files.
CHCPPSRC   =

CHCPPSRCPP = $(CHIBIOS)/os/various/cpp_wrappers/ch.cpp

CHCPPINC   = $(CHIBIOS)/os/various/cpp_wrappers

# Shared variables
ALLCSRC   += $(CHCPPSRC)
ALLCPPSRC += $(CHCPPSRCPP)
ALLINC    += $(CHCPPINC)
