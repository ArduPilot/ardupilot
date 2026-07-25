# List of all the Posix platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/simulator/posix/hal_lld.c \
              ${CHIBIOS}/os/hal/ports/simulator/posix/hal_serial_lld.c \
              ${CHIBIOS}/os/hal/ports/simulator/console.c \
              ${CHIBIOS}/os/hal/ports/simulator/hal_pal_lld.c \
              ${CHIBIOS}/os/hal/ports/simulator/hal_efl_lld.c \
              ${CHIBIOS}/os/hal/ports/simulator/hal_st_lld.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/simulator/posix \
              ${CHIBIOS}/os/hal/ports/simulator

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
