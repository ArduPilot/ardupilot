# List of all the Win32 platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/simulator/win32/hal_lld.c \
              ${CHIBIOS}/os/hal/ports/simulator/win32/hal_serial_lld.c \
              ${CHIBIOS}/os/hal/ports/simulator/console.c \
              ${CHIBIOS}/os/hal/ports/simulator/hal_pal_lld.c \
              ${CHIBIOS}/os/hal/ports/simulator/hal_st_lld.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/simulator/win32 \
              ${CHIBIOS}/os/hal/ports/simulator

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
