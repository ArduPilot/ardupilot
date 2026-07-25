# List of all the LPC214x platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/LPC/LPC214x/hal_lld.c \
              ${CHIBIOS}/os/hal/ports/LPC/LPC214x/hal_pal_lld.c \
              ${CHIBIOS}/os/hal/ports/LPC/LPC214x/hal_serial_lld.c \
              ${CHIBIOS}/os/hal/ports/LPC/LPC214x/hal_spi_lld.c \
              ${CHIBIOS}/os/hal/ports/LPC/LPC214x/hal_st_lld.c \
              ${CHIBIOS}/os/hal/ports/LPC/LPC214x/vic.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/LPC/LPC214x

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
