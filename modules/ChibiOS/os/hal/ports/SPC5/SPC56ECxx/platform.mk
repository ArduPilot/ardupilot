# List of all the SPC56ECxx platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/SPC5/SPC56ECxx/hal_lld.c \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/EDMA_v1/spc5_edma.c \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/SIUL_v1/hal_pal_lld.c \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/LINFlex_v1/hal_serial_lld.c \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/DSPI_v1/hal_spi_lld.c \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/STM_v1/hal_st_lld.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/SPC5/SPC56ECxx \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/EDMA_v1 \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/SIUL_v1 \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/LINFlex_v1 \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/DSPI_v1 \
              ${CHIBIOS}/os/hal/ports/SPC5/LLD/STM_v1

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
