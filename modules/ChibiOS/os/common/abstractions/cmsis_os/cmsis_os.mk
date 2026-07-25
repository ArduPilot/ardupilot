# List of the ChibiOS/RT CMSIS RTOS wrapper.
CMSISRTOSSRC = ${CHIBIOS}/os/common/abstractions/cmsis_os/cmsis_os.c
 
CMSISRTOSINC = ${CHIBIOS}/os/common/abstractions/cmsis_os

# Shared variables
ALLCSRC += $(CMSISRTOSSRC)
ALLINC  += $(CMSISRTOSINC)
