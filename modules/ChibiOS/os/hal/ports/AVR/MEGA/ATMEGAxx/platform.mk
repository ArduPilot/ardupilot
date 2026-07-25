# List of all the AVR platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/AVR/MEGA/ATMEGAxx/hal_lld.c \

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/AVR/MEGA/ATMEGAxx

# Drivers compatible with the platform.
include $(CHIBIOS)/os/hal/ports/AVR/MEGA/LLD/ADCv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/MEGA/LLD/GPIOv2/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/MEGA/LLD/I2Cv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/MEGA/LLD/SPIv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/MEGA/LLD/SYSTICKv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/MEGA/LLD/TIMv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/MEGA/LLD/USARTv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/MEGA/LLD/USBv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
