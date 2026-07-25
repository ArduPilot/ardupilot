This directory contains the ChibiOS/RT "official" bindings with the FatFS
library by ChaN: http://elm-chan.org

In order to use FatFS within ChibiOS/RT project:
1. unzip FatFS under ./ext/fatfs  [See Note 2]
2. include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk in your makefile.
3. Add $(FATFSSRC) to $(CSRC)
4. Add $(FATFSINC) to $(INCDIR)

Note:
1. These files modified for use with version 0.13 of fatfs.
2. In the original distribution, the source directory is called 'source' rather than 'src'
