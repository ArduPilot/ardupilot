# List of files for HTTPD-to-FatFS bindings.
HTTPDFATFSSRC = $(CHIBIOS)/os/various/httpd_fatfs_bindings/httpd_fatfs.c

HTTPDFATFSNC  = $(CHIBIOS)/os/various/httpd_fatfs_bindings

# Shared variables
ALLCSRC += $(HTTPDFATFSSRC)
ALLINC  += $(HTTPDFATFSNC)
