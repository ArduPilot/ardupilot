# RT Shell files.
STREAMSSRC = $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
             $(CHIBIOS)/os/hal/lib/streams/chscanf.c \
             $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
             $(CHIBIOS)/os/hal/lib/streams/nullstreams.c \
             $(CHIBIOS)/os/hal/lib/streams/bufstreams.c

STREAMSINC = $(CHIBIOS)/os/hal/lib/streams

# Shared variables
ALLCSRC += $(STREAMSSRC)
ALLINC  += $(STREAMSINC)
