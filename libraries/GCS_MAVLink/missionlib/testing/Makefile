VERSION = 1.00
CC      = gcc
CFLAGS  = -I ../../include/common -Wall -Werror -DVERSION=\"$(MAVLINK_VERSION)\" -std=c99
LDFLAGS = ""

OBJ = udp.o

udptest: $(OBJ)
	$(CC) $(CFLAGS) -o udptest $(OBJ) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $<