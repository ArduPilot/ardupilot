
# minimal makefile setup for ARM linux targets

CC=arm-linux-gnueabihf-gcc
CFLAGS=-Wall -fPIC -g -shared

all: moduletest.so

moduletest.so: moduletest.c
	$(CC) $(CFLAGS) -o moduletest.so moduletest.c -ldl

clean:
	rm -f moduletest.so
