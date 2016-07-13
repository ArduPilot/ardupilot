
# minimal makefile setup for ARM linux targets

COMPILER ?= arm-linux-gnueabihf-gcc
AP_MODULE_PATH="../../.."
CFLAGS=-Wall -fPIC -g -shared -I$(AP_MODULE_PATH)

all: moduletest.so

moduletest.so: moduletest.c
	$(COMPILER) $(CFLAGS) -o moduletest.so moduletest.c -ldl

clean:
	rm -f moduletest.so
