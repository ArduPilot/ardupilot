all: rangefinder.stripped rangefinder.lst

CC=clpru
LD=lnkpru
AR=arpru
OBJDUMP=dispru
STRIP=strippru
HEX=hexpru

CFLAGS= --endian=little -v3 -s -al --c99 --gcc -O3 --printf_support=minimal --symdebug:none

LDFLAGS=-cr --diag_warning=225 -lAM335x_PRU.cmd -x

STRIPFLAGS=-p

OBJS1:=rangefinder.obj

%.obj: %.c
	$(CC) $(CFLAGS) -c $< -ea=.s

%.obj: %.asm
	$(CC) $(CFLAGS) -c $<

rangefinder: $(OBJS1)
	$(CC) $(CFLAGS) $^ -q -z $(LDFLAGS) -o $@

rangefinder.stripped: rangefinder
	$(STRIP) $(STRIPFLAGS) $< -o $@
	size rangefinder.stripped
	$(HEX) rangefinder.stripped HexUtil_PRU.cmd

rangefinder.lst: rangefinder
	$(OBJDUMP) -1 $< > $@

install:
	cp rangefinderprutext.bin /lib/firmware
	cp rangefinderprudata.bin /lib/firmware

clean:
	rm -f \
		rangefinder rangefinder.asm rangefinder.s \
		*.bin *.obj *.lst *.out *.stripped \
