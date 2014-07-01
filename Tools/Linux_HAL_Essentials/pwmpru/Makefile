#
# Makefile for PWM PRU program
# It is assumed that the PRU compiler environment has been set
#

.PHONY: all clean

all: pwmpru1.stripped pwmpru1.lst

CC=clpru
LD=lnkpru
AR=arpru
OBJDUMP=dispru
STRIP=strippru

# -v3				PRU version 3
# --c99 			C99 support
# --gcc 			Enable GCC extensions
# -O3				Optimization level maximum
# --printf_support=minimal 	Minimal printf
# -ppd				Generate dependencies *.pp
# -ppa				Continue after generating deps
# -DDEBUG			Enable debug
# CFLAGS= -v3 --c99 --gcc -O3 --printf_support=minimal -ppd -ppa -DDEBUG 
CFLAGS= -v3 -s -al -O3 --c99 --gcc --printf_support=minimal --symdebug:none

# -cr 				Link using RAM auto init model (loader assisted)
# -x				Reread libs until no unresolved symbols found
LDFLAGS=-cr --diag_warning=225 -llnk-am33xx.cmd -x

STRIPFLAGS=-p

OBJS1:=pwmpru1.obj

%.obj: %.c
	$(CC) $(CFLAGS) -c $< -ea=.s

%.obj: %.asm
	$(CC) $(CFLAGS) -c $<

pwmpru1: $(OBJS1)
	$(CC) $(CFLAGS) $^ -q -z $(LDFLAGS) -o $@

pwmpru1.stripped: pwmpru1
	$(STRIP) $(STRIPFLAGS) $< -o $@

pwmpru1.lst: pwmpru1
	$(OBJDUMP) -1 $< > $@

clean:
	rm -f \
		pwmpru1 pwmpru1.asm \
		*.obj *.lst *.out *.stripped \
		tags

distclean: clean
	rm -f *.pp

# include any generated deps
-include $(patsubst %.obj,%.pp,$(OBJS))
