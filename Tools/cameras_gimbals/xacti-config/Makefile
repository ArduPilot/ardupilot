################################################################################
# Copyright (c) 2022, Xacti Corporation. All rights reserved.
# Modified by Randy Mackay
################################################################################

APP:= xacti-config
CC = g++

# Optimisation Options
# 
CFLAGOPT  = 
CFLAGOPT += -O0

SRCS:= $(wildcard *.c) $(wildcard *.cpp) 

INCS:= $(wildcard *.h)

PKGS:= x11

OBJS:= $(SRCS:.c=.o)
OBJS:= $(OBJS:.cpp=.o)

CFLAGS+= -g 
CFLAGS+= $(CFLAGOPT)

LIBS+= -lm \
       -Wl,-rpath \
	   -lpthread -pthread -luvc

CFLAGS+= `pkg-config --cflags $(PKGS)`

LIBS+= `pkg-config --libs $(PKGS)`

all: $(APP)

%.o: %.c $(INCS) Makefile
	$(CC) -c -o $@ $(CFLAGS) $<
%.o: %.cpp $(INCS) Makefile
	$(CC) -c -o $@ $(CFLAGS) $<

$(APP): $(OBJS) Makefile
	$(CC) -o $(APP) $(OBJS) $(LIBS)

clean:
	rm -rf $(OBJS) $(APP)

exec:
	./$(APP)
