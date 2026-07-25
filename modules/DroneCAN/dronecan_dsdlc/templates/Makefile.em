@{from dronecan_dsdlc_helpers import *}@
INC=-I../include
INC+=-I@(get_canard_inc())
INC+=-I.
CXX=g++
CC=gcc
CFLAGS=-Wall -Wextra -Werror -std=c99 -DCANARD_DSDLC_TEST_BUILD $(INC)
CXXFLAGS=-Wall -Wextra -Werror -std=c++11 -DCANARD_DSDLC_TEST_BUILD $(INC)
LDFLAGS=-lstdc++

# get test name from Makefile name
@[    if msg.kind == msg.KIND_SERVICE]@
TEST=@(msg.full_name)_@(msg_kind)
@[    else]@
TEST=@(msg.full_name)
@[    end if]@
TEST_SRC:=test_$(TEST).cpp
# get sources
SRCS:=$(wildcard ../src/*.c) @(get_canard_src())
# generate object files
COBJS:=$(SRCS:.c=.o)
CXXOBJS+=$(TEST_SRC:.cpp=.o)

all: $(TEST)

$(TEST): $(COBJS) $(CXXOBJS)
	$(CXX) $(CXXFLAGS) -o $@@ $(COBJS) $(CXXOBJS) $(LDFLAGS)

$(COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@@

$(CXXOBJS): %.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@@

.PHONY: clean


clean:
	rm -f $(TEST) $(CXXOBJS) $(COBJS)
