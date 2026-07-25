
CXX ?= g++
CXXFLAGS ?= -std=c++11 -g -O0 -Wall -pthread
GENDIR ?= ../../include_v2.0
GTESTSRC ?= /usr/src/gtest/src/gtest-all.cc
MSGDEFDIR ?= ../../../../../message_definitions/v1.0

DIALECTS := $(notdir $(shell find $(GENDIR) -mindepth 1 -maxdepth 1 -type d))
TARGETS := $(foreach d,$(DIALECTS),mavtest_$(d))

all: run_tests

gtest-all.o: $(GTESTSRC)
	$(CXX) $(CXXFLAGS) -I"$(dir $(GTESTSRC))../" -c -o $@ $<

mavtest_%: mtest.cpp gtest-all.o $(foreach d,$(DIALECTS),$(GENDIR)/$(d)/$(d).hpp)
	$(CXX) $(CXXFLAGS) -I $(GENDIR)/$* -o $@ $< gtest-all.o

run_tests: $(TARGETS)
	for f in $(TARGETS); do \
		./$$f; \
	done

gen: $(wildcard $(MSGDEFDIR)/*.xml)
	mkdir -p $(GENDIR)
	for d in $^; do	\
		../../../../tools/mavgen.py -o $(GENDIR) --wire-protocol 2.0 --lang C $$d;	\
		../../../../tools/mavgen.py -o $(GENDIR) --wire-protocol 2.0 --lang C++11 $$d;	\
	done

clean:
	rm -f gtest-all.o $(TARGETS)
