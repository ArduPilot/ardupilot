
PROG	:=	BinCommTest
SRCS	:= 	test.cpp ../APM_BinComm.cpp
OBJS	:=	$(SRCS:.cpp=.o)

BinCommTest:	$(OBJS)
	c++ -g -o $@ $(OBJS)

.cpp.o:
	@echo C++ $< -> $@
	c++ -g -c -I. -o $@ $<

clean:
	rm $(PROG) $(OBJS)
