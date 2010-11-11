
PROG	:=	BinCommTest
SRCS	:= 	test.cpp ../APM_BinComm.cpp
OBJS	:=	$(SRCS:.cpp=.o)

BinCommTest:	$(OBJS)
	/Volumes/Data/Users/msmith/llvm/bin/clang++ -g -o $@ $(OBJS)

.cpp.o:
	@echo C++ $< -> $@
	/Volumes/Data/Users/msmith/llvm/bin/clang++ -g -c -I. -o $@ $<

clean:
	rm $(PROG) $(OBJS)
