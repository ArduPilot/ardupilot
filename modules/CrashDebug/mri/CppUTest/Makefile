#Set this to @ to keep the makefile quiet
SILENCE = @

#--- Inputs ----#
COMPONENT_NAME = CppUTest
ifeq ($(CPPUTEST_USE_STD_C_LIB), N)
	CPP_PLATFORM = GccNoStdC
else
	CPP_PLATFORM = Gcc
endif
CPPUTEST_HOME = .
OLD_MAKE = oldmake

CPPUTEST_ENABLE_DEBUG = Y

SRC_DIRS = \
	src/CppUTest \
	src/Platforms/$(CPP_PLATFORM)
	
TEST_SRC_DIRS = \
	tests

INCLUDE_DIRS =\
  include

include $(CPPUTEST_HOME)/build/MakefileWorker.mk

#these are a sample of the other alternative flag settings
.PHONY: test_all
test_all: start test_old_make
	@echo Building with the default flags.
	make clean
	$(TIME) make 
	./$(TEST_TARGET) -r
	make clean
	@echo Building with the STDC++ new disabled. 
	$(TIME) make CPPUTEST_USE_STD_CPP_LIB=Y extensions 
	make CPPUTEST_USE_STD_CPP_LIB=Y cleanExtensions
	@echo Building with Memory Leak Detection disabled
	$(TIME) make CPPUTEST_USE_MEM_LEAK_DETECTION=N extensions
	make CPPUTEST_USE_MEM_LEAK_DETECTION=N cleanExtensions
	@echo Building with Memory Leak Detection disabled and STD C++ disabled
	$(TIME) make CPPUTEST_USE_MEM_LEAK_DETECTION=N CPPUTEST_USE_STD_CPP_LIB=Y extensions
	make CPPUTEST_USE_MEM_LEAK_DETECTION=N CPPUTEST_USE_STD_CPP_LIB=Y cleanExtensions
	@echo Building with debug disabled
	$(TIME) make CPPUTEST_ENABLE_DEBUG=N extensions
	make CPPUTEST_ENABLE_DEBUG=N cleanExtensions
	@echo Building with overridden CXXFLAGS and CFLAGS and CPPFLAGS
	$(TIME) make CLFAGS="" CXXFLAGS="" CPPFLAGS="-Iinclude"
	make CFLAGS="" CXXFLAGS="" clean
	@echo Building without Standard C library includes
	$(TIME) make CPPUTEST_USE_STD_C_LIB=N all_no_tests
	make CPPUTEST_USE_STD_C_LIB=N clean
	@echo Building with overridden CXXFLAGS and CFLAGS and memory leak and STDC++ disabled
	$(TIME) make CLFAGS="" CXXFLAGS="" CPPFLAGS="-Iinclude -DCPPUTEST_STD_CPP_LIB_DISABLED -DCPPUTEST_MEM_LEAK_DETECTION_DISABLED"
	make CFLAGS="" CXXFLAGS="" CPPFLAGS="-DCPPUTEST_STD_CPP_LIB_DISABLED -DCPPUTEST_MEM_LEAK_DETECTION_DISABLED" clean
	@echo Building CppUTest with Google Test 
	$(SILENCE) if [ -z $$GTEST_HOME ]; then  \
	   echo "******** WARNING: Can't test with CPPUTEST_USE_REAL_GTEST because can't find Google Test -- no GTEST_HOME set ********"; \
	   else $(TIME) make extensions CPPUTEST_USE_REAL_GTEST=Y; fi
	@echo Building CppUTest with Google Mock 
	$(SILENCE) if [ -z $$GMOCK_HOME ]; then  \
	   echo "******** WARNING: Can't test with CPPUTEST_USE_REAL_GMOCK because can't find Google Mock -- no GMOCK_HOME set ********"; \
	   else $(TIME) make extensions CPPUTEST_USE_REAL_GMOCK=Y; fi
	@echo Building examples 
	make cleanExamples
	$(TIME) make examples
	make cleanExamples
	@echo Testing JUnit output
	$(TIME) make
	$(SILENCE)./$(TEST_TARGET) -ojunit > junit_run_output
	$(SILENCE)if [ -s junit_run_output ]; then echo "JUnit run has output. Build failed!"; exit 1; fi
	make clean
	make CPPUTEST_MAP_FILE=map.txt
	make clean
	@echo Testing GCOV usage
	$(TIME) make CPPUTEST_USE_GCOV=Y everythingInstall
	make gcov
	make -f Makefile_CppUTestExt gcov
	make -C examples gcov
	make cleanEverythingInstall
	@echo Testing VPATH usage
	$(TIME) make CPPUTEST_USE_GCOV=Y CPPUTEST_USE_VPATH=Y everythingInstall
	make CPPUTEST_USE_VPATH=Y gcov
	make CPPUTEST_USE_VPATH=Y -f Makefile_CppUTestExt gcov
	make CPPUTEST_USE_VPATH=Y -C examples gcov	
	make clean
	@echo Testing VPATH usage
	$(TIME) make CPPUTEST_USE_VPATH=Y everythingInstall
	make CPPUTEST_USE_VPATH=Y cleanEverythingInstall
	make flags
	make debug
	
HAS_OLD_MAKE = $(shell $(OLD_MAKE) -v 2>/dev/null)

test_old_make:
	$(SILENCE)if [ "$(HAS_OLD_MAKE)" = "" ]; then \
		echo "Old make with the name $(OLD_MAKE) not found. Skipping testing with old make version"; \
	else \
		$(OLD_MAKE) -f Makefile_for_old_make clean && \
		$(OLD_MAKE) -f Makefile_for_old_make && \
		$(OLD_MAKE) -f Makefile_for_old_make extensions && \
		$(OLD_MAKE) -f Makefile_for_old_make clean; \
	fi

.PHONY: examples
examples: $(TEST_TARGET) extensions
	$(TIME) make -C examples  all CPPUTEST_USE_STD_CPP_LIB=$(CPPUTEST_USE_STD_CPP_LIB) CPPUTEST_USE_REAL_GTEST=$(CPPUTEST_USE_REAL_GTEST) CPPUTEST_USE_MEM_LEAK_DETECTION=$(CPPUTEST_USE_MEM_LEAK_DETECTION)

extensions: $(TEST_TARGET)
	$(TIME) make -f Makefile_CppUTestExt all CPPUTEST_USE_STD_CPP_LIB=$(CPPUTEST_USE_STD_CPP_LIB) CPPUTEST_USE_REAL_GTEST=$(CPPUTEST_USE_REAL_GTEST) CPPUTEST_USE_MEM_LEAK_DETECTION=$(CPPUTEST_USE_MEM_LEAK_DETECTION)

cleanExtensions: clean
	$(TIME) make -f Makefile_CppUTestExt clean CPPUTEST_USE_STD_CPP_LIB=$(CPPUTEST_USE_STD_CPP_LIB) CPPUTEST_USE_REAL_GTEST=$(CPPUTEST_USE_REAL_GTEST) CPPUTEST_USE_MEM_LEAK_DETECTION=$(CPPUTEST_USE_MEM_LEAK_DETECTION)
	
cleanExamples: clean cleanExtensions
	$(TIME) make -C examples clean CPPUTEST_USE_STD_CPP_LIB=$(CPPUTEST_USE_STD_CPP_LIB) CPPUTEST_USE_REAL_GTEST=$(CPPUTEST_USE_REAL_GTEST) CPPUTEST_USE_MEM_LEAK_DETECTION=$(CPPUTEST_USE_MEM_LEAK_DETECTION)

.PHONY: everythingInstall
everythingInstall: all extensions examples

.PHONY: cleanEverythingInstall
cleanEverythingInstall: clean cleanExtensions cleanExamples

