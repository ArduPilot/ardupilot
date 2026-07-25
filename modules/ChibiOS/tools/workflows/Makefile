# Used by Jenkins for automation purposes to build projects, run tests,
# gather test results, build documentation.

ROOT := $(realpath ../..)

# Directories to store results of tests and docs
COVERAGE_RESULTS := $(ROOT)/coverage_results
DOC_RESULTS := $(ROOT)/doc_results
TEST_RESULTS := $(ROOT)/test_results

# Directory with auxiliary scripts
WORKFLOWS := $(ROOT)/tools/workflows

# Defines what targets to build in makefile based projects, the order matter.
TARGETS := all clean

# If set to true, use virtual environment for python dependencies. Otherwise,
# use system libraries.
USE_VENV := false

# Makefile based projects
makefile_dirs := $(addprefix $(ROOT)/,demos test testex testhal testrt os/sb/apps)
makefiles := $(shell $(WORKFLOWS)/find.py --makefile --make --no-overlaps \
			   $(makefile_dirs))

single_makefiles := $(filter-out %.make,$(makefiles))
multi_makefiles := $(filter %.make,$(makefiles))

# External dependencies
external := $(addprefix $(ROOT)/ext/,fatfs lwip pico-sdk wolfssl littlefs)

# Documents
docs := $(shell find $(ROOT)/doc -type f -name Doxyfile_html)

# Linter scripts
linters := $(shell find $(ROOT)/tools/style -type f -name "*.sh")

# Python virtual environment
venv := $(WORKFLOWS)/.venv
venv_activate := $(venv)/bin/activate
venv_python := $(venv)/bin/python

ifeq ($(USE_VENV),true)
	python := $(venv_python)
    makefile_venv := $(venv_activate)
else
	python := python
    makefile_venv :=
endif

.PHONY : all makefiles external docs info lint coverage clean venv
.PHONY : $(makefiles) $(linters) $(docs)

# Build makefile based projects
all : makefiles
makefiles : $(makefiles)
	$(python) $(WORKFLOWS)/report.py -t $(TEST_RESULTS) \
		-r $(TEST_RESULTS)/test_results

$(single_makefiles) : $(external) $(makefile_venv)
	@mkdir -p $(TEST_RESULTS)/$(subst $(ROOT)/,,$(@D))
	+$(python) $(WORKFLOWS)/make.py -C $(@D) -f $(@F) -p $(ROOT)/ \
		-r $(TEST_RESULTS) -s $(WORKFLOWS)/skip.yaml \
		-t $(WORKFLOWS)/target.yaml $(TARGETS)

$(multi_makefiles) : $(external) $(makefile_venv)
	@mkdir -p $(TEST_RESULTS)/$(subst $(ROOT)/,,$(@D))
	+$(python) $(WORKFLOWS)/make.py -C $(dir $(@D)) -f make/$(@F) -p $(ROOT)/ \
		-r $(TEST_RESULTS) -s $(WORKFLOWS)/skip.yaml \
		-t $(WORKFLOWS)/target.yaml $(TARGETS)

# External projects
external : $(external)
$(external) :
	7z x $@*.7z -o$(@D)

# Build documentation
docs : $(docs)
$(docs) : %Doxyfile_html:
	@mkdir -p $(DOC_RESULTS)
	cd $(@D) && doxygen $(@F)
	tar -czf $(DOC_RESULTS)/$(subst /,_,$(subst $(ROOT)/,,$(@D)))_html.tar.gz \
		--transform "s/^\./$(subst /,_,$(subst $(ROOT)/,,$(@D)))_html/g" \
		-C $(@D)/html .

# Linter checks
lint : $(linters)
	@! grep -r -m 1 ^error: $(TEST_RESULTS)/lint 2>&1 > /dev/null || ( \
		echo "Exiting because there are linter errors"; \
		exit 1)

$(linters) :
	@mkdir -p $(TEST_RESULTS)/lint
	cd $(ROOT)/tools/style && bash $(@F) 2>&1 | \
		tee $(TEST_RESULTS)/lint/$(basename $(@F)).log

# Print versions of used tools
info :
	@which make && make -v
	@echo
	@which ccache && ccache -V || echo "ccache not present"
	@echo
	@which arm-none-eabi-gcc && arm-none-eabi-gcc -v || \
		echo "arm-none-eabi-gcc not present"
	@echo
	@which avr-gcc && avr-gcc -v || echo "avr-gcc not present"
	@echo
	@which doxygen && doxygen -v || echo "doxygen not present"
	@echo
	@which dot && dot -V || echo "graphviz not present"
	@echo
	@which perl && perl -v || echo "perl not present"
	@echo
	@which python3 && python3 -V || echo "python3 not present"
	@echo
	@which rustup && rustup -V && rustup show -v || echo "rustup not present"
	@echo

# Prepare Python virtual environment
venv : $(venv_activate)
$(venv_activate) :
	python3 -mvenv $(venv)
	$(venv)/bin/pip install \
		-r $(WORKFLOWS)/requirements.txt \
		-c $(WORKFLOWS)/constraints.txt

# Generate test coverage report
coverage :
	@mkdir -p $(COVERAGE_RESULTS)
	gcovr -r $(ROOT) \
		--use-gcov-files --gcov-ignore-parse-errors \
		--xml-pretty --xml $(COVERAGE_RESULTS)/coverage.xml 

# Clean up results of tests and docs
clean :
	rm -rf $(TEST_RESULTS)
	rm -rf $(COVERAGE_RESULTS)
	rm -rf $(DOC_RESULTS)
	rm -rf $(external)
	rm -rf $(venv)
