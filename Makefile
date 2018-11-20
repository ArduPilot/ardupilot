ROOT = $(dir $(lastword $(MAKEFILE_LIST)))

WAF_BINARY = $(realpath $(ROOT)/modules/waf/waf-light)
WAF = $(WAF_BINARY) $(WAF_FLAGS)

EXPLICIT_COMMANDS = check check-all clean list_boards

VEHICLES = copter plane rover

all: $(WAF_BINARY)
	@$(WAF) build

$(WAF_BINARY):
	@git submodule init && git submodule update

waf-%: $(WAF_BINARY)
	@$(WAF) $*

%-configure: $(WAF_BINARY)
	@$(WAF) configure --board $*

$(EXPLICIT_COMMANDS): $(WAF_BINARY)
	@$(WAF) $@

$(VEHICLES): $(WAF_BINARY)
	@echo Build for vehicle $@
	@$(WAF) $@

.DEFAULT: %-configure
	@$(WAF) configure --board $@ build

help:
	@echo "Ardupilot Building"
	@echo "=================="
	@echo "This is a make wrapper for Ardupilot's Waf build system. This wrapper is"
	@echo "intended to provide convenience for basic and common build tasks. If you need"
	@echo "more than what this wrapper provides, it's a good idea to use waf directly."
	@echo "The waf executable is at '$(WAF_BINARY)'."
	@echo ""
	@echo "WARNING: Ardupilot's Waf build system is still a work in progress, it's still"
	@echo "missing features from the current official build system."
	@echo ""
	@echo "Note: The examples in this help consider this wrapper being named as Makefile"
	@echo "or makefile. In a Linux environment, alias make='make -f THIS_MAKEFILE'"
	@echo "should make them work."
	@echo ""
	@echo "Boards"
	@echo "------"
	@echo "In order to trigger the build for a board/platform, the name of the board is"
	@echo "used as the target. Example: make linux"
	@echo "If no target is passed, then the build will be triggered for the last board"
	@echo "used. You can suffix the board/platform with '-configure' in order to just "
	@echo "configure without triggering a build command."
	@echo ""
	@echo "Vehicles"
	@echo "--------"
	@echo "It's possible to build for a specific vehicle by defining the target as one of:"
	@echo "    $(VEHICLES)"
	@echo ""
	@echo "Not that if it's your first time building or you want to change the target "
	@echo "board/platform, you'll need to configure the build before (e.g"
	@echo "make linux-configure)"
	@echo ""
	@echo "Check"
	@echo "-----"
	@echo "Check targets are used for running tests. There are two targets available:"
	@echo "    check:     for running tests that are still failing or that are new or"
	@echo "               have been modified"
	@echo "    check-all: to run all tests"
	@echo ""
	@echo "Waf commands"
	@echo "------------"
	@echo "Waf commands can be explicitly called with targets prefixed by 'waf-'. Example:"
	@echo "    make waf-clean"
	@echo "    make waf-build"
	@echo ""
	@echo "Common commands"
	@echo "---------------"
	@echo "Some Waf commands can be executed without the need of prefixing the target name"
	@echo "with 'waf-'. They are: $(EXPLICIT_COMMANDS)"
	@echo ""
	@echo "Waf flags"
	@echo "---------"
	@echo "The variable WAF_FLAGS can be used to set any waf command line options that"
	@echo "come to be necessary. Ex: make linux WAF_FLAGS='-c no'"
	@echo ""

# Don't run in parallel, let waf take care of that.
.NOTPARALLEL:
