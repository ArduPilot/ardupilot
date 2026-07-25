##############################################################################
# Multi-project makefile rules
#

SUB_MAKES := $(sort $(wildcard make/*.make))

all:
	@for m in $(SUB_MAKES); do \
		echo; \
		echo "=== Building $$m ==="; \
		$(MAKE) --no-print-directory -f $$m all || exit $$?; \
		echo; \
	done
	@echo

clean:
	@for m in $(SUB_MAKES); do \
		echo "=== Cleaning $$m ==="; \
		$(MAKE) --no-print-directory -f $$m clean || exit $$?; \
		echo; \
	done
#
##############################################################################
