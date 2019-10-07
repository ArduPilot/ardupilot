#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME:=ardurover
CDIR:=$(dir $(lastword $(MAKEFILE_LIST)))
BOARD:=$(shell cat $(abspath $(CDIR)..)/board.txt)
BUILD_DIR_BASE:=$(abspath $(CDIR)../../../../build/$(BOARD)/idf-rover)
SDKCONFIG:=$(abspath $(CDIR)..)/sdkconfig
SDKCONFIG_DEFAULTS:=$(abspath $(CDIR)..)/sdkconfig.defaults

ifeq (,$(BOARD))
$(error Please run 'waf configure' first)
endif

include $(IDF_PATH)/make/project.mk

showinc:
	echo INCLUDES: $(COMPONENT_INCLUDES)

