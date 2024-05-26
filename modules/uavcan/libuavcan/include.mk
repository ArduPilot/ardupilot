#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

LIBUAVCAN_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

UAVCAN_DIR := $(LIBUAVCAN_DIR)../

#
# Library sources
#
LIBUAVCAN_SRC := $(shell find $(LIBUAVCAN_DIR)src -type f -name '*.cpp')

LIBUAVCAN_INC := $(LIBUAVCAN_DIR)include

#
# DSDL compiler executable
#
LIBUAVCAN_DSDLC := $(LIBUAVCAN_DIR)dsdl_compiler/libuavcan_dsdlc

#
# Standard DSDL definitions
#
UAVCAN_DSDL_DIR := $(UAVCAN_DIR)dsdl/uavcan
