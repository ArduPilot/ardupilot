#
#include $(SKETCHBOOK)/modules/uavcan/libuavcan/include.mk

ifeq ($(UAVCAN_DIRECTORY),)
UAVCAN_DIRECTORY := $(SKETCHBOOK)/modules/uavcan
endif

UAVCAN_GEN     := $(UAVCAN_DIRECTORY)/libuavcan/include/#$(BUILDROOT)/modules/uavcan/
UAVCAN_GEN_INC := $(UAVCAN_GEN)dsdlc_generated/
UAVCAN_HEADERS := $(UAVCAN_GEN_INC)/uavcan/Timestamp.hpp $(wildcard $(UAVCAN_GEN_INC),*.hpp)
UAVCAN_DSDL_MESSAGE_DEFINITIONS := $(wildcard $(UAVCAN_DIRECTORY)/dsdl/uavcan/,*.uavcan) # $(UAVCAN_DIRECTORY)/dsdl/uavcan/Timestamp.uavcan 

UAVCANgen: $(UAVCAN_HEADERS)

$(UAVCAN_HEADERS): $(UAVCAN_DSDL_MESSAGE_DEFINITIONS)
	@echo "Generating UAVCAN headers..."

#	PYTHONPATH=$(UAVCAN_DIR)libuavcan/dsdl_compiler/python $(UAVCAN_DIR)libuavcan/dsdl_compiler/setup.py build
#	PYTHONPATH=$(UAVCAN_DIR)libuavcan/dsdl_compiler/ python $(UAVCAN_DIR)libuavcan/dsdl_compiler/setup.py install

	PYTHONPATH=$(UAVCAN_DIRECTORY)/libuavcan/ python $(UAVCAN_DIRECTORY)/libuavcan/dsdl_compiler/libuavcan_dsdlc "$(UAVCAN_DIRECTORY)/dsdl/uavcan" -O"$(UAVCAN_GEN_INC)"
