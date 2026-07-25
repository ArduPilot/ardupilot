
CANARD_BASE=../..

CC=gcc
CFLAGS = -g -Wall -Idsdl_generated/include -I$(CANARD_BASE) -I$(CANARD_BASE)/drivers/linux

LIBS=$(CANARD_BASE)/canard.c -lm

# add socketcan and multicast drivers for linux
LIBS+=$(CANARD_BASE)/drivers/socketcan/socketcan.c
LIBS+=$(CANARD_BASE)/drivers/mcast/mcast.c
LIBS+=$(CANARD_BASE)/drivers/linux/linux.c

# add in generated code
LIBS+=dsdl_generated/src/uavcan.protocol.NodeStatus.c
LIBS+=dsdl_generated/src/uavcan.protocol.GetNodeInfo_res.c
LIBS+=dsdl_generated/src/uavcan.equipment.range_sensor.Measurement.c
LIBS+=dsdl_generated/src/uavcan.protocol.dynamic_node_id.Allocation.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.GetSet_req.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.GetSet_res.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_req.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_res.c

all: rangefinder

DSDL:
	git clone --depth 1 https://github.com/DroneCAN/DSDL

dronecan_dsdlc:
	git clone --depth 1 https://github.com/DroneCAN/dronecan_dsdlc

dsdl_generated: DSDL dronecan_dsdlc
	python3 dronecan_dsdlc/dronecan_dsdlc.py -O dsdl_generated DSDL/dronecan DSDL/uavcan DSDL/com DSDL/ardupilot

rangefinder: dsdl_generated rangefinder.c $(LIBS)
	$(CC) -o rangefinder rangefinder.c $(LIBS) $(CFLAGS)

clean:
	rm -rf rangefinder DSDL dsdl_generated dronecan_dsdlc
