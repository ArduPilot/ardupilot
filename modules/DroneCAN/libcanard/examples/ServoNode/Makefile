
CANARD_BASE=../..

CC=gcc
CFLAGS = -g -Wall -Idsdl_generated/include -I$(CANARD_BASE) -I$(CANARD_BASE)/drivers/linux

LIBS=$(CANARD_BASE)/canard.c

# add socketcan driver for linux
LIBS+=$(CANARD_BASE)/drivers/socketcan/socketcan.c
LIBS+=$(CANARD_BASE)/drivers/mcast/mcast.c
LIBS+=$(CANARD_BASE)/drivers/linux/linux.c

# add in generated code
LIBS+=dsdl_generated/src/uavcan.protocol.NodeStatus.c
LIBS+=dsdl_generated/src/uavcan.protocol.GetNodeInfo_res.c
LIBS+=dsdl_generated/src/uavcan.protocol.dynamic_node_id.Allocation.c
LIBS+=dsdl_generated/src/uavcan.equipment.actuator.ArrayCommand.c
LIBS+=dsdl_generated/src/uavcan.equipment.actuator.Status.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.GetSet_req.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.GetSet_res.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_req.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_res.c
LIBS+=dsdl_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_req.c
LIBS+=dsdl_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_res.c
LIBS+=dsdl_generated/src/uavcan.protocol.file.Read_req.c
LIBS+=dsdl_generated/src/uavcan.protocol.file.Read_res.c

all: servo_node

DSDL:
	git clone --depth 1 https://github.com/DroneCAN/DSDL

dronecan_dsdlc:
	git clone --depth 1 https://github.com/DroneCAN/dronecan_dsdlc

dsdl_generated: DSDL dronecan_dsdlc
	python3 dronecan_dsdlc/dronecan_dsdlc.py -O dsdl_generated DSDL/dronecan DSDL/uavcan DSDL/com DSDL/ardupilot

servo_node: dsdl_generated servo_node.c $(LIBS)
	$(CC) -o servo_node servo_node.c $(LIBS) $(CFLAGS)

clean:
	rm -rf servo_node DSDL dsdl_generated dronecan_dsdlc
