ROOT=../../

all: DSDL build

test: DSDL build
	@echo "Building and running test suite"
	python3 $(ROOT)/dronecan_dsdlc.py -O dsdlc_generated DSDL/uavcan DSDL/dronecan DSDL/com --run-test

build:
	@echo "Generating headers from DSDL"
	python3 $(ROOT)/dronecan_dsdlc.py -O dsdlc_generated DSDL/uavcan DSDL/dronecan DSDL/com

DSDL:
	@echo Downloading latest DSDL
	wget https://github.com/DroneCAN/DSDL/zipball/master -O DSDL-master.zip
	unzip -q DSDL-master.zip
	mv dronecan-DSDL-* DSDL
	rm -f DSDL-master.zip

clean:
	rm -rf DSDL dsdlc_generated
