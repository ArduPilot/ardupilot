# top level makefile to build SITL for primary vehicle targets. 
# Useful for static analysis tools

all: sitl

sitl: TARGET=sitl
sitl: plane copter rover sub antennatracker hau

linux: TARGET=linux
linux: plane copter rover sub antennatracker hau

clean: TARGET=clean
clean: plane copter rover sub antennatracker hau

.PHONY: plane copter rover sub antennatracker hau

plane:
	$(MAKE) -C ArduPlane $(TARGET)

copter:
	$(MAKE) -C ArduCopter $(TARGET)

rover:
	$(MAKE) -C APMrover2 $(TARGET)

sub:
	$(MAKE) -C ArduSub $(TARGET)

antennatracker:
	$(MAKE) -C AntennaTracker $(TARGET)
	
hau:
	$(MAKE) -C ArduHAU $(TARGET)
