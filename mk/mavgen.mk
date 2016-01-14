
# mavlink header generation
MAVLINK_DIR := $(SKETCHBOOK)/modules/mavlink/
MESSAGE_DEFINITIONS := $(SKETCHBOOK)/modules/mavlink/message_definitions/v1.0
MAVLINK_HEADERS := $(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink/v1.0/

.PHONY: MAV_GEN

MAV_GEN: $(MAVLINK_HEADERS) CHECK_MODULES

$(MAVLINK_HEADERS): $(MESSAGE_DEFINITIONS)/ardupilotmega.xml $(MESSAGE_DEFINITIONS)/common.xml
	echo "Generating MAVLink headers..."
	#goto mavlink module directory and run the most recent generator script
	echo "Generating C code using mavgen.py located at" $(SKETCHBOOK)/modules/mavlink/
	PYTHONPATH=$(MAVLINK_DIR) python $(MAVLINK_DIR)/pymavlink/tools/mavgen.py --lang=C --wire-protocol=1.0 --output=$@ $(MAVLINK_DIR)/message_definitions/v1.0/ardupilotmega.xml