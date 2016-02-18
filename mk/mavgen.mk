
# mavlink header generation
MAVLINK_DIR := $(SKETCHBOOK)/modules/mavlink/
MESSAGE_DEFINITIONS := $(SKETCHBOOK)/modules/mavlink/message_definitions/v1.0
MAVLINK_HEADERS := $(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h $(wildcard $(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink/v1.0/,*.h) $(wildcard $(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink/v1.0/ardupilotmega,*.h)
MAVLINK_OUTPUT_DIR := $(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink/v1.0

$(MAVLINK_HEADERS): $(MESSAGE_DEFINITIONS)/ardupilotmega.xml $(MESSAGE_DEFINITIONS)/common.xml
	echo "Generating MAVLink headers..."
	#goto mavlink module directory and run the most recent generator script
	echo "Generating C code using mavgen.py located at" $(SKETCHBOOK)/modules/mavlink/
	-PYTHONPATH=$(MAVLINK_DIR) python $(MAVLINK_DIR)/pymavlink/tools/mavgen.py --lang=C --wire-protocol=1.0 --output=$(MAVLINK_OUTPUT_DIR) $(MAVLINK_DIR)/message_definitions/v1.0/ardupilotmega.xml
