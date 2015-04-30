#include "MsgHandler_MSG.h"
#include <AP_AHRS.h>
#include <VehicleType.h>

void MsgHandler_MSG::process_message(uint8_t *msg)
{
    const uint8_t msg_text_len = 64;
    char msg_text[msg_text_len];
    require_field(msg, "Message", msg_text, msg_text_len);

    if (strncmp(msg_text, "ArduPlane", strlen("ArduPlane")) == 0) {
	vehicle = VehicleType::VEHICLE_PLANE;
	::printf("Detected Plane\n");
	ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
	ahrs.set_fly_forward(true);
    } else if (strncmp(msg_text, "ArduCopter", strlen("ArduCopter")) == 0 ||
	       strncmp(msg_text, "APM:Copter", strlen("APM:Copter")) == 0) {
	vehicle = VehicleType::VEHICLE_COPTER;
	::printf("Detected Copter\n");
	ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);
	ahrs.set_fly_forward(false);
    } else if (strncmp(msg_text, "ArduRover", strlen("ArduRover")) == 0) {
	vehicle = VehicleType::VEHICLE_ROVER;
	::printf("Detected Rover\n");
	ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);
	ahrs.set_fly_forward(true);
    }
    dataflash.Log_Write_Message(msg_text);
}
