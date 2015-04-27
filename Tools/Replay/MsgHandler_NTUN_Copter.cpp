#include "MsgHandler_NTUN_Copter.h"

void MsgHandler_NTUN_Copter::process_message(uint8_t *msg)
{
    inavpos = Vector3f(require_field_float(msg, "PosX") * 0.01f,
		       require_field_float(msg, "PosY") * 0.01f,
		       0);
}
