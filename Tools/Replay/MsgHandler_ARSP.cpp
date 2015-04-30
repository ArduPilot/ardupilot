#include "MsgHandler_ARSP.h"

void MsgHandler_ARSP::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    airspeed.setHIL(require_field_float(msg, "Airspeed"),
		    require_field_float(msg, "DiffPress"),
		    require_field_float(msg, "Temp"));

    dataflash.Log_Write_Airspeed(airspeed);
}
