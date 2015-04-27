#include "MsgHandler_BARO.h"

void MsgHandler_BARO::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    baro.setHIL(0,
		require_field_float(msg, "Press"),
		require_field_int16_t(msg, "Temp") * 0.01f);
    dataflash.Log_Write_Baro(baro);
}
