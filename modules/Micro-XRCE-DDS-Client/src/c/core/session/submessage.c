#include "submessage_internal.h"
#include "../serialization/xrce_subheader_internal.h"

//==================================================================
//                             PUBLIC
//==================================================================
bool uxr_buffer_submessage_header(
        ucdrBuffer* ub,
        uint8_t submessage_id,
        uint16_t length,
        uint8_t flags)
{
    ucdr_align_to(ub, 4);
    ub->endianness = UCDR_MACHINE_ENDIANNESS;
    flags = (uint8_t)(flags | ub->endianness);
    uxr_serialize_submessage_header(ub, submessage_id, flags, length);

    return ucdr_buffer_remaining(ub) >= length;
}

bool uxr_read_submessage_header(
        ucdrBuffer* ub,
        uint8_t* submessage_id,
        uint16_t* length,
        uint8_t* flags)
{
    ucdr_align_to(ub, 4);
    bool ready_to_read = ucdr_buffer_remaining(ub) >= SUBHEADER_SIZE;
    if (ready_to_read)
    {
        uxr_deserialize_submessage_header(ub, submessage_id, flags, length);

        uint8_t endiannes_flag = *flags & FLAG_ENDIANNESS;
        *flags = (uint8_t)(*flags & ~endiannes_flag);
        ub->endianness = endiannes_flag ? UCDR_LITTLE_ENDIANNESS : UCDR_BIG_ENDIANNESS;
    }

    return ready_to_read;
}

size_t uxr_submessage_padding(
        size_t length)
{
    return (length % SUBHEADER_SIZE != 0) ? SUBHEADER_SIZE - (length % SUBHEADER_SIZE) : 0;
}
