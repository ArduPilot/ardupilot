#include <uxr/client/core/session/object_id.h>

//==================================================================
//                             PUBLIC
//==================================================================
uxrObjectId uxr_object_id(
        uint16_t id,
        uint8_t type)
{
    uxrObjectId object_id;
    object_id.id = id;
    object_id.type = type;
    return object_id;
}

uxrObjectId uxr_object_id_from_raw(
        const uint8_t* raw)
{
    uxrObjectId object_id;
    object_id.id = (uint16_t)((((uint16_t)raw[0]) << 4) + (raw[1] >> 4));
    object_id.type = raw[1] & 0x0F;
    return object_id;
}

void uxr_object_id_to_raw(
        uxrObjectId object_id,
        uint8_t* raw)
{
    raw[0] = (uint8_t) (object_id.id >> 4);
    raw[1] = (uint8_t)((((uint8_t) (object_id.id)) << 4) + (object_id.type & 0x0F));
}
