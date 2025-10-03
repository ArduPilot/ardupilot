#pragma once
// MESSAGE MANUAL_CONTROL PACKING

#define MAVLINK_MSG_ID_MANUAL_CONTROL 69

MAVPACKED(
typedef struct __mavlink_manual_control_t {
 int16_t x; /*<  X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.*/
 int16_t y; /*<  Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.*/
 int16_t z; /*<  Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.*/
 int16_t r; /*<  R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.*/
 uint16_t buttons; /*<  A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.*/
 uint8_t target; /*<  The system to be controlled.*/
 uint16_t buttons2; /*<  A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 16.*/
 uint8_t enabled_extensions; /*<  Set bits to 1 to indicate which of the following extension fields contain valid data: bit 0: pitch, bit 1: roll, bit 2: aux1, bit 3: aux2, bit 4: aux3, bit 5: aux4, bit 6: aux5, bit 7: aux6*/
 int16_t s; /*<  Pitch-only-axis, normalized to the range [-1000,1000]. Generally corresponds to pitch on vehicles with additional degrees of freedom. Valid if bit 0 of enabled_extensions field is set. Set to 0 if invalid.*/
 int16_t t; /*<  Roll-only-axis, normalized to the range [-1000,1000]. Generally corresponds to roll on vehicles with additional degrees of freedom. Valid if bit 1 of enabled_extensions field is set. Set to 0 if invalid.*/
 int16_t aux1; /*<  Aux continuous input field 1. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 2 of enabled_extensions field is set. 0 if bit 2 is unset.*/
 int16_t aux2; /*<  Aux continuous input field 2. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 3 of enabled_extensions field is set. 0 if bit 3 is unset.*/
 int16_t aux3; /*<  Aux continuous input field 3. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 4 of enabled_extensions field is set. 0 if bit 4 is unset.*/
 int16_t aux4; /*<  Aux continuous input field 4. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 5 of enabled_extensions field is set. 0 if bit 5 is unset.*/
 int16_t aux5; /*<  Aux continuous input field 5. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 6 of enabled_extensions field is set. 0 if bit 6 is unset.*/
 int16_t aux6; /*<  Aux continuous input field 6. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 7 of enabled_extensions field is set. 0 if bit 7 is unset.*/
}) mavlink_manual_control_t;

#define MAVLINK_MSG_ID_MANUAL_CONTROL_LEN 30
#define MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN 11
#define MAVLINK_MSG_ID_69_LEN 30
#define MAVLINK_MSG_ID_69_MIN_LEN 11

#define MAVLINK_MSG_ID_MANUAL_CONTROL_CRC 243
#define MAVLINK_MSG_ID_69_CRC 243



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MANUAL_CONTROL { \
    69, \
    "MANUAL_CONTROL", \
    16, \
    {  { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_manual_control_t, target) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_manual_control_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_manual_control_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_manual_control_t, z) }, \
         { "r", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_manual_control_t, r) }, \
         { "buttons", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_manual_control_t, buttons) }, \
         { "buttons2", NULL, MAVLINK_TYPE_UINT16_T, 0, 11, offsetof(mavlink_manual_control_t, buttons2) }, \
         { "enabled_extensions", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_manual_control_t, enabled_extensions) }, \
         { "s", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_manual_control_t, s) }, \
         { "t", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_manual_control_t, t) }, \
         { "aux1", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_manual_control_t, aux1) }, \
         { "aux2", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_manual_control_t, aux2) }, \
         { "aux3", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_manual_control_t, aux3) }, \
         { "aux4", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_manual_control_t, aux4) }, \
         { "aux5", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_manual_control_t, aux5) }, \
         { "aux6", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_manual_control_t, aux6) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MANUAL_CONTROL { \
    "MANUAL_CONTROL", \
    16, \
    {  { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_manual_control_t, target) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_manual_control_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_manual_control_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_manual_control_t, z) }, \
         { "r", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_manual_control_t, r) }, \
         { "buttons", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_manual_control_t, buttons) }, \
         { "buttons2", NULL, MAVLINK_TYPE_UINT16_T, 0, 11, offsetof(mavlink_manual_control_t, buttons2) }, \
         { "enabled_extensions", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_manual_control_t, enabled_extensions) }, \
         { "s", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_manual_control_t, s) }, \
         { "t", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_manual_control_t, t) }, \
         { "aux1", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_manual_control_t, aux1) }, \
         { "aux2", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_manual_control_t, aux2) }, \
         { "aux3", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_manual_control_t, aux3) }, \
         { "aux4", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_manual_control_t, aux4) }, \
         { "aux5", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_manual_control_t, aux5) }, \
         { "aux6", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_manual_control_t, aux6) }, \
         } \
}
#endif

/**
 * @brief Pack a manual_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target  The system to be controlled.
 * @param x  X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
 * @param y  Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
 * @param z  Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
 * @param r  R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
 * @param buttons  A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
 * @param buttons2  A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 16.
 * @param enabled_extensions  Set bits to 1 to indicate which of the following extension fields contain valid data: bit 0: pitch, bit 1: roll, bit 2: aux1, bit 3: aux2, bit 4: aux3, bit 5: aux4, bit 6: aux5, bit 7: aux6
 * @param s  Pitch-only-axis, normalized to the range [-1000,1000]. Generally corresponds to pitch on vehicles with additional degrees of freedom. Valid if bit 0 of enabled_extensions field is set. Set to 0 if invalid.
 * @param t  Roll-only-axis, normalized to the range [-1000,1000]. Generally corresponds to roll on vehicles with additional degrees of freedom. Valid if bit 1 of enabled_extensions field is set. Set to 0 if invalid.
 * @param aux1  Aux continuous input field 1. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 2 of enabled_extensions field is set. 0 if bit 2 is unset.
 * @param aux2  Aux continuous input field 2. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 3 of enabled_extensions field is set. 0 if bit 3 is unset.
 * @param aux3  Aux continuous input field 3. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 4 of enabled_extensions field is set. 0 if bit 4 is unset.
 * @param aux4  Aux continuous input field 4. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 5 of enabled_extensions field is set. 0 if bit 5 is unset.
 * @param aux5  Aux continuous input field 5. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 6 of enabled_extensions field is set. 0 if bit 6 is unset.
 * @param aux6  Aux continuous input field 6. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 7 of enabled_extensions field is set. 0 if bit 7 is unset.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manual_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions, int16_t s, int16_t t, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MANUAL_CONTROL_LEN];
    _mav_put_int16_t(buf, 0, x);
    _mav_put_int16_t(buf, 2, y);
    _mav_put_int16_t(buf, 4, z);
    _mav_put_int16_t(buf, 6, r);
    _mav_put_uint16_t(buf, 8, buttons);
    _mav_put_uint8_t(buf, 10, target);
    _mav_put_uint16_t(buf, 11, buttons2);
    _mav_put_uint8_t(buf, 13, enabled_extensions);
    _mav_put_int16_t(buf, 14, s);
    _mav_put_int16_t(buf, 16, t);
    _mav_put_int16_t(buf, 18, aux1);
    _mav_put_int16_t(buf, 20, aux2);
    _mav_put_int16_t(buf, 22, aux3);
    _mav_put_int16_t(buf, 24, aux4);
    _mav_put_int16_t(buf, 26, aux5);
    _mav_put_int16_t(buf, 28, aux6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
#else
    mavlink_manual_control_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.r = r;
    packet.buttons = buttons;
    packet.target = target;
    packet.buttons2 = buttons2;
    packet.enabled_extensions = enabled_extensions;
    packet.s = s;
    packet.t = t;
    packet.aux1 = aux1;
    packet.aux2 = aux2;
    packet.aux3 = aux3;
    packet.aux4 = aux4;
    packet.aux5 = aux5;
    packet.aux6 = aux6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MANUAL_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_CRC);
}

/**
 * @brief Pack a manual_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target  The system to be controlled.
 * @param x  X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
 * @param y  Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
 * @param z  Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
 * @param r  R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
 * @param buttons  A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
 * @param buttons2  A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 16.
 * @param enabled_extensions  Set bits to 1 to indicate which of the following extension fields contain valid data: bit 0: pitch, bit 1: roll, bit 2: aux1, bit 3: aux2, bit 4: aux3, bit 5: aux4, bit 6: aux5, bit 7: aux6
 * @param s  Pitch-only-axis, normalized to the range [-1000,1000]. Generally corresponds to pitch on vehicles with additional degrees of freedom. Valid if bit 0 of enabled_extensions field is set. Set to 0 if invalid.
 * @param t  Roll-only-axis, normalized to the range [-1000,1000]. Generally corresponds to roll on vehicles with additional degrees of freedom. Valid if bit 1 of enabled_extensions field is set. Set to 0 if invalid.
 * @param aux1  Aux continuous input field 1. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 2 of enabled_extensions field is set. 0 if bit 2 is unset.
 * @param aux2  Aux continuous input field 2. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 3 of enabled_extensions field is set. 0 if bit 3 is unset.
 * @param aux3  Aux continuous input field 3. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 4 of enabled_extensions field is set. 0 if bit 4 is unset.
 * @param aux4  Aux continuous input field 4. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 5 of enabled_extensions field is set. 0 if bit 5 is unset.
 * @param aux5  Aux continuous input field 5. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 6 of enabled_extensions field is set. 0 if bit 6 is unset.
 * @param aux6  Aux continuous input field 6. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 7 of enabled_extensions field is set. 0 if bit 7 is unset.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manual_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions, int16_t s, int16_t t, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MANUAL_CONTROL_LEN];
    _mav_put_int16_t(buf, 0, x);
    _mav_put_int16_t(buf, 2, y);
    _mav_put_int16_t(buf, 4, z);
    _mav_put_int16_t(buf, 6, r);
    _mav_put_uint16_t(buf, 8, buttons);
    _mav_put_uint8_t(buf, 10, target);
    _mav_put_uint16_t(buf, 11, buttons2);
    _mav_put_uint8_t(buf, 13, enabled_extensions);
    _mav_put_int16_t(buf, 14, s);
    _mav_put_int16_t(buf, 16, t);
    _mav_put_int16_t(buf, 18, aux1);
    _mav_put_int16_t(buf, 20, aux2);
    _mav_put_int16_t(buf, 22, aux3);
    _mav_put_int16_t(buf, 24, aux4);
    _mav_put_int16_t(buf, 26, aux5);
    _mav_put_int16_t(buf, 28, aux6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
#else
    mavlink_manual_control_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.r = r;
    packet.buttons = buttons;
    packet.target = target;
    packet.buttons2 = buttons2;
    packet.enabled_extensions = enabled_extensions;
    packet.s = s;
    packet.t = t;
    packet.aux1 = aux1;
    packet.aux2 = aux2;
    packet.aux3 = aux3;
    packet.aux4 = aux4;
    packet.aux5 = aux5;
    packet.aux6 = aux6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MANUAL_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a manual_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target  The system to be controlled.
 * @param x  X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
 * @param y  Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
 * @param z  Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
 * @param r  R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
 * @param buttons  A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
 * @param buttons2  A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 16.
 * @param enabled_extensions  Set bits to 1 to indicate which of the following extension fields contain valid data: bit 0: pitch, bit 1: roll, bit 2: aux1, bit 3: aux2, bit 4: aux3, bit 5: aux4, bit 6: aux5, bit 7: aux6
 * @param s  Pitch-only-axis, normalized to the range [-1000,1000]. Generally corresponds to pitch on vehicles with additional degrees of freedom. Valid if bit 0 of enabled_extensions field is set. Set to 0 if invalid.
 * @param t  Roll-only-axis, normalized to the range [-1000,1000]. Generally corresponds to roll on vehicles with additional degrees of freedom. Valid if bit 1 of enabled_extensions field is set. Set to 0 if invalid.
 * @param aux1  Aux continuous input field 1. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 2 of enabled_extensions field is set. 0 if bit 2 is unset.
 * @param aux2  Aux continuous input field 2. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 3 of enabled_extensions field is set. 0 if bit 3 is unset.
 * @param aux3  Aux continuous input field 3. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 4 of enabled_extensions field is set. 0 if bit 4 is unset.
 * @param aux4  Aux continuous input field 4. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 5 of enabled_extensions field is set. 0 if bit 5 is unset.
 * @param aux5  Aux continuous input field 5. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 6 of enabled_extensions field is set. 0 if bit 6 is unset.
 * @param aux6  Aux continuous input field 6. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 7 of enabled_extensions field is set. 0 if bit 7 is unset.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manual_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target,int16_t x,int16_t y,int16_t z,int16_t r,uint16_t buttons,uint16_t buttons2,uint8_t enabled_extensions,int16_t s,int16_t t,int16_t aux1,int16_t aux2,int16_t aux3,int16_t aux4,int16_t aux5,int16_t aux6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MANUAL_CONTROL_LEN];
    _mav_put_int16_t(buf, 0, x);
    _mav_put_int16_t(buf, 2, y);
    _mav_put_int16_t(buf, 4, z);
    _mav_put_int16_t(buf, 6, r);
    _mav_put_uint16_t(buf, 8, buttons);
    _mav_put_uint8_t(buf, 10, target);
    _mav_put_uint16_t(buf, 11, buttons2);
    _mav_put_uint8_t(buf, 13, enabled_extensions);
    _mav_put_int16_t(buf, 14, s);
    _mav_put_int16_t(buf, 16, t);
    _mav_put_int16_t(buf, 18, aux1);
    _mav_put_int16_t(buf, 20, aux2);
    _mav_put_int16_t(buf, 22, aux3);
    _mav_put_int16_t(buf, 24, aux4);
    _mav_put_int16_t(buf, 26, aux5);
    _mav_put_int16_t(buf, 28, aux6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
#else
    mavlink_manual_control_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.r = r;
    packet.buttons = buttons;
    packet.target = target;
    packet.buttons2 = buttons2;
    packet.enabled_extensions = enabled_extensions;
    packet.s = s;
    packet.t = t;
    packet.aux1 = aux1;
    packet.aux2 = aux2;
    packet.aux3 = aux3;
    packet.aux4 = aux4;
    packet.aux5 = aux5;
    packet.aux6 = aux6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MANUAL_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_CRC);
}

/**
 * @brief Encode a manual_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param manual_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_manual_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_manual_control_t* manual_control)
{
    return mavlink_msg_manual_control_pack(system_id, component_id, msg, manual_control->target, manual_control->x, manual_control->y, manual_control->z, manual_control->r, manual_control->buttons, manual_control->buttons2, manual_control->enabled_extensions, manual_control->s, manual_control->t, manual_control->aux1, manual_control->aux2, manual_control->aux3, manual_control->aux4, manual_control->aux5, manual_control->aux6);
}

/**
 * @brief Encode a manual_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param manual_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_manual_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_manual_control_t* manual_control)
{
    return mavlink_msg_manual_control_pack_chan(system_id, component_id, chan, msg, manual_control->target, manual_control->x, manual_control->y, manual_control->z, manual_control->r, manual_control->buttons, manual_control->buttons2, manual_control->enabled_extensions, manual_control->s, manual_control->t, manual_control->aux1, manual_control->aux2, manual_control->aux3, manual_control->aux4, manual_control->aux5, manual_control->aux6);
}

/**
 * @brief Encode a manual_control struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param manual_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_manual_control_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_manual_control_t* manual_control)
{
    return mavlink_msg_manual_control_pack_status(system_id, component_id, _status, msg,  manual_control->target, manual_control->x, manual_control->y, manual_control->z, manual_control->r, manual_control->buttons, manual_control->buttons2, manual_control->enabled_extensions, manual_control->s, manual_control->t, manual_control->aux1, manual_control->aux2, manual_control->aux3, manual_control->aux4, manual_control->aux5, manual_control->aux6);
}

/**
 * @brief Send a manual_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target  The system to be controlled.
 * @param x  X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
 * @param y  Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
 * @param z  Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
 * @param r  R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
 * @param buttons  A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
 * @param buttons2  A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 16.
 * @param enabled_extensions  Set bits to 1 to indicate which of the following extension fields contain valid data: bit 0: pitch, bit 1: roll, bit 2: aux1, bit 3: aux2, bit 4: aux3, bit 5: aux4, bit 6: aux5, bit 7: aux6
 * @param s  Pitch-only-axis, normalized to the range [-1000,1000]. Generally corresponds to pitch on vehicles with additional degrees of freedom. Valid if bit 0 of enabled_extensions field is set. Set to 0 if invalid.
 * @param t  Roll-only-axis, normalized to the range [-1000,1000]. Generally corresponds to roll on vehicles with additional degrees of freedom. Valid if bit 1 of enabled_extensions field is set. Set to 0 if invalid.
 * @param aux1  Aux continuous input field 1. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 2 of enabled_extensions field is set. 0 if bit 2 is unset.
 * @param aux2  Aux continuous input field 2. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 3 of enabled_extensions field is set. 0 if bit 3 is unset.
 * @param aux3  Aux continuous input field 3. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 4 of enabled_extensions field is set. 0 if bit 4 is unset.
 * @param aux4  Aux continuous input field 4. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 5 of enabled_extensions field is set. 0 if bit 5 is unset.
 * @param aux5  Aux continuous input field 5. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 6 of enabled_extensions field is set. 0 if bit 6 is unset.
 * @param aux6  Aux continuous input field 6. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 7 of enabled_extensions field is set. 0 if bit 7 is unset.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_manual_control_send(mavlink_channel_t chan, uint8_t target, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions, int16_t s, int16_t t, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MANUAL_CONTROL_LEN];
    _mav_put_int16_t(buf, 0, x);
    _mav_put_int16_t(buf, 2, y);
    _mav_put_int16_t(buf, 4, z);
    _mav_put_int16_t(buf, 6, r);
    _mav_put_uint16_t(buf, 8, buttons);
    _mav_put_uint8_t(buf, 10, target);
    _mav_put_uint16_t(buf, 11, buttons2);
    _mav_put_uint8_t(buf, 13, enabled_extensions);
    _mav_put_int16_t(buf, 14, s);
    _mav_put_int16_t(buf, 16, t);
    _mav_put_int16_t(buf, 18, aux1);
    _mav_put_int16_t(buf, 20, aux2);
    _mav_put_int16_t(buf, 22, aux3);
    _mav_put_int16_t(buf, 24, aux4);
    _mav_put_int16_t(buf, 26, aux5);
    _mav_put_int16_t(buf, 28, aux6);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_CONTROL, buf, MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_CRC);
#else
    mavlink_manual_control_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.r = r;
    packet.buttons = buttons;
    packet.target = target;
    packet.buttons2 = buttons2;
    packet.enabled_extensions = enabled_extensions;
    packet.s = s;
    packet.t = t;
    packet.aux1 = aux1;
    packet.aux2 = aux2;
    packet.aux3 = aux3;
    packet.aux4 = aux4;
    packet.aux5 = aux5;
    packet.aux6 = aux6;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_CRC);
#endif
}

/**
 * @brief Send a manual_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_manual_control_send_struct(mavlink_channel_t chan, const mavlink_manual_control_t* manual_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_manual_control_send(chan, manual_control->target, manual_control->x, manual_control->y, manual_control->z, manual_control->r, manual_control->buttons, manual_control->buttons2, manual_control->enabled_extensions, manual_control->s, manual_control->t, manual_control->aux1, manual_control->aux2, manual_control->aux3, manual_control->aux4, manual_control->aux5, manual_control->aux6);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_CONTROL, (const char *)manual_control, MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_MANUAL_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_manual_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions, int16_t s, int16_t t, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, x);
    _mav_put_int16_t(buf, 2, y);
    _mav_put_int16_t(buf, 4, z);
    _mav_put_int16_t(buf, 6, r);
    _mav_put_uint16_t(buf, 8, buttons);
    _mav_put_uint8_t(buf, 10, target);
    _mav_put_uint16_t(buf, 11, buttons2);
    _mav_put_uint8_t(buf, 13, enabled_extensions);
    _mav_put_int16_t(buf, 14, s);
    _mav_put_int16_t(buf, 16, t);
    _mav_put_int16_t(buf, 18, aux1);
    _mav_put_int16_t(buf, 20, aux2);
    _mav_put_int16_t(buf, 22, aux3);
    _mav_put_int16_t(buf, 24, aux4);
    _mav_put_int16_t(buf, 26, aux5);
    _mav_put_int16_t(buf, 28, aux6);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_CONTROL, buf, MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_CRC);
#else
    mavlink_manual_control_t *packet = (mavlink_manual_control_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->r = r;
    packet->buttons = buttons;
    packet->target = target;
    packet->buttons2 = buttons2;
    packet->enabled_extensions = enabled_extensions;
    packet->s = s;
    packet->t = t;
    packet->aux1 = aux1;
    packet->aux2 = aux2;
    packet->aux3 = aux3;
    packet->aux4 = aux4;
    packet->aux5 = aux5;
    packet->aux6 = aux6;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_CONTROL, (const char *)packet, MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN, MAVLINK_MSG_ID_MANUAL_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE MANUAL_CONTROL UNPACKING


/**
 * @brief Get field target from manual_control message
 *
 * @return  The system to be controlled.
 */
static inline uint8_t mavlink_msg_manual_control_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field x from manual_control message
 *
 * @return  X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
 */
static inline int16_t mavlink_msg_manual_control_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field y from manual_control message
 *
 * @return  Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
 */
static inline int16_t mavlink_msg_manual_control_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field z from manual_control message
 *
 * @return  Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
 */
static inline int16_t mavlink_msg_manual_control_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field r from manual_control message
 *
 * @return  R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
 */
static inline int16_t mavlink_msg_manual_control_get_r(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field buttons from manual_control message
 *
 * @return  A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
 */
static inline uint16_t mavlink_msg_manual_control_get_buttons(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field buttons2 from manual_control message
 *
 * @return  A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 16.
 */
static inline uint16_t mavlink_msg_manual_control_get_buttons2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  11);
}

/**
 * @brief Get field enabled_extensions from manual_control message
 *
 * @return  Set bits to 1 to indicate which of the following extension fields contain valid data: bit 0: pitch, bit 1: roll, bit 2: aux1, bit 3: aux2, bit 4: aux3, bit 5: aux4, bit 6: aux5, bit 7: aux6
 */
static inline uint8_t mavlink_msg_manual_control_get_enabled_extensions(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field s from manual_control message
 *
 * @return  Pitch-only-axis, normalized to the range [-1000,1000]. Generally corresponds to pitch on vehicles with additional degrees of freedom. Valid if bit 0 of enabled_extensions field is set. Set to 0 if invalid.
 */
static inline int16_t mavlink_msg_manual_control_get_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field t from manual_control message
 *
 * @return  Roll-only-axis, normalized to the range [-1000,1000]. Generally corresponds to roll on vehicles with additional degrees of freedom. Valid if bit 1 of enabled_extensions field is set. Set to 0 if invalid.
 */
static inline int16_t mavlink_msg_manual_control_get_t(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field aux1 from manual_control message
 *
 * @return  Aux continuous input field 1. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 2 of enabled_extensions field is set. 0 if bit 2 is unset.
 */
static inline int16_t mavlink_msg_manual_control_get_aux1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field aux2 from manual_control message
 *
 * @return  Aux continuous input field 2. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 3 of enabled_extensions field is set. 0 if bit 3 is unset.
 */
static inline int16_t mavlink_msg_manual_control_get_aux2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field aux3 from manual_control message
 *
 * @return  Aux continuous input field 3. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 4 of enabled_extensions field is set. 0 if bit 4 is unset.
 */
static inline int16_t mavlink_msg_manual_control_get_aux3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field aux4 from manual_control message
 *
 * @return  Aux continuous input field 4. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 5 of enabled_extensions field is set. 0 if bit 5 is unset.
 */
static inline int16_t mavlink_msg_manual_control_get_aux4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field aux5 from manual_control message
 *
 * @return  Aux continuous input field 5. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 6 of enabled_extensions field is set. 0 if bit 6 is unset.
 */
static inline int16_t mavlink_msg_manual_control_get_aux5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field aux6 from manual_control message
 *
 * @return  Aux continuous input field 6. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 7 of enabled_extensions field is set. 0 if bit 7 is unset.
 */
static inline int16_t mavlink_msg_manual_control_get_aux6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Decode a manual_control message into a struct
 *
 * @param msg The message to decode
 * @param manual_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_manual_control_decode(const mavlink_message_t* msg, mavlink_manual_control_t* manual_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    manual_control->x = mavlink_msg_manual_control_get_x(msg);
    manual_control->y = mavlink_msg_manual_control_get_y(msg);
    manual_control->z = mavlink_msg_manual_control_get_z(msg);
    manual_control->r = mavlink_msg_manual_control_get_r(msg);
    manual_control->buttons = mavlink_msg_manual_control_get_buttons(msg);
    manual_control->target = mavlink_msg_manual_control_get_target(msg);
    manual_control->buttons2 = mavlink_msg_manual_control_get_buttons2(msg);
    manual_control->enabled_extensions = mavlink_msg_manual_control_get_enabled_extensions(msg);
    manual_control->s = mavlink_msg_manual_control_get_s(msg);
    manual_control->t = mavlink_msg_manual_control_get_t(msg);
    manual_control->aux1 = mavlink_msg_manual_control_get_aux1(msg);
    manual_control->aux2 = mavlink_msg_manual_control_get_aux2(msg);
    manual_control->aux3 = mavlink_msg_manual_control_get_aux3(msg);
    manual_control->aux4 = mavlink_msg_manual_control_get_aux4(msg);
    manual_control->aux5 = mavlink_msg_manual_control_get_aux5(msg);
    manual_control->aux6 = mavlink_msg_manual_control_get_aux6(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MANUAL_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_MANUAL_CONTROL_LEN;
        memset(manual_control, 0, MAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
    memcpy(manual_control, _MAV_PAYLOAD(msg), len);
#endif
}
