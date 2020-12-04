#pragma once

#include <AP_Logger/AP_Logger.h>
#include "VehicleType.h"

#include <stdio.h>

// ignore cast errors in this case to keep complexity down
// on x86 where replay is run we don't care about cast alignment
#pragma GCC diagnostic ignored "-Wcast-align"

#define LOGREADER_MAX_FIELDS 30

#define streq(x, y) (!strcmp(x, y))

class MsgHandler {
public:
    // constructor - create a parser for a MavLink message format
    MsgHandler(const struct log_Format &f);

    // retrieve a comma-separated list of all labels
    void string_for_labels(char *buffer, uint32_t bufferlen);

    // field_value - retrieve the value of a field from the supplied message
    // these return false if the field was not found
    template<typename R>
    bool field_value(uint8_t *msg, const char *label, R &ret);

    bool field_value(uint8_t *msg, const char *label, Vector3f &ret);
    bool field_value(uint8_t *msg, const char *label,
		     char *buffer, uint8_t bufferlen);
    
    template <typename R>
    void require_field(uint8_t *msg, const char *label, R &ret)
        {   
            if (! field_value(msg, label, ret)) {
                field_not_found(msg, label);
            }
        }
    void require_field(uint8_t *msg, const char *label, char *buffer, uint8_t bufferlen);
    float require_field_float(uint8_t *msg, const char *label);
    uint8_t require_field_uint8_t(uint8_t *msg, const char *label);
    int32_t require_field_int32_t(uint8_t *msg, const char *label);
    uint16_t require_field_uint16_t(uint8_t *msg, const char *label);
    int16_t require_field_int16_t(uint8_t *msg, const char *label);

private:

    void add_field(const char *_label, uint8_t _type, uint8_t _offset,
                   uint8_t length);

    template<typename R>
    void field_value_for_type_at_offset(uint8_t *msg, uint8_t type,
                                        uint8_t offset, R &ret);

    struct format_field_info { // parsed field information
        char *label;
        uint8_t type;
        uint8_t offset;
        uint8_t length;
    };
    struct format_field_info field_info[LOGREADER_MAX_FIELDS];

    uint8_t next_field;
    size_t size_for_type_table[52]; // maps field type (e.g. 'f') to e.g 4 bytes

    struct format_field_info *find_field_info(const char *label);

    void parse_format_fields();
    void init_field_types();
    void add_field_type(char type, size_t size);
    uint8_t size_for_type(char type);

protected:
    struct log_Format f; // the format we are a parser for

    void location_from_msg(uint8_t *msg, Location &loc, const char *label_lat,
			   const char *label_long, const char *label_alt);

    void ground_vel_from_msg(uint8_t *msg,
			     Vector3f &vel,
			     const char *label_speed,
			     const char *label_course,
			     const char *label_vz);

    void attitude_from_msg(uint8_t *msg,
			   Vector3f &att,
			   const char *label_roll,
			   const char *label_pitch,
			   const char *label_yaw);
    [[noreturn]] void field_not_found(uint8_t *msg, const char *label);
};

template<typename R>
bool MsgHandler::field_value(uint8_t *msg, const char *label, R &ret)
{
    struct format_field_info *info = find_field_info(label);
    if (info == NULL) {
        return false;
    }

    uint8_t offset = info->offset;
    if (offset == 0) {
        return false;
    }

    field_value_for_type_at_offset(msg, info->type, offset, ret);

    return true;
}


template<typename R>
inline void MsgHandler::field_value_for_type_at_offset(uint8_t *msg,
                                                      uint8_t type,
                                                      uint8_t offset,
                                                      R &ret)
{
    /* we register the types - add_field_type - so can we do without
     * this switch statement somehow? */
    switch (type) {
    case 'B':
        ret = (R)(((uint8_t*)&msg[offset])[0]);
        break;
    case 'c':
    case 'h':
        ret = (R)(((int16_t*)&msg[offset])[0]);
        break;
    case 'H':
        ret = (R)(((uint16_t*)&msg[offset])[0]);
        break;
    case 'C':
        ret = (R)(((uint16_t*)&msg[offset])[0]);
        break;
    case 'f':
        ret = (R)(((float*)&msg[offset])[0]);
        break;
    case 'I':
    case 'E':
        ret = (R)(((uint32_t*)&msg[offset])[0]);
        break;
    case 'L':
    case 'e':
        ret = (R)(((int32_t*)&msg[offset])[0]);
        break;
    case 'q':
        ret = (R)(((int64_t*)&msg[offset])[0]);
        break;
    case 'Q':
        ret = (R)(((uint64_t*)&msg[offset])[0]);
        break;
    default:
        ::printf("Unhandled format type (%c)\n", type);
        exit(1);
    }
}
