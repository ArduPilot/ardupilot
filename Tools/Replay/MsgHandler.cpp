#include "MsgHandler.h"

void fatal(const char *msg) {
    ::printf("%s",msg);
    ::printf("\n");
    exit(1);
}

char *xstrdup(const char *string)
{
    char *ret = strdup(string);
    if (ret == NULL) {
        perror("strdup");
        fatal("strdup failed");
    }
    return ret;
}

void MsgHandler::add_field_type(char type, size_t size)
{
    size_for_type_table[(type > 'A' ? (type-'A') : (type-'a'))] = size;
}

uint8_t MsgHandler::size_for_type(char type)
{
    uint8_t ret = size_for_type_table[(uint8_t)(type > 'A' ? (type-'A') : (type-'a'))];
    if (ret == 0) {
        ::fprintf(stderr, "Unknown type (%c)\n", type);
        abort();
    }
    return ret;
}

void MsgHandler::init_field_types()
{
    add_field_type('b', sizeof(int8_t));
    add_field_type('c', sizeof(int16_t));
    add_field_type('d', sizeof(double));
    add_field_type('e', sizeof(int32_t));
    add_field_type('f', sizeof(float));
    add_field_type('h', sizeof(int16_t));
    add_field_type('i', sizeof(int32_t));
    add_field_type('n', sizeof(char[4]));
    add_field_type('B', sizeof(uint8_t));
    add_field_type('C', sizeof(uint16_t));
    add_field_type('E', sizeof(uint32_t));
    add_field_type('H', sizeof(uint16_t));
    add_field_type('I', sizeof(uint32_t));
    add_field_type('L', sizeof(int32_t));
    add_field_type('M', sizeof(uint8_t));
    add_field_type('N', sizeof(char[16]));
    add_field_type('Z', sizeof(char[64]));
    add_field_type('q', sizeof(int64_t));
    add_field_type('Q', sizeof(uint64_t));
}

struct MsgHandler::format_field_info *MsgHandler::find_field_info(const char *label)
{
    for(uint8_t i=0; i<next_field; i++) {
        if (streq(field_info[i].label, label)) {
            return &field_info[i];
        }
    }
    return NULL;
}

MsgHandler::MsgHandler(const struct log_Format &_f) : next_field(0), f(_f)
{
    init_field_types();
    parse_format_fields();
}

void MsgHandler::add_field(const char *_label, uint8_t _type, uint8_t _offset,
                          uint8_t _length)
{
    field_info[next_field].label = xstrdup(_label);
    field_info[next_field].type = _type;
    field_info[next_field].offset = _offset;
    field_info[next_field].length = _length;
    next_field++;
}

void MsgHandler::parse_format_fields()
{
    char *labels = xstrdup(f.labels);
    char * arg = labels;
    uint8_t label_offset = 0;
    char *next_label;
    uint8_t msg_offset = 3; // 3 bytes for the header

    while ((next_label = strtok(arg, ",")) != NULL) {
	if (label_offset > strlen(f.format)) {
	    free(labels);
	    printf("too few field times for labels %s (format=%s) (labels=%s)\n",
		   f.name, f.format, f.labels);
	    exit(1);
	}
        uint8_t field_type = f.format[label_offset];
        uint8_t length = size_for_type(field_type);
        add_field(next_label, field_type, msg_offset, length);
        arg = NULL;
        msg_offset += length;
        label_offset++;
    }

    if (label_offset != strlen(f.format)) {
        printf("too few labels for format (format=%s) (labels=%s)\n",
               f.format, f.labels);
    }

    free(labels);
}

bool MsgHandler::field_value(uint8_t *msg, const char *label, char *ret, uint8_t retlen)
{
    struct format_field_info *info = find_field_info(label);
    if (info == NULL) {
      ::printf("No info for (%s)\n",label);
      exit(1);
    }

    uint8_t offset = info->offset;
    if (offset == 0) {
        return false;
    }

    memset(ret, '\0', retlen);

    memcpy(ret, &msg[offset], (retlen < info->length) ? retlen : info->length);

    return true;
}


bool MsgHandler::field_value(uint8_t *msg, const char *label, Vector3f &ret)
{
    const char *axes = "XYZ";
    uint8_t i;
    for(i=0; i<next_field; i++) {
	if (!strncmp(field_info[i].label, label, strlen(label)) &&
	    strlen(field_info[i].label) == strlen(label)+1) {
	    for (uint8_t j=0; j<3; j++) {
		if (field_info[i].label[strlen(label)] == axes[j]) {
                    field_value_for_type_at_offset(msg,
                                                   field_info[i].type,
                                                   field_info[i].offset,
                                                   ret[j]);
                    break; // break from finding-label loop
                }
            }
        }
        if (i == next_field) {
            return 0; // not found
        }
    }

    return true;
}


void MsgHandler::string_for_labels(char *buffer, uint bufferlen)
{
    memset(buffer, '\0', bufferlen);
    bufferlen--;

    char *pos = buffer;
    for (uint8_t k=0; k<LOGREADER_MAX_FIELDS; k++) {
        if (field_info[k].label != NULL) {
            uint8_t remaining = bufferlen - (pos - buffer);
            uint8_t label_length = strlen(field_info[k].label);
            uint8_t required = label_length;
            if (pos != buffer) { // room for a comma
                required++;
            }
            if (required+1 > remaining) { // null termination
                break;
            }

            if (pos != buffer) {
                *pos++ = ',';
            }

            memcpy(pos, field_info[k].label, label_length);
            pos += label_length;
        }
    }
}

MsgHandler::~MsgHandler()
{
    for (uint8_t k=0; k<LOGREADER_MAX_FIELDS; k++) {
        if (field_info[k].label != NULL) {
            free(field_info[k].label);
        }
    }
}

void MsgHandler::location_from_msg(uint8_t *msg,
                                  Location &loc,
                                  const char *label_lat,
                                  const char *label_long,
                                  const char *label_alt)
{
    loc.lat = require_field_int32_t(msg, label_lat);
    loc.lng = require_field_int32_t(msg, label_long);
    loc.alt = require_field_int32_t(msg, label_alt);
    loc.options = 0;
}

void MsgHandler::ground_vel_from_msg(uint8_t *msg,
                                    Vector3f &vel,
                                    const char *label_speed,
                                    const char *label_course,
                                    const char *label_vz)
{
    float ground_speed;
    float ground_course;
    // in older logs speed and course are integers
    if (!field_value(msg, label_speed, ground_speed)) {
        uint32_t speed_cms;
        require_field(msg, label_speed, speed_cms);
        ground_speed = speed_cms * 0.01f;
    }
    if (!field_value(msg, label_course, ground_course)) {
        uint32_t course_cd;
        require_field(msg, label_course, course_cd);
        ground_course = course_cd * 0.01f;
    }
    require_field(msg, label_course, ground_course);
    vel[0] = ground_speed*cosf(radians(ground_course));
    vel[1] = ground_speed*sinf(radians(ground_course));
    vel[2] = require_field_float(msg, label_vz);
}

void MsgHandler::attitude_from_msg(uint8_t *msg,
				   Vector3f &att,
				   const char *label_roll,
				   const char *label_pitch,
				   const char *label_yaw)
{
    att[0] = require_field_int16_t(msg, label_roll) * 0.01f;
    att[1] = require_field_int16_t(msg, label_pitch) * 0.01f;
    att[2] = require_field_uint16_t(msg, label_yaw) * 0.01f;
}

void MsgHandler::field_not_found(uint8_t *msg, const char *label)
{
    char all_labels[256];
    uint8_t type = msg[2];
    string_for_labels(all_labels, 256);
    ::printf("Field (%s) not found for id=%d; options are (%s)\n",
             label, type, all_labels);
    abort();
}

void MsgHandler::require_field(uint8_t *msg, const char *label, char *buffer, uint8_t bufferlen)
{
    if (! field_value(msg, label, buffer, bufferlen)) {
        field_not_found(msg,label);
    }
}

float MsgHandler::require_field_float(uint8_t *msg, const char *label)
{
    float ret;
    require_field(msg, label, ret);
    return ret;
}
uint8_t MsgHandler::require_field_uint8_t(uint8_t *msg, const char *label)
{
    uint8_t ret;
    require_field(msg, label, ret);
    return ret;
}
int32_t MsgHandler::require_field_int32_t(uint8_t *msg, const char *label)
{
    int32_t ret;
    require_field(msg, label, ret);
    return ret;
}
uint16_t MsgHandler::require_field_uint16_t(uint8_t *msg, const char *label)
{
    uint16_t ret;
    require_field(msg, label, ret);
    return ret;
}
int16_t MsgHandler::require_field_int16_t(uint8_t *msg, const char *label)
{
    int16_t ret;
    require_field(msg, label, ret);
    return ret;
}
