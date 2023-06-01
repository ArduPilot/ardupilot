#include "AP_JSONParser_config.h"

#if AP_JSONPARSER_ENABLED

#include "AP_JSONParser.h"

#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>

bool AP_JSONParser::init(uint16_t max_tokens)
{
    t = new jsmntok_t[max_tokens];
    t_count = max_tokens;

    return t != nullptr;
}

AP_JSONParser::~AP_JSONParser()
{
    delete[] t;
    t = nullptr;
    t_count = 0;
    free(buf);
    buf = nullptr;
}

#if AP_FILESYSTEM_FILE_READING_ENABLED
bool AP_JSONParser::parse_filepath(const char *filepath)
{
    assert_storage_size<jsmntok_t, 16> __assert_token_size;
    (void)__assert_token_size;

    struct stat st;
    if (AP::FS().stat(filepath, &st) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "failed to stat json %s", filepath);
        return false;
    }
    int fd = AP::FS().open(filepath, O_RDONLY);
    if (fd == -1) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "failed to open json %s", filepath);
        return false;
    }
    buf = (char*)malloc(st.st_size+1);
    memset(buf, '\0', st.st_size+1);  // keep valgrind happy
    if (buf == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "alloc failed %u", unsigned(st.st_size+1));
        AP::FS().close(fd);
        return false;
    }

    if (AP::FS().read(fd, buf, st.st_size) != st.st_size) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "failed to read json %s", filepath);
        AP::FS().close(fd);
        return false;
    }
    AP::FS().close(fd);

    char *start = strchr(buf, '{');
    if (!start) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Invalid json %s", filepath);
        return false;
    }

    /*
      remove comments, as not allowed by the parser
     */
    for (char *p = strchr(buf,'#'); p; p=strchr(p+1, '#')) {
        // clear to end of line
        do {
            *p++ = ' ';
        } while (*p != '\n' && *p != '\r' && *p);
    }

    jsmn_init(&parser);

    const int r = jsmn_parse(&parser,
                             buf,
                             st.st_size,
                             t,
                             t_count);
    if (r < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to parse %s", filepath);
        return false;
    }

    num_used_tokens = uint16_t(r);

    if (t[0].type != JSMN_OBJECT) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "malformed JSON %s", filepath);
        return false;
    }

    token_offset = 0;

    return true;
}

#endif

bool AP_JSONParser::get_string_for_token(uint16_t token_ofs, char *ret, uint8_t len) const
{
    const uint16_t json_text_len = t[token_ofs].end - t[token_ofs].start;
    if (len <= json_text_len) {
        return false;
    }
    strncpy(ret, &buf[t[token_ofs].start], json_text_len);
    ret[json_text_len] = 0;
    return true;
}

bool AP_JSONParser::get_string_for_current_token(char *ret, uint8_t len) const
{
    return get_string_for_token(token_offset, ret, len);
}

bool AP_JSONParser::shift_token_string(char *ret, uint8_t len)
{
    if (token_offset >= num_used_tokens) {
        return false;
    }
    if (t[token_offset].type != JSMN_STRING) {
        return false;
    }
    if (!get_string_for_current_token(ret, len)) {
        return false;
    }
    token_offset++;  // shift
    return true;
}

bool AP_JSONParser::shift_token_float(float &ret)
{
    if (token_offset >= num_used_tokens) {
        return false;
    }
    if (t[token_offset].type != JSMN_PRIMITIVE) {
        return false;
    }

    char tmp_value[20];
    if (!get_string_for_current_token(tmp_value, ARRAY_SIZE(tmp_value))) {
        return false;
    }

    errno = 0;
    ret = strtof(tmp_value, nullptr);
    if (errno != 0) {
        return false;
    }

    token_offset++;

    return true;
}

bool AP_JSONParser::shift_token_int32_t(int32_t &ret)
{
    if (token_offset >= num_used_tokens) {
        return false;
    }
    if (t[token_offset].type != JSMN_PRIMITIVE) {
        return false;
    }

    char tmp_value[20];
    if (!get_string_for_current_token(tmp_value, ARRAY_SIZE(tmp_value))) {
        return false;
    }

    errno = 0;
    ret = strtol(tmp_value, nullptr, 10);
    if (errno != 0) {
        return false;
    }

    token_offset++;
    return true;
}

bool AP_JSONParser::shift_token_vector3f(Vector3f &ret)
{
    if (t[token_offset].type != JSMN_ARRAY) {
        return false;
    }
    if (t[token_offset].size != 3) {
        return false;
    }
    if (token_offset+t[token_offset].size >= num_used_tokens) {
        return false;
    }
    const uint8_t count = t[token_offset].size;
    uint8_t tmp_token_offset = token_offset;  // atomic consumption of vector3f
    tmp_token_offset++;  // consume array token
    for (uint8_t i=0; i<count; i++) {
        if (t[tmp_token_offset].type != JSMN_PRIMITIVE) {
            return false;
        }
        char tmp_value[20];
        if (!get_string_for_token(tmp_token_offset, tmp_value, ARRAY_SIZE(tmp_value))) {
            return false;
        }
        errno = 0;
        ret[i] = strtof(tmp_value, nullptr);
        if (errno != 0) {
            return false;
        }
        tmp_token_offset++;  // consume primitive token
    }
    token_offset = tmp_token_offset;
    return true;
}

bool AP_JSONParser::shift_token_object(uint16_t &ret)
{
    if (token_offset >= num_used_tokens) {
        return false;
    }
    if (t[token_offset].type != JSMN_OBJECT) {
        return false;
    }

    ret = t[token_offset].size;

    token_offset++;
    return true;
}

#endif  // AP_JSONPARSER_ENABLED
