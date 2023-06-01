#pragma once

/*
  Convenience class for parsing json files.

  The parsed data output consists of a stream of tokens which can be
  retrieved from this object.

  Once you have supplied a file to parse with parse_filepath, you
  should be able to shift an object from your instance of this class.
  The number returned (via parameter) is the number of key/value pairs
  in the object.  Keys will always be strings.  Use shift_token_string
  to get a string, then one of the shift_token_* methods to retrieve
  the value (which may be another object with its count of key/value
  pairs).

  See tests directory for examples.

 */

#include "AP_JSONParser_config.h"
#include <AP_Filesystem/AP_Filesystem_config.h>

#if AP_JSONPARSER_ENABLED

#include <AP_Math/vector3.h>

// jsmn.h can't be included multiple times across a compilation as it
// defines functions.  If they're static that's not a problem, but
// they also aren't used in the .h file, so we need to declare them
// static:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#define JSMN_STATIC
#include "jsmn.h"
#pragma GCC diagnostic pop

class AP_JSONParser
{
public:
    AP_JSONParser() {}
    ~AP_JSONParser();

    // init - ready parser for parsing; allocated maximum number of
    // tokens to be found in document.
    bool init(uint16_t max_tokens) WARN_IF_UNUSED;

#if AP_FILESYSTEM_FILE_READING_ENABLED
    // try to parse supplied filepath
    bool parse_filepath(const char *filepath) WARN_IF_UNUSED;
#endif

    // methods to extract the next token from the stream of a particular form:
    bool shift_token_string(char *ret, uint8_t len) WARN_IF_UNUSED;
    bool shift_token_float(float &ret) WARN_IF_UNUSED;
    bool shift_token_int32_t(int32_t &ret) WARN_IF_UNUSED;
    bool shift_token_vector3f(Vector3f &ret) WARN_IF_UNUSED;
    bool shift_token_object(uint16_t &count) WARN_IF_UNUSED;

private:

    // this object may be declared on the stack, so initialise:
    char *buf = nullptr;

    jsmntok_t *t = nullptr;  // see static assert in .cpp; 16 bytes each
    uint16_t t_count = 0;  // number of jsmntok_t in t
    jsmn_parser parser {};

    // valid after jsmn_parse call:
    uint16_t token_offset;
    uint16_t num_used_tokens;

    bool get_string_for_current_token(char *ret, uint8_t len) const WARN_IF_UNUSED;
    // return a string for a specific token:
    bool get_string_for_token(uint16_t token_ofs, char *ret, uint8_t len) const;
};

#endif  // AP_JSONPARSER_ENABLED
