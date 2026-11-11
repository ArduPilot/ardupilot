/*
  streaming, zero allocation JSON field parser, see AP_JSON_FieldParser.h
 */

#include "AP_JSON_FieldParser.h"
#include "AP_JSON_Number.h"

#include <AP_Math/crc.h>
#include <math.h>
#include <string.h>

typedef AP_JSON_FieldParser::Result Result;

// words that can stand for a value. Apart from true, false and null they
// are not JSON, but are how common encoders write infinity and NaN
static const char *const literals[] = {
    "true", "false", "null", "nan", "NaN", "inf", "Infinity",
};

// index of the lowest set bit; __builtin_ctzll is a library call on
// 32 bit cores
static inline uint8_t lowest_bit(uint64_t v)
{
    const uint32_t lo = uint32_t(v);
    return lo != 0 ? uint8_t(__builtin_ctz(lo)) : uint8_t(32 + __builtin_ctz(uint32_t(v >> 32)));
}

AP_JSON_FieldParser::AP_JSON_FieldParser(const Field *fields, uint8_t num_fields) :
    _fields(fields),
    _num_fields(num_fields > MAX_FIELDS ? MAX_FIELDS : num_fields),
    _found(0),
    _full_path_set(0),
    _len(0),
    _crc(0),
    _crc_received(0),
    _in_object(false),
    _had_crc(false),
    _crc_digits(0),
    _depth(0),
    _is_object(0),
    _array_discard(0),
    _target(Target::SKIP),
    _target_field(-1),
    _target_discard(false),
    _target_candidates(0)
{
    _number_len = 0;
    _special = Special::NONE;
    // the bucket a key of n characters hashes to
    const auto bucket = [](const char *key, uint8_t n) {
        uint8_t h = 0;
        for (uint8_t k = 0; k < n; k++) {
            h = key_hash(h, uint8_t(key[k]));
        }
        return h % KEY_BUCKETS;
    };
    memset(_bucket, 0, sizeof(_bucket));
    for (uint8_t i = 0; i < _num_fields; i++) {
        memset(_part_len[i], 0, sizeof(_part_len[i]));
        const char *path = _fields[i].path;
        uint8_t part = 0;
        bool ok = path != nullptr && path[0] != 0;
        for (const char *p = path; ok && *p != 0; part++) {
            const char *dot = strchr(p, '.');
            const size_t n = dot != nullptr ? size_t(dot - p) : strlen(p);
            // an empty part, a part no key can match or too many parts
            // make the field unmatchable
            if (n == 0 || n > MAX_KEY_LEN || part >= MAX_PATH_DEPTH) {
                ok = false;
                break;
            }
            _part_len[i][part] = uint8_t(n);
            p += n + (dot != nullptr ? 1 : 0);
            if (dot != nullptr && *p == 0) {
                ok = false;
            }
        }
        if (ok) {
            _bucket[bucket(path, _part_len[i][0])] |= 1ULL << i;
        } else {
            memset(_part_len[i], 0, sizeof(_part_len[i]));
        }
        const char *alias = _fields[i].alias;
        _alias_len[i] = 0;
        if (alias != nullptr) {
            const size_t alias_len = strlen(alias);
            if (alias_len != 0 && alias_len <= MAX_KEY_LEN && strchr(alias, '.') == nullptr) {
                _alias_len[i] = uint8_t(alias_len);
                _bucket[bucket(alias, _alias_len[i])] |= 1ULL << i;
            }
        }
    }
}

Result AP_JSON_FieldParser::fail(uint8_t c, const char *msg)
{
    _error = msg;
    _in_object = false;
    // skip the rest of the line, unless this byte already ended it
    _state = (c == '\n') ? State::IDLE : State::RESYNC;
    return Result::ERROR;
}

__attribute__((always_inline)) inline Result AP_JSON_FieldParser::feed_byte(uint8_t c)
{
    if (c == 0) {
        // C senders may end a packet with the string's terminator
        c = '\n';
    }
    switch (_state) {
    case State::IDLE:
        if (c == '\n' || is_ws(c)) {
            return Result::NONE;
        }
        if (c != '{') {
            return fail(c, "packet does not start with '{'");
        }
        _found = 0;
        _full_path_set = 0;
        _alias_set = 0;
        _had_crc = false;
        _len = 1;
        _crc = 0xFFFF;
        _in_object = true;
        _depth = 0;
        _is_object = 0;
        _target = Target::DESCEND;
        _target_candidates = (_num_fields == 64) ? ~0ULL : ((1ULL << _num_fields) - 1);
        return push(c, true);

    case State::RESYNC:
        if (c == '\n') {
            _state = State::IDLE;
        }
        return Result::NONE;

    default:
        break;
    }

    if (++_len > MAX_PACKET_LEN) {
        return fail(c, "packet too long");
    }
    return step(c);
}

__attribute__((always_inline)) inline Result AP_JSON_FieldParser::step(uint8_t c)
{
    if (c == '\n' && _state != State::AFTER_OBJECT && _state != State::CRC_END) {
        return fail(c, "line ended inside the packet");
    }

    // only a byte ending a number is processed twice, by the loop
    while (true) {
        switch (_state) {
        case State::OBJ_FIRST:
            if (is_ws(c)) {
                return Result::NONE;
            }
            if (c == '}') {
                return close_container(c, true);
            }
            if (c != '"') {
                return fail(c, "expected a key or '}'");
            }
            _string_is_key = true;
            _key_len = 0;
            _key_hash = 0;
            _key_matchable = true;
            _expect_low_surrogate = false;
            _state = State::STRING;
            return Result::NONE;

        case State::OBJ_KEY:
            if (is_ws(c)) {
                return Result::NONE;
            }
            if (c != '"') {
                return fail(c, "expected a key");
            }
            _string_is_key = true;
            _key_len = 0;
            _key_hash = 0;
            _key_matchable = true;
            _expect_low_surrogate = false;
            _state = State::STRING;
            return Result::NONE;

        case State::COLON:
            if (is_ws(c)) {
                return Result::NONE;
            }
            if (c != ':') {
                return fail(c, "expected ':'");
            }
            _state = State::VALUE;
            return Result::NONE;

        case State::VALUE:
            if (is_ws(c)) {
                return Result::NONE;
            }
            return start_value(c);

        case State::ARR_FIRST:
            if (is_ws(c)) {
                return Result::NONE;
            }
            if (c == ']') {
                return close_container(c, false);
            }
            if (_array_field[_depth - 1] >= 0) {
                _target = Target::ELEMENT;
                _target_field = _array_field[_depth - 1];
                _target_discard = (_array_discard >> (_depth - 1)) & 1U;
            } else {
                _target = Target::SKIP;
            }
            return start_value(c);

        case State::AFTER_VALUE:
            if (is_ws(c)) {
                return Result::NONE;
            }
            if (c == ',') {
                if (_is_object & (1U << (_depth - 1))) {
                    _state = State::OBJ_KEY;
                } else {
                    if (_array_field[_depth - 1] >= 0) {
                        _target = Target::ELEMENT;
                        _target_field = _array_field[_depth - 1];
                        _target_discard = (_array_discard >> (_depth - 1)) & 1U;
                    } else {
                        _target = Target::SKIP;
                    }
                    _state = State::VALUE;
                }
                return Result::NONE;
            }
            if (c == '}' || c == ']') {
                return close_container(c, c == '}');
            }
            return fail(c, "expected ',' or the end of a container");

        case State::STRING:
            if (c == '"') {
                if (_expect_low_surrogate) {
                    return fail(c, "unpaired surrogate in string");
                }
                if (_string_is_key) {
                    return end_key();
                }
                after_value();
                return Result::NONE;
            }
            if (c < 0x20) {
                return fail(c, "control character in string");
            }
            if (c == '\\') {
                _key_matchable = false;
                _state = State::STRING_ESCAPE;
                return Result::NONE;
            }
            if (_expect_low_surrogate) {
                return fail(c, "unpaired surrogate in string");
            }
            if (_string_is_key && _key_matchable) {
                if (c == '.' || _key_len >= MAX_KEY_LEN) {
                    _key_matchable = false;
                } else {
                    _key[_key_len++] = char(c);
                    _key_hash = key_hash(_key_hash, c);
                }
            }
            return Result::NONE;

        case State::STRING_ESCAPE:
            if (_expect_low_surrogate && c != 'u') {
                return fail(c, "unpaired surrogate in string");
            }
            switch (c) {
            case '"':
            case '\\':
            case '/':
            case 'b':
            case 'f':
            case 'n':
            case 'r':
            case 't':
                _state = State::STRING;
                return Result::NONE;
            case 'u':
                _hex_digits = 0;
                _hex_value = 0;
                _state = State::STRING_HEX;
                return Result::NONE;
            default:
                return fail(c, "invalid escape in string");
            }

        case State::STRING_HEX: {
            uint8_t nibble;
            if (c >= '0' && c <= '9') {
                nibble = c - '0';
            } else if (c >= 'a' && c <= 'f') {
                nibble = c - 'a' + 10;
            } else if (c >= 'A' && c <= 'F') {
                nibble = c - 'A' + 10;
            } else {
                return fail(c, "invalid \\u escape");
            }
            _hex_value = (_hex_value << 4) | nibble;
            if (++_hex_digits < 4) {
                return Result::NONE;
            }
            const bool high = _hex_value >= 0xD800 && _hex_value <= 0xDBFF;
            const bool low = _hex_value >= 0xDC00 && _hex_value <= 0xDFFF;
            if (_expect_low_surrogate ? !low : low) {
                return fail(c, "unpaired surrogate in string");
            }
            _expect_low_surrogate = high;
            _state = State::STRING;
            return Result::NONE;
        }

        case State::NUMBER: {
            const bool digit = c >= '0' && c <= '9';
            bool ends = false;
            switch (_number_state) {
            case NumberState::MINUS:
                if (c == 'I' || c == 'i' || c == 'n' || c == 'N') {
                    return start_literal(c, true);
                }
                if (!digit) {
                    return fail(c, "invalid number");
                }
                _number_state = (c == '0') ? NumberState::ZERO : NumberState::INTEGER;
                break;
            case NumberState::ZERO:
            case NumberState::INTEGER:
                if (digit) {
                    if (_number_state == NumberState::ZERO) {
                        return fail(c, "leading zero in number");
                    }
                } else if (c == '.') {
                    _number_state = NumberState::POINT;
                } else if (c == 'e' || c == 'E') {
                    _number_state = NumberState::EXP;
                } else {
                    ends = true;
                }
                break;
            case NumberState::POINT:
                if (!digit) {
                    return fail(c, "invalid number");
                }
                _number_state = NumberState::FRACTION;
                break;
            case NumberState::FRACTION:
                if (c == 'e' || c == 'E') {
                    _number_state = NumberState::EXP;
                } else if (!digit) {
                    ends = true;
                }
                break;
            case NumberState::EXP:
                if (c == '+' || c == '-') {
                    _number_state = NumberState::EXP_SIGN;
                } else if (digit) {
                    _number_state = NumberState::EXP_DIGITS;
                } else {
                    return fail(c, "invalid number");
                }
                break;
            case NumberState::EXP_SIGN:
                if (!digit) {
                    return fail(c, "invalid number");
                }
                _number_state = NumberState::EXP_DIGITS;
                break;
            case NumberState::EXP_DIGITS:
                if (!digit) {
                    ends = true;
                }
                break;
            }
            if (!ends) {
                if (_target == Target::FIELD || _target == Target::ELEMENT) {
                    if (_number_len >= sizeof(_number) - 1) {
                        return fail(c, "number too long");
                    }
                    _number[_number_len++] = char(c);
                }
                return Result::NONE;
            }
            // this byte follows the number: convert it, then process
            // the byte again as what comes after a value
            if (end_number(c) == Result::ERROR) {
                return Result::ERROR;
            }
            continue;
        }

        case State::LITERAL:
            if (c != uint8_t(_literal[_literal_pos])) {
                // another word with the same start, as null and nan
                const char *other = nullptr;
                for (const char *w : literals) {
                    if (strncmp(w, _literal, _literal_pos) == 0 && uint8_t(w[_literal_pos]) == c) {
                        other = w;
                        break;
                    }
                }
                if (other == nullptr) {
                    return fail(c, "invalid literal");
                }
                _literal = other;
            }
            if (_literal[++_literal_pos] == 0) {
                return end_literal(c);
            }
            return Result::NONE;

        case State::AFTER_OBJECT:
            if (is_ws(c)) {
                return Result::NONE;
            }
            if (c == '*') {
                _crc_digits = 0;
                _crc_received = 0;
                _state = State::CRC_DIGITS;
                return Result::NONE;
            }
            if (c == '\n') {
                _state = State::IDLE;
                return Result::PACKET;
            }
            return fail(c, "unexpected data after the packet");

        case State::CRC_DIGITS: {
            uint8_t nibble;
            if (c >= '0' && c <= '9') {
                nibble = c - '0';
            } else if (c >= 'a' && c <= 'f') {
                nibble = c - 'a' + 10;
            } else if (c >= 'A' && c <= 'F') {
                nibble = c - 'A' + 10;
            } else {
                return fail(c, "invalid CRC");
            }
            _crc_received = (_crc_received << 4) | nibble;
            if (++_crc_digits == 4) {
                _state = State::CRC_END;
            }
            return Result::NONE;
        }

        case State::CRC_END:
            if (is_ws(c)) {
                return Result::NONE;
            }
            if (c != '\n') {
                return fail(c, "unexpected data after the CRC");
            }
            if (_crc_received != _crc) {
                return fail(c, "CRC mismatch");
            }
            _had_crc = true;
            _state = State::IDLE;
            return Result::PACKET;

        case State::IDLE:
        case State::RESYNC:
            // handled in feed()
            return Result::NONE;
        }
        return Result::NONE;
    }
}

Result AP_JSON_FieldParser::start_value(uint8_t c)
{
    const bool for_field = _target == Target::FIELD || _target == Target::ELEMENT;
    const Type type = for_field ? _fields[_target_field].type : Type::FLOAT;
    const bool array_field = _target == Target::FIELD &&
                             (type == Type::FLOAT_ARRAY || type == Type::DOUBLE_ARRAY);

    switch (c) {
    case '{':
        if (for_field) {
            return fail(c, "object where a field value was expected");
        }
        return push(c, true);

    case '[':
        if (for_field && !array_field) {
            return fail(c, "array where a single value was expected");
        }
        return push(c, false);

    case '"':
        if (for_field) {
            return fail(c, "string where a field value was expected");
        }
        _string_is_key = false;
        _expect_low_surrogate = false;
        _state = State::STRING;
        return Result::NONE;

    case 't':
    case 'f':
    case 'n':
    case 'N':
    case 'i':
    case 'I':
        return start_literal(c, false);

    default:
        if (c != '-' && (c < '0' || c > '9')) {
            return fail(c, "invalid value");
        }
        if (array_field) {
            return fail(c, "number where an array was expected");
        }
        _number_len = 0;
        _special = Special::NONE;
        if (for_field) {
            _number[_number_len++] = char(c);
        }
        _number_state = (c == '-') ? NumberState::MINUS :
                        (c == '0') ? NumberState::ZERO : NumberState::INTEGER;
        _state = State::NUMBER;
        return Result::NONE;
    }
}

Result AP_JSON_FieldParser::start_literal(uint8_t c, bool negative)
{
    _literal = nullptr;
    for (const char *w : literals) {
        if (uint8_t(w[0]) == c) {
            _literal = w;
            break;
        }
    }
    if (_literal == nullptr) {
        return fail(c, "invalid value");
    }
    if (_target == Target::FIELD || _target == Target::ELEMENT) {
        const Type type = _fields[_target_field].type;
        const bool boolean = c == 't' || c == 'f';
        // true and false only for a BOOL field, words for numbers only
        // for a number. An array field wants '[' first
        if (_target == Target::FIELD && (type == Type::FLOAT_ARRAY || type == Type::DOUBLE_ARRAY)) {
            return fail(c, "wrong type of value for field");
        }
        if (boolean != (_target == Target::FIELD && type == Type::BOOL)) {
            return fail(c, "wrong type of value for field");
        }
    }
    _literal_pos = 1;
    _literal_negative = negative;
    // a word for a number has no digits: none of the last number's may
    // be converted, nor _number_len left as it was
    _number_len = 0;
    _number[0] = 0;
    _state = State::LITERAL;
    return Result::NONE;
}

Result AP_JSON_FieldParser::push(uint8_t c, bool object)
{
    if (_depth >= MAX_DEPTH) {
        return fail(c, "nesting too deep");
    }
    const uint8_t d = _depth++;
    if (object) {
        _is_object |= (1U << d);
        _candidates[d] = (_target == Target::DESCEND) ? _target_candidates : 0;
        if (d < MAX_PATH_DEPTH - 1) {
            _descended[d] = 0;
        }
        _state = State::OBJ_FIRST;
    } else {
        _is_object &= ~(1U << d);
        _array_field[d] = (_target == Target::FIELD) ? _target_field : -1;
        _array_count[d] = 0;
        if (_target == Target::FIELD && _target_discard) {
            _array_discard |= (1U << d);
        } else {
            _array_discard &= ~(1U << d);
        }
        _state = State::ARR_FIRST;
    }
    return Result::NONE;
}

Result AP_JSON_FieldParser::close_container(uint8_t c, bool object)
{
    const uint8_t d = _depth - 1;
    if (((_is_object >> d) & 1U) != (object ? 1U : 0U)) {
        return fail(c, "mismatched bracket");
    }
    if (!object && _array_field[d] >= 0) {
        const uint8_t field = _array_field[d];
        if (_array_count[d] != _fields[field].count) {
            return fail(c, "array field has the wrong number of elements");
        }
        if (((_array_discard >> d) & 1U) == 0) {
            mark_found(field);
        }
    }
    _depth--;
    if (_depth == 0) {
        _in_object = false;
        _state = State::AFTER_OBJECT;
    } else {
        after_value();
    }
    return Result::NONE;
}

/*
  a key has been read: work out what its value is for
 */
Result AP_JSON_FieldParser::end_key()
{
    const uint8_t level = _depth - 1;
    uint64_t candidates = _candidates[level];
    if (level == 0) {
        candidates &= _bucket[_key_hash % KEY_BUCKETS];
    }
    _target = Target::SKIP;
    _state = State::COLON;
    if (!_key_matchable || _key_len == 0 || (candidates == 0 && level != 0)) {
        return Result::NONE;
    }

    int8_t leaf = -1;
    uint64_t below = 0;
    while (candidates != 0) {
        const uint8_t i = lowest_bit(candidates);
        candidates &= candidates - 1;
        const uint8_t *part_len = _part_len[i];
        if (part_len[level] != _key_len) {
            continue;
        }
        // offset of part number `level` in the path
        uint8_t offset = 0;
        for (uint8_t k = 0; k < level; k++) {
            offset += part_len[k] + 1;
        }
        const char *part = _fields[i].path + offset;
        if (part[0] != _key[0] || memcmp(part, _key, _key_len) != 0) {
            continue;
        }
        if (level + 1 == MAX_PATH_DEPTH || part_len[level + 1] == 0) {
            if (leaf < 0) {
                leaf = i;
            }
        } else {
            below |= 1ULL << i;
        }
    }

    if (leaf >= 0) {
        if ((_full_path_set & (1ULL << leaf)) != 0) {
            return fail('"', "duplicate key");
        }
        _target = Target::FIELD;
        _target_field = leaf;
        _target_discard = false;
        _full_path_set |= 1ULL << leaf;
        return Result::NONE;
    }

    if (level == 0) {
        uint64_t aliases = _bucket[_key_hash % KEY_BUCKETS];
        while (aliases != 0) {
            const uint8_t i = lowest_bit(aliases);
            aliases &= aliases - 1;
            if (_alias_len[i] != _key_len) {
                continue;
            }
            const char *alias = _fields[i].alias;
            if (alias[0] != _key[0] || memcmp(alias, _key, _key_len) != 0) {
                continue;
            }
            if ((_alias_set & (1ULL << i)) != 0) {
                return fail('"', "duplicate key");
            }
            _alias_set |= 1ULL << i;
            // the full path wins if a packet has both, but the value is
            // still checked so the result does not depend on key order
            _target = Target::FIELD;
            _target_field = i;
            _target_discard = (_full_path_set & (1ULL << i)) != 0;
            return Result::NONE;
        }
    }

    if (below != 0) {
        // within one object, the same key always leads to the same
        // fields and different keys to different ones
        if ((below & _descended[level]) != 0) {
            return fail('"', "duplicate key");
        }
        _descended[level] |= below;
        _target = Target::DESCEND;
        _target_candidates = below;
    }
    return Result::NONE;
}

float AP_JSON_FieldParser::number_as_float() const
{
    switch (_special) {
    case Special::NOT_A_NUMBER:
        return NAN;
    case Special::PLUS_INFINITY:
        return INFINITY;
    case Special::MINUS_INFINITY:
        return -INFINITY;
    case Special::NONE:
        break;
    }
    return AP_JSON_Number::to_float(_number);
}

double AP_JSON_FieldParser::number_as_double() const
{
    if (_special != Special::NONE) {
        return double(number_as_float());
    }
    return AP_JSON_Number::to_double(_number);
}

Result AP_JSON_FieldParser::end_number(uint8_t c)
{
    if (_target == Target::ELEMENT) {
        const Field &f = _fields[_target_field];
        const uint8_t d = _depth - 1;
        const uint8_t idx = _array_count[d];
        if (idx >= f.count) {
            return fail(c, "array field has the wrong number of elements");
        }
        if (!_target_discard) {
            _number[_number_len] = 0;
            if (f.type == Type::FLOAT_ARRAY) {
                static_cast<float *>(f.ptr)[idx] = number_as_float();
            } else {
                static_cast<double *>(f.ptr)[idx] = number_as_double();
            }
        }
        _array_count[d]++;
    } else if (_target == Target::FIELD && !_target_discard) {
        _number[_number_len] = 0;
        const Field &f = _fields[_target_field];
        switch (f.type) {
        case Type::FLOAT:
            *static_cast<float *>(f.ptr) = number_as_float();
            break;
        case Type::DOUBLE:
            *static_cast<double *>(f.ptr) = number_as_double();
            break;
        case Type::BOOL: {
            const double v = AP_JSON_Number::to_double(_number);
            set_bool(v > 0 || v < 0);
            break;
        }
        case Type::FLOAT_ARRAY:
        case Type::DOUBLE_ARRAY:
            // rejected in start_value()
            break;
        }
        mark_found(_target_field);
    }
    after_value();
    return Result::NONE;
}

Result AP_JSON_FieldParser::end_literal(uint8_t c)
{
    const char first = _literal[0];
    if (first == 't' || first == 'f') {
        if (_target == Target::FIELD && !_target_discard) {
            // start_literal() lets only a BOOL field get here
            set_bool(first == 't');
            mark_found(_target_field);
        }
        after_value();
        return Result::NONE;
    }
    if (_literal_negative && _literal[1] == 'u') {
        return fail(c, "invalid number");   // -null
    }
    _special = (first == 'n' || first == 'N') ? Special::NOT_A_NUMBER :
               _literal_negative ? Special::MINUS_INFINITY : Special::PLUS_INFINITY;
    return end_number(c);
}

void AP_JSON_FieldParser::after_value()
{
    _state = State::AFTER_VALUE;
}

void AP_JSON_FieldParser::set_bool(bool b)
{
    *static_cast<bool *>(_fields[_target_field].ptr) = b;
}

void AP_JSON_FieldParser::mark_found(uint8_t field)
{
    _found |= 1ULL << field;
}

Result AP_JSON_FieldParser::feed(const uint8_t *data, size_t len, size_t &consumed)
{
    size_t i = 0;
    Result r = Result::NONE;
    // the CRC covers the root object, from '{' to '}'. It is computed
    // over runs of bytes rather than one byte at a time: crc_from is the
    // start of the run not yet included
    size_t crc_from = 0;
    while (i < len && r == Result::NONE) {
        // runs of plain characters in a string, or of digits in a number,
        // are the bulk of a packet: take them without going through the
        // state machine for each byte
        if (_state == State::STRING && !_expect_low_surrogate) {
            // locals, so the compiler need not reload members after
            // every byte stored in _key
            const bool collect = _string_is_key && _key_matchable;
            uint8_t key_len = _key_len;
            uint8_t hash = _key_hash;
            bool matchable = collect;
            const uint8_t *p = data + i;
            const uint8_t *const end = data + len;
            if (matchable) {
                char *key = _key;
                while (p < end) {
                    const uint8_t c = *p;
                    if (c == '"' || c == '\\' || c < 0x20) {
                        break;
                    }
                    if (c == '.' || key_len >= MAX_KEY_LEN) {
                        matchable = false;
                    } else if (matchable) {
                        key[key_len++] = char(c);
                        hash = key_hash(hash, c);
                    }
                    p++;
                }
            } else {
                while (p < end) {
                    const uint8_t c = *p;
                    if (c == '"' || c == '\\' || c < 0x20) {
                        break;
                    }
                    p++;
                }
            }
            const size_t n = size_t(p - (data + i));
            if (n > 0) {
                if (collect) {
                    _key_len = key_len;
                    _key_hash = hash;
                    _key_matchable = matchable;
                }
                _len += n;
                i += n;
                if (_len > MAX_PACKET_LEN) {
                    consumed = i;
                    return fail(data[i - 1], "packet too long");
                }
                continue;
            }
        } else if (_state == State::NUMBER) {
            // the grammar of the rest of the number, checked here so most
            // number bytes never reach the state machine
            const bool store = _target == Target::FIELD || _target == Target::ELEMENT;
            NumberState ns = _number_state;
            uint8_t number_len = _number_len;
            char *number = _number;
            size_t n = 0;
            const uint8_t *p = data + i;
            while (p + n < data + len) {
                const uint8_t c = p[n];
                NumberState next;
                if (c >= '0' && c <= '9') {
                    if (ns == NumberState::ZERO) {
                        break;      // leading zero: the state machine reports it
                    }
                    next = (ns == NumberState::MINUS) ? (c == '0' ? NumberState::ZERO : NumberState::INTEGER) :
                           (ns == NumberState::POINT) ? NumberState::FRACTION :
                           (ns == NumberState::EXP || ns == NumberState::EXP_SIGN) ? NumberState::EXP_DIGITS : ns;
                } else if (c == '.' && (ns == NumberState::ZERO || ns == NumberState::INTEGER)) {
                    next = NumberState::POINT;
                } else if ((c == 'e' || c == 'E') &&
                           (ns == NumberState::ZERO || ns == NumberState::INTEGER || ns == NumberState::FRACTION)) {
                    next = NumberState::EXP;
                } else if ((c == '+' || c == '-') && ns == NumberState::EXP) {
                    next = NumberState::EXP_SIGN;
                } else {
                    break;          // the end of the number, or an error: the state machine decides
                }
                if (store) {
                    if (number_len >= sizeof(_number) - 1) {
                        consumed = i + n + 1;
                        return fail(c, "number too long");
                    }
                    number[number_len++] = char(c);
                }
                ns = next;
                n++;
            }
            if (n > 0) {
                _number_state = ns;
                _number_len = number_len;
                _len += n;
                i += n;
                if (_len > MAX_PACKET_LEN) {
                    consumed = i;
                    return fail(data[i - 1], "packet too long");
                }
                continue;
            }
        }
        const bool was_in_object = _in_object;
        r = feed_byte(data[i++]);
        if (_in_object != was_in_object) {
            if (_in_object) {
                // the '{' starting a packet
                crc_from = i - 1;
            } else if (r != Result::ERROR) {
                // the '}' ending the root object
                _crc = crc16_ccitt(data + crc_from, uint32_t(i - crc_from), _crc);
            }
        }
    }
    if (_in_object) {
        _crc = crc16_ccitt(data + crc_from, uint32_t(i - crc_from), _crc);
    }
    consumed = i;
    return r;
}
