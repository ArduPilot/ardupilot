/*
  ArduPilot JSON parser, based on picojson.h cloned from:
    https://github.com/kazuho/picojson
 */
/*
  Picojson copyright:

 * Copyright 2009-2010 Cybozu Labs, Inc.
 * Copyright 2011-2014 Kazuho Oku
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma GCC optimize("Os")

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_JSON.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

// defined below, used by load_json()
template <> bool AP_JSON::value::is<AP_JSON::value::object>() const;

/*
  load JSON file, returning a value object or nullptr on failure
 */
AP_JSON::value *AP_JSON::load_json(const char *filename)
{
    struct stat st;
    if (AP::FS().stat(filename, &st) != 0) {
        ::printf("No such json file %s\n", filename);
        return nullptr;
    }
    int fd = AP::FS().open(filename, O_RDONLY);
    if (fd == -1) {
        ::printf("failed to open json %s\n", filename);
        return nullptr;
    }
    char *buf = NEW_NOTHROW char[st.st_size+1];
    if (buf == nullptr) {
        AP::FS().close(fd);
        ::printf("failed to allocate json %s\n", filename);
        return nullptr;
    }
    if (AP::FS().read(fd, buf, st.st_size) != st.st_size) {
        ::printf("failed to read json %s\n", filename);
        delete[] buf;
        AP::FS().close(fd);
        return nullptr;
    }
    AP::FS().close(fd);

    // skip a UTF-8 byte order mark, which some editors write
    char *text = buf;
    if (st.st_size >= 3 && memcmp(text, "\xEF\xBB\xBF", 3) == 0) {
        text += 3;
    }
    // the whole file, so a NUL in it is a parse error
    const char *end = buf + st.st_size;

    /*
      remove comments, as not allowed by the parser, leaving any # in a
      string alone
     */
    bool in_string = false;
    for (char *p = text; p < end; p++) {
        if (in_string) {
            if (*p == '\\' && p + 1 < end) {
                p++;
            } else if (*p == '"') {
                in_string = false;
            }
        } else if (*p == '"') {
            in_string = true;
        } else if (*p == '#') {
            // clear to end of line
            while (p < end && *p != '\n' && *p != '\r') {
                *p++ = ' ';
            }
            p--;
        }
    }

    AP_JSON::value *obj = NEW_NOTHROW AP_JSON::value;
    if (obj == nullptr) {
        ::printf("Invalid allocate json for %s\n", filename);
        delete[] buf;
        return nullptr;
    }
    std::string err = AP_JSON::parse(*obj, std::string(text, end - text));
    if (!err.empty()) {
        ::printf("parse failed for json %s: %s\n", filename, err.c_str());
        delete obj;
        delete[] buf;
        return nullptr;
    }
    // callers expect an object
    if (!obj->is<AP_JSON::value::object>()) {
        ::printf("json %s is not an object\n", filename);
        delete obj;
        delete[] buf;
        return nullptr;
    }

    delete[] buf;
    return obj;
}

typedef AP_JSON::value::array array;
typedef AP_JSON::value::object object;
typedef AP_JSON::value value;
typedef AP_JSON::null null;

enum {
    null_type,
    boolean_type,
    number_type,
    string_type,
    array_type,
    object_type
};


AP_JSON::value::value() : type_(null_type), u_()
{
}

AP_JSON::value::value(int type, bool) : type_(type), u_()
{
    switch (type) {
#define INIT(p, v)                           \
  case p##type:                              \
    u_.p = v;                                \
    break
        INIT(boolean_, false);
        INIT(number_, 0.0);
        INIT(string_, NEW_NOTHROW std::string());
        INIT(array_, NEW_NOTHROW array());
        INIT(object_, NEW_NOTHROW object());
#undef INIT
    default:
        break;
    }
}

AP_JSON::value::value(bool b) : type_(boolean_type), u_()
{
    u_.boolean_ = b;
}

AP_JSON::value::value(double n) : type_(number_type), u_()
{
    u_.number_ = n;
}

AP_JSON::value::value(const std::string &s) : type_(string_type), u_()
{
    u_.string_ = NEW_NOTHROW std::string(s);
}

AP_JSON::value::value(const array &a) : type_(array_type), u_()
{
    u_.array_ = NEW_NOTHROW array(a);
}

AP_JSON::value::value(const object &o) : type_(object_type), u_()
{
    u_.object_ = NEW_NOTHROW object(o);
}

AP_JSON::value::value(std::string &&s) : type_(string_type), u_()
{
    u_.string_ = NEW_NOTHROW std::string(std::move(s));
}

AP_JSON::value::value(array &&a) : type_(array_type), u_()
{
    u_.array_ = NEW_NOTHROW array(std::move(a));
}

AP_JSON::value::value(object &&o) : type_(object_type), u_()
{
    u_.object_ = NEW_NOTHROW object(std::move(o));
}

AP_JSON::value::value(const char *s) : type_(string_type), u_()
{
    u_.string_ = NEW_NOTHROW std::string(s);
}

AP_JSON::value::value(const char *s, size_t len) : type_(string_type), u_()
{
    u_.string_ = NEW_NOTHROW std::string(s, len);
}

void AP_JSON::value::clear()
{
    switch (type_) {
#define DEINIT(p)           \
  case p##type:             \
    delete u_.p;            \
    break
        DEINIT(string_);
        DEINIT(array_);
        DEINIT(object_);
#undef DEINIT
    default:
        break;
    }
}

AP_JSON::value::~value()
{
    clear();
}

AP_JSON::value::value(const value &x) : type_(x.type_), u_()
{
    switch (type_) {
#define INIT(p, v)          \
  case p##type:             \
    u_.p = v;               \
    break
        INIT(string_, NEW_NOTHROW std::string(*x.u_.string_));
        INIT(array_, NEW_NOTHROW array(*x.u_.array_));
        INIT(object_, NEW_NOTHROW object(*x.u_.object_));
#undef INIT
    default:
        u_ = x.u_;
        break;
    }
}

value &AP_JSON::value::operator=(const value &x)
{
    if (this != &x) {
        value t(x);
        swap(t);
    }
    return *this;
}

AP_JSON::value::value(value &&x) : type_(null_type), u_()
{
    swap(x);
}
value &AP_JSON::value::operator=(value &&x)
{
    swap(x);
    return *this;
}

void AP_JSON::value::swap(value &x)
{
    std::swap(type_, x.type_);
    std::swap(u_, x.u_);
}

#define IS(ctype, jtype)                                    \
  template <> bool AP_JSON::value::is<ctype>() const {      \
    return type_ == jtype##_type;                           \
  }
IS(null, null)
IS(bool, boolean)
IS(std::string, string)
IS(array, array)
IS(object, object)
#undef IS
template <> bool AP_JSON::value::is<double>() const
{
    return type_ == number_type;
}

#define GET(ctype, var)                                                \
  template <> const ctype &AP_JSON::value::get<ctype>() const {        \
    return var;                                                        \
  }                                                                    \
  template <> ctype &AP_JSON::value::get<ctype>() {                    \
    return var;                                                        \
  }
GET(bool, u_.boolean_)
GET(std::string, *u_.string_)
GET(array, *u_.array_)
GET(object, *u_.object_)
GET(double, u_.number_)
#undef GET

#define SET(ctype, jtype, setter)                                      \
  template <> void AP_JSON::value::set<ctype>(const ctype &_val) {     \
    clear();                                                           \
    type_ = jtype##_type;                                              \
    setter                                                             \
  }
SET(bool, boolean, u_.boolean_ = _val;)
SET(std::string, string, u_.string_ = NEW_NOTHROW std::string(_val);)
SET(array, array, u_.array_ = NEW_NOTHROW array(_val);)
SET(object, object, u_.object_ = NEW_NOTHROW object(_val);)
SET(double, number, u_.number_ = _val;)
#undef SET

#define MOVESET(ctype, jtype, setter)                                  \
  template <> void AP_JSON::value::set<ctype>(ctype && _val) {         \
    clear();                                                           \
    type_ = jtype##_type;                                              \
    setter                                                             \
  }
MOVESET(std::string, string, u_.string_ = NEW_NOTHROW std::string(std::move(_val));)
MOVESET(array, array, u_.array_ = NEW_NOTHROW array(std::move(_val));)
MOVESET(object, object, u_.object_ = NEW_NOTHROW object(std::move(_val));)
#undef MOVESET

bool AP_JSON::value::evaluate_as_boolean() const
{
    switch (type_) {
    case null_type:
        return false;
    case boolean_type:
        return u_.boolean_;
    case number_type:
        return !is_zero(u_.number_);
    case string_type:
        return !u_.string_->empty();
    default:
        return true;
    }
}

const value &AP_JSON::value::get(const size_t idx) const
{
    static value s_null;
    return type_ == array_type && idx < u_.array_->size() ? (*u_.array_)[idx] : s_null;
}

const value &AP_JSON::value::get(const std::string &key) const
{
    static value s_null;
    if (type_ != object_type) {
        return s_null;
    }
    object::const_iterator i = u_.object_->find(key);
    return i != u_.object_->end() ? i->second : s_null;
}

bool AP_JSON::value::contains(const size_t idx) const
{
    return type_ == array_type && idx < u_.array_->size();
}

bool AP_JSON::value::contains(const std::string &key) const
{
    if (type_ != object_type) {
        return false;
    }
    object::const_iterator i = u_.object_->find(key);
    return i != u_.object_->end();
}

std::string AP_JSON::value::to_str() const
{
    switch (type_) {
    case null_type:
        return "null";
    case boolean_type:
        return u_.boolean_ ? "true" : "false";
    case number_type: {
        char buf[256];
        double tmp;
        snprintf(buf, sizeof(buf), fabs(u_.number_) < (1ULL << 53) && is_zero(modf(u_.number_, &tmp)) ? "%.f" : "%.17g", u_.number_);
        return buf;
    }
    case string_type:
        return *u_.string_;
    case array_type:
        return "array";
    case object_type:
        return "object";
    default:
        break;
    }
    return std::string();
}

template <typename Iter> void copy(const std::string &s, Iter oi)
{
    std::copy(s.begin(), s.end(), oi);
}

template <typename Iter> class input
{
protected:
    Iter cur_, end_;
    bool consumed_;
    int line_;

public:
    input(const Iter &first, const Iter &last) : cur_(first), end_(last), consumed_(false), line_(1)
    {
    }
    int getc()
    {
        if (consumed_) {
            if (*cur_ == '\n') {
                ++line_;
            }
            ++cur_;
        }
        if (cur_ == end_) {
            consumed_ = false;
            return -1;
        }
        consumed_ = true;
        return *cur_ & 0xff;
    }
    void ungetc()
    {
        consumed_ = false;
    }
    Iter cur() const
    {
        if (consumed_) {
            input<Iter> *self = const_cast<input<Iter> *>(this);
            self->consumed_ = false;
            ++self->cur_;
        }
        return cur_;
    }
    int line() const
    {
        return line_;
    }
    void skip_ws()
    {
        while (1) {
            int ch = getc();
            if (!(ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r')) {
                ungetc();
                break;
            }
        }
    }
    bool expect(const int expected)
    {
        skip_ws();
        if (getc() != expected) {
            ungetc();
            return false;
        }
        return true;
    }
    bool match(const std::string &pattern)
    {
        for (std::string::const_iterator pi(pattern.begin()); pi != pattern.end(); ++pi) {
            if (getc() != *pi) {
                ungetc();
                return false;
            }
        }
        return true;
    }
};

template <typename Iter> int _parse_quadhex(input<Iter> &in)
{
    int uni_ch = 0, hex;
    for (int i = 0; i < 4; i++) {
        if ((hex = in.getc()) == -1) {
            return -1;
        }
        if ('0' <= hex && hex <= '9') {
            hex -= '0';
        } else if ('A' <= hex && hex <= 'F') {
            hex -= 'A' - 0xa;
        } else if ('a' <= hex && hex <= 'f') {
            hex -= 'a' - 0xa;
        } else {
            in.ungetc();
            return -1;
        }
        uni_ch = uni_ch * 16 + hex;
    }
    return uni_ch;
}

template <typename String, typename Iter> bool _parse_codepoint(String &out, input<Iter> &in)
{
    int uni_ch;
    if ((uni_ch = _parse_quadhex(in)) == -1) {
        return false;
    }
    if (0xd800 <= uni_ch && uni_ch <= 0xdfff) {
        if (0xdc00 <= uni_ch) {
            // a second 16-bit of a surrogate pair appeared
            return false;
        }
        // first 16-bit of surrogate pair, get the next one
        if (in.getc() != '\\' || in.getc() != 'u') {
            in.ungetc();
            return false;
        }
        const int second = _parse_quadhex(in);
        if (!(0xdc00 <= second && second <= 0xdfff)) {
            return false;
        }
        uni_ch = ((uni_ch - 0xd800) << 10) | ((second - 0xdc00) & 0x3ff);
        uni_ch += 0x10000;
    }
    if (uni_ch < 0x80) {
        out.push_back(static_cast<char>(uni_ch));
    } else {
        if (uni_ch < 0x800) {
            out.push_back(static_cast<char>(0xc0 | (uni_ch >> 6)));
        } else {
            if (uni_ch < 0x10000) {
                out.push_back(static_cast<char>(0xe0 | (uni_ch >> 12)));
            } else {
                out.push_back(static_cast<char>(0xf0 | (uni_ch >> 18)));
                out.push_back(static_cast<char>(0x80 | ((uni_ch >> 12) & 0x3f)));
            }
            out.push_back(static_cast<char>(0x80 | ((uni_ch >> 6) & 0x3f)));
        }
        out.push_back(static_cast<char>(0x80 | (uni_ch & 0x3f)));
    }
    return true;
}

template <typename String, typename Iter> bool _parse_string(String &out, input<Iter> &in)
{
    while (1) {
        int ch = in.getc();
        if (ch < ' ') {
            in.ungetc();
            return false;
        } else if (ch == '"') {
            return true;
        } else if (ch == '\\') {
            if ((ch = in.getc()) == -1) {
                return false;
            }
            switch (ch) {
#define MAP(sym, val)                                                                                                              \
  case sym:                                                                                                                        \
    out.push_back(val);                                                                                                            \
    break
                MAP('"', '\"');
                MAP('\\', '\\');
                MAP('/', '/');
                MAP('b', '\b');
                MAP('f', '\f');
                MAP('n', '\n');
                MAP('r', '\r');
                MAP('t', '\t');
#undef MAP
            case 'u':
                if (!_parse_codepoint(out, in)) {
                    return false;
                }
                break;
            default:
                return false;
            }
        } else {
            out.push_back(static_cast<char>(ch));
        }
    }
    return false;
}

template <typename Context, typename Iter> bool _parse_array(Context &ctx, input<Iter> &in)
{
    if (!ctx.parse_array_start()) {
        return false;
    }
    size_t idx = 0;
    if (in.expect(']')) {
        return ctx.parse_array_stop(idx);
    }
    do {
        if (!ctx.parse_array_item(in, idx)) {
            return false;
        }
        idx++;
    } while (in.expect(','));
    return in.expect(']') && ctx.parse_array_stop(idx);
}

template <typename Context, typename Iter> bool _parse_object(Context &ctx, input<Iter> &in)
{
    if (!ctx.parse_object_start()) {
        return false;
    }
    if (in.expect('}')) {
        return true;
    }
    do {
        std::string key;
        if (!in.expect('"') || !_parse_string(key, in) || !in.expect(':')) {
            return false;
        }
        if (!ctx.parse_object_item(in, key)) {
            return false;
        }
    } while (in.expect(','));
    return in.expect('}');
}

/*
  scan a number following the JSON grammar exactly, returning an empty
  string if it is malformed
 */
template <typename Iter> std::string _parse_number(input<Iter> &in)
{
    std::string num_str;
    int ch = in.getc();
#define PUSH() do { num_str.push_back(static_cast<char>(ch)); ch = in.getc(); } while (0)
#define DIGIT() ('0' <= ch && ch <= '9')
    if (ch == '-') {
        PUSH();
    }
    if (ch == '0') {
        PUSH();
    } else if (DIGIT()) {
        while (DIGIT()) {
            PUSH();
        }
    } else {
        return std::string();
    }
    if (ch == '.') {
        PUSH();
        if (!DIGIT()) {
            return std::string();
        }
        while (DIGIT()) {
            PUSH();
        }
    }
    if (ch == 'e' || ch == 'E') {
        PUSH();
        if (ch == '+' || ch == '-') {
            PUSH();
        }
        if (!DIGIT()) {
            return std::string();
        }
        while (DIGIT()) {
            PUSH();
        }
    }
#undef PUSH
#undef DIGIT
    in.ungetc();
    return num_str;
}

template <typename Context, typename Iter> bool _parse(Context &ctx, input<Iter> &in)
{
    in.skip_ws();
    int ch = in.getc();
    switch (ch) {
#define IS(ch, text, op)                                                                                                           \
  case ch:                                                                                                                         \
    if (in.match(text) && op) {                                                                                                    \
      return true;                                                                                                                 \
    } else {                                                                                                                       \
      return false;                                                                                                                \
    }
        IS('n', "ull", ctx.set_null());
        IS('f', "alse", ctx.set_bool(false));
        IS('t', "rue", ctx.set_bool(true));
#undef IS
    case '"':
        return ctx.parse_string(in);
    case '[':
        return _parse_array(ctx, in);
    case '{':
        return _parse_object(ctx, in);
    default:
        if (('0' <= ch && ch <= '9') || ch == '-') {
            in.ungetc();
            const std::string num_str(_parse_number(in));
            if (num_str.empty()) {
                return false;
            }
            ctx.set_number(strtod(num_str.c_str(), nullptr));
            return true;
        }
        break;
    }
    in.ungetc();
    return false;
}

// nested arrays and objects allowed, so a hostile document can not
// recurse deep enough to overflow the stack
#define AP_JSON_MAX_DEPTH 16

class default_parse_context
{
protected:
    value *out_;
    // arrays and objects that may still be opened below this value
    size_t depth_;

public:
    default_parse_context(value *out, size_t depth = AP_JSON_MAX_DEPTH) : out_(out), depth_(depth)
    {
    }
    bool set_null()
    {
        *out_ = value();
        return true;
    }
    bool set_bool(bool b)
    {
        *out_ = value(b);
        return true;
    }
    bool set_number(double f)
    {
        *out_ = value(f);
        return true;
    }
    template <typename Iter> bool parse_string(input<Iter> &in)
    {
        *out_ = value(string_type, false);
        return _parse_string(out_->get<std::string>(), in);
    }
    bool parse_array_start()
    {
        if (depth_ == 0) {
            return false;
        }
        *out_ = value(array_type, false);
        return true;
    }
    template <typename Iter> bool parse_array_item(input<Iter> &in, size_t)
    {
        array &a = out_->get<array>();
        a.push_back(value());
        default_parse_context ctx(&a.back(), depth_ - 1);
        return _parse(ctx, in);
    }
    bool parse_array_stop(size_t)
    {
        return true;
    }
    bool parse_object_start()
    {
        if (depth_ == 0) {
            return false;
        }
        *out_ = value(object_type, false);
        return true;
    }
    template <typename Iter> bool parse_object_item(input<Iter> &in, const std::string &key)
    {
        object &o = out_->get<object>();
        // a repeated key would silently replace the earlier value
        std::pair<object::iterator, bool> item = o.emplace(key, value());
        if (!item.second) {
            return false;
        }
        default_parse_context ctx(&item.first->second, depth_ - 1);
        return _parse(ctx, in);
    }

private:
    default_parse_context(const default_parse_context &);
    default_parse_context &operator=(const default_parse_context &);
};

template <typename Context, typename Iter> Iter _parse(Context &ctx, const Iter &first, const Iter &last, std::string *err)
{
    input<Iter> in(first, last);
    if (!_parse(ctx, in) && err != NULL) {
        char buf[64];
        snprintf(buf, sizeof(buf), "syntax error at line %d near: ", in.line());
        *err = buf;
        while (1) {
            int ch = in.getc();
            if (ch == -1 || ch == '\n') {
                break;
            } else if (ch >= ' ') {
                err->push_back(static_cast<char>(ch));
            }
        }
    }
    return in.cur();
}

template <typename Iter> Iter parse(value &out, const Iter &first, const Iter &last, std::string *err)
{
    default_parse_context ctx(&out);
    return _parse(ctx, first, last, err);
}

std::string AP_JSON::parse(value &out, const std::string &s)
{
    std::string err;
    std::string::const_iterator p = ::parse(out, s.begin(), s.end(), &err);
    if (err.empty()) {
        // only whitespace may follow the value
        while (p != s.end() && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
            p++;
        }
        if (p != s.end()) {
            err = "trailing characters after JSON value";
        }
    }
    return err;
}
