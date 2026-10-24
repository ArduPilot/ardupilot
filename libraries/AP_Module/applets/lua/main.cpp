// SPDX-License-Identifier: GPL-3.0-or-later
#include <ap_module.hpp>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <ctype.h>
extern "C" {
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
}

using namespace ap_module;

namespace
{
Context *host;
lua_State *active_state;
uint32_t random_state;
uint32_t instructions_left;
bool overtime;
int failure_status;

[[noreturn]] void fail(const char *message)
{
    luaL_error(active_state, "%s", message);
    __builtin_unreachable();
}
void check(Error error)
{
    if (error != Error::Ok) {
        luaL_error(active_state, "module API error %d", int(error));
    }
}
template<class T> T checked(Result<T> result)
{
    check(result.error);
    return result.value;
}
StringView string_arg(lua_State *L, int index)
{
    size_t size;
    const char *text = luaL_checklstring(L, index, &size);
    return {text, size};
}
uint32_t unsigned_arg(lua_State *L, int index, uint32_t maximum)
{
    const lua_Integer value = luaL_checkinteger(L, index);
    luaL_argcheck(L, value >= 0 && uint32_t(value) <= maximum, index, "out of range");
    return uint32_t(value);
}

// Lua's snprintf calls contain one conversion. Integer/string formatting stays
// local; floating-point rendering uses the same HAL formatter as normal Lua.
struct Output {
    char *data;
    size_t capacity;
    size_t length;
    void put(char c)
    {
        if (length + 1 < capacity) {
            data[length] = c;
        }
        length++;
    }
    void text(const char *s, size_t n)
    {
        for (size_t i = 0; i < n; i++) {
            put(s[i]);
        }
    }
};
int format(char *dst, size_t capacity, const char *fmt, va_list args)
{
    Output output{dst, capacity, 0};
    while (*fmt) {
        if (*fmt != '%') {
            output.put(*fmt++);
            continue;
        }
        const char *start = fmt++;
        if (*fmt == '%') {
            output.put(*fmt++);
            continue;
        }
        bool left = false, zero = false, plus = false, space = false, alternate = false;
        for (; *fmt && strchr("-+ #0", *fmt); fmt++) {
            left |= *fmt == '-'; zero |= *fmt == '0'; plus |= *fmt == '+';
            space |= *fmt == ' '; alternate |= *fmt == '#';
        }
        unsigned width = 0;
        while (*fmt >= '0' && *fmt <= '9') {
            width = width * 10 + unsigned(*fmt++ - '0');
            if (width > 255) {
                fail("format width exceeds 255");
            }
        }
        int precision = -1;
        if (*fmt == '.') {
            precision = 0;
            fmt++;
            while (*fmt >= '0' && *fmt <= '9') {
                precision = precision * 10 + (*fmt++ - '0');
                if (precision > 99) {
                    fail("format precision exceeds 99");
                }
            }
        }
        unsigned longs = 0;
        while (*fmt == 'l') {
            longs++;
            fmt++;
        }
        const char kind = *fmt++;
        char field[256];
        size_t count = 0;
        const char *text = field;
        if (kind && strchr("fFeEgG", kind)) {
            PrintfPacket packet{};
            const size_t n = size_t(fmt - start);
            if (n >= sizeof(packet.format.data) || precision > 60) {
                fail("unsupported float format");
            }
            memcpy(packet.format.data, start, n);
            packet.count = 1;
            packet.tags.data[0] = 5;
            const double value = va_arg(args, double);
            memcpy(&packet.values.data[0], &value, sizeof(value));
            const auto result = checked(host->string().format_printf(packet));
            if (result.length > 255) {
                fail("invalid formatter result");
            }
            output.text(reinterpret_cast<const char *>(result.text.data), result.length);
            continue;
        }
        if (kind == 's') {
            text = va_arg(args, const char *);
            count = strlen(text);
            if (precision >= 0 && count > unsigned(precision)) {
                count = unsigned(precision);
            }
        } else if (kind == 'c') {
            field[count++] = char(va_arg(args, int));
        } else if (kind && strchr("diuoxXp", kind)) {
            const bool signed_value = kind == 'd' || kind == 'i';
            const unsigned base = kind == 'o' ? 8 : (kind == 'x' || kind == 'X' || kind == 'p' ? 16 : 10);
            uint64_t value;
            bool negative = false;
            if (kind == 'p') {
                value = uintptr_t(va_arg(args, void *));
                alternate = true;
            } else if (signed_value) {
                const int64_t number = longs >= 2 ? va_arg(args, long long) :
                                       longs ? va_arg(args, long) : va_arg(args, int);
                negative = number < 0;
                value = negative ? -uint64_t(number) : uint64_t(number);
            } else {
                value = longs >= 2 ? va_arg(args, unsigned long long) :
                        longs ? va_arg(args, unsigned long) : va_arg(args, unsigned);
            }
            char digits[64];
            unsigned n = 0;
            const char *alphabet = kind == 'X' ? "0123456789ABCDEF" : "0123456789abcdef";
            const bool nonzero = value != 0;
            const bool explicit_precision = precision >= 0;
            if (nonzero || precision != 0) {
                do {
                    digits[n++] = alphabet[value % base];
                    value /= base;
                } while (value);
            }
            if (negative) {
                field[count++] = '-';
            } else if (signed_value && (plus || space)) {
                field[count++] = plus ? '+' : ' ';
            }
            if (alternate && base == 8 && (n == 0 || digits[n - 1] != '0') && precision <= int(n)) {
                precision = int(n + 1);
            }
            if (alternate && base == 16 && (nonzero || kind == 'p')) {
                field[count++] = '0';
                field[count++] = kind == 'X' ? 'X' : 'x';
            }
            unsigned zeros = precision > int(n) ? unsigned(precision) - n : 0;
            if (zero && !left && !explicit_precision && width > count + n) {
                zeros = width - unsigned(count) - n;
            }
            while (zeros--) {
                field[count++] = '0';
            }
            while (n) {
                field[count++] = digits[--n];
            }
        } else {
            // %a is deliberately unsupported until a local hexadecimal float
            // formatter is supplied; never pass an unsupported format to HAL.
            fail("unsupported module format conversion");
        }
        if (!left) {
            for (size_t n = count; n < width; n++) {
                output.put(' ');
            }
        }
        output.text(text, count);
        if (left) {
            for (size_t n = count; n < width; n++) {
                output.put(' ');
            }
        }
    }
    if (capacity) {
        dst[output.length < capacity ? output.length : capacity - 1] = 0;
    }
    return int(output.length);
}

void *allocate(void *, void *ptr, size_t old_size, size_t new_size)
{
    if (new_size == 0) {
        host->deallocate(ptr);
        return nullptr;
    }
    // Lua requires shrink to succeed, even when the heap is exhausted.
    if (ptr != nullptr && new_size <= old_size) {
        return ptr;
    }
    if (new_size > UINT32_MAX) {
        return nullptr;
    }
    void *next = host->allocate(uint32_t(new_size), alignof(max_align_t));
    if (next != nullptr) {
        if (ptr != nullptr) {
            memcpy(next, ptr, old_size);
            host->deallocate(ptr);
        }
    }
    return next;
}

void hook(lua_State *L, lua_Debug *)
{
    if (host->stop_requested() || overtime || instructions_left <= 100) {
        overtime = true;
        lua_sethook(L, hook, LUA_MASKCOUNT, 1);
        luaL_error(L, host->stop_requested() ? "module stopped" : "Exceeded CPU time");
    }
    instructions_left -= 100;
}
void reset_hook(lua_State *L)
{
    overtime = false;
    instructions_left = LUA_MODULE_INSTRUCTIONS;
    lua_sethook(L, hook, LUA_MASKCOUNT, 100);
}

int send_text(lua_State *L)
{
    const auto severity = unsigned_arg(L, 2, 7);
    const auto text = string_arg(L, 3);
    check(host->gcs().send_text(MavSeverity(severity), text).error);
    return 0;
}
int send_named_float(lua_State *L)
{
    const auto name = string_arg(L, 2);
    const float value = luaL_checknumber(L, 3);
    check(host->gcs().send_named_float(name, value).error);
    return 0;
}
int send_named_int(lua_State *L)
{
    const auto name = string_arg(L, 2);
    const int32_t value = luaL_checkinteger(L, 3);
    check(host->gcs().send_named_int(name, value).error);
    return 0;
}
int attitude(lua_State *L)
{
    const int axis = lua_tointeger(L, lua_upvalueindex(1));
    const auto result = axis == 0 ? host->ahrs().get_roll_rad() :
                        axis == 1 ? host->ahrs().get_pitch_rad() : host->ahrs().get_yaw_rad();
    lua_pushnumber(L, checked(result));
    return 1;
}
int vector_field(lua_State *L)
{
    auto *v = static_cast<Vector3f *>(luaL_checkudata(L, 1, "Vector3f"));
    const int axis = lua_tointeger(L, lua_upvalueindex(1));
    float &value = axis == 0 ? v->x : axis == 1 ? v->y : v->z;
    if (lua_gettop(L) == 2) {
        value = luaL_checknumber(L, 2);
        return 0;
    }
    lua_pushnumber(L, value);
    return 1;
}
int new_vector(lua_State *L)
{
    auto *v = static_cast<Vector3f *>(lua_newuserdata(L, sizeof(Vector3f)));
    *v = {};
    luaL_setmetatable(L, "Vector3f");
    return 1;
}
int gyro(lua_State *L)
{
    const Vector3f value = checked(host->ahrs().get_gyro());
    new_vector(L);
    *static_cast<Vector3f *>(lua_touserdata(L, -1)) = value;
    return 1;
}
uint32_t uint_value(lua_State *L, int index)
{
    const auto *boxed = static_cast<uint32_t *>(luaL_testudata(L, index, "uint32_t"));
    if (boxed != nullptr) {
        return *boxed;
    }
    if (lua_isinteger(L, index)) {
        return uint32_t(lua_tointeger(L, index));
    }
    const double n = luaL_checknumber(L, index);
    luaL_argcheck(L, n >= 0 && n <= UINT32_MAX, index, "uint32 out of range");
    return uint32_t(n);
}
int push_uint(lua_State *L, uint32_t value)
{
    *static_cast<uint32_t *>(lua_newuserdata(L, sizeof(value))) = value;
    luaL_setmetatable(L, "uint32_t");
    return 1;
}
int uint_constructor(lua_State *L)
{
    return push_uint(L, lua_gettop(L) ? uint_value(L, 1) : 0);
}
int uint_float(lua_State *L)
{
    lua_pushnumber(L, float(uint_value(L, 1)));
    return 1;
}
int uint_int(lua_State *L)
{
    lua_pushinteger(L, int32_t(uint_value(L, 1)));
    return 1;
}
int uint_string(lua_State *L)
{
    char text[12];
    snprintf(text, sizeof(text), "%u", unsigned(uint_value(L, 1)));
    lua_pushstring(L, text);
    return 1;
}
int uint_operator(lua_State *L)
{
    const uint32_t a = uint_value(L, 1), b = uint_value(L, 2);
    switch (lua_tointeger(L, lua_upvalueindex(1))) {
    case 0: return push_uint(L, a + b);
    case 1: return push_uint(L, a - b);
    case 2: lua_pushboolean(L, a < b); break;
    case 3: lua_pushboolean(L, a <= b); break;
    default: lua_pushboolean(L, a == b); break;
    }
    return 1;
}
int clock_value(lua_State *L)
{
    uint64_t value = host->time_us();
    if (lua_tointeger(L, lua_upvalueindex(1)) == 0) {
        value /= 1000;
    }
    return push_uint(L, uint32_t(value));
}
int gps_status(lua_State *L)
{
    const uint8_t index = uint8_t(unsigned_arg(L, 2, 255));
    lua_pushinteger(L, checked(host->gps().status(index)));
    return 1;
}
int param_get(lua_State *L)
{
    const auto value = checked(host->param().get(string_arg(L, 2)));
    if (!value.present) {
        return 0;
    }
    lua_pushnumber(L, value.value);
    return 1;
}
int param_set(lua_State *L)
{
    const auto name = string_arg(L, 2);
    const float value = luaL_checknumber(L, 3);
    const auto result = host->param().set(name, value);
    lua_pushboolean(L, result.error == Error::Invalid ? false : checked(result));
    return 1;
}
int param_add_table(lua_State *L)
{
    const auto key = unsigned_arg(L, 2, 200);
    const auto name = string_arg(L, 3);
    const auto count = unsigned_arg(L, 4, 63);
    lua_pushboolean(L, checked(host->param().add_table(uint8_t(key), name, uint8_t(count))));
    return 1;
}
int param_add(lua_State *L)
{
    const auto key = unsigned_arg(L, 2, 200);
    const auto index = unsigned_arg(L, 3, 63);
    const auto name = string_arg(L, 4);
    const float value = luaL_checknumber(L, 5);
    lua_pushboolean(L, checked(host->param().add_param(uint8_t(key), uint8_t(index), name, value)));
    return 1;
}
struct ParameterStorage {
    void *data;
};
ParameterStorage *parameter(lua_State *L)
{
    return static_cast<ParameterStorage *>(luaL_checkudata(L, 1, "Parameter"));
}
int parameter_gc(lua_State *L)
{
    auto *p = parameter(L);
    if (p->data != nullptr) {
        host->deallocate(p->data);
        p->data = nullptr;
    }
    return 0;
}
int parameter_init(lua_State *L)
{
    auto *p = parameter(L);
    const auto name = string_arg(L, 2);
    if (p->data == nullptr) {
        const auto layout = checked(host->parameter_cache().__layout());
        p->data = host->allocate(layout.size, layout.alignment);
        if (p->data == nullptr) {
            return luaL_error(L, "Parameter: out of memory");
        }
    }
    lua_pushboolean(L, checked(host->parameter_cache().init(uintptr_t(p->data), name)));
    return 1;
}
int parameter_constructor(lua_State *L)
{
    const bool named = !lua_isnoneornil(L, 1);
    if (named) {
        luaL_checkstring(L, 1);
    }
    auto *p = static_cast<ParameterStorage *>(lua_newuserdata(L, sizeof(ParameterStorage)));
    p->data = nullptr;
    luaL_setmetatable(L, "Parameter");
    if (named) {
        lua_insert(L, 1);
        parameter_init(L);
        if (!lua_toboolean(L, -1)) {
            return luaL_error(L, "No parameter: %s", lua_tostring(L, 2));
        }
        lua_settop(L, 1);
    }
    return 1;
}
int parameter_get(lua_State *L)
{
    const auto *p = parameter(L);
    if (p->data == nullptr) {
        return 0;
    }
    const auto result = checked(host->parameter_cache().get(uintptr_t(p->data)));
    if (!result.present) {
        return 0;
    }
    lua_pushnumber(L, result.value);
    return 1;
}
int parameter_set(lua_State *L)
{
    const auto *p = parameter(L);
    const float value = luaL_checknumber(L, 2);
    if (p->data == nullptr) {
        return luaL_error(L, "Parameter is not initialized");
    }
    const auto result = lua_tointeger(L, lua_upvalueindex(1)) != 0 ?
                        host->parameter_cache().set_and_save(uintptr_t(p->data), value) :
                        host->parameter_cache().set(uintptr_t(p->data), value);
    lua_pushboolean(L, checked(result));
    return 1;
}

void function(lua_State *L, const char *name, lua_CFunction fn, int selector = -1)
{
    if (selector < 0) {
        lua_pushcfunction(L, fn);
    } else {
        lua_pushinteger(L, selector);
        lua_pushcclosure(L, fn, 1);
    }
    lua_setfield(L, -2, name);
}
void metatable(lua_State *L, const char *name)
{
    luaL_newmetatable(L, name);
    lua_pushvalue(L, -1);
    lua_setfield(L, -2, "__index");
    lua_pushstring(L, name);
    lua_setfield(L, -2, "__metatable");
}
void environment(lua_State *L)
{
    lua_newtable(L);
    luaopen_base_sandbox(L);
    luaopen_math(L); lua_setfield(L, -2, "math");
    luaopen_string(L); lua_setfield(L, -2, "string");
    luaopen_table(L); lua_setfield(L, -2, "table");
    lua_newtable(L);
    function(L, "send_text", send_text);
    function(L, "send_named_float", send_named_float);
    function(L, "send_named_int", send_named_int);
    lua_setfield(L, -2, "gcs");
    lua_newtable(L);
    function(L, "get_roll_rad", attitude, 0);
    function(L, "get_pitch_rad", attitude, 1);
    function(L, "get_yaw_rad", attitude, 2);
    function(L, "get_gyro", gyro);
    lua_setfield(L, -2, "ahrs");
    lua_newtable(L);
    function(L, "get", param_get);
    function(L, "set", param_set);
    function(L, "add_table", param_add_table);
    function(L, "add_param", param_add);
    lua_setfield(L, -2, "param");
    lua_newtable(L);
    function(L, "status", gps_status);
    lua_setfield(L, -2, "gps");
    function(L, "Vector3f", new_vector);
    function(L, "uint32_t", uint_constructor);
    function(L, "Parameter", parameter_constructor);
    function(L, "millis", clock_value, 0);
    function(L, "micros", clock_value, 1);
}

constexpr unsigned MAX_SCRIPTS = 16;
constexpr uint32_t MAX_SOURCE = 65536;
struct Script {
    Array<uint8_t, 256> path;
    uint64_t deadline;
    int function_ref;
};
Script scripts[MAX_SCRIPTS];
unsigned script_count;

struct Reader {
    Script *script;
    uint32_t offset;
    uint32_t size;
    uint32_t crc;
    bool shebang;
    Array<uint8_t, 1024> bytes;
};
const char *read_source(lua_State *L, void *raw, size_t *length)
{
    auto &reader = *static_cast<Reader *>(raw);
    for (;;) {
        if (host->stop_requested()) {
            return (luaL_error(L, "module stopped"), nullptr);
        }
        if (reader.offset == reader.size) {
            *length = 0;
            return nullptr;
        }
        const uint32_t left = reader.size - reader.offset;
        const auto result = checked(host->filesystem().read_at(reader.script->path, reader.offset,
                                    uint16_t(left > 1024 ? 1024 : left)));
        if (!result.present || result.value.length == 0 || result.value.length > left || result.value.length > 1024) {
            return (luaL_error(L, "script read failed"), nullptr);
        }
        *length = result.value.length;
        memcpy(reader.bytes.data, result.value.data.data, *length);
        for (size_t i = 0; i < *length; i++) {
            reader.crc ^= reader.bytes.data[i];
            for (unsigned bit = 0; bit < 8; bit++) {
                reader.crc = (reader.crc >> 1) ^ (0xedb88320U & (0U - (reader.crc & 1)));
            }
        }
        size_t skip = 0;
        if (reader.offset == 0) {
            if (*length >= 3 && memcmp(reader.bytes.data, "\xef\xbb\xbf", 3) == 0) {
                skip = 3;
            }
            reader.shebang = skip < *length && reader.bytes.data[skip] == '#';
        }
        reader.offset += uint32_t(*length);
        if (reader.shebang) {
            while (skip < *length && reader.bytes.data[skip] != '\n') {
                skip++;
            }
            if (skip == *length) {
                continue;
            }
            reader.shebang = false;
        }
        *length -= skip;
        return reinterpret_cast<const char *>(reader.bytes.data + skip);
    }
}
bool load_script(lua_State *L, Script &script)
{
    const char *path = reinterpret_cast<const char *>(script.path.data);
    check(host->gcs().send_text(MavSeverity::INFO, {path, strlen(path)}).error);
    const auto before = checked(host->filesystem().stat(script.path));
    if (!before.present) {
        luaL_error(L, "cannot stat Lua script: %s", path);
    }
    if (before.value.directory) {
        return false;
    }
    if (before.value.size > MAX_SOURCE) {
        char message[256];
        snprintf(message, sizeof(message), "Lua script size %u exceeds %u byte limit: %s",
                 unsigned(before.value.size), unsigned(MAX_SOURCE), path);
        fail(message);
    }
    if (script_count == MAX_SCRIPTS) {
        fail("too many Lua scripts (maximum 16)");
    }
    Reader reader{&script, 0, before.value.size, 0xffffffffU, false, {}};
    char chunk[257];
    chunk[0] = '@';
    memcpy(chunk + 1, script.path.data, strlen(reinterpret_cast<const char *>(script.path.data)) + 1);
    const int status = lua_load(L, read_source, &reader, chunk, "t");
    if (status != LUA_OK) {
        failure_status = status;
        lua_error(L);
    }
    if (reader.offset != reader.size) {
        fail("incomplete script read");
    }
    const auto after = checked(host->filesystem().stat(script.path));
    if (!after.present || after.value.size != before.value.size || after.value.mtime != before.value.mtime) {
        fail("script changed during load");
    }
    environment(L);
    lua_setupvalue(L, -2, 1);
    script.function_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    char message[80];
    snprintf(message, sizeof(message), "Lua module: script %u size=%u crc=%08x", script_count + 1,
             unsigned(reader.size), unsigned(~reader.crc));
    check(host->gcs().send_text(MavSeverity::INFO, {message, strlen(message)}).error);
    return true;
}

int engine(lua_State *L)
{
    metatable(L, "Vector3f");
    function(L, "x", vector_field, 0); function(L, "y", vector_field, 1); function(L, "z", vector_field, 2);
    lua_pop(L, 1);
    metatable(L, "uint32_t");
    function(L, "tofloat", uint_float); function(L, "toint", uint_int); function(L, "__tostring", uint_string);
    function(L, "__add", uint_operator, 0); function(L, "__sub", uint_operator, 1);
    function(L, "__lt", uint_operator, 2); function(L, "__le", uint_operator, 3); function(L, "__eq", uint_operator, 4);
    lua_pop(L, 1);
    metatable(L, "Parameter");
    function(L, "init", parameter_init); function(L, "get", parameter_get);
    function(L, "set", parameter_set, 0); function(L, "set_and_save", parameter_set, 1);
    function(L, "__gc", parameter_gc);
    lua_pop(L, 1);
    // A private string-method table is shared, as in the built-in runtime.
    lua_pushliteral(L, "");
    lua_newtable(L);
    luaopen_string(L);
    lua_setfield(L, -2, "__index");
    lua_setmetatable(L, -2);
    lua_pop(L, 1);

    Array<uint8_t, 256> directory{};
    memcpy(directory.data, LUA_MODULE_DIRECTORY, sizeof(LUA_MODULE_DIRECTORY));
    for (uint32_t index = 0; index < 65536; index++) {
        if (host->stop_requested()) {
            fail("module stopped");
        }
        const auto entry = checked(host->filesystem().dir_entry(directory, uint16_t(index)));
        if (!entry.present) {
            break;
        }
        const auto &name = entry.value;
        if (name.length < 5 || name.length > 190 || memcmp(name.name.data + name.length - 4, ".lua", 4)) {
            continue;
        }
        Script script{};
        char basename[191];
        memcpy(basename, name.name.data, name.length);
        basename[name.length] = 0;
        const int length = snprintf(reinterpret_cast<char *>(script.path.data), sizeof(script.path.data), "%s/%s",
                                    LUA_MODULE_DIRECTORY, basename);
        if (length >= int(sizeof(script.path.data))) {
            fail("script path exceeds 255 bytes");
        }
        if (load_script(L, script)) {
            scripts[script_count++] = script;
        }
    }
    if (script_count == 0) {
        luaL_error(L, "no Lua scripts found in %s", LUA_MODULE_DIRECTORY);
    }
    while (!host->stop_requested()) {
        bool running = false;
        for (unsigned i = 0; i < script_count; i++) {
            auto &script = scripts[i];
            if (script.function_ref == LUA_NOREF) {
                continue;
            }
            running = true;
            const uint64_t now = host->time_us() / 1000;
            if (script.deadline > now) {
                continue;
            }
            reset_hook(L);
            const int top = lua_gettop(L);
            lua_rawgeti(L, LUA_REGISTRYINDEX, script.function_ref);
            const int status = lua_pcall(L, 0, LUA_MULTRET, 0);
            if (status != LUA_OK) {
                failure_status = status;
                return lua_error(L);
            }
            luaL_unref(L, LUA_REGISTRYINDEX, script.function_ref);
            script.function_ref = LUA_NOREF;
            const int count = lua_gettop(L) - top;
            if (count == 2) {
                if (!lua_isfunction(L, -2) || lua_type(L, -1) != LUA_TNUMBER) {
                    fail("expected callback and delay");
                }
                const double delay = lua_tonumber(L, -1);
                if (!(delay >= 0 && delay <= UINT32_MAX)) {
                    fail("invalid callback delay");
                }
                script.deadline = now + uint32_t(delay);
                lua_pop(L, 1);
                script.function_ref = luaL_ref(L, LUA_REGISTRYINDEX);
            } else if (count != 0) {
                fail("invalid callback result count");
            }
            lua_settop(L, top);
            reset_hook(L);
            lua_gc(L, LUA_GCCOLLECT, 0);
        }
        if (!running) {
            return 0;
        }
        // A positive sleep is required even for overdue/zero-delay callbacks:
        // ChibiOS does not time-slice equal-priority module workers.
        uint64_t next = UINT64_MAX;
        for (unsigned i = 0; i < script_count; i++) {
            if (scripts[i].function_ref != LUA_NOREF && scripts[i].deadline < next) {
                next = scripts[i].deadline;
            }
        }
        if (next == UINT64_MAX) {
            return 0;
        }
        const uint64_t now = host->time_us() / 1000;
        const uint64_t wait = next > now ? next - now : 1;
        const auto slept = host->sleep_ms(uint32_t(wait > 1000 ? 1000 : wait));
        if (!slept) {
            check(slept.error);
        }
    }
    return 0;
}
} // namespace

extern "C" int snprintf(char *dst, size_t size, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    const int result = format(dst, size, fmt, args);
    va_end(args);
    return result;
}
extern "C" void lua_abort()
{
    ap_module::fatal("unprotected Lua error");
}
extern "C" int rand()
{
    random_state = random_state * 1664525U + 1013904223U;
    return int(random_state >> 1);
}
extern "C" void srand(unsigned seed)
{
    random_state = seed;
}
extern "C" time_t time(time_t *result)
{
    const time_t value = time_t(host->time_us() / 1000000);
    if (result != nullptr) {
        *result = value;
    }
    return value;
}
extern "C" float strtof(const char *text, char **end)
{
    Array<uint8_t, 256> input{};
    const char *start = text;
    while (isspace(static_cast<unsigned char>(*start))) {
        start++;
    }
    const size_t size = strlen(start);
    memcpy(input.data, start, size > 255 ? 255 : size);
    const auto result = checked(host->string().strtof(input));
    if (result.consumed > size) {
        fail("invalid number conversion result");
    }
    // Failed or truncated conversions follow the C no-conversion contract.
    // This also lets tonumber(long_non_numeric_string) return nil normally.
    if (result.consumed == 0 || (size > 255 && result.consumed == 255 && !isspace(start[255]))) {
        if (end != nullptr) {
            *end = const_cast<char *>(text);
        }
        return 0;
    }
    if (end != nullptr) {
        *end = const_cast<char *>(start + result.consumed);
    }
    return result.value;
}
#define LUA_LIBM1(name) extern "C" float name(float x) { return checked(host->libm().name(x)); }
#define LUA_LIBM2(name) extern "C" float name(float x, float y) { return checked(host->libm().name(x, y)); }
LUA_LIBM1(acosf)
LUA_LIBM1(asinf)
LUA_LIBM1(ceilf)
LUA_LIBM1(cosf)
LUA_LIBM1(expf)
LUA_LIBM1(floorf)
LUA_LIBM1(logf)
LUA_LIBM1(sinf)
LUA_LIBM1(sqrtf)
LUA_LIBM1(tanf)
LUA_LIBM2(atan2f)
LUA_LIBM2(fmodf)
LUA_LIBM2(powf)

Result<void> lua_module_run(Context &context)
{
    host = &context;
    script_count = 0;
    failure_status = LUA_OK;
    memset(scripts, 0, sizeof(scripts));
    random_state = uint32_t(context.time_us());
    lua_State *L = lua_newstate(allocate, nullptr);
    if (L == nullptr) {
        return {Error::NoMemory};
    }
    active_state = L;
    lua_pushcfunction(L, engine);
    reset_hook(L);
    const int status = lua_pcall(L, 0, 0, 0);
    if (status != LUA_OK && !context.stop_requested()) {
        // Converting a numeric error object to text can allocate. Reporting
        // runs outside Lua protection, so only read an existing string here.
        const char *message = lua_type(L, -1) == LUA_TSTRING ? lua_tostring(L, -1) : nullptr;
        const char *error = status == LUA_ERRMEM || failure_status == LUA_ERRMEM ?
                            "Lua module: not enough memory; increase --heap-size and MOD_HEAP_MAX" :
                            message != nullptr ? message : "Lua module error";
        const size_t length = strlen(error);
        context.gcs().send_text(MavSeverity::ERROR, {error, length > 255 ? 255 : length});
    }
    // No nontrivial C++ automatic object spans a Lua API that can longjmp.
    // Finalizers only release module-owned storage; firmware reaps handles
    // after this entry returns and its worker has joined.
    // lua_close can execute script finalizers. Retain the hook so a finalizer
    // cannot prevent stop/reload with an infinite Lua loop.
    reset_hook(L);
    lua_close(L);
    active_state = nullptr;
    host = nullptr;
    return {context.stop_requested() ? Error::Stopped : status == LUA_OK ? Error::Ok : Error::Host};
}
