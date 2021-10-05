#include <AP_Common/AP_Common.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/HAL.h>
#include <AP_Logger/AP_Logger.h>

#include "lua_bindings.h"

#include "lua_boxed_numerics.h"
#include <AP_Scripting/lua_generated_bindings.h>

#include <AP_Scripting/AP_Scripting.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

int check_arguments(lua_State *L, int expected_arguments, const char *fn_name);
int check_arguments(lua_State *L, int expected_arguments, const char *fn_name) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
    if (expected_arguments < 0) {
       AP_HAL::panic("Lua: Attempted to check for negative arguments");
    }
#endif

    const int args = lua_gettop(L);
    if (args != expected_arguments) {
        return luaL_error(L, "%s expected %d arguments got %d", fn_name, expected_arguments, args);
    }
    return 0;
}

// millis
static int lua_millis(lua_State *L) {
    check_arguments(L, 0, "millis");

    new_uint32_t(L);
    *check_uint32_t(L, -1) = AP_HAL::millis();

    return 1;
}

// micros
static int lua_micros(lua_State *L) {
    check_arguments(L, 0, "micros");

    new_uint32_t(L);
    *check_uint32_t(L, -1) = AP_HAL::micros();

    return 1;
}

static int lua_mission_receive(lua_State *L) {
    check_arguments(L, 0, "mission_receive");

    ObjectBuffer<struct AP_Scripting::scripting_mission_cmd> *input = AP::scripting()->mission_data;

    if (input == nullptr) {
        // no mission items ever received
        return 0;
    }

    struct AP_Scripting::scripting_mission_cmd cmd;

    if (!input->pop(cmd)) {
        // no new item
        return 0;
    }

    new_uint32_t(L);
    *check_uint32_t(L, -1) = cmd.time_ms;

    lua_pushinteger(L, cmd.p1);
    lua_pushnumber(L, cmd.content_p1);
    lua_pushnumber(L, cmd.content_p2);
    lua_pushnumber(L, cmd.content_p3);

    return 5;
}

static const luaL_Reg global_functions[] =
{
    {"millis", lua_millis},
    {"micros", lua_micros},
    {"mission_receive", lua_mission_receive},
    {NULL, NULL}
};

static int AP_Logger_Write(lua_State *L) {
    AP_Logger * AP_logger = AP_Logger::get_singleton();
    if (AP_logger == nullptr) {
        return luaL_argerror(L, 1, "logger not supported on this firmware");
    }

    // check we have at least 4 arguments passed in
    const int args = lua_gettop(L);
    if (args < 4) {
        return luaL_argerror(L, args, "too few arguments");
    }

    const char * name = luaL_checkstring(L, 1);
    const char * labels = luaL_checkstring(L, 2);
    const char * fmt = luaL_checkstring(L, 3);

    // cheack the name, labels and format are not too long
    if (strlen(name) >= LS_NAME_SIZE) {
        return luaL_error(L, "Name must be 4 or less chars long");
    }
    uint8_t length = strlen(labels);
    if (length >= (LS_LABELS_SIZE - 7)) { // need 7 chars to add 'TimeUS,'
        return luaL_error(L, "labels must be less than 58 chars long");
    }
    // Count the number of commas
    uint8_t commas = 1;
    for (uint8_t i=0; i<length; i++) {
        if (labels[i] == ',') {
            commas++;
        }
    }

    length = strlen(fmt);
    if (length >= (LS_FORMAT_SIZE - 1)) { // need 1 char to add timestamp
        return luaL_error(L, "format must be less than 15 chars long");
    }

    // check the number of arguments matches the number of values in the label
    if (length != commas) {
        return luaL_argerror(L, args, "label does not match format");
    }

    bool have_units = false;
    if (args - 5 == length) {
        // check if there are enough arguments for units and multiplyers
        have_units = true;
    } else if (args - 3 != length) {
        // check the number of arguments matches the length of the foramt string
        return luaL_argerror(L, args, "format does not match No. of arguments");
    }

    // prepend timestamp to format and labels
    char label_cat[LS_LABELS_SIZE];
    strcpy(label_cat,"TimeUS,");
    strcat(label_cat,labels);
    char fmt_cat[LS_FORMAT_SIZE];
    strcpy(fmt_cat,"Q");
    strcat(fmt_cat,fmt);

    // Need to declare these here so they don't go out of scope
    char units_cat[LS_FORMAT_SIZE];
    char multipliers_cat[LS_FORMAT_SIZE];

    uint8_t field_start = 4;
    struct AP_Logger::log_write_fmt *f;
    if (!have_units) {
        // ask for a mesage type
        f = AP_logger->msg_fmt_for_name(name, label_cat, nullptr, nullptr, fmt_cat, true);

    } else {
        // read in units and multiplers strings
        field_start += 2;
        const char * units = luaL_checkstring(L, 4);
        const char * multipliers = luaL_checkstring(L, 5);

        if (length != strlen(units)) {
            return luaL_error(L, "units must be same length as format");
        }
        if (length != strlen(multipliers)) {
            return luaL_error(L, "multipliers must be same length as format");
        }

        // prepend timestamp to units and multiplyers
        strcpy(units_cat,"s");
        strcat(units_cat,units);

        strcpy(multipliers_cat,"F");
        strcat(multipliers_cat,multipliers);

        // ask for a mesage type
        f = AP_logger->msg_fmt_for_name(name, label_cat, units_cat, multipliers_cat, fmt_cat, true);
    }

    if (f == nullptr) {
        // unable to map name to a messagetype; could be out of
        // msgtypes, could be out of slots, ...
        return luaL_argerror(L, args, "could not map message type");
    }

    // work out how long the block will be
    int16_t msg_len = AP_logger->Write_calc_msg_len(fmt_cat);
    if (msg_len == -1) {
        return luaL_argerror(L, args, "unknown format");
    }

    luaL_Buffer buffer;
    luaL_buffinit(L, &buffer);

    // add logging headers
    const char header[2] = {(char)HEAD_BYTE1, (char)HEAD_BYTE2};
    luaL_addlstring(&buffer, header, sizeof(header));
    luaL_addlstring(&buffer, (char *)&f->msg_type, sizeof(f->msg_type));

    // timestamp is always first value
    const uint64_t now = AP_HAL::micros64();
    luaL_addlstring(&buffer, (char *)&now, sizeof(uint64_t));

    for (uint8_t i=field_start; i<=args; i++) {
        uint8_t charlen = 0;
        uint8_t index = have_units ? i-5 : i-3;
        switch(fmt_cat[index]) {
            // logger varable types not available to scripting
            // 'b': int8_t
            // 'h': int16_t
            // 'c': int16_t
            // 'd': double
            // 'H': uint16_t
            // 'C': uint16_t
            // 'Q': uint64_t
            // 'q': int64_t
            // 'a': arrays
            case 'i':
            case 'L':
            case 'e': {
                const lua_Integer tmp1 = luaL_checkinteger(L, i);
                luaL_argcheck(L, ((tmp1 >= INT32_MIN) && (tmp1 <= INT32_MAX)), i, "argument out of range");
                int32_t tmp = tmp1;
                luaL_addlstring(&buffer, (char *)&tmp, sizeof(int32_t));
                break;
            }
            case 'f': {
                float tmp = luaL_checknumber(L, i);
                luaL_argcheck(L, ((tmp >= -INFINITY) && (tmp <= INFINITY)), i, "argument out of range");
                luaL_addlstring(&buffer, (char *)&tmp, sizeof(float));
                break;
            }
            case 'n': {
                charlen = 4;
                break;
            }
            case 'M':
            case 'B': {
                const lua_Integer tmp1 = luaL_checkinteger(L, i);
                luaL_argcheck(L, ((tmp1 >= 0) && (tmp1 <= UINT8_MAX)), i, "argument out of range");
                uint8_t tmp = static_cast<uint8_t>(tmp1);
                luaL_addlstring(&buffer, (char *)&tmp, sizeof(uint8_t));
                break;
            }
            case 'I':
            case 'E': {
                const uint32_t tmp = coerce_to_uint32_t(L, i);
                luaL_addlstring(&buffer, (char *)&tmp, sizeof(uint32_t));
                break;
            }
            case 'N': {
                charlen = 16;
                break;
            }
            case 'Z': {
                charlen = 64;
                break;
            }
            default: {
                return luaL_error(L, "%c unsupported format",fmt_cat[i-3]);
            }
        }
        if (charlen != 0) {
            const char *tmp = luaL_checkstring(L, i);
            const size_t slen = strlen(tmp);
            if (slen > charlen) {
                return luaL_error(L, "arg %i too long for %c format",i,fmt_cat[i-3]);
            }
            char tstr[charlen];
            memcpy(tstr, tmp, slen);
            if (slen < charlen) {
                memset(&tstr[slen], 0, charlen-slen);
            }
            luaL_addlstring(&buffer, tstr, charlen);
        }
    }

    AP_logger->Safe_Write_Emit_FMT(f);

    luaL_pushresult(&buffer);
    AP_logger->WriteBlock(buffer.b,msg_len);

    return 0;
}

const luaL_Reg AP_Logger_functions[] = {
    {"write", AP_Logger_Write},
    {NULL, NULL}
};

static int lua_get_i2c_device(lua_State *L) {

    const int args = lua_gettop(L);
    if (args < 2) {
        return luaL_argerror(L, args, "require i2c bus and address");
    }
    if (args > 4) {
        return luaL_argerror(L, args, "too many arguments");
    }

    const lua_Integer bus_in = luaL_checkinteger(L, 1);
    luaL_argcheck(L, ((bus_in >= 0) && (bus_in <= 4)), 1, "bus out of range");
    const uint8_t bus = static_cast<uint8_t>(bus_in);

    const lua_Integer address_in = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((address_in >= 0) && (address_in <= 128)), 2, "address out of range");
    const uint8_t address = static_cast<uint8_t>(address_in);

    // optional arguments, use the same defaults as the hal get_device function
    uint32_t bus_clock = 400000;
    bool use_smbus = false;

    if (args > 2) {
        bus_clock = coerce_to_uint32_t(L, 3);

        if (args > 3) {
            use_smbus = static_cast<bool>(lua_toboolean(L, 4));
        }
    }

    static_assert(SCRIPTING_MAX_NUM_I2C_DEVICE >= 0, "There cannot be a negative number of I2C devices");
    if (AP::scripting()->num_i2c_devices >= SCRIPTING_MAX_NUM_I2C_DEVICE) {
        return luaL_argerror(L, 1, "no i2c devices available");;
    }

    AP::scripting()->_i2c_dev[AP::scripting()->num_i2c_devices] = new AP_HAL::OwnPtr<AP_HAL::I2CDevice>;
    if (AP::scripting()->_i2c_dev[AP::scripting()->num_i2c_devices] == nullptr) {
        return luaL_argerror(L, 1, "i2c device nullptr");;
    }

    *AP::scripting()->_i2c_dev[AP::scripting()->num_i2c_devices] = std::move(hal.i2c_mgr->get_device(bus, address, bus_clock, use_smbus));

    if (AP::scripting()->_i2c_dev[AP::scripting()->num_i2c_devices] == nullptr || AP::scripting()->_i2c_dev[AP::scripting()->num_i2c_devices]->get() == nullptr) {
        return luaL_argerror(L, 1, "i2c device nullptr");;
    }

    new_AP_HAL__I2CDevice(L);
    *check_AP_HAL__I2CDevice(L, -1) = AP::scripting()->_i2c_dev[AP::scripting()->num_i2c_devices]->get();

    AP::scripting()->num_i2c_devices++;

    return 1;
}

const luaL_Reg i2c_functions[] = {
    {"get_device", lua_get_i2c_device},
    {NULL, NULL}
};

void load_lua_bindings(lua_State *L) {
    lua_pushstring(L, "logger");
    luaL_newlib(L, AP_Logger_functions);
    lua_settable(L, -3);

    lua_pushstring(L, "i2c");
    luaL_newlib(L, i2c_functions);
    lua_settable(L, -3);

    luaL_setfuncs(L, global_functions, 0);
}

