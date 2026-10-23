#pragma once

// for inclusion at the start of the generated .cpp

int binding_argcheck(lua_State *L, int expected_arg_count) {
    const int args = lua_gettop(L);
    if (args > expected_arg_count) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < expected_arg_count) {
        return luaL_argerror(L, args, "too few arguments");
    }
    return 0;
}

int field_argerror(lua_State *L) {
    return binding_argcheck(L, -1); // force too many args error
}

bool userdata_zero_arg_check(lua_State *L) {
    if (lua_gettop(L) == 0) {
        return false;
    }
    lua_Debug ar;
    if (lua_getstack(L, 1, &ar)) {
        lua_getinfo(L, "Sl", &ar);
        if (ar.currentline != 0) {
            if (lua_getstack(L, 0, &ar)) {
                lua_getinfo(L, "n", &ar);
                if (ar.name != NULL) {
                    lua_scripts::set_and_print_new_error_message(MAV_SEVERITY_WARNING, "%s:%d Warning: %s does not take arguments, will be fatal in future", ar.short_src, ar.currentline, ar.name);
                    return true;
                }
            }
        }
    }
    lua_scripts::set_and_print_new_error_message(MAV_SEVERITY_WARNING, "Warning: userdata creation does not take arguments, will be fatal in future");
    return true;
}

lua_Integer get_integer(lua_State *L, int arg_num, lua_Integer min_val, lua_Integer max_val) {
    const lua_Integer lua_int = luaL_checkinteger(L, arg_num);
    luaL_argcheck(L, (lua_int >= min_val) && (lua_int <= max_val), arg_num, "out of range");
    return lua_int;
}

// helpers for full range types
int8_t get_int8_t(lua_State *L, int arg_num) {
    return static_cast<int8_t>(get_integer(L, arg_num, INT8_MIN, INT8_MAX));
}

int16_t get_int16_t(lua_State *L, int arg_num) {
    return static_cast<int16_t>(get_integer(L, arg_num, INT16_MIN, INT16_MAX));
}

uint8_t get_uint8_t(lua_State *L, int arg_num) {
    return static_cast<uint8_t>(get_integer(L, arg_num, 0, UINT8_MAX));
}

uint16_t get_uint16_t(lua_State *L, int arg_num) {
    return static_cast<uint16_t>(get_integer(L, arg_num, 0, UINT16_MAX));
}

float get_number(lua_State *L, int arg_num, float min_val, float max_val) {
    const float lua_num = luaL_checknumber(L, arg_num);
    luaL_argcheck(L, (lua_num >= min_val) && (lua_num <= max_val), arg_num, "out of range");
    return lua_num;
}

uint32_t get_uint32(lua_State *L, int arg_num, uint32_t min_val, uint32_t max_val) {
    const uint32_t lua_unint32 = coerce_to_uint32_t(L, arg_num);
    luaL_argcheck(L, (lua_unint32 >= min_val) && (lua_unint32 <= max_val), arg_num, "out of range");
    return lua_unint32;
}

void ** check_ap_object(lua_State *L, int arg_num, const char * name) {
    void ** data = (void **)luaL_checkudata(L, arg_num, name);
    if (*data == NULL) {
        luaL_error(L, "internal error: %s is null", name); // does not return
    }
    return data;
}

static int not_supported_error(lua_State *L, int arg, const char* name) {
    char error_msg[50];
    snprintf(error_msg, sizeof(error_msg), "%s not supported on this firmware", name);
    return luaL_argerror(L, arg, error_msg);
}

struct userdata_enum {
    const char *name;
    int value;
};

struct userdata_meta {
    const char *name;
    lua_CFunction func;
    const luaL_Reg *operators;
};

static int load_function(lua_State *L, const luaL_Reg *list, const uint8_t length) {
    const char * name = luaL_checkstring(L, 2);
    for (uint8_t i = 0; i < length; i++) {
        if (strcmp(name,list[i].name) == 0) {
            lua_pushcfunction(L, list[i].func);
            return 1;
        }
    }
    return 0;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static int load_enum(lua_State *L, const userdata_enum *list, const uint8_t length) {
    const char * name = luaL_checkstring(L, 2);
    for (uint8_t i = 0; i < length; i++) {
        if (strcmp(name,list[i].name) == 0) {
            lua_pushinteger(L, list[i].value);
            return 1;
        }
    }
    return 0;
}
#pragma GCC diagnostic pop
