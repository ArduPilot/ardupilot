#pragma once

// for inclusion at the end of the generated .cpp

void * new_ap_object(lua_State *L, size_t size, const char * name) {
    void * ud = lua_newuserdata(L, size);
    if (luaL_newmetatable(L, name)) { // metatable just created, set it up
        for (uint32_t i = 0; i < ARRAY_SIZE(ap_object_fun); i++) {
            if (strcmp(name, ap_object_fun[i].name) == 0) {
                lua_pushcfunction(L, ap_object_fun[i].func);
                lua_setfield(L, -2, "__index");
                break;
            }
        }
        // if no name matched, the metatable will be pointless but otherwise
        // valid, so it's safe to continue. since this function isn't
        // accessible to user code, it's probably not worth raising an error.
    }
    // correct metatable is on stack regardless of if we just set it up
    lua_setmetatable(L, -2);
    return ud;
}

void set_userdata_metatable(lua_State *L, const char * name) {
    if (luaL_newmetatable(L, name)) { // metatable just created, set it up
        for (uint32_t i = 0; i < ARRAY_SIZE(userdata_fun); i++) {
            if (strcmp(name, userdata_fun[i].name) == 0) {
                lua_pushcfunction(L, userdata_fun[i].func);
                lua_setfield(L, -2, "__index");
                if (userdata_fun[i].operators != nullptr) {
                    luaL_setfuncs(L, userdata_fun[i].operators, 0);
                }
                break;
            }
        }
        // if no name matched, the metatable will be pointless but otherwise
        // valid, so it's safe to continue. since this function isn't
        // accessible to user code, it's probably not worth raising an error.
    }
    // correct metatable is on stack regardless of if we just set it up
    lua_setmetatable(L, -2);
}

static int binding_index(lua_State *L) {
    const char * name = luaL_checkstring(L, 2);

    bool found = false;
    for (uint32_t i = 0; i < ARRAY_SIZE(singleton_fun); i++) {
        if (strcmp(name, singleton_fun[i].name) == 0) {
            lua_newuserdata(L, 0);
            if (luaL_newmetatable(L, name)) { // need to create metatable
                lua_pushcfunction(L, singleton_fun[i].func);
                lua_setfield(L, -2, "__index");
            }
            lua_setmetatable(L, -2);
            found = true;
            break;
        }
    }
    if (!found) {
        for (uint32_t i = 0; i < ARRAY_SIZE(new_userdata); i++) {
            if (strcmp(name, new_userdata[i].name) == 0) {
                lua_pushcfunction(L, new_userdata[i].fun);
                found = true;
                break;
            }
        }
    }
    if (!found) {
        return 0;
    }

    // store found value to avoid a re-index
    lua_pushvalue(L, -2);
    lua_pushvalue(L, -2);
    lua_settable(L, -5);

    return 1;
}

void load_generated_sandbox(lua_State *L) {
    lua_createtable(L, 0, 1);
    lua_pushcfunction(L, binding_index);
    lua_setfield(L, -2, "__index");
    lua_setmetatable(L, -2);
}
