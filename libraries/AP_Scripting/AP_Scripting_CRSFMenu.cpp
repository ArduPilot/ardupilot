#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include "AP_Scripting_CRSFMenu.h"

#if AP_CRSF_SCRIPTING_ENABLED

#include <AP_Scripting/lua_generated_bindings.h>

AP_CRSF_Telem::ScriptedParameter* CRSFMenu::add_parameter(uint8_t length, const char* data)
{
    return menu->add_parameter(length, data);
}

AP_CRSF_Telem::ScriptedMenu* CRSFMenu::add_menu(const char* menu_name)
{
    return menu->add_menu(menu_name, 2, menu->id);
}

int lua_CRSF_new_menu(lua_State *L)
{
    binding_argcheck(L, 1);

    const char * name = luaL_checkstring(L, 1);

    // create Lua object first in case of errors so we don't leak the menu
    auto *m = new_CRSFMenu(L);

    AP_CRSF_Telem::ScriptedMenu* menu = AP::crsf_telem()->add_menu(name);

    if (menu == nullptr) {
        return luaL_error(L, "out of memory");
    }

    *m = CRSFMenu(menu);

    return 1;
}

int lua_CRSF_get_menu_event(lua_State *L)
{
    binding_argcheck(L, 2);

    const uint8_t events = get_uint8_t(L, 2);
    uint8_t param = 0;
    AP_CRSF_Telem::ScriptedPayload payload {};
    const uint8_t data = static_cast<uint8_t>(AP::crsf_telem()->get_menu_event(
            events,
            param,
            payload));

#if 3 > LUA_MINSTACK
    luaL_checkstack(L, 3, nullptr);
#endif

    lua_pushinteger(L, param);
    lua_pushlstring(L, (const char*)payload.payload, payload.payload_length);
    lua_pushinteger(L, data);
    return 3;
}

int lua_CRSF_peek_menu_event(lua_State *L)
{
    binding_argcheck(L, 1);

    uint8_t param = 0;
    uint8_t events = 0;
    AP_CRSF_Telem::ScriptedPayload payload {};
    const uint8_t count = AP::crsf_telem()->peek_menu_event(param, payload, events);

#if 4 > LUA_MINSTACK
    luaL_checkstack(L, 4, nullptr);
#endif

    lua_pushinteger(L, count);
    lua_pushinteger(L, param);
    lua_pushlstring(L, (const char*)payload.payload, payload.payload_length);
    lua_pushinteger(L, events);
    return 4;
}

int lua_CRSF_send_response(lua_State *L)
{
    binding_argcheck(L, 2);

    size_t len = 0;
    const char * payload= luaL_checklstring(L, 2, &len);
    const bool data = static_cast<bool>(AP::crsf_telem()->send_write_response(
            len,
            payload));

    lua_pushboolean(L, data);
    return 1;
}

int lua_CRSF_add_parameter(lua_State *L)
{
    binding_argcheck(L, 2);

    CRSFMenu* ud = check_CRSFMenu(L, 1);
    size_t len = 0;
    const char * data = luaL_checklstring(L, 2, &len);

    // create Lua object first in case of errors so we don't leak the param
    auto *p = new_CRSFParameter(L);

    AP_CRSF_Telem::ScriptedParameter* param = ud->add_parameter(len, data);

    if (param) {
        *p = CRSFParameter(ud->get_menu(), param);
        return 1;
    }
    return 0;
}

int lua_CRSF_add_menu(lua_State *L)
{
    binding_argcheck(L, 2);

    CRSFMenu* ud = check_CRSFMenu(L, 1);
    const char * name = luaL_checkstring(L, 2);

    // create Lua object first in case of errors so we don't leak the menu
    auto *m = new_CRSFMenu(L);

    AP_CRSF_Telem::ScriptedMenu* menu = ud->add_menu(name);

    if (menu) {
        *m = CRSFMenu(menu);
        return 1;
    }
    return 0;
}

int lua_CRSF_add_root_menu(lua_State *L)
{
    binding_argcheck(L, 2);

    const char * name = luaL_checkstring(L, 2);

    // create Lua object first in case of errors so we don't leak the menu
    auto *m = new_CRSFMenu(L);

    AP_CRSF_Telem::ScriptedMenu* menu = AP::crsf_telem()->add_menu(name);

    if (menu) {
        *m = CRSFMenu(menu);
        return 1;
    }
    return 0;
}

int lua_CRSF_param_data(lua_State *L)
{
    binding_argcheck(L, 1);

    CRSFParameter * ud = check_CRSFParameter(L, 1);
    AP_CRSF_Telem::ScriptedParameter* param = ud->get_parameter();
    if (param == nullptr) {
        return luaL_error(L, "internal error: %s is null", "CRSFParameter");
    }

    lua_pushlstring(L, param->data, param->length);
    return 1;
}

#endif // AP_CRSF_SCRIPTING_ENABLED
#endif // AP_SCRIPTING
