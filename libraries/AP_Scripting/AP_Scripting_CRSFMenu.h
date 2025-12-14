#pragma once

#include <AP_RCTelemetry/AP_CRSF_Telem.h>
#include "lua/src/lua.hpp"

#if AP_CRSF_SCRIPTING_ENABLED

int lua_CRSF_new_menu(lua_State *L);
int lua_CRSF_get_menu_event(lua_State *L);
int lua_CRSF_peek_menu_event(lua_State *L);
int lua_CRSF_send_response(lua_State *L);
int lua_CRSF_add_parameter(lua_State *L);
int lua_CRSF_add_menu(lua_State *L);
int lua_CRSF_add_root_menu(lua_State *L);
int lua_CRSF_param_data(lua_State *L);

// wrappers to underlying CRSF menu and parameter types that mean the 
// underlying data is always the actual data rather than a copy
class CRSFParameter {
public:
    CRSFParameter() {}
    CRSFParameter(AP_CRSF_Telem::ScriptedMenu* m, AP_CRSF_Telem::ScriptedParameter* p) : menu(m), param(p->AP_CRSF_Telem::ScriptedParameter::id) {}

    uint8_t id() const { return param; }
    // parameter reallocation means that the pointer could change, so index via the menu
    AP_CRSF_Telem::ScriptedParameter* get_parameter() { return menu->find_parameter(param); }

private:
    AP_CRSF_Telem::ScriptedMenu* menu;
    uint8_t param;
};

class CRSFMenu {
public:
    CRSFMenu() {}
    CRSFMenu(AP_CRSF_Telem::ScriptedMenu* m) : menu(m) {}

    uint8_t id() const { return menu->id; }
    const char* name() const { return menu->name; }
    uint8_t num_params() const { return menu->num_params; }
    AP_CRSF_Telem::ScriptedMenu* get_menu() const { return menu; }
    AP_CRSF_Telem::ScriptedParameter* add_parameter(uint8_t length, const char* data);
    AP_CRSF_Telem::ScriptedMenu* add_menu(const char* menu_name);


private:
    AP_CRSF_Telem::ScriptedMenu* menu;
};

#endif // AP_CRSF_SCRIPTING_ENABLED
