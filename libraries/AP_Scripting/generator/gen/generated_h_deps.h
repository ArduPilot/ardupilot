#pragma once

// for inclusion by the generated .h

void load_generated_sandbox(lua_State *L);
int binding_argcheck(lua_State *L, int expected_arg_count);
int field_argerror(lua_State *L);
bool userdata_zero_arg_check(lua_State *L);
lua_Integer get_integer(lua_State *L, int arg_num, lua_Integer min_val, lua_Integer max_val);
int8_t get_int8_t(lua_State *L, int arg_num);
int16_t get_int16_t(lua_State *L, int arg_num);
uint8_t get_uint8_t(lua_State *L, int arg_num);
uint16_t get_uint16_t(lua_State *L, int arg_num);
float get_number(lua_State *L, int arg_num, float min_val, float max_val);
uint32_t get_uint32(lua_State *L, int arg_num, uint32_t min_val, uint32_t max_val);
void * new_ap_object(lua_State *L, size_t size, const char * name);
void ** check_ap_object(lua_State *L, int arg_num, const char * name);
void set_userdata_metatable(lua_State *L, const char * name);
