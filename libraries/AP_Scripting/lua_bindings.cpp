#include <AP_Common/AP_Common.h>
#include <AP_HAL/HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include "lua_bindings.h"

#include "lua_boxed_numerics.h"
#include <AP_Scripting/lua_generated_bindings.h>

#include <AP_Scripting/AP_Scripting.h>
#include <string.h>

extern "C" {
#include "lua/src/lmem.h"
}

extern const AP_HAL::HAL& hal;

// millis
int lua_millis(lua_State *L) {
    binding_argcheck(L, 0);

    new_uint32_t(L);
    *check_uint32_t(L, -1) = AP_HAL::millis();

    return 1;
}

// micros
int lua_micros(lua_State *L) {
    binding_argcheck(L, 0);

    new_uint32_t(L);
    *check_uint32_t(L, -1) = AP_HAL::micros();

    return 1;
}

int lua_mission_receive(lua_State *L) {
    binding_argcheck(L, 0);

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

int AP_Logger_Write(lua_State *L) {
    AP_Logger * AP_logger = AP_Logger::get_singleton();
    if (AP_logger == nullptr) {
        return luaL_argerror(L, 1, "logger not supported on this firmware");
    }

    // Allow : and . access
    const int arg_offset = (luaL_testudata(L, 1, "logger") != NULL) ? 1 : 0;

    // check we have at least 4 arguments passed in
    const int args = lua_gettop(L) - arg_offset;
    if (args < 4) {
        return luaL_argerror(L, args, "too few arguments");
    }

    const char * name = luaL_checkstring(L, 1 + arg_offset);
    const char * labels = luaL_checkstring(L, 2 + arg_offset);
    const char * fmt = luaL_checkstring(L, 3 + arg_offset);

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
        f = AP_logger->msg_fmt_for_name(name, label_cat, nullptr, nullptr, fmt_cat, true, true);

    } else {
        // read in units and multiplers strings
        field_start += 2;
        const char * units = luaL_checkstring(L, 4 + arg_offset);
        const char * multipliers = luaL_checkstring(L, 5 + arg_offset);

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
        f = AP_logger->msg_fmt_for_name(name, label_cat, units_cat, multipliers_cat, fmt_cat, true, true);
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

    // note that luaM_malloc will never return null, it will fault instead
    char *buffer = (char*)luaM_malloc(L, msg_len);

    // add logging headers
    uint8_t offset = 0;
    buffer[offset++] = HEAD_BYTE1;
    buffer[offset++] = HEAD_BYTE2;
    buffer[offset++] = f->msg_type;

    // timestamp is always first value
    const uint64_t now = AP_HAL::micros64();
    memcpy(&buffer[offset], &now, sizeof(uint64_t));
    offset += sizeof(uint64_t);

    for (uint8_t i=field_start; i<=args; i++) {
        uint8_t charlen = 0;
        uint8_t index = have_units ? i-5 : i-3;
        uint8_t arg_index = i + arg_offset;
        switch(fmt_cat[index]) {
            // logger variable types not available to scripting
            // 'd': double
            // 'Q': uint64_t
            // 'q': int64_t
            // 'a': arrays
            case 'b': { // int8_t
                int isnum;
                const lua_Integer tmp1 = lua_tointegerx(L, arg_index, &isnum);
                if (!isnum || (tmp1 < INT8_MIN) || (tmp1 > INT8_MAX)) {
                    luaM_free(L, buffer);
                    luaL_argerror(L, arg_index, "argument out of range");
                    // no return
                }
                int8_t tmp = static_cast<int8_t>(tmp1);
                memcpy(&buffer[offset], &tmp, sizeof(int8_t));
                offset += sizeof(int8_t);
                break;
            }
            case 'h': // int16_t
            case 'c': { // int16_t * 100
                int isnum;
                const lua_Integer tmp1 = lua_tointegerx(L, arg_index, &isnum);
                if (!isnum || (tmp1 < INT16_MIN) || (tmp1 > INT16_MAX)) {
                    luaM_free(L, buffer);
                    luaL_argerror(L, arg_index, "argument out of range");
                    // no return
                }
                int16_t tmp = static_cast<int16_t>(tmp1);
                memcpy(&buffer[offset], &tmp, sizeof(int16_t));
                offset += sizeof(int16_t);
                break;
            }
            case 'H': // uint16_t
            case 'C': { // uint16_t * 100
                int isnum;
                const lua_Integer tmp1 = lua_tointegerx(L, arg_index, &isnum);
                if (!isnum || (tmp1 < 0) || (tmp1 > UINT16_MAX)) {
                    luaM_free(L, buffer);
                    luaL_argerror(L, arg_index, "argument out of range");
                    // no return
                }
                uint16_t tmp = static_cast<uint16_t>(tmp1);
                memcpy(&buffer[offset], &tmp, sizeof(uint16_t));
                offset += sizeof(uint16_t);
                break;
            }
            case 'i': // int32_t
            case 'L': // int32_t (lat/long)
            case 'e': { // int32_t * 100
                int isnum;
                const lua_Integer tmp1 = lua_tointegerx(L, arg_index, &isnum);
                if (!isnum) {
                    luaM_free(L, buffer);
                    luaL_argerror(L, arg_index, "argument out of range");
                    // no return
                }
                const int32_t tmp = tmp1;
                memcpy(&buffer[offset], &tmp, sizeof(int32_t));
                offset += sizeof(int32_t);
                break;
            }
            case 'f': { // float
                int isnum;
                const lua_Number tmp1 = lua_tonumberx(L, arg_index, &isnum);
                if (!isnum) {
                    luaM_free(L, buffer);
                    luaL_argerror(L, arg_index, "argument out of range");
                    // no return
                }
                const float tmp = tmp1;
                memcpy(&buffer[offset], &tmp, sizeof(float));
                offset += sizeof(float);
                break;
            }
            case 'n': { // char[4]
                charlen = 4;
                break;
            }
            case 'M': // uint8_t (flight mode)
            case 'B': { // uint8_t
                int isnum;
                const lua_Integer tmp1 = lua_tointegerx(L, arg_index, &isnum);
                if (!isnum || (tmp1 < 0) || (tmp1 > UINT8_MAX)) {
                    luaM_free(L, buffer);
                    luaL_argerror(L, arg_index, "argument out of range");
                    // no return
                }
                uint8_t tmp = static_cast<uint8_t>(tmp1);
                memcpy(&buffer[offset], &tmp, sizeof(uint8_t));
                offset += sizeof(uint8_t);
                break;
            }
            case 'I': // uint32_t
            case 'E': { // uint32_t * 100
                uint32_t tmp;
                void * ud = luaL_testudata(L, arg_index, "uint32_t");
                if (ud != nullptr) {
                    tmp = *static_cast<uint32_t *>(ud);
                } else {
                    int success;
                    const lua_Integer v_int = lua_tointegerx(L, arg_index, &success);
                    if (success) {
                        tmp = v_int;
                    } else {
                        const lua_Number v_float = lua_tonumberx(L, arg_index, &success);
                        if (!success || (v_float < 0) || (v_float > float(UINT32_MAX))) {
                            luaM_free(L, buffer);
                            luaL_argerror(L, arg_index, "argument out of range");
                            // no return
                        }
                        tmp = v_float;
                    }
                }
                memcpy(&buffer[offset], &tmp, sizeof(uint32_t));
                offset += sizeof(uint32_t);
                break;
            }
            case 'N': { // char[16]
                charlen = 16;
                break;
            }
            case 'Z': { // char[64]
                charlen = 64;
                break;
            }
            default: {
                luaM_free(L, buffer);
                luaL_error(L, "%c unsupported format",fmt_cat[index]);
                // no return
            }
        }
        if (charlen != 0) {
            size_t slen;
            const char *tmp = lua_tolstring(L, arg_index, &slen);
            if (tmp == nullptr) {
                luaM_free(L, buffer);
                luaL_argerror(L, arg_index, "argument out of range");
                // no return
            }
            if (slen > charlen) {
                luaM_free(L, buffer);
                luaL_error(L, "arg %d too long for %c format",arg_index,fmt_cat[index]);
                // no return
            }
            memcpy(&buffer[offset], tmp, slen);
            memset(&buffer[offset+slen], 0, charlen-slen);
            offset += charlen;
        }
    }

    AP_logger->Safe_Write_Emit_FMT(f);

    AP_logger->WriteBlock(buffer,msg_len);

    luaM_free(L, buffer);

    return 0;
}

int lua_get_i2c_device(lua_State *L) {

    // Allow : and . access
    const int arg_offset = (luaL_testudata(L, 1, "i2c") != NULL) ? 1 : 0;

    const int args = lua_gettop(L) - arg_offset;
    if (args < 2) {
        return luaL_argerror(L, args, "require i2c bus and address");
    }
    if (args > 4) {
        return luaL_argerror(L, args, "too many arguments");
    }

    const lua_Integer bus_in = get_integer(L, 1 + arg_offset, 0, 4);
    const uint8_t bus = static_cast<uint8_t>(bus_in);

    const lua_Integer address_in = get_integer(L, 2 + arg_offset, 0, 128);
    const uint8_t address = static_cast<uint8_t>(address_in);

    // optional arguments, use the same defaults as the hal get_device function
    uint32_t bus_clock = 400000;
    bool use_smbus = false;

    if (args > 2) {
        bus_clock = coerce_to_uint32_t(L, 3 + arg_offset);

        if (args > 3) {
            use_smbus = static_cast<bool>(lua_toboolean(L, 4 + arg_offset));
        }
    }

    auto *scripting = AP::scripting();

    static_assert(SCRIPTING_MAX_NUM_I2C_DEVICE >= 0, "There cannot be a negative number of I2C devices");
    if (scripting->num_i2c_devices >= SCRIPTING_MAX_NUM_I2C_DEVICE) {
        return luaL_argerror(L, 1, "no i2c devices available");
    }

    scripting->_i2c_dev[scripting->num_i2c_devices] = new AP_HAL::OwnPtr<AP_HAL::I2CDevice>;
    if (scripting->_i2c_dev[scripting->num_i2c_devices] == nullptr) {
        return luaL_argerror(L, 1, "i2c device nullptr");
    }

    *scripting->_i2c_dev[scripting->num_i2c_devices] = std::move(hal.i2c_mgr->get_device(bus, address, bus_clock, use_smbus));

    if (scripting->_i2c_dev[scripting->num_i2c_devices] == nullptr || scripting->_i2c_dev[scripting->num_i2c_devices]->get() == nullptr) {
        return luaL_argerror(L, 1, "i2c device nullptr");
    }

    new_AP_HAL__I2CDevice(L);
    *((AP_HAL::I2CDevice**)luaL_checkudata(L, -1, "AP_HAL::I2CDevice")) = scripting->_i2c_dev[scripting->num_i2c_devices]->get();

    scripting->num_i2c_devices++;

    return 1;
}

int AP_HAL__I2CDevice_read_registers(lua_State *L) {
    const int args = lua_gettop(L);
    bool multi_register;
    if (args == 2) {
        multi_register = false;
    } else if (args == 3) {
        multi_register = true;
    } else {
        return luaL_argerror(L, args, "expected 1 or 2 arguments");
    }

    AP_HAL::I2CDevice * ud = *check_AP_HAL__I2CDevice(L, 1);

    const uint8_t first_reg = get_uint8_t(L, 2);

    uint8_t recv_length = 1;
    if (multi_register) {
        recv_length = get_uint8_t(L, 3);
    }

    uint8_t data[recv_length];

    ud->get_semaphore()->take_blocking();
    const bool success = static_cast<bool>(ud->read_registers(first_reg, data, recv_length));
    ud->get_semaphore()->give();

    if (success) {
        if (!multi_register) {
            lua_pushinteger(L, data[0]);
        } else {
            // push to table
            lua_newtable(L);
            for (uint8_t i=0; i < recv_length; i++) {
                lua_pushinteger(L, i+1);
                lua_pushinteger(L, data[i]);
                lua_settable(L, -3);
            }
        }
    }
    return success;
}

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
int lua_get_CAN_device(lua_State *L) {

    // Allow : and . access
    const int arg_offset = (luaL_testudata(L, 1, "CAN") != NULL) ? 1 : 0;

    binding_argcheck(L, 1 + arg_offset);

    const uint32_t raw_buffer_len = get_uint32(L, 1 + arg_offset, 1, 25);
    const uint32_t buffer_len = static_cast<uint32_t>(raw_buffer_len);

    auto *scripting = AP::scripting();

    if (scripting->_CAN_dev == nullptr) {
        scripting->_CAN_dev = new ScriptingCANSensor(AP_CAN::Protocol::Scripting);
        if (scripting->_CAN_dev == nullptr) {
            return luaL_argerror(L, 1, "CAN device nullptr");
        }
    }

    new_ScriptingCANBuffer(L);
    *((ScriptingCANBuffer**)luaL_checkudata(L, -1, "ScriptingCANBuffer")) = scripting->_CAN_dev->add_buffer(buffer_len);

    return 1;
}

int lua_get_CAN_device2(lua_State *L) {

    // Allow : and . access
    const int arg_offset = (luaL_testudata(L, 1, "CAN") != NULL) ? 1 : 0;

    binding_argcheck(L, 1 + arg_offset);

    const uint32_t raw_buffer_len = get_uint32(L, 1 + arg_offset, 1, 25);
    const uint32_t buffer_len = static_cast<uint32_t>(raw_buffer_len);

    auto *scripting = AP::scripting();

    if (scripting->_CAN_dev2 == nullptr) {
        scripting->_CAN_dev2 = new ScriptingCANSensor(AP_CAN::Protocol::Scripting2);
        if (scripting->_CAN_dev2 == nullptr) {
            return luaL_argerror(L, 1, "CAN device nullptr");
        }
    }

    new_ScriptingCANBuffer(L);
    *((ScriptingCANBuffer**)luaL_checkudata(L, -1, "ScriptingCANBuffer")) = scripting->_CAN_dev2->add_buffer(buffer_len);

    return 1;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

/*
  directory listing, return table of files in a directory
 */
int lua_dirlist(lua_State *L) {
    binding_argcheck(L, 1);

    struct dirent *entry;
    int i;
    const char *path = luaL_checkstring(L, 1);
    
    /* open directory */
    auto dir = AP::FS().opendir(path);
    if (dir == nullptr) {  /* error opening the directory? */
        lua_pushnil(L);  /* return nil and ... */
        lua_pushstring(L, strerror(errno));  /* error message */
        return 2;  /* number of results */
    }
    
    /* create result table */
    lua_newtable(L);
    i = 1;
    while ((entry = AP::FS().readdir(dir)) != nullptr) {
        lua_pushnumber(L, i++);  /* push key */
        lua_pushstring(L, entry->d_name);  /* push value */
        lua_settable(L, -3);
    }
    
    AP::FS().closedir(dir);
    return 1;  /* table is already on top */
}

/*
  remove a file
 */
int lua_removefile(lua_State *L) {
    binding_argcheck(L, 1);
    const char *filename = luaL_checkstring(L, 1);
    return luaL_fileresult(L, remove(filename) == 0, filename);
}

// Manual binding to allow SRV_Channels table to see safety state
int SRV_Channels_get_safety_state(lua_State *L) {
    binding_argcheck(L, 1);
    const bool data = hal.util->safety_switch_state() !=  AP_HAL::Util::SAFETY_ARMED;
    lua_pushboolean(L, data);
    return 1;
}

int lua_get_PWMSource(lua_State *L) {
    binding_argcheck(L, 0);

    auto *scripting = AP::scripting();

    static_assert(SCRIPTING_MAX_NUM_PWM_SOURCE >= 0, "There cannot be a negative number of PWMSources");
    if (scripting->num_pwm_source >= SCRIPTING_MAX_NUM_PWM_SOURCE) {
        return luaL_argerror(L, 1, "no PWMSources available");
    }

    scripting->_pwm_source[scripting->num_pwm_source] = new AP_HAL::PWMSource;
    if (scripting->_pwm_source[scripting->num_pwm_source] == nullptr) {
        return luaL_argerror(L, 1, "PWMSources device nullptr");
    }

    new_AP_HAL__PWMSource(L);
    *((AP_HAL::PWMSource**)luaL_checkudata(L, -1, "AP_HAL::PWMSource")) = scripting->_pwm_source[scripting->num_pwm_source];

    scripting->num_pwm_source++;

    return 1;
}
