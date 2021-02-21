// lua.hpp
// Lua header files for C++
// <<extern "C">> not supplied automatically because Lua also compiles as C++

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Filesystem/posix_compat.h>

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}
