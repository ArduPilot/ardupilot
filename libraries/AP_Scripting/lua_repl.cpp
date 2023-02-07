// this implements a Lua REPL, and is based off of a cut down version of
// lua/src/lua.c. It overall modified the functions to the minimum amount
// required, with the exception of fixing whitespace/indentation on if's


#include "lua_scripts.h"
#include <AP_Scripting/lua_generated_bindings.h>

#include "lua/src/lua.h"
#include "lua/src/lauxlib.h"
#include "lua/src/lualib.h"

#include <AP_Logger/LogStructure.h>

#if !defined(LUA_MAXINPUT)
#define LUA_MAXINPUT    256
#endif

#if !defined(LUA_PROMPT)
#define LUA_PROMPT		"> "
#define LUA_PROMPT2		">> "
#endif

extern const AP_HAL::HAL& hal;

/*
** Message handler used to run all chunks
*/
static int msghandler(lua_State *L) {
    const char *msg = lua_tostring(L, 1);
    if (msg == NULL) {  /* is error object not a string? */
        if (luaL_callmeta(L, 1, "__tostring") &&  /* does it have a metamethod */
            lua_type(L, -1) == LUA_TSTRING) { /* that produces a string? */
            return 1;  /* that is the message */
        } else {
            msg = lua_pushfstring(L, "(error object is a %s value)",
                                     luaL_typename(L, 1));
        }
    }
    luaL_traceback(L, L, msg, 1);  /* append a standard traceback */
    return 1;  /* return the traceback */
}


/*
** Interface to 'lua_pcall', which sets appropriate message function
** and C-signal handler. Used to run all chunks.
*/
int lua_scripts::docall(lua_State *L, int narg, int nres) const {
    int status;
    int base = lua_gettop(L) - narg;  /* function index */
    lua_rawgeti(L, LUA_REGISTRYINDEX, sandbox_ref);
    lua_setupvalue(L, -2, 1);
    lua_pushcfunction(L, msghandler);  /* push message handler */
    lua_insert(L, base);  /* put it under function and args */
    status = lua_pcall(L, narg, nres, base);
    lua_remove(L, base);  /* remove message handler from the stack */
    return status;
}


/*
** Returns the string to be used as a prompt by the interpreter.
*/
const char * lua_scripts::get_prompt(lua_State *L, int firstline) {
    const char *p;
    lua_getglobal(L, firstline ? "_PROMPT" : "_PROMPT2");
    p = lua_tostring(L, -1);
    if (p == NULL) {
        p = (firstline ? LUA_PROMPT : LUA_PROMPT2);
    }
    return p;
}

/* mark in error messages for incomplete statements */
#define EOFMARK		"<eof>"
#define marklen		(sizeof(EOFMARK)/sizeof(char) - 1)


/*
** Check whether 'status' signals a syntax error and the error
** message at the top of the stack ends with the above mark for
** incomplete statements.
*/
int lua_scripts::incomplete(lua_State *L, int status) {
    if (status == LUA_ERRSYNTAX) {
        size_t lmsg;
        const char *msg = lua_tolstring(L, -1, &lmsg);
        if (lmsg >= marklen && strcmp(msg + lmsg - marklen, EOFMARK) == 0) {
            lua_pop(L, 1);
            return 1;
        }
    }
    return 0;  /* else... */
}


/*
** Prompt the user, read a line, and push it into the Lua stack.
*/
int lua_scripts::pushline(lua_State *L, int firstline) {
    char buffer[LUA_MAXINPUT + 1] = {};
    size_t l = 0;

    // send prompt to the user
    terminal_print(get_prompt(L, firstline));

    while (terminal.session) {
        // reseek to where we need input from, as invalid reads could have done weird stuff, and we want to start from the last valid input
        int input_fd = AP::FS().open(REPL_IN, O_RDONLY);
        if (input_fd != -1) {
            AP::FS().lseek(input_fd, terminal.input_offset, SEEK_SET);
            ssize_t read_bytes = AP::FS().read(input_fd, buffer, ARRAY_SIZE(buffer) - 1);
            AP::FS().close(input_fd);
            if (read_bytes > 0) {
                // locate the first newline
                char * newline_chr = strchr(buffer, '\n');
                if (newline_chr != NULL) {
                    newline_chr[0] = '\0';
                    // only advance to the newline
                    l = strlen(buffer);
                    terminal.input_offset += l + 1;
                    break;
                }
            }
        }
        // wait for any input
        hal.scheduler->delay(100);
    }
  
    lua_pop(L, 1);  /* remove prompt */
    lua_pushlstring(L, buffer, l);
    return 1;
}


/*
** Try to compile line on the stack as 'return <line>;'; on return, stack
** has either compiled chunk or original line (if compilation failed).
*/
int lua_scripts::addreturn(lua_State *L) {
    const char *line = lua_tostring(L, -1);  /* original line */
    const char *retline = lua_pushfstring(L, "return %s;", line);
    int status = luaL_loadbuffer(L, retline, strlen(retline), "=stdin");
    if (status == LUA_OK) {
        lua_remove(L, -2);  /* remove modified line */
    } else {
        lua_pop(L, 2);  /* pop result from 'luaL_loadbuffer' and modified line */
    }
    return status;
}


/*
** Read multiple lines until a complete Lua statement
*/
int lua_scripts::multiline (lua_State *L) {
    for (;;) {  /* repeat until gets a complete statement */
        size_t len;
        const char *line = lua_tolstring(L, 1, &len);  /* get what it has */
        int status = luaL_loadbuffer(L, line, len, "=stdin");  /* try it */
        if (!incomplete(L, status) || !pushline(L, 0)) {
            return status;  /* cannot or should not try to add continuation line */
        }
        lua_pushliteral(L, "\n");  /* add newline... */
        lua_insert(L, -2);  /* ...between the two lines */
        lua_concat(L, 3);  /* join them */
    }
}


/*
** Read a line and try to load (compile) it first as an expression (by
** adding "return " in front of it) and second as a statement. Return
** the final status of load/call with the resulting function (if any)
** in the top of the stack.
*/
int lua_scripts::loadline(lua_State *L) {
    int status;
    lua_settop(L, 0);
    if (!pushline(L, 1)) {
        return -1;  /* no input */
    }
    if ((status = addreturn(L)) != LUA_OK) {  /* 'return ...' did not work? */
        status = multiline(L);  /* try as command, maybe with continuation lines */
    } else {
    }
    lua_remove(L, 1);  /* remove line from the stack */
    lua_assert(lua_gettop(L) == 1);
    return status;
}

// push the tring into the terminal, blocks until it's queued
void lua_scripts::terminal_print(const char *str) {
    if ((AP::FS().write(terminal.output_fd, str, strlen(str)) == -1) ||
        (AP::FS().fsync(terminal.output_fd) != 0)) {
        terminal.session = false;
    }
}

/*
** Prints (calling the Lua 'print' function) any values on the stack
*/
void lua_scripts::l_print(lua_State *L) {
    int n = lua_gettop(L);
    if (n > 0) {  /* any result to be printed? */
        luaL_checkstack(L, LUA_MINSTACK, "too many results to print");
        // grab all the internal functions via the sandbox
        lua_rawgeti(L, LUA_REGISTRYINDEX, sandbox_ref);
        lua_getfield(L, -1, "string");
        lua_getfield(L, -1, "format");
        lua_insert(L, 1);
        lua_remove(L, -2);
        lua_getfield(L, -1, "rep");
        lua_remove(L, -2);
        lua_pushliteral(L, "%s");
        lua_pushinteger(L, n);
        lua_pushliteral(L, "\t");
        if (lua_pcall(L, 3, 1, 0) != LUA_OK) {
            // should never happen
            lua_error(L);
        }
        lua_insert(L, 2);
        if (lua_pcall(L, n + 1, 1, 0) != LUA_OK) {
            terminal_print(lua_pushfstring(L, "error calling 'print' (%s)\n", lua_tostring(L, -1)));
        } else {
            terminal_print(lua_pushfstring(L, "%s\n", lua_tostring(L, -1)));
        }
    }
}

/*
** Do the REPL: repeatedly read (load) a line, evaluate (call) it, and
** print any results.
*/
void lua_scripts::doREPL(lua_State *L) {
    int status;
    // clear out any old script results
    reset_loop_overtime(L);
    // prep the sandbox
    create_sandbox(L);
    sandbox_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    terminal.input_offset = 0;
    while (((status = loadline(L)) != -1) && terminal.session) {
        if (status == LUA_OK) {
            status = docall(L, 0, LUA_MULTRET);
        }
        if (status == LUA_OK) {
            l_print(L);
        } else {
            terminal_print(lua_pushfstring(L, "%s\n", lua_tostring(L, -1)));
        }
        reset_loop_overtime(L);
    }
    lua_settop(L, 0);  /* clear stack */
    luaL_unref(L, LUA_REGISTRYINDEX, sandbox_ref);
    repl_cleanup();
}

