//
// Simple commandline menu system.
#include "AP_Menu.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// statics
char *Menu::_inbuf;
Menu::arg *Menu::_argv;
AP_HAL::BetterStream *Menu::_port;


// constructor
Menu::Menu(const char *prompt, const Menu::command *commands, uint8_t entries, preprompt ppfunc) :
    _prompt(prompt),
    _commands(commands),
    _entries(entries),
    _ppfunc(ppfunc),
    _commandline_max(MENU_COMMANDLINE_MAX),
    _args_max(MENU_ARGS_MAX)
{
    // the buffers are initially nullptr, then they are allocated on
    // first use
    _inbuf = nullptr;
    _argv = nullptr;
}

/**
   check for another input byte on the port and accumulate
   return true if we have a full line ready to process
 */
bool
Menu::_check_for_input(void)
{
    if (_port->available() <= 0) {
        return false;
    }

    // loop reading characters from the input
    int c = _port->read();

    // carriage return -> process command
    if ('\r' == c || '\n' == c) {
        _inbuf[_input_len] = '\0';
        _port->write('\r');
        _port->write('\n');
        // we have a full line to process
        return true;
    }

    // backspace
    if ('\b' == c) {
        if (_input_len > 0) {
            _input_len--;
            _port->write('\b');
            _port->write(' ');
            _port->write('\b');
            return false;
        }
    }

    // printable character
    if (isprint(c) && (_input_len < (_commandline_max - 1))) {
        _inbuf[_input_len++] = c;
        _port->write((char)c);
    }

    return false;
}

// display the prompt
void
Menu::_display_prompt(void)
{
    _port->printf("%s] ", _prompt);
}

// run the menu
bool
Menu::_run_command(bool prompt_on_enter)
{
    int8_t ret;
    uint8_t i;
    uint8_t argc;
    char *s = nullptr;

    _input_len = 0;

    // split the input line into tokens
    argc = 0;
    s = nullptr;
    _argv[argc++].str = strtok_r(_inbuf, " ", &s);

    // XXX should an empty line by itself back out of the current menu?
    while (argc <= _args_max) {
        _argv[argc].str = strtok_r(nullptr, " ", &s);
        if (_argv[argc].str == nullptr || '\0' == _argv[argc].str[0])
            break;
        _argv[argc].i = atol(_argv[argc].str);
        _argv[argc].f = strtof(_argv[argc].str, NULL);
        argc++;
    }
    
    if (_argv[0].str == nullptr) {
        // we got a blank line, re-display the prompt
        if (prompt_on_enter) {
            _display_prompt();
        }
        return false;
    }
    
    // populate arguments that have not been specified with "" and 0
    // this is safer than NULL in the case where commands may look
    // without testing argc
    i = argc;
    while (i <= _args_max) {
        _argv[i].str = "";
        _argv[i].i = 0;
        _argv[i].f = 0;
        i++;
    }
    
    bool cmd_found = false;
    // look for a command matching the first word (note that it may be empty)
    for (i = 0; i < _entries; i++) {
        if (!strcasecmp(_argv[0].str, _commands[i].command)) {
            ret = _call(i, argc);
            cmd_found=true;
            if (-2 == ret)
                return true;
            break;
        }
    }
    
    // implicit commands
    if (i == _entries) {
        if (!strcmp(_argv[0].str, "?") || (!strcasecmp(_argv[0].str, "help"))) {
            _help();
            cmd_found=true;
        } else if (!strcasecmp(_argv[0].str, "exit")) {
            // exit the menu
            return true;
        }
    }

    if (cmd_found==false)
    {
        _port->printf("Invalid command, type 'help'\n");
    }

    return false;
}


// run the menu
void
Menu::run(void)
{
	if (_port == nullptr) {
		// default to main serial port
		_port = hal.console;
	}

    _allocate_buffers();

    _display_prompt();

    // loop performing commands
    for (;;) {

        // run the pre-prompt function, if one is defined
        if (_ppfunc) {
            if (!_ppfunc())
                return;
            _display_prompt();
        }

        // loop reading characters from the input
        _input_len = 0;

        for (;; ) {
            if (_check_for_input()) {
                break;
            }
            hal.scheduler->delay(20);
        }

        // we have a full command to run
        if (_run_command(false)) break;

        _display_prompt();
    }    
}

// check for new user input
bool
Menu::check_input(void)
{
	if (_port == nullptr) {
		// default to main serial port
		_port = hal.console;
	}

    _allocate_buffers();

    if (_check_for_input()) {
        return _run_command(true);
    }    

    return false;
}

// display the list of commands in response to the 'help' command
void
Menu::_help(void)
{
    int i;

    _port->printf("Commands:\n");
    for (i = 0; i < _entries; i++) {
		hal.scheduler->delay(10);
        _port->printf("  %s\n", _commands[i].command);
	}
}

// run the n'th command in the menu
int8_t
Menu::_call(uint8_t n, uint8_t argc)
{
    return _commands[n].func(argc, &_argv[0]);
}

/**
   set limits on max args and command line length
*/
void
Menu::set_limits(uint8_t commandline_max, uint8_t args_max)
{
    if (_inbuf != nullptr) {
        delete[] _inbuf;
        _inbuf = nullptr;
    }
    if (_argv != nullptr) {
        delete[] _argv;
        _argv = nullptr;
    }
    // remember limits, the buffers will be allocated by allocate_buffers()
    _commandline_max = commandline_max;
    _args_max = args_max;
}

void
Menu::_allocate_buffers(void)
{
    /* only allocate if the buffers are nullptr */
    if (_inbuf == nullptr) {
        _inbuf = NEW_NOTHROW char[_commandline_max];
        memset(_inbuf, 0, _commandline_max);
    }
    if (_argv == nullptr) {
        _argv = NEW_NOTHROW arg[_args_max+1];
        memset(_argv, 0, (_args_max+1) * sizeof(_argv[0]));
    }
}
