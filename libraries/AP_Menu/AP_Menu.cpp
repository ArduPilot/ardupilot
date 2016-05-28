// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple commandline menu system.
//

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>

#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "AP_Menu.h"

extern const AP_HAL::HAL& hal;

// statics
char *Menu::_inbuf;
Menu::arg *Menu::_argv;
AP_HAL::BetterStream *Menu::_port;


// constructor
Menu::Menu(const prog_char *prompt, const Menu::command *commands, uint8_t entries, preprompt ppfunc) :
    _prompt(prompt),
    _commands(commands),
    _entries(entries),
    _ppfunc(ppfunc),
    _commandline_max(MENU_COMMANDLINE_MAX),
    _args_max(MENU_ARGS_MAX)
{
    // the buffers are initially NULL, then they are allocated on
    // first use
    _inbuf = NULL;
    _argv = NULL;
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
    _port->printf_P(PSTR("%S] "), FPSTR(_prompt));    
}

// run the menu
bool
Menu::_run_command(bool prompt_on_enter)
{
    int8_t ret;
    uint8_t i;
    uint8_t argc;
    char *s = NULL;

    _input_len = 0;

    // split the input line into tokens
    argc = 0;
    s = NULL;
    _argv[argc++].str = strtok_r(_inbuf, " ", &s);

    // XXX should an empty line by itself back out of the current menu?
    while (argc <= _args_max) {
        _argv[argc].str = strtok_r(NULL, " ", &s);
        if ('\0' == _argv[argc].str)
            break;
        _argv[argc].i = atol(_argv[argc].str);
        _argv[argc].f = atof(_argv[argc].str);      // calls strtod, > 700B !
        argc++;
    }
    
    if (_argv[0].str == NULL) {
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
        if (!strcasecmp_P(_argv[0].str, _commands[i].command)) {
            ret = _call(i, argc);
            cmd_found=true;
            if (-2 == ret)
                return true;
            break;
        }
    }
    
    // implicit commands
    if (i == _entries) {
        if (!strcmp(_argv[0].str, "?") || (!strcasecmp_P(_argv[0].str, PSTR("help")))) {
            _help();
            cmd_found=true;
        } else if (!strcasecmp_P(_argv[0].str, PSTR("exit"))) {
            // exit the menu
            return true;
        }
    }

    if (cmd_found==false)
    {
        _port->println_P(PSTR("Invalid command, type 'help'"));
    }

    return false;
}


// run the menu
void
Menu::run(void)
{
	if (_port == NULL) {
		// default to main serial port
		_port = hal.console;
	}

    _allocate_buffers();

    _display_prompt();

    // loop performing commands
    for (;;) {

        // run the pre-prompt function, if one is defined
        if (NULL != _ppfunc) {
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
	if (_port == NULL) {
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

    _port->println_P(PSTR("Commands:"));
    for (i = 0; i < _entries; i++) {
		hal.scheduler->delay(10);
        _port->printf_P(PSTR("  %S\n"), FPSTR(_commands[i].command));
	}
}

// run the n'th command in the menu
int8_t
Menu::_call(uint8_t n, uint8_t argc)
{
    func fn;

    fn = (func)pgm_read_pointer(&_commands[n].func);
    return(fn(argc, &_argv[0]));
}

/**
   set limits on max args and command line length
*/
void
Menu::set_limits(uint8_t commandline_max, uint8_t args_max)
{
    if (_inbuf != NULL) {
        delete[] _inbuf;
        _inbuf = NULL;
    }
    if (_argv != NULL) {
        delete[] _argv;
        _argv = NULL;
    }
    // remember limits, the buffers will be allocated by allocate_buffers()
    _commandline_max = commandline_max;
    _args_max = args_max;
}

void
Menu::_allocate_buffers(void)
{
    /* only allocate if the buffers are NULL */
    if (_inbuf == NULL) {
        _inbuf = new char[_commandline_max];
        memset(_inbuf, 0, _commandline_max);
    }
    if (_argv == NULL) {
        _argv = new arg[_args_max+1];
        memset(_argv, 0, (_args_max+1) * sizeof(_argv[0]));
    }
}
