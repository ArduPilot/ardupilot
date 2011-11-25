// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	menu.h
/// @brief	Simple commandline menu subsystem.
/// @discussion
/// The Menu class implements a simple CLI that accepts commands typed by
/// the user, and passes the arguments to those commands to a function
/// defined as handing the command.
///
/// Commands are defined in an array of Menu::command structures passed
/// to the constructor.  Each entry in the array defines one command.
///
/// Arguments passed to the handler function are pre-converted to both
/// long and float for convenience.

#ifndef __AP_COMMON_MENU_H
#define __AP_COMMON_MENU_H

#include <inttypes.h>

#define MENU_COMMANDLINE_MAX	32	///< maximum input line length
#define MENU_ARGS_MAX			4	///< maximum number of arguments
#define MENU_COMMAND_MAX		14	///< maximum size of a command name

/// Class defining and handling one menu tree
class Menu {
public:
    /// argument passed to a menu function
    ///
    /// Space-delimited arguments are parsed from the commandline and
    /// separated into these structures.
    ///
    /// If the argument cannot be parsed as a float or a long, the value
    /// of f or i respectively is undefined.  You should range-check
    /// the inputs to your function.
    ///
    struct arg {
        const char	*str;			///< string form of the argument
        long		i;				///< integer form of the argument (if a number)
        float		f;				///< floating point form of the argument (if a number)
    };

    /// menu command function
    ///
    /// Functions called by menu array entries are expected to be of this
    /// type.
    ///
    /// @param	argc		The number of valid arguments, including the
    ///						name of the command in argv[0].  Will never be
    ///						more than MENU_ARGS_MAX.
    /// @param	argv		Pointer to an array of Menu::arg structures
    ///						detailing any optional arguments given to the
    ///						command.  argv[0] is always the name of the
    ///						command, so that the same function can be used
    ///						to handle more than one command.
    ///
    typedef int8_t			(*func)(uint8_t argc, const struct arg *argv);

    /// menu pre-prompt function
    ///
    /// Called immediately before waiting for the user to type a command; can be
    /// used to display help text or status, for example.
    ///
    /// If this function returns false, the menu exits.
    ///
    typedef bool			(*preprompt)(void);

    /// menu command description
    ///
    struct command {
        /// Name of the command, as typed or received.
        /// Command names are limited in size to keep this structure compact.
        ///
        const char	command[MENU_COMMAND_MAX];

        /// The function to call when the command is received.
        ///
        /// The argc argument will be at least 1, and no more than
        /// MENU_ARGS_MAX.  The argv array will be populated with
        /// arguments typed/received up to MENU_ARGS_MAX.  The command
        /// name will always be in argv[0].
        ///
        /// Commands may return -2 to cause the menu itself to exit.
        /// The "?", "help" and "exit" commands are always defined, but
        /// can be overridden by explicit entries in the command array.
        ///
        int8_t			(*func)(uint8_t argc, const struct arg *argv);	///< callback function
    };

    /// constructor
    ///
    /// Note that you should normally not call the constructor directly.  Use
    /// the MENU and MENU2 macros defined below.
    ///
    /// @param prompt		The prompt to be displayed with this menu.
    /// @param commands		An array of ::command structures in program memory (PROGMEM).
    /// @param entries		The number of entries in the menu.
    ///
    Menu(const char *prompt, const struct command *commands, uint8_t entries, preprompt ppfunc = 0);

    /// menu runner
    void				run(void);

private:
    /// Implements the default 'help' command.
    ///
    void				_help(void);					///< implements the 'help' command

    /// calls the function for the n'th menu item
    ///
    /// @param n			Index for the menu item to call
    /// @param argc			Number of arguments prepared for the menu item
    ///
    int8_t				_call(uint8_t n, uint8_t argc);

    const char			*_prompt;						///< prompt to display
    const command		*_commands;						///< array of commands
    const uint8_t		_entries;						///< size of the menu
    const preprompt		_ppfunc;						///< optional pre-prompt action

    static char			_inbuf[MENU_COMMANDLINE_MAX];	///< input buffer
    static arg			_argv[MENU_ARGS_MAX + 1];		///< arguments
};

/// Macros used to define a menu.
///
/// The commands argument should be an arary of Menu::command structures, one
/// per command name.  The array does not need to be terminated with any special
/// record.
///
/// Use name.run() to run the menu.
///
/// The MENU2 macro supports the optional pre-prompt printing function.
///
#define MENU(name, prompt, commands)							\
	static const char __menu_name__ ##name[] PROGMEM = prompt;	\
	static Menu name(__menu_name__ ##name, commands, sizeof(commands) / sizeof(commands[0]))

#define MENU2(name, prompt, commands, preprompt)				\
	static const char __menu_name__ ##name[] PROGMEM = prompt;	\
	static Menu name(__menu_name__ ##name, commands, sizeof(commands) / sizeof(commands[0]), preprompt)

#endif
