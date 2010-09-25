// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	menu.h
/// @brief	Simple commandline menu subsystem.

#define MENU_COMMANDLINE_MAX	32	///< maximum input line length
#define MENU_ARGS_MAX			4	///< maximum number of arguments
#define MENU_COMMAND_MAX		14	///< maximum size of a command name

/// Class defining and handling one menu tree
class Menu {
public:
	/// argument passed to a menu function
	struct arg {
		const char	*str;			///< string form of the argument
		long		i;				///< integer form of the argument (if a number)
		float		f;				///< floating point form of the argument (if a number)
	};

	/// menu command function
	///
	typedef int8_t			(*func)(uint8_t argc, const struct arg *argv);

	/// menu pre-prompt function
	///
	/// If this function returns false, the menu exits.
	typedef bool			(*preprompt)(void);
	
	/// menu command description
	///
	/// Note that the array of menu commands is expected to be in program 
	/// memory.
	struct command {
		/// Name of the command, as typed or received.
		/// Command names are limited in size to keep this structure compact.
		const char	command[MENU_COMMAND_MAX];

		/// The function to call when the command is received.
		/// The \a argc argument will be at least 1, and no more than
		/// MENU_ARGS_MAX.  The argv array will be populated with
		/// arguments typed/received up to MENU_ARGS_MAX.  The command
		/// name will always be in argv[0].
		/// Commands may return -2 to cause the menu itself to exit.
		/// The "?", "help" and "exit" commands are always defined.
		int8_t			(*func)(uint8_t argc, const struct arg *argv);	///< callback function
	};

	/// constructor
	///
	/// @param prompt		The prompt to be displayed with this menu.
	/// @param commands		An array of ::command structures.
	/// @param entries		The number of entries in the menu.
	///
	Menu(const char *prompt, const struct command *commands, uint8_t entries, preprompt ppfunc = 0);

	/// menu runner
	void				run(void);
	
private:
	/// Implements the default 'help' command
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
	
