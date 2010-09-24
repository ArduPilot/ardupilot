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
	typedef int			(*func)(uint8_t argc, const struct arg *argv);
	
	/// menu command description
	///
	/// Note that the array of menu commands is expected to be in program 
	/// memory.
	struct command {
		const char	command[MENU_COMMAND_MAX];						///< name of the command
		int			(*func)(uint8_t argc, const struct arg *argv);	///< callback function
	};

	/// constructor
	///
	/// @param prompt		The prompt to be displayed with this menu.
	/// @param commands		An array of ::command structures.
	/// @param entries		The number of entries in the menu.
	///
	Menu(const char *prompt, const struct command *commands, uint8_t entries);

	/// menu runner
	void				run(void);
	
private:
	void				_help(void);					///< implements the 'help' command
	const char			*_prompt;						///< prompt to display
	const command		*_commands;						///< array of commands
	const uint8_t		_entries;						///< size of the menu

	static char			_inbuf[MENU_COMMANDLINE_MAX];	///< input buffer
	static arg			_argv[MENU_ARGS_MAX + 1];		///< arguments
};

/// Macro used to define a menu.
///
/// Use name.run() to run the menu.
///
#define MENU(name, prompt, commands)							\
	static const char __menu_name__ ##name[] PROGMEM = prompt;	\
	static Menu name(__menu_name__ ##name, commands, sizeof(commands) / sizeof(commands[0]))
	
