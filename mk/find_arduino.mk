#
# Find Arduino, if not explicitly specified
#
ifeq ($(ARDUINO),)

  #
  # List locations that might be valid ARDUINO settings
  #
  ifeq ($(SYSTYPE),Darwin)
    # use Spotlight to find Arduino.app
    ARDUINO_QUERY	=	'kMDItemContentType == com.apple.application-bundle && kMDItemFSName == Arduino.app'
    ARDUINOS		:=	$(addsuffix /Contents/Resources/Java,$(shell mdfind -literal $(ARDUINO_QUERY)))
    ifeq ($(ARDUINOS),)
      $(error ERROR: Spotlight cannot find Arduino on your system.)
    endif
  endif

  ifeq ($(SYSTYPE),Linux)
    ARDUINO_SEARCHPATH	=	/usr/share/arduino* /usr/local/share/arduino*
    ARDUINOS		:=	$(wildcard $(ARDUINO_SEARCHPATH))
  endif

  ifneq ($(findstring CYGWIN, $(SYSTYPE)),)
	# Most of the following commands are simply to deal with whitespaces in the path
	# Read the "Program Files" system directory from the windows registry
	PROGRAM_FILES		:=	$(shell cat /proc/registry/HKEY_LOCAL_MACHINE/SOFTWARE/Microsoft/Windows/CurrentVersion/ProgramFilesDir)
	# Convert the path delimiters to /
	PROGRAM_FILES		:=	$(shell cygpath -m ${PROGRAM_FILES})
	# Escape the space with a backslash
	PROGRAM_FILES		:=	$(shell echo $(PROGRAM_FILES) | sed s/\ /\\\\\ / )
	# Use DOS paths because they do not contain spaces
	PROGRAM_FILES		:=	$(shell cygpath -d ${PROGRAM_FILES})
	# Convert the path delimiters to /
	PROGRAM_FILES	:=	$(subst \,/,$(PROGRAM_FILES))
	# Search for an Arduino instalation in a couple of paths
	ARDUINO_SEARCHPATH	:=	c:/arduino* $(PROGRAM_FILES)/arduino*
    ARDUINOS		:=	$(wildcard $(ARDUINO_SEARCHPATH))
  endif

  #
  # Pick the first option if more than one candidate is found.
  #
  ARDUINO		:=	$(firstword $(ARDUINOS))
  ifeq ($(ARDUINO),)
    $(error ERROR: Cannot find Arduino on this system, please specify on the commandline with ARDUINO=<path> or on the config.mk file)
  endif

  ifneq ($(words $(ARDUINOS)),1)
    $(warning WARNING: More than one copy of Arduino was found, using $(ARDUINO))
  endif

endif
