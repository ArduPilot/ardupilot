To keep access to waf convenient, use the following alias from the
root ArduPilot directory

    alias waf="$PWD/waf"

that way waf can be called from subdirectories to trigger partial
builds.

Differently from the make-based build, with waf there's a configure step
to choose the board to be used

    # Configure the Linux board.
    waf configure --board=linux

by default the board used is 'sitl'. This must be called from the root
ardupilot directory. Other commands may be issued from anywhere in the
tree.

To build, use the 'waf build' command. This is the default command, so
calling just 'waf' is enough

    # From the root ardupilot directory, build everything.
    waf

    # Waf also accepts '-j' option to parallelize the build.
    waf -j8

In subdirectories of vehicles, examples and tools (they contain a
wscript file), it's possible to trigger a build of just that program
either by calling waf in the subdirectory or by specifying it as part of
targets

    # Will build only ArduCopter
    cd ArduCopter; waf -j9; cd -

    # From the top directory, note the board name used in the target
    waf --targets=ArduCopter.linux

    # List all the targets available
    waf list

By default all the files produced by the build will be inside the build/
subdirectory. The binaries will also be there, with the name identifying
the target board.

To clean things up use

    # Clean the build products, but keep configure information
    waf clean

    # Clean everything, will need to call configure again
    waf distclean

using git to clean the files also work fine.


TODO: Add explanation on how the build system is organized once we
settle down.
