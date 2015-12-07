To keep access to waf convenient, use the following alias from the
root ardupilot directory:

    alias waf="$PWD/modules/waf/waf-light"

Waf should always be called from the ardupilot's root.

Differently from the make-based build, with waf there's a configure step
to choose the board to be used

    # Configure the Linux board.
    waf configure --board=linux

by default the board used is 'sitl'.

To build, use the 'waf build' command. This is the default command, so
calling just 'waf' is enough

    # From the root ardupilot directory, build everything.
    waf

    # Waf also accepts '-j' option to parallelize the build.
    waf -j8

It's possible to build for just a vehicle or an example by specifying it as the
target:

    # From the top directory
    waf --target ArduCopter

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

There's also a make wrapper called "Makefile.waf". You can use
`make -f Makefile.waf help` for instructions on how to use it.

TODO: Add explanation on how the build system is organized once we
settle down.
