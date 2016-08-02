# WAF Build #

Ardupilot is gradually moving from the make-based build system to
[Waf](https://waf.io/).

You can read the [Waf Book](https://waf.io/book/) if you want to learn more
about Waf.

## Calling waf ##

Waf should always be called from the ardupilot's root directory.

Differently from the make-based build, with Waf there's a configure step
to choose the board to be used (default is `sitl`):

## Basic use ##

There are several commands in the build system for advanced usages, but here we
list some basic examples.

### Build ArduCopter for only one board ###

Here we use minlure as an example of Linux board. See more advanced usages
below to get a list of supported boards.

```bash
./waf configure --board minlure
./waf copter
```

This will buil all copter frames for minlure board. The configure command
needs to be given only when you are switching from a board (or sitl) to another
one so usually it's just one command to keep a change/test/debug cycle.
The build will always be incremental and there's no reason to do a clean
build

Building ArduCopter Quad for Pixhawk:

```bash
./waf configure --board px4-v2
./waf --targets bin/arducopter-quad
```

px4-v2 board supports the --upload option, so the last command could be:

```bash
./waf --targets bin/arducopter-quad --upload
```

In this case it would build and upload to a connected Pixhawk board.

## Advanced use ##

Waf build system is composed of commands. For example, the command below
(`configure`) is for configuring the build.

```bash
# Configure the Linux board
./waf configure --board=linux
```

Consequently, in order to build, a "build" command is issued, thus `waf build`.
That is the default command, so calling just `waf` is enough:

```bash
# Build programs from bin group
./waf

# Waf also accepts '-j' option to parallelize the build.
./waf -j8
```

To clean things up, use the `clean` or `distclean` command:

```bash
# Clean the build products, but keep configure information
./waf clean

# Clean everything, will need to call configure again
./waf distclean
```

Using git to clean the files also work fine.

To list the task generator names that can be used for the option `--targets`,
use the `list`command:

```bash
./waf list
```

## Program groups ##

Program groups are used to represent a class of programs. They can be used to
build all programs of a certain class without having to specify each program.
It's possible for two groups to overlap, except when both groups are main
groups. In other words, a program can belong to more than one group, but only
to one main group.

There's a special group, called "all", that comprises all programs.

### Main groups ###

The main groups form a partition of all programs. Besides separating the
programs logically, they also define where they are built.

The main groups are:

 - bin: *the main binaries, that is, ardupilot's main products - the vehicles and
   Antenna Tracker*
 - tools
 - examples: *programs that show how certain libraries are used or to simply
   test their operation*
 - benchmarks: *requires `--enable-benchmarks` during configurarion*
 - tests: *basically unit tests to ensure changes don't break the system's
   logic*

All build files are placed under `build/<board>/`, where `<board>` represents
the board/platform you selected during configuration. Each main program group
has a folder with its name directly under `build/<board>/`. Thus, a program
will be stored in `build/<board>/<main_group>/`, where `<main_group>` is the
main group the program belongs to. For example, for a linux build, arduplane,
which belongs to the main group "bin", will be located at
`build/linux/bin/arduplane`.

### Main products groups ###

Those are groups for ardupilot's main products. They contain programs for the
product they represent. Currently only the "copter" group has more than one
program - one for each frame type.

The main products groups are:

 - antennatracker
 - copter
 - plane
 - rover

## Building a program group ##

Ardupilot adds to waf an option called `--program-group`, which receives as
argument the group you want it to build. For a build command, if you don't pass
any of `--targets` or `--program-group`, then the group "bin" is selected by
default. The option `--program-group` can be passed multiple times.

Examples:

```bash
# Group bin is the default one
./waf

# Build all vehicles and Antenna Tracker
./waf --program-group bin

# Build all benchmarks and tests
./waf --program-group benchmarks --program-group tests
```
### Shortcut for program groups ###

For less typing, you can use the group name as the command to waf. Examples:

```bash
# Build all vehicles and Antenna Tracker
./waf bin

# Build all examples
./waf examples

# Build arducopter binaries
./waf copter
```

## Building a specific program ##

In order to build a specific program, you just need to pass its path relative
to `build/<board>/` to the option `--targets`. Example:

```bash
# Build arducopter for quad frame
./waf --targets bin/arducopter-quad

# Build vectors unit test
./waf --targets tests/test_vectors
```

## Uploading ##

There's a build option `--upload` that can be used to tell the build that it
must upload the program(s) addressed by `--targets` arguments. The
implementation is board-specific and not all boards may have that implemented.
Example:

```bash
# PX4 supports uploading the program through a USB connection
./waf configure --board px4-v2
# Build arducopter-quad and upload it to my board
./waf --targets bin/arducopter-quad --upload
```

## Checking ##

The command `check` builds all programs and then run the relevant tests. In
that context, a relevant test is a program from the group "tests" that makes
one of the following statements true:

 - it's the first time the test is built since the last cleanup or when the
   project was cloned.
 - the program had to be rebuilt (due to modifications in the code or
   dependencies, for example)
 - the test program failed in the previous check.

That is, the tests are run only if necessary. If you want waf to run all tests,
then you can use either option `--alltests` or the shortcut command
`check-all`.

Examples:

```bash
# Build everything and run relevant tests
./waf check

# Build everything and run all tests
./waf check --alltests

# Build everything and run all tests
./waf check-all
```

## Debugging ##

It's possible to pass the option `--debug` to the `configure` command. That
will set compiler flags to store debugging information in the binaries so that
you can use them with `gdb`, for example. The build directory will be set to
`build/<board>-debug/`. That option might come handy when using SITL.

## Make wrapper ##

There's also a make wrapper called `Makefile.waf`. You can use
`make -f Makefile.waf help` for instructions on how to use it.

## Command line help ##

You can use `waf --help` to see information about commands and options built-in
to waf as well as some quick help on those added by ardupilot.
