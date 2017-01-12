# Building ArduPilot #

Ardupilot is gradually moving from the make-based build system to
[Waf](https://waf.io/). The instructions below should be enough for you to
build Ardupilot, but you can also read more about the build system in the
[Waf Book](https://waf.io/book/).

Waf should always be called from the ardupilot's root directory. Differently
from the make-based build, with Waf there's a configure step to choose the
board to be used (default is `sitl`).

## Basic usage ##

There are several commands in the build system for advanced usages, but here we
list some basic and more used commands as example.

* **Build ArduCopter**

    Here we use minlure as an example of Linux board. Other boards can be used
    and the next section shows how to get a list of available boards.

    ```sh
    ./waf configure --board minlure
    ./waf copter
    ```

    The first command should be called only once or when you want to change a
    configuration option. One configuration often used is the `--board` option to
    switch from one board to another one. For example we could switch to
    Pixhawk and build again:

    ```sh
    ./waf configure --board px4-v2
    ./waf copter
    ```

* **List available boards**


    It's possible to get a list of supported boards on ArduPilot with the command
    below

    ```sh
    ./waf list_boards

    ```

* **Clean the build**

    Commands `clean` and `distclean` can be used to clean the objects produced by
    the build. The first keeps the `configure` information, cleaning only the
    objects for the current board. The second cleans everything for every board,
    including the saved `configure` information.

    Cleaning the build is very often not necessary and discouraged. We do
    incremental builds reducing the build time by orders of magnitude.


* **Upload or install**

    Build commands have a `--upload` option in order to upload the binary built
    to a connected board. This option is supported by Pixhawk. The command below
    uses the `--targets` option that is explained in the next item.

    ```sh
    ./waf --targets bin/arducopter --upload
    ```

    Currently Linux boards don't support the upload option, but there's an
    install command, which will install to a certain directory. This can be
    used by distributors to create .deb, .rpm or other package types:

    ```sh
    ./waf copter
    DESTDIR=/my/temporary/location ./waf install
    ```

* **Use different targets**

    The build commands in the items above use `copter` as argument. This
    builds all binaries that fall under the "copter" group. See the
    section [Advanced usage](#advanced-usage) below for more details regarding
    groups.

    This shows a list of all possible targets:

    ```
    ./waf list
    ```

    For example, to build only a single binary:

    ```
    # Quad frame of ArduCopter
    ./waf --targets bin/arducopter

    # unit test of our math functions
    ./waf --targets tests/test_math
    ```

* **Other options**

    It's possible to see all available commands and options:

    ```
    ./waf -h
    ```

    Also, take a look on the [Advanced section](#advanced-usage) below.

## Advanced usage ##

This section contains some explanations on how the Waf build system works
and how you can use more advanced features.

Waf build system is composed of commands. For example, the command below
(`configure`) is for configuring the build with all the options used by this
particular build.

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

By default waf tries to parallelize the build automatically to all processors
so the `-j` option is usually not needed, unless you are using icecc (thus
you want a bigger value) or you don't want to stress your machine with
the build.

### Program groups ###

Program groups are used to represent a class of programs. They can be used to
build all programs of a certain class without having to specify each program.
It's possible for two groups to overlap, except when both groups are main
groups. In other words, a program can belong to more than one group, but only
to one main group.

There's a special group, called "all", that comprises all programs.

#### Main groups ####

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

#### Main product groups ####

Those are groups for ardupilot's main products. They contain programs for the
product they represent. Currently only the "copter" group has more than one
program - one for each frame type.

The main product groups are:

 - antennatracker
 - copter
 - plane
 - rover

#### Building a program group ####

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
#### Shortcut for program groups ####

For less typing, you can use the group name as the command to waf. Examples:

```bash
# Build all vehicles and Antenna Tracker
./waf bin

# Build all examples
./waf examples

# Build arducopter binaries
./waf copter
```

### Building a specific program ###

In order to build a specific program, you just need to pass its path relative
to `build/<board>/` to the option `--targets`. Example:

```bash
# Build arducopter for quad frame
./waf --targets bin/arducopter

# Build vectors unit test
./waf --targets tests/test_vectors
```

### Checking ###

The command `check` builds all programs and then executes the relevant tests.
In that context, a relevant test is a program from the group "tests" that makes
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

### Debugging ###

It's possible to pass the option `--debug` to the `configure` command. That
will set compiler flags to store debugging information in the binaries so that
you can use them with `gdb`, for example. The build directory will be set to
`build/<board>-debug/`. That option might come handy when using SITL.

### Build-system wrappers ###

The `waf` binary on root tree is actually a wrapper to the real `waf` that's
maintained in its own submodule.  It's possible to call the latter directly via
`./modules/waf/waf-light` or to use an alias if you prefer typing `waf` over
`./waf`.

```sh
alias waf="<ardupilot-directory>/modules/waf/waf-light"

```

There's also a make wrapper called `Makefile.waf`. You can use
`make -f Makefile.waf help` for instructions on how to use it.

### Command line help ###

You can use `waf --help` to see information about commands and options built-in
to waf as well as some quick help on those added by ardupilot.
