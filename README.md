# ArduPilot Project

## Getting the source

You can either download the source using the "ZIP" button at the top of the
github page, or you can make a clone using git:

```
git clone git://github.com/diydrones/ardupilot.git
```

## Building using Arduino IDE

ArduPilot is no longer compatible with the standard Arduino distribution.
You need to use a patched Arduino IDE to build ArduPilot.

1. The patched ArduPilot Arduino IDE is available for Mac and Windows from
   the [downloads page][1]. On Linux, you should use the makefile build.

2. Unpack and launch the ArduPilot Arduino IDE. In the preferences menu, set
   your sketchbook location to your downloaded or cloned `ardupilot` directory.

3. In the ArduPilot Arduino IDE, select your ArduPilot type (APM1 or APM2) from
   the ArduPilot menu (in the top menubar).

4. Restart the ArduPilot Arduino IDE. You should now be able to build ArduPlane
   or ArduCopter from source.

5. Remember that, after changing ArduPilot type (APM1 or APM2) in the IDE,
   you'll need to close and restart the IDE before continuing.

[1]: http://code.google.com/p/ardupilot-mega/downloads/list


## Building using make 

 1. Before you build the project for the first time, you'll need to run
    `make configure` from a  sketch directory (i.e. ArduPlane, ArduCopter, etc...).
    This will create a `config.mk` file at the top level of the repository. You can
    set some defaults in `config.mk`

 2. In the sketch directory, type `make` to build for APM2. Alternatively,
    `make apm1` will build for the APM1.  The binaries will generated in
    `/tmp/<i>sketchname</i>.build`.

 3. Type `make upload` to upload. You may need to set the correct default
    serial port in your `config.mk`.

# User Technical Support

ArduPilot users should use the DIYDrones.com forums for technical support.

# Development Team

The ArduPilot project is open source and maintained by a team of volunteers.

To contribute, you can send a pull request on Github. You can also join the
[development discussion on Google Groups][2]. Note that the Google Groups
mailing lists are NOT for user tech support, and are moderated for new users to
prevent off-topic discussion.

[2]: https://groups.google.com/forum/?fromgroups#!forum/drones-discuss
