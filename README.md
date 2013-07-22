# ArduPilot Project

You can find lots of development information at the [ArduPilot development site](http://dev.ardupilot.com)

## Getting the source

You can either download the source using the "ZIP" button at the top
of the github page, or you can make a clone using git:

```
git clone git://github.com/diydrones/ardupilot.git
```

## Prerequisites

### Ubuntu Linux

The following packages are required to build ardupilot for the
APM1/APM2 (Arduino) platform in Ubuntu: `gawk make git arduino-core
g++`

To build ardupilot for the PX4 platform, you'll first need to install
the PX4 toolchain and download the PX4 source code.  See the [PX4
toolchain installation
page](https://pixhawk.ethz.ch/px4/dev/toolchain_installation_lin).

The easiest way to install all these prerequisites is to run the
`ardupilot/Tools/scripts/install-prereqs-ubuntu.sh` script, which will
install all the required packages and download all the required
software.


## Building using the Arduino IDE

ArduPilot is no longer compatible with the standard Arduino
distribution.  You need to use a patched Arduino IDE to build
ArduPilot.

Do not try to use the Arduino IDE to build in Linux--you should follow
the instructions in the "Building using make" section.

1. The patched ArduPilot Arduino IDE is available for Mac and Windows
   from the [downloads
   page](http://firmware.diydrones.com).

2. Unpack and launch the ArduPilot Arduino IDE. In the preferences
   menu, set your sketchbook location to your downloaded or cloned
   `ardupilot` directory.

3. In the ArduPilot Arduino IDE, select your ArduPilot type (APM1 or
   APM2) from the ArduPilot menu (in the top menubar).

4. Restart the ArduPilot Arduino IDE. You should now be able to build
   ArduPlane or ArduCopter from source.

5. Remember that, after changing ArduPilot type (APM1 or APM2) in the
   IDE, you'll need to close and restart the IDE before continuing.


## Building using make

 1. Before you build the project for the first time, you'll need to run `make
    configure` from a  sketch directory (i.e. ArduPlane, ArduCopter, etc...).
    This will create a `config.mk` file at the top level of the repository. You
    can set some defaults in `config.mk`

 2. In the sketch directory, type `make` to build for APM2. Alternatively,
    `make apm1` will build for the APM1 and `make px4` will build for the PX4.
    The binaries will generated in `/tmp/<i>sketchname</i>.build`.

 3. Type `make upload` to upload. You may need to set the correct default
    serial port in your `config.mk`.


## Development using VirtualBox

ardupilot has a standardized Linux virtual machine (VM) setup script
that uses the free VirtualBox virtualization software.  You can use it
to create a standard, reproducible development environment in just a
few minutes in Linux, OS X, or Windows.

 1. [Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)
 for your Mac, Windows or Linux machine.

 2. [Install vagrant](http://docs.vagrantup.com/v2/installation/).

 4. In the `ardupilot` directory, run `vagrant up` from the command
 line.  This will create a new Ubuntu Linux VM.

 5. Run `vagrant ssh -c
 "ardupilot/Tools/scripts/install-prereqs-ubuntu.sh -y"`.  This will
 install all the prerequisites for doing ardupilot development.

You can now run `vagrant ssh` to log in to the development
environment.  The `~/ardupilot` directory in the VM is actually the
`ardupilot` directory in your host operating system--changes in either
directory show up in the other.

Once you've followed the instructions above, here's how you would
build ArduCopter for PX4 in the development environment:

```
$ vagrant ssh
# cd ardupilot/ArduCopter
# make configure
```

Now edit `ardupilot/config.mk` so it has the following line:

```
PX4_ROOT = ~/PX4-Firmware
```

Back at the terminal:

```
# cd ArduCopter
# make px4
# make px4-upload (optional)
```

# User Technical Support

ArduPilot users should use the DIYDrones.com forums for technical support.

# Development Team

The ArduPilot project is open source and maintained by a team of volunteers.

To contribute, you can send a pull request on Github. You can also
join the [development discussion on Google
Groups](https://groups.google.com/forum/?fromgroups#!forum/drones-discuss). Note
that the Google Groups mailing lists are NOT for user tech support,
and are moderated for new users to prevent off-topic discussion.
