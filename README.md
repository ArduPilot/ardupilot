# ArduPilot Project

You can find lots of development information at the [ArduPilot development site](http://dev.ardupilot.com)

#### To compile APM2.x Ardupilot after version 3.1 please follow the instructions found at 

[Dev.Ardupilot] (http://dev.ardupilot.com/wiki/building-ardupilot-with-arduino-windows/) 


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


## Development using VirtualBox and Vagrants

ardupilot has a standardized Linux virtual machine (VM) setup script
that uses the free VirtualBox virtualization software.  You can use it
to create a standard, reproducible development environment in just a
few minutes in Linux, OS X, or Windows.

### Getting the source

Vagrants will map shared folders so you can build from source from your host.  This is visible in the `Vagrantfile` shown in the snippet below:

```
[..]
config.vm.synced_folder(".", "/home/vagrant/ardupilot" )
config.vm.synced_folder("../diydrones/PX4Firmware", "/home/vagrant/PX4Firmware" )
config.vm.synced_folder("../diydrones/PX4NuttX", "/home/vagrant/PX4NuttX" )
config.vm.synced_folder("../mavlink/mavlink", "/home/vagrant/mavlink" )
config.vm.synced_folder("../tridge/jsbsim", "/home/vagrant/jsbsim" )
[..]
```

This assumes that all git clones were created as follows:

```
git clone git://github.com/diydrones/ardupilot.git C:\<your git working root>\diydrones\ardupilot
git clone git://github.com/diydrones/PX4Firmware.git C:\<your git working root>\diydrones\PX4Firmware
git clone git://github.com/diydrones/PX4NuttX.git C:\<your git working root>\diydrones\PX4NuttX
git clone git://github.com/mavlink/mavlink.git C:\<your git working root>\mavlink\mavlink
git clone git://github.com/tridge/jsbsim.git C:\<your git working root>\tridge\jsbsim
```

### Installation Steps

 1. [Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)
 for your Mac, Windows or Linux machine.

 2. [Install vagrant](http://docs.vagrantup.com/v2/installation/).
 
 3. <strong>Enable Virtualization in BIOS</strong>  This Vagrantfile will instruct Vbox provisioner
 to enable multi-core of 4 with a CPU cap of 80%. If you do not, the SITL will be extremely slow.  If you don't
 enable Virtualization options, the box will fail to boot.
 
 4. <strong>Edit the Vagrantfile</strong> shared_folders to map to the three required projects: ardupilot, PX4Nuttx, PX4Firmware, and jsbsim.

 5. <strong>Optionally, edit your location for your Ubuntu archives</strong> From the list here [Ubuntu Mirrors](https://launchpad.net/ubuntu/+archivemirrors) search and replace the values in `~ardupilot/puppet/modules/preconditionals/files/sources.list` to where you want.  By default, it points to Easynews.com since they have a large download pipe. This can help speed up provisioning since apt-get will use an archive mirror closer or with a bigger pipe than the default ubuntu mirror.
 
 5. <strong>Start the Vagrant</strong> In the `ardupilot` directory, run `vagrant up` from the command
 line.  This will create a new Ubuntu Linux VM.  This box is provisioned via puppet.
 any updates to the provisioning can be applied by running `vagrant provision`.
 The puppet files mimic the setup in "ardupilot/Tools/scripts/install-prereqs-ubuntu.sh -y"` as well
 as the requirements for SITL.  You do NOT need to run the script.  <strong>Puppet handles the provisioning.</strong>
 
 6. <strong>Wait for Puppet to complete provisioning</strong> Once the provisioning is complete (it takes a while (30min or so) due to the large UI install packages)
 
 7. <strong>Reboot</strong> to get a nice UI run `vagrant reload` and upon reboot, the UI will appear. I recommend logging in using GNOME 2d (no effects) and installing the 
 GuestBoxAdditions from the Guest machine.

 8. <strong>Login</strong> Login password is "vagrant" as well as sudo password.
 
 9. <strong>Fixing the guest additions defect in VBox 4.3.10</strong> VirtualBox 4.3.10 has a a defect in the VBoxAdditions.  You will need to run the script "sudo vbox_patch.sh" located in
 home directory then reload the machine with `vagrant reload` otherwise vagrant will fail to mount the shared folders AFTER
 you've installed the VBoxAdditions.
 
Once you've followed the instructions above, here's how you would
build ArduCopter for PX4 in the development environment:

```
# cd ardupilot/ArduCopter
# make configure
```

Back at the terminal:

```
# make px4
# make px4-upload  # (optional)
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
