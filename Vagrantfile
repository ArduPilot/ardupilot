# -*- mode: ruby -*-
# vi: set ft=ruby :

# Testing an ArduPilot VM:
# sim_vehicle.py # in the starting directory should start a Copter simulation
# xterm # X11 forwarding should work
# sim_vehicle.py --debug --gdb
# sim_vehicle.py --valgrind
# cd /vagrant && ./waf configure --board=px4-v2 && ./waf build --target=bin/ardusub
# cd /vagrant ./waf configure --board=navio2 && ./waf build --target=bin/arduplane
# cd /vagrant ./Tools/autotest/sim_vehicle.py -v ArduPlane # should test JSBSim
# cd /vagrant ./Tools/autotest/autotest.py build.APMrover2 drive.APMrover2

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  config.vm.provider "virtualbox" do |vb|
      # Don't boot with headless mode
      #   vb.gui = true
      #
      #   # Use VBoxManage to customize the VM. For example to change memory:
      vb.customize ["modifyvm", :id, "--memory", "2048"]
      vb.customize ["modifyvm", :id, "--ioapic", "on"]
      vb.customize ["modifyvm", :id, "--cpus", "2"]
      # Make some effort to avoid clock skew
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-threshold", "5000"]
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-start"]
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-on-restore", "1"]
  end

  # If you are on windows then you must use a version of git >= 1.8.x
  # to update the submodules in order to build. Older versions of git
  # use absolute paths for submodules which confuses things.

  # removing this line causes "A box must be specified." error:
  config.vm.box = "ubuntu/artful32"

  # LTS, EOL April, 2019:
  config.vm.define "trusty64", autostart: false do |trusty64|
    config.vm.box = "ubuntu/trusty64"
    config.vm.provision "trusty64", type: "shell", path: "Tools/vagrant/initvagrant.sh"
    config.vm.name = "ArduPilot (Trusty64)"
  end

  # LTS, EOL April 2021
  config.vm.define "xenial32", autostart: false do |xenial32|
    config.vm.box = "ubuntu/xenial32"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    config.vm.name = "ArduPilot (Xenial32)"
    config.vm.gui = true
  end

  # EOL January 2018
  config.vm.define "zesty32", autostart: false do |zesty32|
    config.vm.box = "ubuntu/zesty32"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
  end

  # EOL July 2018
  config.vm.define "artful32", primary: true do |artful32|
    config.vm.box = "ubuntu/artful32"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
  end

  config.vm.define "bionic32", autostart: false do |bionic32|
    config.vm.box = "ubuntu/bionic32"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
  end

end

