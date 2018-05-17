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

  # removing this line causes "A box must be specified." error
  # and this is the default box that will be booted if no name is specified
  config.vm.box = "ubuntu/artful32"

  # LTS, EOL April, 2019:
  config.vm.define "trusty32", autostart: false do |trusty32|
    config.vm.box = "ubuntu/trusty32"
    config.vm.provision "trusty32", type: "shell", path: "Tools/vagrant/initvagrant.sh"
    config.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (trusty32)"
      vb.gui = true
    end
  end

  # 14.04.5 LTS, EOL April, 2019:
  config.vm.define "trusty64", autostart: false do |trusty64|
    config.vm.box = "ubuntu/trusty64"
    config.vm.provision "trusty64", type: "shell", path: "Tools/vagrant/initvagrant.sh"
    config.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (trusty64)"
      vb.gui = true
    end
  end

  # LTS, EOL April 2021
  # this VM is useful for running valgrind on!
  config.vm.define "xenial32", autostart: false do |xenial32|
    config.vm.box = "ubuntu/xenial32"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    config.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (xenial32)"
      vb.gui = true
    end
  end

  # NO LONGER AVAILABLE FOR DOWNLOAD, EOL January 2018
  # EOL January 2018
  # Only kept around for those few dev's who have already got this image and continue to use it.
  config.vm.define "zesty32", autostart: false do |zesty32|
    config.vm.box = "ubuntu/zesty32"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    config.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (zesty32)"
      vb.gui = true
    end
  end

  # 17.10, EOL July 2018
  config.vm.define "artful32", primary: true do |artful32|
    config.vm.box = "ubuntu/artful32"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    config.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (artful32)"
      vb.gui = true
    end
  end

  # 18.04 LTS , bleeding edge.
  config.vm.define "bionic32", autostart: false do |bionic32|
    config.vm.box = "ubuntu/bionic32"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    config.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (bionic32)"
      vb.gui = true
    end
  end

  # 18.04 LTS , bleeding edge.
  config.vm.define "bionic64", autostart: false do |bionic64|
    config.vm.box = "ubuntu/bionic64"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    config.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (bionic64)"
      vb.gui = true
    end
  end

end

