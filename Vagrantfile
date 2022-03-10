# -*- mode: ruby -*-
# vi: set ft=ruby :

# Testing an ArduPilot VM:
# rm -rf /vagrant/build
# dpkg -l | grep modemmanager
# sim_vehicle.py --map --console # in the starting directory should start a Copter simulation
# sim_vehicle.py --debug --gdb
# sim_vehicle.py --debug --valgrind
# time (cd /vagrant && ./waf configure --board=fmuv2 && ./waf build --target=bin/ardusub) # ~9 minutes
# time (cd /vagrant && ./waf configure --board=fmuv3 && ./waf build --target=bin/ardusub) # ~ minutes (after building fmuv2)
# time (cd /vagrant && ./waf configure --board=navio2 && ./waf build --target=bin/arduplane)
# time (cd /vagrant && ./Tools/autotest/sim_vehicle.py --map --console -v ArduPlane -f jsbsim) # should test JSBSim
# time (cd /vagrant && ./Tools/autotest/autotest.py build.Rover test.Rover)

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.ssh.forward_x11 = true

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  config.vm.provider "virtualbox" do |vb|
      # Don't boot with headless mode
      #   vb.gui = true
      #
      #   # Use VBoxManage to customize the VM. For example to change memory:
      vb.customize ["modifyvm", :id, "--memory", "3192"]
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
  config.vm.box = "ubuntu/bionic64"
  config.vm.boot_timeout = 1500

  # LTS, EOL April, 2019:
  config.vm.define "trusty32", autostart: false do |trusty32|
    trusty32.vm.box = "ubuntu/trusty32"
    trusty32.vm.provision "trusty32", type: "shell", path: "Tools/vagrant/initvagrant.sh"
    trusty32.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (trusty32)"
    end
  end

  # 14.04.5 LTS, EOL April, 2019:
  config.vm.define "trusty64", autostart: false do |trusty64|
    trusty64.vm.box = "ubuntu/trusty64"
    trusty64.vm.provision "trusty64", type: "shell", path: "Tools/vagrant/initvagrant.sh"
    trusty64.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (trusty64)"
    end
  end

  # LTS, EOL April 2021
  # this VM is useful for running valgrind on!
  config.vm.define "xenial32", autostart: false do |xenial32|
    xenial32.vm.box = "ubuntu/xenial32"
    xenial32.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    xenial32.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (xenial32)"
    end
  end

  config.vm.define "xenial64", autostart: false do |xenial64|
    xenial64.vm.box = "ubuntu/xenial64"
    xenial64.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    xenial64.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (xenial64)"
    end
  end

  # NO LONGER AVAILABLE FOR DOWNLOAD, EOL January 2018
  # EOL January 2018
  # Only kept around for those few dev's who have already got this image and continue to use it.
  config.vm.define "zesty32", autostart: false do |zesty32|
    zesty32.vm.box = "ubuntu/zesty32"
    zesty32.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    zesty32.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (zesty32)"
    end
  end

  # 17.10, EOL July 2018
  # Only kept around for those few dev's who have already got this image and continue to use it; not available for download
  config.vm.define "artful32", autostart: false do |artful32|
    artful32.vm.box = "ubuntu/artful32"
    artful32.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    artful32.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (artful32)"
    end
  end

  # 18.04 LTS EOL April 2023
  # Only kept around for those few dev's who have already got this image and continue to use it; not available for download
  config.vm.define "bionic32", autostart: false do |bionic32|
    bionic32.vm.box = "ubuntu/bionic32"
    bionic32.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    bionic32.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (bionic32)"
    end
  end

  # 18.04 LTS EOL April 2023
  config.vm.define "bionic64", primary: true do |bionic64|
    bionic64.vm.box = "ubuntu/bionic64"
    bionic64.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    bionic64.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (bionic64)"
    end
  end

  # 18.04 LTS EOL April 2023
  config.vm.define "bionic64-desktop", autostart: false do |bionic64|
    bionic64.vm.box = "ubuntu/bionic64"
    bionic64.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    bionic64.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (bionic64-desktop)"
      vb.gui = true
    end
  end

  # 18.10
  config.vm.define "cosmic32", autostart: false do |cosmic32|
    cosmic32.vm.box = "ubuntu/cosmic32"
    cosmic32.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    cosmic32.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (cosmic32)"
    end
  end

  # 18.10
  config.vm.define "cosmic64", autostart: false do |cosmic64|
    cosmic64.vm.box = "ubuntu/cosmic64"
    cosmic64.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    cosmic64.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (cosmic64)"
    end
  end

  # 19.04
  config.vm.define "disco64", autostart: false do |disco64|
    disco64.vm.box = "ubuntu/disco64"
    disco64.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    disco64.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (disco64)"
    end
  end

  # 19.10 - broken; fails to use NamedTemporaryFile to create file for gdb
#  config.vm.define "eoan", autostart: false do |eoan|
#    eoan.vm.box = "ubuntu/eoan64"
#    eoan.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
#    eoan.vm.provider "virtualbox" do |vb|
#      vb.name = "ArduPilot (eoan)"
#    end
#    eoan.vm.boot_timeout = 1200
#  end

  # 20.04 LTS  EOL April 2025
  config.vm.define "focal", autostart: false do |focal|
    focal.vm.box = "ubuntu/focal64"
    focal.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    focal.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (focal)"
    end
    focal.vm.boot_timeout = 1200
  end
  config.vm.define "focal-desktop", autostart: false do |focal|
    focal.vm.box = "ubuntu/focal64"
    focal.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    focal.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (focal-desktop)"
      vb.gui = true
    end
    focal.vm.boot_timeout = 1500
  end

  # 20.10
  config.vm.define "groovy", autostart: false do |groovy|
    groovy.vm.box = "ubuntu/groovy64"
    groovy.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    groovy.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (groovy)"
    end
    groovy.vm.boot_timeout = 1200
  end

  # 21.04
  config.vm.define "hirsute", autostart: false do |hirsute|
    hirsute.vm.box = "ubuntu/hirsute64"
    hirsute.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    hirsute.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (hirsute)"
    end
    hirsute.vm.boot_timeout = 1200
  end
  config.vm.define "hirsute-desktop", autostart: false do |hirsute|
    hirsute.vm.box = "ubuntu/hirsute64"
    hirsute.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    hirsute.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (hirsute-desktop)"
      vb.gui = true
    end
    hirsute.vm.boot_timeout = 1200
  end

  # 21.10
  config.vm.define "impish", autostart: false do |impish|
    impish.vm.box = "ubuntu/impish64"
    impish.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    impish.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (impish)"
    end
    impish.vm.boot_timeout = 1200
  end

  # 22.04 LTS EOL Apr 2032
  config.vm.define "jammy", autostart: false do |jammy|
    jammy.vm.box = "ubuntu/jammy64"
    jammy.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    jammy.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (jammy)"
    end
    jammy.vm.boot_timeout = 1200
  end
  config.vm.define "jammy-desktop", autostart: false do |jammy|
    jammy.vm.box = "ubuntu/jammy64"
    jammy.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    jammy.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (jammy-desktop)"
      vb.gui = true
    end
    jammy.vm.boot_timeout = 1200
  end

end
