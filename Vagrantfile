# -*- mode: ruby -*-
# vi: set ft=ruby :

# Testing an ArduPilot VM:
# rm -rf /vagrant/build
# dpkg -l | grep modemmanager
# sim_vehicle.py --map --console # in the starting directory should start a Copter simulation
# sim_vehicle.py --debug --gdb
# sim_vehicle.py --debug --valgrind
#
# Make testing rather faster:
# time rsync -aPH /vagrant/ $HOME/ardupilot  # real	50s
# time (cd $HOME/ardupilot && ./waf configure --board=fmuv2 && ./waf build --target=bin/ardusub) # ~4 minutes
# time (cd $HOME/ardupilot && ./waf configure --board=fmuv3 && ./waf build --target=bin/ardusub) # ~4 minutes (after building fmuv2)
# time (cd $HOME/ardupilot && ./waf configure --board=navio2 && ./waf build --target=bin/arduplane)  # ~6 minutes
# time (cd $HOME/ardupilot && ./Tools/autotest/sim_vehicle.py --map --console -v ArduPlane -f jsbsim) # should test JSBSim
# time (cd $HOME/ardupilot && ./Tools/autotest/autotest.py build.Rover test.Rover)

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
  config.vm.boot_timeout = 1500

  config.vm.define "autotest-server", primary: true do |autotest|
    autotest.vm.box = "ubuntu/jammy64"
    autotest.vm.provision :shell, path: "Tools/vagrant/initvagrant-autotest-server.sh"
    autotest.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (autotest-server)"
    end
    autotest.vm.boot_timeout = 1200
  end

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

  # 22.04 LTS EOL Apr 2032
  config.vm.define "jammy", primary: true do |jammy|
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

  # 23.04 EOL Jan 2024
  config.vm.define "lunar", autostart: false do |lunar|
    lunar.vm.box = "ubuntu/lunar64"
    lunar.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    lunar.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (lunar)"
    end
    lunar.vm.boot_timeout = 1200
  end
  config.vm.define "lunar-desktop", autostart: false do |lunar|
    lunar.vm.box = "ubuntu/lunar64"
    lunar.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    lunar.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (lunar-desktop)"
      vb.gui = true
    end
    lunar.vm.boot_timeout = 1200
  end

  # 23.10 EOL Jul 2024
  config.vm.define "mantic", autostart: false do |mantic|
    mantic.vm.box = "ubuntu/mantic64"
    mantic.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    mantic.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (mantic)"
    end
    mantic.vm.boot_timeout = 1200
  end
  config.vm.define "mantic-desktop", autostart: false do |mantic|
    mantic.vm.box = "ubuntu/mantic64"
    mantic.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    mantic.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (mantic-desktop)"
      vb.gui = true
    end
    mantic.vm.boot_timeout = 1200
  end

  # 24.04 end of standard support Jun 2029
  # note the use of "bento" here; Ubuntu stopped providing Vagrant
  # images due to Hashicorp adopting the "Business Source License".
  config.vm.define "noble", autostart: false do |noble|
    noble.vm.box = "bento/ubuntu-24.04"
    noble.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    noble.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (noble)"
    end
    noble.vm.boot_timeout = 1200
  end
  config.vm.define "noble-desktop", autostart: false do |noble|
    noble.vm.box = "bento/ubuntu-24.04"
    noble.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    noble.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (noble-desktop)"
      vb.gui = true
    end
    noble.vm.boot_timeout = 1200
  end

  # 24.10 end of standard support ??
  # note the use of "alvistack" here; Ubuntu stopped providing Vagrant
  # images due to Hashicorp adopting the "Business Source License".
  config.vm.define "oracular", autostart: false do |oracular|
    oracular.vm.box = "alvistack/ubuntu-24.10"
    oracular.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    oracular.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (oracular)"
    end
    oracular.vm.boot_timeout = 1200
  end
  config.vm.define "oracular-desktop", autostart: false do |oracular|
    oracular.vm.box = "alvistack/ubuntu-24.10"
    oracular.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    oracular.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (oracular-desktop)"
      vb.gui = true
    end
    oracular.vm.boot_timeout = 1200
  end

  # 25.04 end of standard support ??
  # note the use of "alvistack" here; Ubuntu stopped providing Vagrant
  # images due to Hashicorp adopting the "Business Source License".
  config.vm.define "plucky", autostart: false do |plucky|
    plucky.vm.box = "alvistack/ubuntu-25.04"
    plucky.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    plucky.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (plucky)"
    end
    plucky.vm.boot_timeout = 1200
  end
  config.vm.define "plucky-desktop", autostart: false do |plucky|
    plucky.vm.box = "alvistack/ubuntu-25.04"
    plucky.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    plucky.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (plucky-desktop)"
      vb.gui = true
    end
    plucky.vm.boot_timeout = 1200
  end

  # 25.10 end of standard support ??
  # note the use of "alvistack" here; Ubuntu stopped providing Vagrant
  # images due to Hashicorp adopting the "Business Source License".
  config.vm.define "questing", autostart: false do |questing|
    questing.vm.box = "alvistack/ubuntu-25.10"
    questing.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    questing.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (questing)"
    end
    questing.vm.boot_timeout = 1200
  end
  config.vm.define "questing-desktop", autostart: false do |questing|
    questing.vm.box = "alvistack/ubuntu-25.10"
    questing.vm.provision :shell, path: "Tools/vagrant/initvagrant-desktop.sh"
    questing.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot (questing-desktop)"
      vb.gui = true
    end
    questing.vm.boot_timeout = 1200
  end
end
