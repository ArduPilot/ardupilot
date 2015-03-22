# -*- mode: ruby -*-
# vi: set ft=ruby :

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|

  config.vm.box = "ubuntu/trusty32"
  # push.app = "geeksville/ardupilot-sitl"

  # The following forwarding is not necessary (or possible), because our sim_vehicle.sh is smart enough to send packets
  # out to the containing OS
  # config.vm.network "forwarded_port", guest: 14550, host: 14550, protocol: "udp"

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
      # NuttX needs symlinks. If you want to go that route you need this setting, but rsync is easier. 
#      vb.customize ["setextradata", :id, "VBoxInternal2/SharedFoldersEnableSymlinksCreate/PX4NuttX", "1"]
      # Make some effort to avoid clock skew
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-threshold", "5000"]
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-start"]
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-on-restore", "1"]
  end

  config.vm.synced_folder "../PX4Firmware", "/PX4Firmware"
  config.vm.synced_folder "../PX4NuttX", "/PX4NuttX", type: "rsync"
  config.vm.synced_folder "../uavcan", "/uavcan"

  config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"  
end

